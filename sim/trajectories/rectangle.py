import numpy as np
from .base import BaseTrajectory


class RectangleTrajectory(BaseTrajectory):
    """
    Rounded-corner rectangle on the ground plane at constant speed.

    The rectangle has half-lengths (Lx, Ly) and corner radius r.
    The path consists of 4 straight segments connected by 4 quarter-circle
    arcs, giving a C¹-continuous trajectory (continuous position and
    velocity) with piecewise-constant curvature — zero on straights,
    1/r at corners.

    The target starts on the perimeter (midpoint of the +x side) and moves
    counter-clockwise.

    Benchmarking value:
      - Straight sides  → tests steady-state tracking at constant velocity
      - Rounded corners → tests high-curvature response (tunable via r)
      - Clean separation of easy (straight) vs. hard (corner) segments
    """

    def __init__(self, args):
        amp = args.amplitude if getattr(args, 'amplitude', None) is not None else 3.0

        # Rectangle half-lengths.  Using 2:1 aspect ratio.
        self._Lx = amp            # half-length along x
        self._Ly = amp * 0.5      # half-length along y

        # Corner radius: near-zero for sharp 90° turns.
        # The car is a mocap body (no physics) so it handles the
        # near-instantaneous velocity change — the challenge is on the drone.
        self._r = 0.02

        # Total perimeter = 4 straight segments + 4 quarter-circle arcs
        Sx = self._Lx - self._r   # half straight length along x
        Sy = self._Ly - self._r   # half straight length along y
        self._perimeter = 2 * (2 * Sx + 2 * Sy) + 2 * np.pi * self._r
        # = 4*(Lx + Ly) - 8*r + 2*pi*r  (true rounded-rect perimeter)

        # Compute speed → omega-like parameter
        if getattr(args, 'target_speed', None) is not None:
            self._speed = args.target_speed
        else:
            self._speed = 1.0  # default 1 m/s

        self._period = self._perimeter / self._speed

        # Pre-compute the 8-segment layout: alternating straight, arc, ...
        # Segments (CCW from midpoint of right side):
        #   0: straight up   (+y direction, right side)
        #   1: arc  top-right corner
        #   2: straight left  (−x direction, top side)
        #   3: arc  top-left corner
        #   4: straight down  (−y direction, left side)
        #   5: arc  bottom-left corner
        #   6: straight right (+x direction, bottom side)
        #   7: arc  bottom-right corner
        r  = self._r
        Sx = self._Lx - r
        Sy = self._Ly - r

        # Each segment: (type, length)
        # Straight lengths: 2*Sy for vertical sides, 2*Sx for horizontal sides
        self._segments = [
            ('S', 2 * Sy),          # 0: right side upward
            ('A', 0.5 * np.pi * r), # 1: top-right arc
            ('S', 2 * Sx),          # 2: top side leftward
            ('A', 0.5 * np.pi * r), # 3: top-left arc
            ('S', 2 * Sy),          # 4: left side downward
            ('A', 0.5 * np.pi * r), # 5: bottom-left arc
            ('S', 2 * Sx),          # 6: bottom side rightward
            ('A', 0.5 * np.pi * r), # 7: bottom-right arc
        ]

        # Cumulative arc-length boundaries
        self._cum_lengths = np.zeros(len(self._segments) + 1)
        for i, (_, length) in enumerate(self._segments):
            self._cum_lengths[i + 1] = self._cum_lengths[i] + length

        # Corner centres (Lx-r, Ly-r) etc.
        self._corner_centres = [
            np.array([+(self._Lx - r), +(self._Ly - r)]),  # top-right
            np.array([-(self._Lx - r), +(self._Ly - r)]),  # top-left
            np.array([-(self._Lx - r), -(self._Ly - r)]),  # bottom-left
            np.array([+(self._Lx - r), -(self._Ly - r)]),  # bottom-right
        ]

        # Starting angles for each corner arc (radians, CCW from +x)
        # Top-right:     start at angle 0 (pointing +x → rotate CCW to +y)
        # Top-left:      start at π/2
        # Bottom-left:   start at π
        # Bottom-right:  start at 3π/2
        self._corner_start_angles = [0.0, np.pi / 2, np.pi, 3 * np.pi / 2]

        print(f" Rectangle: {2*self._Lx:.1f}×{2*self._Ly:.1f} m, "
              f"corner_r={self._r:.3f} m, speed={self._speed:.2f} m/s, "
              f"period={self._period:.2f} s")

    def __call__(self, t: float):
        # Map t to arc-length along perimeter
        s = (t * self._speed) % self._perimeter

        # Find which segment we're in
        seg_idx = 0
        for i in range(len(self._segments)):
            if s < self._cum_lengths[i + 1]:
                seg_idx = i
                break
        else:
            seg_idx = len(self._segments) - 1

        s_local = s - self._cum_lengths[seg_idx]
        seg_type, seg_len = self._segments[seg_idx]

        Lx = self._Lx
        Ly = self._Ly
        r  = self._r
        speed = self._speed

        if seg_type == 'S':
            # Straight segment
            if seg_idx == 0:    # right side, going +y
                pos = np.array([Lx, -(Ly - r) + s_local, 0.0])
                vel = np.array([0.0, speed, 0.0])
            elif seg_idx == 2:  # top side, going -x
                pos = np.array([(Lx - r) - s_local, Ly, 0.0])
                vel = np.array([-speed, 0.0, 0.0])
            elif seg_idx == 4:  # left side, going -y
                pos = np.array([-Lx, (Ly - r) - s_local, 0.0])
                vel = np.array([0.0, -speed, 0.0])
            elif seg_idx == 6:  # bottom side, going +x
                pos = np.array([-(Lx - r) + s_local, -Ly, 0.0])
                vel = np.array([speed, 0.0, 0.0])
            else:
                pos = np.zeros(3)
                vel = np.zeros(3)
        else:
            # Arc segment (quarter circle)
            # Map seg_idx to corner index: seg 1→0, 3→1, 5→2, 7→3
            corner_idx = seg_idx // 2
            centre = self._corner_centres[corner_idx]
            start_angle = self._corner_start_angles[corner_idx]

            # Angle swept so far within this arc (CCW)
            theta = start_angle + s_local / r

            pos = np.array([
                centre[0] + r * np.cos(theta),
                centre[1] + r * np.sin(theta),
                0.0,
            ])
            # Velocity is tangent to arc (CCW = perpendicular to radius)
            vel = speed * np.array([
                -np.sin(theta),
                 np.cos(theta),
                 0.0,
            ])

        return pos, vel

    @property
    def period(self) -> float:
        return self._period

    @property
    def name(self) -> str:
        return "rectangle"
