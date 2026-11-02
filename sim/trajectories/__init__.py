from .lissajous     import LissajousTrajectory
from .circle         import CircleTrajectory
from .zigzag         import ZigzagTrajectory
from .straight_line  import StraightLineTrajectory
from .sine_wave      import SineWaveTrajectory
from .rectangle      import RectangleTrajectory

# Wrapper, not a selectable path — deliberately kept out of REGISTRY.
from .speed_ramp     import SpeedRampTrajectory

# Maps --trajectory CLI value → class.
# To add a new trajectory: create trajectories/my_new.py and add one line here.
REGISTRY = {
    "lissajous":     LissajousTrajectory,
    "circle":        CircleTrajectory,
    "zigzag":        ZigzagTrajectory,
    "straight_line": StraightLineTrajectory,
    "sine_wave":     SineWaveTrajectory,
    "rectangle":     RectangleTrajectory,
}
