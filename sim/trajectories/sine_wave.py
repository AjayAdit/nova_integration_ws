import numpy as np
from .base import BaseTrajectory

class SineWaveTrajectory(BaseTrajectory):
    """
    Sine wave trajectory moving back and forth along the X-axis.
    
    x(t) = moves back and forth between -x_amp and +x_amp at a constant speed
    y(t) = amplitude * sin(2 * pi * x(t) / wavelength)
    """

    def __init__(self, args):
        self._amp = args.amplitude if getattr(args, 'amplitude', None) is not None else 2.0
        self._wavelength = getattr(args, 'wavelength', 30.0)
        
        if getattr(args, 'target_speed', None) is not None:
            self._speed = args.target_speed
        else:
            self._speed = 3.0  # default 3 m/s
            
        self._x_amp = 40.0 # How far it travels along X before bouncing back
        self._period = 4.0 * self._x_amp / self._speed
        
        print(f" SineWave: speed={self._speed:.2f} m/s  "
              f"y-amplitude={self._amp:.2f} m  wavelength={self._wavelength:.2f} m")

    def __call__(self, t: float):
        T = self._period
        phase = t % T
        quarter_T = T / 4.0
        three_quarter_T = 3.0 * T / 4.0

        if phase < quarter_T:
            # Moving forward: 0 → +x_amp
            x = self._speed * phase
            vx = self._speed
        elif phase < three_quarter_T:
            # Moving backward: +x_amp → -x_amp
            x = self._x_amp - self._speed * (phase - quarter_T)
            vx = -self._speed
        else:
            # Moving forward again: -x_amp → 0
            x = -self._x_amp + self._speed * (phase - three_quarter_T)
            vx = self._speed

        # y(t) = A * sin(k * x)
        k = 2.0 * np.pi / self._wavelength
        y = self._amp * np.sin(k * x)
        
        # dy/dt = dy/dx * dx/dt
        # dy/dt = A * k * cos(k * x) * vx
        vy = self._amp * k * np.cos(k * x) * vx

        pos = np.array([x, y, 0.0])
        vel = np.array([vx, vy, 0.0])
        return pos, vel

    @property
    def period(self) -> float:
        return self._period

    @property
    def name(self) -> str:
        return "sine_wave"
