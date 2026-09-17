"""drivelab -- a standardized bench for comparing rotary drive transmissions.

The premise: every transmission, however it is built, is a *two-port element*
between a motor and a load. It is fully characterized by five behaviours:

  1. kinematic map      theta_out = theta_m/N + e(theta_m)   (ratio + error)
  2. compliance         K(delta)                             (series stiffness)
  3. loss               tau_loss(tau_j, omega)               (efficiency, both ways)
  4. reflected inertia  N^2 * J_motor
  5. limits             tau_max, stroke, slip

Every drive in `drivelab.drives` implements exactly that interface, so the same
test bench, the same plant and the same metrics apply to all of them.

Internals are SI. Analysis is dimensionless -- see `drivelab.scaling`.
"""

__version__ = "0.1.0"

from .scaling import Scales, PiGroups
from .plant import LeverArm
from .motor import Motor
from .config import Config
from .sim import simulate, SimResult

__all__ = [
    "Scales", "PiGroups", "LeverArm", "Motor", "Config",
    "simulate", "SimResult",
]
