"""The standardized test plant: a lever arm on a revolute joint.

One rigid link of length L with a point payload at the tip, rotating in a
vertical plane. theta = 0 places the arm horizontal, i.e. at *maximum* gravity
load -- the worst case, and the natural reference for a robot joint.

This plant is the common ground of the whole bench. It is identical for every
drive under test, and every characteristic scale is taken from it.
"""

import math
from dataclasses import dataclass

G = 9.80665


@dataclass(frozen=True)
class LeverArm:
    name: str
    length: float          # L, joint to payload        [m]
    payload_mass: float    # point mass at the tip      [kg]
    link_mass: float = 0.0 # uniform rod mass           [kg]
    damping: float = 0.0   # viscous drag at the joint  [N*m/(rad/s)]

    @property
    def inertia(self):
        """J_l about the joint [kg*m^2] -- point mass plus uniform rod."""
        return (self.payload_mass * self.length ** 2
                + self.link_mass * self.length ** 2 / 3.0)

    @property
    def gravity_moment(self):
        """m g L, the peak gravity torque at theta = 0 [N*m]. This is the
        bench's torque scale."""
        return (self.payload_mass * self.length
                + self.link_mass * self.length / 2.0) * G

    @property
    def omega_scale(self):
        """sqrt(m g L / J_l) [rad/s]."""
        return math.sqrt(self.gravity_moment / self.inertia)

    def gravity_torque(self, theta):
        """Joint torque from gravity [N*m]; theta measured from horizontal."""
        return -self.gravity_moment * math.cos(theta)

    def torque(self, theta, omega):
        return self.gravity_torque(theta) - self.damping * omega
