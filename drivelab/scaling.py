"""Non-dimensionalization.

Characteristic scales are taken from the *plant*, never from the drive -- that
is what makes the comparison fair. Two drives are compared on a load that does
not know which drive is attached to it.

    torque scale   tau*   = m g L        peak gravity torque of the lever arm
    time scale     t*     = sqrt(J_l / tau*)
    angle scale    1 rad

The dimensionless equations of motion are then

    Pi_J * thm'' = tau_m_hat - tau_j_hat - tau_loss_hat
          thl''  = tau_j_hat - cos(thl) - 2 zeta_l thl' + tau_ext_hat
        tau_j_hat = Om_n^2 * delta + 2 zeta_d Om_n * delta'

so the *entire* behaviour of any drive is captured by:

    Pi_J    = N^2 J_m / J_l          reflected inertia ratio
    Om_n    = sqrt(K_out / (m g L))  stiffness, in units of the plant time scale
    zeta_d                           drive internal damping
    eta_f                            forward efficiency  (backward follows)
    tau_max_hat = tau_max / (m g L)  torque margin
    e(.)                             kinematic error function, rad at output

Any two transmissions that land on the same six numbers are indistinguishable
to the load. That is the claim the bench is built to test.
"""

import math
from dataclasses import dataclass, asdict


@dataclass(frozen=True)
class Scales:
    """Characteristic scales derived from the plant."""
    tau: float      # N*m   -- peak gravity torque m g L
    inertia: float  # kg*m2 -- link inertia about the joint
    time: float     # s     -- sqrt(J_l / tau)

    @classmethod
    def from_plant(cls, plant):
        tau = plant.gravity_moment          # m g L  [N*m]
        J = plant.inertia                   # J_l    [kg*m^2]
        if tau <= 0.0:
            # Gravity-free plant: fall back on a unit torque scale so the
            # normalization stays well defined (useful for horizontal-plane
            # test rigs where gravity does no work on the joint).
            tau = 1.0
        return cls(tau=tau, inertia=J, time=math.sqrt(J / tau))

    @property
    def omega(self):
        """Angular velocity scale [rad/s] -- also the small-oscillation
        frequency of the lever arm hanging under gravity."""
        return 1.0 / self.time

    @property
    def power(self):
        return self.tau / self.time

    @property
    def energy(self):
        return self.tau  # tau * 1 rad

    def nd_torque(self, t):
        return t / self.tau

    def nd_time(self, t):
        return t / self.time

    def nd_omega(self, w):
        return w * self.time

    def nd_stiffness(self, k):
        return k / self.tau

    def nd_energy(self, e):
        return e / self.energy


@dataclass(frozen=True)
class PiGroups:
    """The six numbers that fully describe a drive as seen by the load."""
    Pi_J: float          # N^2 J_m / J_l          reflected inertia ratio
    Omega_n: float       # sqrt(K_out / (m g L))  normalized stiffness
    zeta_d: float        # drive damping ratio
    eta_f: float         # forward efficiency
    tau_max_hat: float   # tau_max / (m g L)      torque margin
    err_amp_arcmin: float  # peak kinematic error at the output [arcmin]

    # Derived, reported for convenience -- not independent.
    @property
    def eta_b(self):
        """Backdriving efficiency implied by eta_f under the load-proportional
        loss model. Negative values mean self-locking."""
        if self.eta_f <= 0.0:
            return float("-inf")
        return 2.0 - 1.0 / self.eta_f

    @property
    def self_locking(self):
        return self.eta_f <= 0.5

    @property
    def f_resonance_ratio(self):
        """Ratio of the drive-on-load resonance to the plant time scale,
        including the reflected inertia (series inertia + spring + inertia)."""
        if self.Pi_J <= 0.0:
            return self.Omega_n
        return self.Omega_n * math.sqrt(1.0 + 1.0 / self.Pi_J)

    def as_dict(self):
        d = asdict(self)
        d["eta_b"] = self.eta_b
        d["self_locking"] = self.self_locking
        d["f_resonance_ratio"] = self.f_resonance_ratio
        return d


def pi_groups(drive, motor, plant):
    """Reduce a (drive, motor, plant) triple to its Pi groups."""
    s = Scales.from_plant(plant)
    J_refl = drive.N ** 2 * motor.inertia + drive.reflected_inertia_extra()
    K = drive.stiffness_out(0.0)
    return PiGroups(
        Pi_J=J_refl / plant.inertia,
        Omega_n=math.sqrt(K / s.tau),
        zeta_d=drive.damping_ratio(plant.inertia),
        eta_f=drive.eta_forward,
        tau_max_hat=drive.tau_max() / s.tau,
        err_amp_arcmin=drive.error_amplitude() * 180.0 / math.pi * 60.0,
    )
