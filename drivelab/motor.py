"""Motor model.

Deliberately simple: the electrical time constant of a small BLDC is one to two
orders of magnitude faster than anything the transmission does, so current
dynamics are dropped. What is kept is the part that matters for the backdrive
test -- the short-circuit damping k_t^2/R, which is often the dominant
resistance to backdriving a low-ratio drive and is routinely forgotten.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Motor:
    name: str
    inertia: float        # rotor inertia J_m            [kg*m^2]
    tau_cont: float       # continuous torque            [N*m]
    tau_peak: float       # peak torque                  [N*m]
    kt: float             # torque constant              [N*m/A]
    resistance: float     # phase-to-phase resistance R  [ohm]
    v_bus: float = 24.0   # bus voltage                  [V]
    damping: float = 0.0  # viscous drag on the rotor    [N*m/(rad/s)]

    @property
    def ke(self):
        """Back-EMF constant [V/(rad/s)] -- equal to kt in SI."""
        return self.kt

    @property
    def omega_noload(self):
        """No-load speed at bus voltage [rad/s]."""
        return self.v_bus / self.ke

    @property
    def short_circuit_damping(self):
        """Damping seen at the rotor with the phases shorted, kt^2/R
        [N*m/(rad/s)]. This is what a 'braked' unpowered motor feels like."""
        return self.kt ** 2 / self.resistance

    def torque_limit(self, omega, peak=False):
        """Available torque at rotor speed `omega`, respecting the voltage
        ceiling (torque falls linearly to zero at no-load speed)."""
        cap = self.tau_peak if peak else self.tau_cont
        head = 1.0 - abs(omega) / self.omega_noload
        if head <= 0.0:
            return 0.0
        return cap * min(1.0, head)

    def copper_loss(self, tau):
        """Resistive loss [W] for a commanded torque -- the thermal proxy."""
        return (tau / self.kt) ** 2 * self.resistance
