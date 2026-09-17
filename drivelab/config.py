"""A `Config` binds one motor, one drive and one plant into a testable system.

Two ways to build one, matching the two study modes:

  Config(motor, drive, plant)     hardware-informed -- real parts from the
                                  catalog, SI throughout, results carry units.

  Config.from_pi_groups(...)      synthetic -- specify the six dimensionless
                                  numbers directly and get a system that
                                  realizes them. Useful for asking "what would
                                  a drive with these properties do?" without
                                  committing to a way of building it.

Both land in the same place, because the bench only ever reads the Pi groups.
"""

import math
from dataclasses import dataclass, field

from .scaling import Scales, PiGroups, pi_groups


@dataclass
class Config:
    motor: object
    drive: object
    plant: object
    name: str = ""

    def __post_init__(self):
        if not self.name:
            self.name = "%s + %s" % (self.drive.name, self.motor.name)

    # ---- derived ---------------------------------------------------------

    @property
    def scales(self):
        return Scales.from_plant(self.plant)

    @property
    def pi(self):
        return pi_groups(self.drive, self.motor, self.plant)

    @property
    def reflected_inertia(self):
        """N^2 J_m plus the drive's own, output-referred [kg*m^2]."""
        return (self.drive.N ** 2 * self.motor.inertia
                + self.drive.reflected_inertia_extra())

    @property
    def total_inertia(self):
        """Rigid-body inertia seen at the output [kg*m^2]."""
        return self.plant.inertia + self.reflected_inertia

    @property
    def tau_motor_max_out(self):
        """Peak motor torque referred to the output [N*m]."""
        return self.drive.N * self.motor.tau_peak

    @property
    def tau_motor_cont_out(self):
        return self.drive.N * self.motor.tau_cont

    @property
    def torque_headroom(self):
        """Continuous motor torque at the output, over peak gravity load.
        Below 1.0 the arm cannot hold itself up horizontally."""
        return self.tau_motor_cont_out / self.plant.gravity_moment

    @property
    def drive_torque_margin(self):
        """Drive capacity over peak gravity load."""
        return self.drive.tau_max() / self.plant.gravity_moment

    def feasibility(self):
        """Structural checks that a metric table would otherwise hide.

        A drive that cannot hold the arm up, or whose stroke cannot reach the
        commanded angle, will still produce plausible-looking plots. These are
        the checks that stop a comparison from quietly being nonsense.
        """
        issues = []
        if self.torque_headroom < 1.0:
            issues.append(
                "motor cannot hold the arm horizontal (%.2fx gravity)"
                % self.torque_headroom)
        if self.drive_torque_margin < 1.0:
            issues.append(
                "drive capacity below peak gravity torque (%.2fx)"
                % self.drive_torque_margin)
        stroke = self.drive.stroke_limit()
        if stroke is not None and stroke < math.pi / 2.0:
            issues.append("stroke limit +/-%.0f deg is under +/-90 deg"
                          % math.degrees(stroke))
        if hasattr(self.drive, "bend_ratio_ok") and not self.drive.bend_ratio_ok():
            issues.append("capstan D/d = %.0f is below the cable minimum %.0f"
                          % (self.drive.bend_ratio,
                             self.drive.cable.min_bend_ratio))
        return issues

    # ---- construction from dimensionless numbers -------------------------

    @classmethod
    def from_pi_groups(cls, Pi_J, Omega_n, zeta_d=0.02, eta_f=0.9,
                       tau_max_hat=float("inf"), err_amp_arcmin=0.0,
                       backlash_arcmin=0.0, no_load_drag_hat=0.0,
                       name="synthetic"):
        """Build a unit-scaled system realizing the given Pi groups.

        The plant is normalized so that m g L = 1 N*m and J_l = 1 kg*m^2, which
        makes the characteristic time 1 s and every SI result numerically equal
        to its dimensionless counterpart.
        """
        from .motor import Motor
        from .plant import LeverArm
        from .drives.ideal import GenericGearedDrive
        from .plant import G

        # Choose L and m so that m g L = 1 and m L^2 = 1  =>  L = g, m = 1/g^2.
        L = G
        m = 1.0 / (G ** 2)
        plant = LeverArm(name="unit", length=L, payload_mass=m)

        # N = 1 makes the reflected inertia condition N^2 J_m = Pi_J trivial.
        motor = Motor(name="unit", inertia=Pi_J, tau_cont=1e6, tau_peak=1e6,
                      kt=1.0, resistance=1.0, v_bus=1e6)
        K = Omega_n ** 2 * plant.gravity_moment
        damping = zeta_d * 2.0 * math.sqrt(K * plant.inertia)
        drive = GenericGearedDrive(
            N=1.0, eta_forward=eta_f, stiffness=K,
            no_load_drag=no_load_drag_hat * plant.gravity_moment,
            backlash=math.radians(backlash_arcmin / 60.0),
            error_amplitude=math.radians(err_amp_arcmin / 60.0),
            damping=damping, inertia_extra=0.0, name=name,
            inertia_ref=plant.inertia)
        drive._tau_max = tau_max_hat * plant.gravity_moment
        drive.tau_max = lambda: drive._tau_max
        return cls(motor=motor, drive=drive, plant=plant, name=name)

    def summary(self):
        d = {"config": self.name}
        d.update(self.drive.summary())
        d.update(self.pi.as_dict())
        d.update({
            "J_reflected": self.reflected_inertia,
            "J_link": self.plant.inertia,
            "tau_scale_Nm": self.plant.gravity_moment,
            "time_scale_s": self.scales.time,
            "torque_headroom": self.torque_headroom,
            "drive_torque_margin": self.drive_torque_margin,
        })
        return d
