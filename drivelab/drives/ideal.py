"""Reference drives used to validate the bench itself.

These exist so that every test has a known-answer case. If T1 does not report
zero lost motion for a RigidIdealDrive, the test is broken, not the drive.
"""

import math
from .base import DriveModel
from ..friction import make_friction


class RigidIdealDrive(DriveModel):
    """Lossless, infinitely stiff, no error, no limits.

    The analytical check: the whole system must reduce to a single rigid
    pendulum of inertia (J_l + N^2 J_m) with no dissipation, oscillating at
    sqrt(m g L / (J_l + N^2 J_m)). Any test that disagrees is wrong.
    """

    physics_derived = True   # trivially

    def __init__(self, N=1.0, stiffness=1.0e9, name="ideal-rigid"):
        self.name = name
        self._k = stiffness
        DriveModel.__init__(
            self, N=N, eta_forward=1.0,
            friction=make_friction("regularized", tau_c0=0.0,
                                   eta_forward=1.0, viscous=0.0))

    def stiffness_out(self, delta=0.0):
        return self._k

    def tau_max(self):
        return float("inf")


class CompliantIdealDrive(DriveModel):
    """Lossless but finitely stiff -- a pure two-inertia resonant system.

    The analytical check: the free system has one rigid-body mode and one
    resonance at sqrt(K/J_eff) with J_eff the series combination of the load
    inertia and the reflected motor inertia. T5 must find that frequency.
    """

    physics_derived = True

    def __init__(self, N, stiffness, damping=0.0, name="ideal-compliant"):
        self.name = name
        self._k = stiffness
        self._c = damping
        DriveModel.__init__(
            self, N=N, eta_forward=1.0,
            friction=make_friction("regularized", tau_c0=0.0,
                                   eta_forward=1.0, viscous=0.0))

    def stiffness_out(self, delta=0.0):
        return self._k

    def damping_out(self):
        return self._c

    def tau_max(self):
        return float("inf")


class GenericGearedDrive(DriveModel):
    """A lossy geared stage described by *datasheet* numbers.

    Read the provenance flag: unlike CapstanDrive, nothing here is derived from
    geometry. You hand it a ratio, an efficiency, a stiffness, a backlash and a
    ripple amplitude, and it behaves accordingly. That makes it useful for
    putting a real harmonic or cycloidal gearbox on the bench today using its
    published figures -- and it makes it useless as evidence about *why* those
    figures come out the way they do.

    Keeping this distinction visible is the whole reason `physics_derived`
    exists. A comparison plot that mixes derived and datasheet drives is still
    valid; it just answers a different question.

    The one thing it does not let you fake is backdrivability: efficiency in
    both directions is locked together through the loss model, so eta_b follows
    from eta_f rather than being a second free knob.
    """

    physics_derived = False

    def __init__(self, N, eta_forward, stiffness, no_load_drag=0.0,
                 backlash=0.0, error_amplitude=0.0, error_cycles=2.0,
                 inertia_extra=0.0, structural_damping_ratio=0.02,
                 tau_rated=float("inf"), stiction_ratio=1.6,
                 friction_model="lugre", name="geared", inertia_ref=1e-4):
        self.name = name
        self._k = stiffness
        self._tau_rated = tau_rated
        # Damping is referred to the same inertia that damping_ratio()
        # normalizes by, so the zeta asked for is the zeta reported. An
        # inconsistency here quietly hands one drive extra damping in every
        # transient test.
        self._c = (structural_damping_ratio * 2.0
                   * math.sqrt(stiffness * inertia_ref))
        self._backlash = float(backlash)          # total, rad at output
        self._err_amp = float(error_amplitude)    # rad at output
        self._err_cycles = float(error_cycles)    # cycles per motor rev
        self._inertia_extra = inertia_extra
        DriveModel.__init__(
            self, N=N, eta_forward=eta_forward,
            friction=make_friction(
                friction_model, tau_c0=no_load_drag,
                eta_forward=eta_forward, viscous=0.0,
                stiction_ratio=stiction_ratio, omega_stribeck=0.01,
                presliding=5.0e-5, tau_ref=max(no_load_drag, 1e-6),
                inertia_ref=inertia_ref))

    def kinematic_error(self, theta_m_out):
        if self._err_amp == 0.0:
            return 0.0
        # Ripple is periodic in *motor* revolutions; theta_m_out is the motor
        # angle referred to the output, so the motor angle is N * theta_m_out.
        return self._err_amp * math.sin(self._err_cycles * self.N
                                        * theta_m_out)

    def d_kinematic_error(self, theta_m_out):
        if self._err_amp == 0.0:
            return 0.0
        w = self._err_cycles * self.N
        return self._err_amp * w * math.cos(w * theta_m_out)

    def error_amplitude(self):
        return self._err_amp

    def spring_torque(self, delta):
        """Deadband backlash: no torque until the teeth take up."""
        half = self._backlash / 2.0
        if delta > half:
            engaged = delta - half
        elif delta < -half:
            engaged = delta + half
        else:
            return 0.0
        return self._k * engaged

    def stiffness_out(self, delta=0.0):
        return self._k

    def damping_out(self):
        return self._c

    def reflected_inertia_extra(self):
        return self._inertia_extra

    def tau_max(self):
        return self._tau_rated

    def summary(self):
        d = DriveModel.summary(self)
        d["backlash_arcmin"] = math.degrees(self._backlash) * 60.0
        return d
