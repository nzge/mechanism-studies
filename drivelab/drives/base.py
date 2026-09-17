"""The two-port transmission interface.

Every drive -- capstan, harmonic, cycloidal, belt, ideal gearbox -- is the same
kind of object: a two-port element sitting between a motor and a load. It is
completely described by five behaviours, and this class is just those five
behaviours with names:

  1. ratio and kinematic error   N, e(theta)
  2. compliance                  stiffness_out(delta), damping_out()
  3. loss                        the friction model
  4. reflected inertia           N^2 J_m + the drive's own
  5. limits                      tau_max(), stroke_limit()

Everything in the bench is written against this interface, so adding a new
transmission means implementing these and nothing else. No test, metric or plot
needs to know which drive it is looking at.

State convention
----------------
All angles and torques are **output-referred**: theta_m_out = theta_m / N, and
motor torque is reported as N * tau_m. This keeps every quantity comparable to
the load without a ratio floating through the equations.

Every drive carries the same two extra states, in this order:

    extra[0] = theta_slip   accumulated irreversible slip [rad, output side]
    extra[1] = z            LuGre bristle deflection      [rad]

Drives that cannot slip simply return zero for the first derivative. A fixed
layout costs one unused state and buys a much simpler integrator.
"""

import math

SLIP, BRISTLE = 0, 1
N_EXTRA = 2


class DriveModel(object):
    """Base class. Subclasses override the hooks; the bench only calls these."""

    name = "drive"
    physics_derived = False   # True if parameters come from geometry+material
                              # rather than from a datasheet. Reported in
                              # comparisons so the provenance stays visible.

    def __init__(self, N, eta_forward, friction):
        if N <= 0:
            raise ValueError("reduction ratio N must be positive")
        self.N = float(N)
        self.eta_forward = float(eta_forward)
        self.friction = friction

    # ---- 1. kinematics ---------------------------------------------------

    def kinematic_error(self, theta_m_out):
        """Transmission error e [rad at the output]. Zero for an ideal drive;
        this is where each technology's signature ripple lives."""
        return 0.0

    def d_kinematic_error(self, theta_m_out):
        """de/dtheta -- needed for the velocity map, not just position."""
        return 0.0

    def error_amplitude(self):
        """Peak kinematic error [rad at output], for the summary table."""
        return 0.0

    # ---- 2. compliance ---------------------------------------------------

    def stiffness_out(self, delta):
        """Output-referred torsional stiffness [N*m/rad]. May depend on the
        windup `delta` (harmonic drives are markedly softer near zero)."""
        raise NotImplementedError

    def damping_out(self):
        """Structural damping in the drive [N*m/(rad/s)], output-referred."""
        return 0.0

    def damping_ratio(self, inertia_load):
        k = self.stiffness_out(0.0)
        if k <= 0.0:
            return 0.0
        return self.damping_out() / (2.0 * math.sqrt(k * inertia_load))

    def spring_torque(self, delta):
        """Elastic torque through the drive [N*m]. Override to add backlash."""
        return self.stiffness_out(delta) * delta

    # ---- 3. loss ---------------------------------------------------------

    def loss_torque(self, omega_m_out, tau_j, z):
        """Loss torque opposing motor-side rotation, output-referred [N*m].

        Note the velocity argument is the *motor* speed, not the relative speed
        across the compliance. Gear teeth and cable wraps slide in proportion to
        throughput; a rigid drive turning at constant speed still dissipates.
        """
        return self.friction.torque(omega_m_out, tau_j, z)

    def dz(self, omega_m_out, tau_j, z):
        return self.friction.dz(omega_m_out, tau_j, z)

    def breakaway_torque(self, tau_j=0.0):
        return self.friction.breakaway(tau_j)

    # ---- 4. inertia ------------------------------------------------------

    def reflected_inertia_extra(self):
        """The drive's own inertia, output-referred [kg*m^2] -- pulleys,
        flexspline, eccentric. Excludes the N^2 J_motor term."""
        return 0.0

    # ---- 5. limits -------------------------------------------------------

    def tau_max(self):
        """Peak output torque the drive can transmit [N*m]."""
        raise NotImplementedError

    def stroke_limit(self):
        """Output travel limit [rad], or None for continuous rotation. This is
        a hard categorical difference no efficiency figure captures."""
        return None

    def slip_rate(self, tau_spring, omega_rel):
        """d(theta_slip)/dt [rad/s]. Non-zero only for drives that can slip."""
        return 0.0

    # ---- evaluation ------------------------------------------------------

    def evaluate(self, theta_m_out, omega_m_out, theta_l, omega_l, extra):
        """Return (tau_j, tau_loss, extra_dot).

        tau_j is the torque delivered to the load; tau_loss opposes the motor.
        """
        th_slip, z = extra[SLIP], extra[BRISTLE]
        e = self.kinematic_error(theta_m_out)
        de = self.d_kinematic_error(theta_m_out)

        delta = theta_m_out + e - theta_l - th_slip
        tau_spring = self.spring_torque(delta)

        # Provisional transmitted torque, with slip not yet resolved. The slip
        # criterion is evaluated on this rather than on the spring term alone:
        # a friction coupling cannot transmit more than its grip limit however
        # fast it is being deformed, so capping only the elastic part lets an
        # impact push arbitrary torque through the damper. Using the
        # provisional value keeps the system explicit, at the cost of resolving
        # the onset of slip one step late.
        ddelta0 = omega_m_out * (1.0 + de) - omega_l
        tau_raw = tau_spring + self.damping_out() * ddelta0

        d_slip = self.slip_rate(tau_raw, omega_m_out - omega_l)

        ddelta = ddelta0 - d_slip
        tau_j = tau_spring + self.damping_out() * ddelta
        cap = self.tau_max()
        if d_slip != 0.0 and abs(tau_j) > cap:
            tau_j = math.copysign(cap, tau_j)

        tau_loss = self.loss_torque(omega_m_out, tau_j, z)
        dz = self.dz(omega_m_out, tau_j, z)
        return tau_j, tau_loss, (d_slip, dz)

    # ---- reporting -------------------------------------------------------

    def efficiency_at(self, tau_out):
        """Efficiency at a given output torque.

        Worth plotting rather than quoting as a single number: the no-load drag
        term means every drive is inefficient at low load, and drives differ
        enormously in how fast they recover. A gearbox whose loss is mostly
        load-proportional has a flat curve; a preload-dominated drive like a
        capstan has a curve that climbs steeply with load.
        """
        if abs(tau_out) <= 0.0:
            return 0.0
        loss = self.friction.tau_c0 + self.friction.c * abs(tau_out)
        return abs(tau_out) / (abs(tau_out) + loss)

    def summary(self):
        from ..friction import eta_backward
        return {
            "name": self.name,
            "N": self.N,
            "eta_forward": self.eta_forward,
            "eta_backward": eta_backward(self.eta_forward),
            "self_locking": self.eta_forward <= 0.5,
            "K_out": self.stiffness_out(0.0),
            "tau_max": self.tau_max(),
            "stroke_limit_deg": (None if self.stroke_limit() is None
                                 else math.degrees(self.stroke_limit())),
            "error_arcmin": math.degrees(self.error_amplitude()) * 60.0,
            "no_load_drag": self.friction.tau_c0,
            "physics_derived": self.physics_derived,
        }

    def __repr__(self):
        return "<%s N=%.1f eta_f=%.3f K=%.3g>" % (
            self.__class__.__name__, self.N, self.eta_forward,
            self.stiffness_out(0.0))
