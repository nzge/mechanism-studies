"""The integrator. One set of equations, every drive.

Everything is output-referred, so the reduction ratio does not appear in the
equations of motion at all -- it is folded into the reflected inertia and into
how motor torque is reported. This is deliberate: it makes two drives with
different ratios directly comparable on the same axes.

    J_refl * w_m' = N*tau_m - tau_j - tau_loss
    J_l    * w_l' = tau_j + tau_gravity(th_l) - b_l*w_l + tau_ext
        tau_j     = spring(delta) + C*delta'
        delta     = th_m + e(th_m) - th_l - th_slip

State vector (6):
    [ th_m_out, w_m_out, th_l, w_l, th_slip, z ]

The solver is Radau by default. The system is genuinely stiff -- a steel-cable
capstan can be four orders of magnitude stiffer than the load dynamics, and the
LuGre bristle adds another fast mode -- so an explicit solver will either crawl
or lie.
"""

import math
import numpy as np
from dataclasses import dataclass, field
from scipy.integrate import solve_ivp

from .drives.base import SLIP, BRISTLE

TH_M, W_M, TH_L, W_L, TH_SLIP, Z = range(6)
N_STATES = 6


# ---------------------------------------------------------------------------
# Motor-side inputs
# ---------------------------------------------------------------------------

class Controller(object):
    """Returns motor-shaft torque [N*m] (not output-referred)."""
    name = "none"

    def torque(self, t, x, cfg):
        return 0.0

    def reference(self, t):
        return None


class FreeMotor(Controller):
    """Unpowered, open circuit. The most permissive backdrive condition."""
    name = "free"


class ShortedMotor(Controller):
    """Unpowered with the phases shorted -- kt^2/R damping at the rotor.

    Routinely left out of backdrive analysis, and for a low-ratio drive it is
    often the *dominant* resistance: the mechanism backdrives easily and the
    motor does not.
    """
    name = "shorted"

    def torque(self, t, x, cfg):
        omega_rotor = x[W_M] * cfg.drive.N
        return -cfg.motor.short_circuit_damping * omega_rotor


class TorqueProfile(Controller):
    """Open-loop motor torque, tau(t) at the motor shaft."""
    name = "open-loop"

    def __init__(self, fn, name="open-loop"):
        self.fn = fn
        self.name = name

    def torque(self, t, x, cfg):
        return self.fn(t)


class PDController(Controller):
    """PD on the *link* angle, with a standardized gain rule.

    Gains are placed for a target closed-loop bandwidth expressed as a multiple
    of the plant time scale, using only the rigid-body inertia. The same alpha
    is used for every drive under test, deliberately: a drive that cannot
    support the bandwidth reveals it by ringing or going unstable, and that is
    a finding rather than a nuisance. Tuning each drive to its own limit would
    hide exactly the difference the test is looking for.
    """
    name = "pd"

    def __init__(self, ref, alpha=3.0, zeta=0.8, gravity_ff=False,
                 name="pd"):
        self.ref = ref            # callable t -> (theta, dtheta)
        self.alpha = alpha
        self.zeta = zeta
        self.gravity_ff = gravity_ff
        self.name = name
        self._kp = None
        self._kd = None

    def gains(self, cfg):
        """Output-referred kp [N*m/rad], kd [N*m/(rad/s)]."""
        if self._kp is None:
            J = cfg.total_inertia
            wc = self.alpha * cfg.plant.omega_scale
            self._kp = J * wc ** 2
            self._kd = 2.0 * self.zeta * J * wc
        return self._kp, self._kd

    def reference(self, t):
        return self.ref(t)

    def torque(self, t, x, cfg):
        kp, kd = self.gains(cfg)
        th_ref, dth_ref = self.ref(t)
        tau_out = kp * (th_ref - x[TH_L]) + kd * (dth_ref - x[W_L])
        if self.gravity_ff:
            tau_out -= cfg.plant.gravity_torque(x[TH_L])
        return tau_out / cfg.drive.N


class MotorPositionLock(Controller):
    """A very stiff servo on the *motor* angle. Used by the lost-motion test to
    hold the input while the output is pushed around."""
    name = "motor-lock"

    def __init__(self, target=0.0, bandwidth_hz=200.0, zeta=1.0):
        self.target = target
        self.w = 2.0 * math.pi * bandwidth_hz
        self.zeta = zeta

    def torque(self, t, x, cfg):
        J = cfg.motor.inertia * cfg.drive.N ** 2
        kp = J * self.w ** 2
        kd = 2.0 * self.zeta * J * self.w
        tgt = self.target(t) if callable(self.target) else self.target
        return (kp * (tgt - x[TH_M]) - kd * x[W_M]) / cfg.drive.N


# ---------------------------------------------------------------------------
# Load-side inputs
# ---------------------------------------------------------------------------

def no_load(t, th, w):
    return 0.0


class VirtualWall(object):
    """A stiff unilateral contact at `angle`. Used for the impact test.

    Stiffness defaults to a hard surface relative to the plant; the point of
    the test is that peak transmitted torque depends on the *drive*, chiefly
    through reflected inertia, not on this number.
    """

    def __init__(self, angle, stiffness, damping=0.0, side=+1):
        self.angle = angle
        self.k = stiffness
        self.c = damping
        self.side = side

    def __call__(self, t, th, w):
        pen = self.side * (th - self.angle)
        if pen <= 0.0:
            return 0.0
        f = -self.side * (self.k * pen + self.c * self.side * w)
        # Contact cannot pull.
        if self.side * f > 0.0:
            return 0.0
        return f


class TorqueRamp(object):
    """Slow external torque ramp at the output -- the backdrive probe."""

    def __init__(self, rate, t_start=0.0, limit=None):
        self.rate = rate
        self.t_start = t_start
        self.limit = limit

    def __call__(self, t, th, w):
        tau = self.rate * max(t - self.t_start, 0.0)
        if self.limit is not None:
            tau = min(tau, self.limit)
        return tau


# ---------------------------------------------------------------------------
# Integration
# ---------------------------------------------------------------------------

def _rhs(t, x, cfg, controller, tau_ext, apply_motor_limit):
    drive, motor, plant = cfg.drive, cfg.motor, cfg.plant

    tau_j, tau_loss, (d_slip, dz) = drive.evaluate(
        x[TH_M], x[W_M], x[TH_L], x[W_L], (x[TH_SLIP], x[Z]))

    tau_m = controller.torque(t, x, cfg)
    if apply_motor_limit:
        lim = motor.torque_limit(x[W_M] * drive.N, peak=True)
        if lim > 0.0:
            tau_m = max(-lim, min(lim, tau_m))
        else:
            tau_m = 0.0

    J_refl = cfg.reflected_inertia
    dw_m = (drive.N * tau_m - tau_j - tau_loss
            - motor.damping * drive.N ** 2 * x[W_M]) / J_refl

    ext = tau_ext(t, x[TH_L], x[W_L])
    dw_l = (tau_j + plant.gravity_torque(x[TH_L])
            - plant.damping * x[W_L] + ext) / plant.inertia

    return [x[W_M], dw_m, x[W_L], dw_l, d_slip, dz]


@dataclass
class SimResult:
    t: np.ndarray
    x: np.ndarray
    cfg: object
    controller: object
    tau_ext_fn: object = None
    label: str = ""
    success: bool = True
    message: str = ""
    motor_limited: bool = True

    # ---- raw channels ----------------------------------------------------

    @property
    def theta_m(self):
        """Motor angle referred to the output [rad]."""
        return self.x[TH_M]

    @property
    def omega_m(self):
        return self.x[W_M]

    @property
    def theta_l(self):
        return self.x[TH_L]

    @property
    def omega_l(self):
        return self.x[W_L]

    @property
    def theta_slip(self):
        return self.x[TH_SLIP]

    def _recompute(self):
        n = len(self.t)
        tau_j = np.zeros(n)
        tau_loss = np.zeros(n)
        tau_m = np.zeros(n)
        ext = np.zeros(n)
        for i in range(n):
            xi = self.x[:, i]
            tj, tl, _ = self.cfg.drive.evaluate(
                xi[TH_M], xi[W_M], xi[TH_L], xi[W_L], (xi[TH_SLIP], xi[Z]))
            tau_j[i] = tj
            tau_loss[i] = tl
            cmd = self.controller.torque(self.t[i], xi, self.cfg)
            if self.motor_limited:
                # Report what the motor actually delivered, not what the
                # controller asked for. Without this a saturated drive reports
                # a peak torque its motor cannot produce, and every derived
                # quantity -- utilization, copper loss, energy -- inherits the
                # error.
                lim = self.cfg.motor.torque_limit(xi[W_M] * self.cfg.drive.N,
                                                  peak=True)
                cmd = max(-lim, min(lim, cmd)) if lim > 0.0 else 0.0
            tau_m[i] = cmd
            if self.tau_ext_fn is not None:
                ext[i] = self.tau_ext_fn(self.t[i], xi[TH_L], xi[W_L])
        return tau_j, tau_loss, tau_m, ext

    @property
    def channels(self):
        if not hasattr(self, "_ch"):
            tj, tl, tm, ex = self._recompute()
            self._ch = {"tau_j": tj, "tau_loss": tl,
                        "tau_m": tm, "tau_ext": ex}
        return self._ch

    @property
    def tau_j(self):
        return self.channels["tau_j"]

    @property
    def tau_loss(self):
        return self.channels["tau_loss"]

    @property
    def tau_m_out(self):
        """Motor torque referred to the output [N*m]."""
        return self.channels["tau_m"] * self.cfg.drive.N

    @property
    def tau_ext(self):
        return self.channels["tau_ext"]

    @property
    def windup(self):
        """delta [rad] -- the lost motion across the drive."""
        e = np.array([self.cfg.drive.kinematic_error(v) for v in self.theta_m])
        return self.theta_m + e - self.theta_l - self.theta_slip

    # ---- energy ----------------------------------------------------------

    def energy_in(self):
        p = self.tau_m_out * self.omega_m
        return float(np.trapz(np.maximum(p, 0.0), self.t))

    def energy_out(self):
        return float(np.trapz(self.tau_j * self.omega_l, self.t))

    def energy_lost(self):
        return float(np.trapz(np.abs(self.tau_loss * self.omega_m), self.t))

    def copper_loss(self):
        i2r = np.array([self.cfg.motor.copper_loss(tm)
                        for tm in self.channels["tau_m"]])
        return float(np.trapz(i2r, self.t))

    # ---- dimensionless views --------------------------------------------

    @property
    def t_hat(self):
        return self.t / self.cfg.scales.time

    def hat(self, torque_array):
        return torque_array / self.cfg.scales.tau

    def did_slip(self, tol=1e-6):
        return float(np.max(np.abs(self.theta_slip))) > tol


def simulate(cfg, controller=None, tau_ext=no_load, t_span=(0.0, 2.0),
             x0=None, n_eval=2000, method="Radau", rtol=1e-7, atol=1e-9,
             apply_motor_limit=True, max_step=np.inf, events=None, label=""):
    """Integrate one configuration. Returns a `SimResult`."""
    if controller is None:
        controller = FreeMotor()
    if x0 is None:
        x0 = np.zeros(N_STATES)
    x0 = np.asarray(x0, dtype=float)

    t_eval = np.linspace(t_span[0], t_span[1], n_eval)
    sol = solve_ivp(
        _rhs, t_span, x0, t_eval=t_eval, method=method,
        rtol=rtol, atol=atol, max_step=max_step, events=events,
        args=(cfg, controller, tau_ext, apply_motor_limit))

    return SimResult(t=sol.t, x=sol.y, cfg=cfg, controller=controller,
                     tau_ext_fn=tau_ext, label=label,
                     success=sol.success, message=sol.message,
                     motor_limited=apply_motor_limit)


def initial_state(theta_l=0.0, theta_m=None, omega_l=0.0, omega_m=0.0):
    """Build a consistent initial state, defaulting the motor to match the
    link so the drive starts unwound."""
    if theta_m is None:
        theta_m = theta_l
    x = np.zeros(N_STATES)
    x[TH_M] = theta_m
    x[W_M] = omega_m
    x[TH_L] = theta_l
    x[W_L] = omega_l
    return x


def quasi_static_state(cfg, theta_l, omega_l=0.0, tau_ext=0.0):
    """Initial state with the drive already wound up to its quasi-static value.

    Starting a test from zero windup rings the drive's internal mode at full
    amplitude, and that transient is an artifact of the initial condition, not
    of the drive. It shows up as a spurious overshoot in the impact test and as
    spurious energy in the tracking test.

    The fix: pre-load the spring to the torque the *rigid* system would be
    transmitting. In a rigid system both inertias share one acceleration
    th'' = tau_g / J_total, so the drive must be carrying tau_j = -J_refl*th''
    to accelerate the motor side. Setting delta = tau_j/K leaves the fast mode
    unexcited and only the rigid-body motion running.
    """
    tau_g = cfg.plant.gravity_torque(theta_l) - cfg.plant.damping * omega_l
    tau_g += tau_ext
    acc = tau_g / cfg.total_inertia
    tau_j = -cfg.reflected_inertia * acc
    K = cfg.drive.stiffness_out(0.0)
    delta = tau_j / K if K > 0.0 else 0.0

    x = np.zeros(N_STATES)
    x[TH_L] = theta_l
    x[W_L] = omega_l
    x[TH_M] = theta_l + delta - cfg.drive.kinematic_error(theta_l + delta)
    x[W_M] = omega_l
    return x


def settle(cfg, controller, theta_l, t=2.0, **kw):
    """Run to a quasi-static equilibrium and return the final state. Used to
    start a test from a loaded, wound-up condition rather than from zero."""
    x0 = initial_state(theta_l=theta_l)
    r = simulate(cfg, controller, t_span=(0.0, t), x0=x0, n_eval=50, **kw)
    return r.x[:, -1].copy()
