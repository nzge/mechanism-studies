"""The five standardized tests.

Every drive faces exactly these, on the same arm, with the same controller
rules. The tests are deliberately simple and deliberately fixed -- the moment a
test gets tuned per drive, the comparison stops meaning anything.

  T1  lost motion      quasi-static torque reversal with the input locked
  T2  cyclic tracking  PD-controlled sinusoid, fixed bandwidth rule
  T3  backdrive        ramp torque at the output, motor unpowered
  T4  impact           swing into a virtual wall
  T5  frequency        chirp, empirical FRF from motor torque to link angle

Times are expressed in units of the plant time scale t*, so the same test is
the same test regardless of how big the arm is.
"""

import math
import numpy as np

from ..sim import (simulate, quasi_static_state, initial_state,
                   FreeMotor, ShortedMotor, PDController, MotorPositionLock,
                   TorqueProfile, VirtualWall, TorqueRamp)


# ---------------------------------------------------------------------------
# T1 -- lost motion and hysteresis
# ---------------------------------------------------------------------------

def t1_lost_motion(cfg, tau_frac=0.3, cycles=1.0, periods=30.0, n_eval=2500,
                   theta_l=-math.pi / 2, **kw):
    """Hold the input, push the output slowly back and forth, watch the loop.

    This is the standard lost-motion measurement, and it is the one test where
    the LuGre bristle model earns its keep: a regularized Coulomb friction
    would trace a sharp corner at reversal, while the real thing traces a
    rounded presliding loop whose width *is* the lost motion. Loop area is the
    energy thrown away per cycle.

    Reported quantities:
      lost_motion   output angle swept at zero torque  [rad]
      stiffness     secant slope over the loaded range [N*m/rad]
      loop_area     dissipation per cycle              [J]
    """
    ts = cfg.scales.time
    T = periods * ts
    tau_amp = tau_frac * min(cfg.drive.tau_max(), 4.0 * cfg.plant.gravity_moment)

    def tri(t):
        """Triangle wave in [-1, 1], `cycles` of them over T."""
        u = (t / T) * cycles
        return 2.0 * abs(2.0 * (u - math.floor(u + 0.5))) - 1.0

    def ext(t, th, w):
        return tau_amp * tri(t + T / (4.0 * cycles))

    x0 = quasi_static_state(cfg, theta_l=theta_l)
    # The lock is in series with the drive, so its compliance is subtracted
    # from the stiffness being measured. Put it far above the drive's own
    # resonance or T1 reports the lock instead of the drive.
    f_res = math.sqrt(cfg.drive.stiffness_out(0.0)
                      / cfg.reflected_inertia) / (2 * math.pi)
    lock = MotorPositionLock(target=x0[0], bandwidth_hz=max(20.0 * f_res, 500.0))
    r = simulate(cfg, lock, tau_ext=ext, t_span=(0.0, T), x0=x0,
                 n_eval=n_eval, label="T1 lost motion", **kw)
    r.meta = {"tau_amp": tau_amp, "theta_0": theta_l}
    return r


# ---------------------------------------------------------------------------
# T2 -- cyclic tracking
# ---------------------------------------------------------------------------

def t2_tracking(cfg, amplitude=0.3, freq_ratio=0.25, periods=3.0, alpha=2.5,
                zeta=0.8, gravity_ff=True, n_eval=2500,
                theta_center=-math.pi / 4, **kw):
    """Track a sinusoid under PD control with the standardized gain rule.

    `alpha` is the target closed-loop bandwidth as a multiple of the plant
    time scale, and it is the *same for every drive*. A drive that cannot
    support it rings or goes unstable, and that is the finding. Tuning each
    drive to its own limit would hide precisely the difference being measured.

    `freq_ratio` is the reference frequency as a fraction of that bandwidth.
    """
    w_ref = freq_ratio * alpha * cfg.plant.omega_scale
    T = periods * 2.0 * math.pi / w_ref

    def ref(t):
        return (theta_center + amplitude * math.sin(w_ref * t),
                amplitude * w_ref * math.cos(w_ref * t))

    ctrl = PDController(ref, alpha=alpha, zeta=zeta, gravity_ff=gravity_ff)
    x0 = quasi_static_state(cfg, theta_l=theta_center)
    r = simulate(cfg, ctrl, t_span=(0.0, T), x0=x0, n_eval=n_eval,
                 label="T2 tracking", **kw)
    r.meta = {"w_ref": w_ref, "amplitude": amplitude,
              "theta_center": theta_center, "ref": ref, "alpha": alpha}
    return r


# ---------------------------------------------------------------------------
# T3 -- backdrive
# ---------------------------------------------------------------------------

def t3_backdrive(cfg, rate_frac=0.5, periods=30.0, shorted=False,
                 n_eval=2000, theta_l=-math.pi / 2, move_threshold=0.02,
                 **kw):
    """Ramp an external torque at the output with the motor unpowered.

    Run it both ways. `shorted=False` is the open-circuit case, which is what
    most analyses quote. `shorted=True` adds kt^2/R damping at the rotor, which
    is what an unpowered motor with its phases connected actually feels -- and
    for a low-ratio drive that term is frequently the dominant resistance. A
    capstan that backdrives beautifully can still feel stiff if the motor is
    braking it.

    The arm starts hanging, where gravity torque is zero, so the measured
    breakaway is the *drive's* threshold and not the arm's weight.
    """
    ts = cfg.scales.time
    T = periods * ts
    rate = rate_frac * cfg.plant.gravity_moment / ts
    cap = 1.5 * min(cfg.drive.tau_max(), 20.0 * cfg.plant.gravity_moment)
    ramp = TorqueRamp(rate=rate, limit=cap)
    ctrl = ShortedMotor() if shorted else FreeMotor()
    x0 = quasi_static_state(cfg, theta_l=theta_l)

    # Stop the moment it breaks away. Without this the ramp keeps climbing and
    # a freely-backdriving drive spins the arm hundreds of revolutions past its
    # own stroke limit, which is both slow and physically meaningless.
    def broke_away(t, x, *a):
        return abs(x[2] - theta_l) - move_threshold
    broke_away.terminal = True
    broke_away.direction = 1

    r = simulate(cfg, ctrl, tau_ext=ramp, t_span=(0.0, T), x0=x0,
                 n_eval=n_eval, label="T3 backdrive", events=broke_away, **kw)
    r.meta = {"rate": rate, "theta_0": theta_l, "shorted": shorted,
              "threshold": move_threshold, "cap": cap}
    return r


# ---------------------------------------------------------------------------
# T4 -- impact
# ---------------------------------------------------------------------------

def t4_impact(cfg, speed_ratio=1.0, wall_offset=0.05, periods=6.0,
              wall_Omega=300.0, n_eval=4000, **kw):
    """Swing the arm into a rigid obstacle and measure what gets transmitted.

    The most design-relevant test in the suite, and the one where a metric
    table of efficiencies and backlash figures is most misleading. Peak
    transmitted torque is dominated by *reflected inertia*: the arm carries its
    own momentum plus N^2 J_m worth of motor momentum, and at 100:1 the second
    term buries the first.

    The wall is referenced to the *plant*, not to the drive: k_wall =
    wall_Omega^2 * m g L, so every drive on a given arm hits the identical
    obstacle. Scaling the wall by each drive's own stiffness would be a
    fairness bug -- a stiff drive would be tested against a stiffer wall, and
    the comparison would partly be measuring the test rig.
    """
    w0 = speed_ratio * cfg.plant.omega_scale
    theta_0 = -math.pi / 2
    wall = theta_0 + wall_offset
    k_wall = wall_Omega ** 2 * cfg.plant.gravity_moment
    c_wall = 2.0 * 0.1 * math.sqrt(k_wall * cfg.plant.inertia)

    ext = VirtualWall(angle=wall, stiffness=k_wall, damping=c_wall, side=+1)
    x0 = quasi_static_state(cfg, theta_l=theta_0, omega_l=w0)
    r = simulate(cfg, FreeMotor(), tau_ext=ext, t_span=(0.0, periods * cfg.scales.time),
                 x0=x0, n_eval=n_eval, label="T4 impact", **kw)
    r.meta = {"omega_0": w0, "wall": wall, "k_wall": k_wall}
    return r


# ---------------------------------------------------------------------------
# T5 -- frequency response
# ---------------------------------------------------------------------------

def t5_frequency(cfg, f_lo_ratio=0.2, f_hi_ratio=60.0, periods=120.0,
                 amp_frac=0.05, n_eval=24000, theta_l=-math.pi / 2, **kw):
    """Empirical FRF from motor torque to link angle, via a linear chirp.

    Measured rather than linearized on purpose. The analytic two-inertia
    transfer function is available in `analytic_frf` below and the two should
    agree in the small-signal limit -- but friction, backlash and the kinematic
    error are all nonlinear, and the gap between the two curves is itself
    informative. A drive whose measured FRF departs from its analytic one at
    small amplitude has a friction or backlash problem that no linear model
    will warn you about.
    """
    ts = cfg.scales.time
    T = periods * ts
    f0 = f_lo_ratio / ts / (2.0 * math.pi)
    f1 = f_hi_ratio / ts / (2.0 * math.pi)
    amp = amp_frac * cfg.plant.gravity_moment / cfg.drive.N

    def chirp(t):
        k = (f1 - f0) / T
        return amp * math.sin(2.0 * math.pi * (f0 * t + 0.5 * k * t * t))

    ctrl = TorqueProfile(chirp, name="chirp")
    x0 = quasi_static_state(cfg, theta_l=theta_l)
    r = simulate(cfg, ctrl, t_span=(0.0, T), x0=x0, n_eval=n_eval,
                 apply_motor_limit=False, label="T5 frequency", **kw)
    r.meta = {"f0": f0, "f1": f1, "amp": amp, "theta_0": theta_l}
    return r


def analytic_frf(cfg, freqs_hz, theta_l=-math.pi / 2):
    """Closed-form small-signal FRF, motor torque (output-referred) -> link
    angle, for the linearized two-inertia plant.

    Gives the resonance/antiresonance pair that sets achievable control
    bandwidth. The antiresonance is the one that bites: it is a notch the
    controller cannot push through, and it sits at sqrt(K/J_l) regardless of
    how good the motor is.
    """
    K = cfg.drive.stiffness_out(0.0)
    C = cfg.drive.damping_out()
    Jm = cfg.reflected_inertia
    Jl = cfg.plant.inertia
    bl = cfg.plant.damping
    # Gravity stiffness about theta_l (from -m g L cos(theta)).
    Kg = -cfg.plant.gravity_moment * math.sin(theta_l)

    w = 2.0 * np.pi * np.asarray(freqs_hz, dtype=float)
    s = 1j * w
    # [Jm s^2 + Cs + K      -(Cs + K)      ] [th_m]   [tau_m]
    # [-(Cs + K)   Jl s^2 + (C+bl)s + K+Kg ] [th_l] = [  0  ]
    a11 = Jm * s ** 2 + C * s + K
    a12 = -(C * s + K)
    a22 = Jl * s ** 2 + (C + bl) * s + K + Kg
    det = a11 * a22 - a12 ** 2
    return -a12 / det      # th_l / tau_m


def resonance_pair(cfg, theta_l=-math.pi / 2):
    """(antiresonance, resonance) in Hz -- the two numbers that bound what a
    controller can do."""
    K = cfg.drive.stiffness_out(0.0)
    Jm = cfg.reflected_inertia
    Jl = cfg.plant.inertia
    w_anti = math.sqrt(K / Jl)
    w_res = math.sqrt(K * (1.0 / Jm + 1.0 / Jl))
    return w_anti / (2 * math.pi), w_res / (2 * math.pi)


SUITE = {
    "T1": t1_lost_motion,
    "T2": t2_tracking,
    "T3": t3_backdrive,
    "T4": t4_impact,
    "T5": t5_frequency,
}
