"""Known-answer checks on the bench itself.

Every one of these has an answer derivable on paper. They exist because a
simulation that compares drive technologies is only as trustworthy as its
ability to reproduce the cases where the answer is already known -- and because
a subtly broken integrator produces plots that look entirely reasonable.

Run:  python tests/test_validation.py
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drivelab import Config, simulate
from drivelab.sim import (FreeMotor, ShortedMotor, initial_state,
                          quasi_static_state)
from drivelab.plant import LeverArm
from drivelab.motor import Motor
from drivelab.drives import RigidIdealDrive, CompliantIdealDrive
from drivelab.friction import eta_backward, loss_coefficient

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, ok, detail))
    print("  [%s] %-46s %s" % ("PASS" if ok else "FAIL", name, detail))
    return ok


def _period_from_crossings(t, y):
    """Mean period from upward zero crossings, linearly interpolated."""
    s = np.sign(y)
    idx = np.where((s[:-1] < 0) & (s[1:] >= 0))[0]
    if len(idx) < 2:
        return None
    xs = []
    for i in idx:
        frac = -y[i] / (y[i + 1] - y[i])
        xs.append(t[i] + frac * (t[i + 1] - t[i]))
    return float(np.mean(np.diff(xs)))


# ---------------------------------------------------------------------------

def test_pendulum_period():
    """A lossless rigid drive must reduce to a pendulum of inertia
    J_l + N^2 J_m oscillating at sqrt(m g L / J_total).

    This is the single most important check in the file: it tests the plant,
    the reflected-inertia bookkeeping, the sign conventions and the integrator
    all at once.

    The drive is given a critically-damped series damper, which costs nothing
    in rigour and a great deal in runtime. A series element damps only
    *relative* motion, and the rigid-body mode has none -- both inertias turn
    together -- so the pendulum period is untouched while the fast internal
    mode is killed, and the solver stops having to resolve a kilohertz
    oscillation that is not part of the question being asked."""
    print("\n1. Rigid lossless drive == plain pendulum")
    arm = LeverArm(name="v", length=0.3, payload_mass=0.5, damping=0.0)
    mot = Motor(name="v", inertia=1e-5, tau_cont=1e3, tau_peak=1e3,
                kt=1.0, resistance=1.0, v_bus=1e6, damping=0.0)

    for N in (1.0, 10.0, 50.0):
        J_r = N ** 2 * mot.inertia
        J_ser = arm.inertia * J_r / (arm.inertia + J_r)
        w_pend = math.sqrt(arm.gravity_moment / (arm.inertia + J_r))
        # Size the internal mode at a fixed 60x the pendulum rather than
        # picking a large stiffness and hoping. The residual shift of the low
        # mode is of order m g L J_r / (K J_tot), which works out below 0.05%
        # here -- well inside the tolerance -- while the fast mode stays cheap
        # enough to integrate.
        K = J_ser * (60.0 * w_pend) ** 2
        drv = CompliantIdealDrive(N=N, stiffness=K,
                                  damping=2.0 * math.sqrt(K * J_ser))
        cfg = Config(mot, drv, arm)
        w_pred = math.sqrt(arm.gravity_moment / cfg.total_inertia)
        T_pred = 2 * math.pi / w_pred

        x0 = quasi_static_state(cfg, theta_l=-math.pi / 2 + 0.02)
        r = simulate(cfg, FreeMotor(), t_span=(0, 2.2 * T_pred), x0=x0,
                     n_eval=6000, apply_motor_limit=False,
                     rtol=1e-9, atol=1e-12)
        T_meas = _period_from_crossings(r.t, r.theta_l + math.pi / 2)
        err = abs(T_meas - T_pred) / T_pred
        check("pendulum period, N=%g" % N, err < 2e-3,
              "pred %.5fs  meas %.5fs  err %.3f%%"
              % (T_pred, T_meas, 100 * err))


def test_energy_conservation():
    """A lossless, undamped drive must conserve total mechanical energy."""
    print("\n2. Lossless drive conserves energy")
    arm = LeverArm(name="v", length=0.3, payload_mass=0.5, damping=0.0)
    mot = Motor(name="v", inertia=2e-5, tau_cont=1e3, tau_peak=1e3,
                kt=1.0, resistance=1.0, v_bus=1e6, damping=0.0)
    drv = CompliantIdealDrive(N=10.0, stiffness=5e2, damping=0.0)
    cfg = Config(mot, drv, arm)

    x0 = quasi_static_state(cfg, theta_l=-math.pi / 2 + 0.25)
    r = simulate(cfg, FreeMotor(), t_span=(0, 1.5), x0=x0, n_eval=3000,
                 apply_motor_limit=False, rtol=1e-11, atol=1e-13)

    K = drv.stiffness_out(0.0)
    E = (0.5 * cfg.plant.inertia * r.omega_l ** 2
         + 0.5 * cfg.reflected_inertia * r.omega_m ** 2
         + arm.gravity_moment * np.sin(r.theta_l)
         + 0.5 * K * r.windup ** 2)
    drift = (E.max() - E.min()) / max(abs(E).max(), 1e-12)
    check("energy drift over 1.5 s", drift < 1e-6, "%.3e relative" % drift)


def test_two_inertia_resonance():
    """The free system must ring at sqrt(K (1/J_refl + 1/J_l))."""
    print("\n3. Two-inertia resonance lands where theory says")
    arm = LeverArm(name="v", length=0.3, payload_mass=0.5, damping=0.0)
    mot = Motor(name="v", inertia=5e-5, tau_cont=1e3, tau_peak=1e3,
                kt=1.0, resistance=1.0, v_bus=1e6, damping=0.0)
    drv = CompliantIdealDrive(N=10.0, stiffness=4e3, damping=0.0)
    cfg = Config(mot, drv, arm)

    w_pred = math.sqrt(drv.stiffness_out(0.0)
                       * (1.0 / cfg.reflected_inertia + 1.0 / arm.inertia))

    # Excite the internal mode on purpose: start with the drive wound up.
    x0 = initial_state(theta_l=0.0, theta_m=0.01)
    r = simulate(cfg, FreeMotor(), t_span=(0, 1.0), x0=x0, n_eval=20000,
                 apply_motor_limit=False, rtol=1e-9, atol=1e-12)

    w = r.windup - np.mean(r.windup)
    fft = np.abs(np.fft.rfft(w * np.hanning(len(w))))
    freqs = np.fft.rfftfreq(len(w), r.t[1] - r.t[0])
    w_meas = 2 * math.pi * freqs[np.argmax(fft)]
    err = abs(w_meas - w_pred) / w_pred
    check("resonance frequency", err < 5e-3,
          "pred %.1f  meas %.1f rad/s  err %.2f%%"
          % (w_pred, w_meas, 100 * err))


def test_efficiency_coupling():
    """eta_b = 2 - 1/eta_f must fall out of the loss coefficient, and 0.5 must
    be exactly the self-locking boundary."""
    print("\n4. Backdrive efficiency is derived, not assumed")
    ok = True
    for e in (1.0, 0.95, 0.8, 0.75, 0.6, 0.5):
        c = loss_coefficient(e)
        ok &= abs((1.0 - c) - eta_backward(e)) < 1e-12
    check("eta_b == 1 - c for all eta_f", ok)
    check("eta_f = 0.5 is exactly self-locking",
          abs(eta_backward(0.5)) < 1e-12, "eta_b = %.1e" % eta_backward(0.5))
    check("eta_f = 0.75 gives eta_b = 2/3",
          abs(eta_backward(0.75) - 2.0 / 3.0) < 1e-12)


def test_capstan_closed_forms():
    """The capstan's torque and stiffness must match the hand formulas."""
    print("\n5. Capstan matches its closed-form solutions")
    from drivelab.drives import CapstanDrive
    from drivelab.catalog import CABLES

    cable = CABLES["steel-7x19"]
    d = CapstanDrive(r_capstan=0.010, R_output=0.080, cable=cable,
                     cable_diameter=0.001, n_wraps=3.0, mu=0.2,
                     pretension=200.0, free_length=0.16)

    beta = 2 * math.pi * 3.0
    tau_hand = 2 * 200.0 * (0.010 + 0.0005) * math.tanh(0.2 * beta / 2)
    check("tau_max = 2 T_p r tanh(mu beta/2)",
          abs(d.tau_max_slip() - tau_hand) / tau_hand < 1e-12,
          "%.5f N*m" % d.tau_max_slip())

    A = 0.58 * math.pi * 0.001 ** 2 / 4
    k_c = cable.E_eff * A / 0.16
    K_hand = 2 * k_c * (0.080 + 0.0005) ** 2
    check("K_out = 2 (EA/L) R^2",
          abs(d.stiffness_out(0.0) - K_hand) / K_hand < 1e-12,
          "%.1f N*m/rad" % d.stiffness_out(0.0))

    check("K_in = K_out / N^2",
          abs(d.stiffness_out(0.0) / d.N ** 2
              - 2 * k_c * (0.010 + 0.0005) ** 2) < 1e-6)

    # The claim made in the module docstring, checked rather than asserted.
    slack_out = 2 * d.R_eff * d.T_p
    check("capstan slips before it goes slack",
          d.tau_max_slip() < slack_out,
          "slip %.3f < slack-out %.3f N*m" % (d.tau_max_slip(), slack_out))


def test_wrap_saturation():
    """The tanh must saturate: past ~3 wraps, more wraps buy almost nothing."""
    print("\n6. Wrap angle saturates (the tanh, checked numerically)")
    from drivelab.drives import CapstanDrive
    from drivelab.catalog import CABLES
    taus = []
    for n in (1, 2, 3, 4, 6, 10):
        d = CapstanDrive(r_capstan=0.010, R_output=0.080,
                         cable=CABLES["steel-7x19"], cable_diameter=0.001,
                         n_wraps=n, mu=0.2, pretension=200.0, free_length=0.16)
        taus.append(d.tau_max_slip())
    gain_3_to_10 = taus[-1] / taus[2] - 1.0
    check("3 -> 10 wraps gains < 5%", gain_3_to_10 < 0.05,
          "+%.2f%% for 7 more wraps" % (100 * gain_3_to_10))
    check("1 -> 3 wraps is the useful range", taus[2] / taus[0] > 1.15,
          "+%.1f%%" % (100 * (taus[2] / taus[0] - 1)))


def test_shorted_motor_matters():
    """Short-circuit damping must actually resist backdriving -- the term is
    routinely omitted and it is often dominant at low ratio."""
    print("\n7. Short-circuit damping resists backdriving")
    from drivelab.catalog.configs import reference_capstan
    from drivelab.sim import TorqueRamp

    cfg = reference_capstan()
    ramp = TorqueRamp(rate=0.5 * cfg.plant.gravity_moment)
    out = {}
    for ctrl in (FreeMotor(), ShortedMotor()):
        x0 = quasi_static_state(cfg, theta_l=-math.pi / 2)
        r = simulate(cfg, ctrl, tau_ext=ramp, t_span=(0, 4.0), x0=x0,
                     n_eval=1200)
        out[ctrl.name] = abs(r.theta_l[-1] - r.theta_l[0])
    check("shorted motor moves less than free motor",
          out["shorted"] < out["free"],
          "free %.4f rad vs shorted %.4f rad" % (out["free"], out["shorted"]))


def test_harmonic_closed_forms():
    """The harmonic's ratio, Bredt stiffness, preload, ripple and stress limit
    must match the hand formulas in its docstring."""
    print("\n8. Harmonic drive matches its closed-form solutions")
    from drivelab.drives import HarmonicDrive

    h = HarmonicDrive(z_flex=200, r_pitch=0.022, rim_width=0.008,
                      cup_wall=0.5e-3, cup_length=0.018, rim_wall=0.4e-3,
                      w0=0.4e-3, cone_length=0.007,
                      cone_radii=(0.016, 0.022))

    check("N = z_flex / (z_ring - z_flex) = 100",
          h.N == 100.0, "N = %g" % h.N)

    G = h.material.G
    k_cup_hand = (2.0 * math.pi * G * h.cup_radius ** 3
                  * h.cup_wall / h.cup_length)
    check("K_cup = 2 pi G r^3 t / L",
          abs(h.k_cup - k_cup_hand) / k_cup_hand < 1e-12,
          "%.1f N*m/rad" % h.k_cup)

    sina = h.cone_sin_alpha
    k_cone_hand = (4.0 * math.pi * G * h.cone_wall * sina
                   / (1.0 / h.cone_a ** 2 - 1.0 / h.cone_b ** 2))
    check("K_cone = 4 pi G t sin(a) / (1/a^2 - 1/b^2)",
          abs(h.k_cone - k_cone_hand) / k_cone_hand < 1e-12,
          "%.1f N*m/rad" % h.k_cone)

    k_series = 1.0 / (1.0 / k_cup_hand + 1.0 / k_cone_hand)
    check("cup and cone carry in series",
          abs(h.stiffness_out(0.0) - k_series) / k_series < 1e-12,
          "%.1f N*m/rad" % h.stiffness_out(0.0))

    i_rim = h.rim_width * h.rim_wall ** 3 / 12.0
    f_p_hand = 9.0 * math.pi * h.material.E * i_rim * h.w0 \
        / (2.0 * h.r_pitch ** 4)
    check("WG preload = 9 pi E I w0 / (2 r^4)",
          abs(h.preload_force - f_p_hand) / f_p_hand < 1e-12,
          "%.1f N" % h.preload_force)

    tau_hand = (2.0 * math.pi * h.cup_radius ** 2 * h.cup_wall
                * 0.577 * h.material.sigma_allow / h.safety_factor)
    check("tau_max = 2 pi r^2 t * 0.577 sigma_allow / sf",
          abs(h.tau_max() - tau_hand) / tau_hand < 1e-12,
          "%.1f N*m" % h.tau_max())

    # The ripple must repeat twice per input revolution: in output-referred
    # motor angle the period is pi/N, and the amplitude is the runout over the
    # pitch radius.
    eps = h.cam_runout / h.r_pitch
    p = math.pi / h.N
    check("ripple amplitude = runout / r_pitch",
          abs(h.error_amplitude() - eps) / eps < 1e-12,
          "%.2f arcmin" % (math.degrees(eps) * 60))
    check("ripple has 2 cycles per input rev",
          abs(h.kinematic_error(0.0)) < 1e-15
          and abs(h.kinematic_error(p / 4.0) - eps) / eps < 1e-12
          and abs(h.kinematic_error(p / 2.0)) < 1e-15)
    check("velocity ripple = 2 N * amplitude",
          abs(h.d_kinematic_error(0.0) - 2.0 * h.N * eps)
          / (2.0 * h.N * eps) < 1e-12)

    # Derived loss: c = c_wg + c_tooth, eta_f = 1 / (1 + c).
    c_wg = (h.mu_wg * h.r_wg_bearing * h.N / h.r_pitch
            * (1.0 + math.tan(h.alpha)))
    c_t = (h.N * h.mu_tooth * 4.0 * h.w0
           / (2.0 * math.pi * h.r_pitch * math.tan(h.alpha)))
    check("eta_f = 1 / (1 + c_wg + c_tooth)",
          abs(h.eta_forward - 1.0 / (1.0 + c_wg + c_t)) < 1e-12,
          "eta_f = %.4f" % h.eta_forward)
    check("tau_c0 = N (2 mu_wg F_p r_b + seal)",
          abs(h._tau_c0 - h.N * (2.0 * h.mu_wg * f_p_hand * h.r_wg_bearing
                                 + h.seal_drag)) < 1e-12,
          "%.3f N*m" % h._tau_c0)


def test_cycloidal_closed_forms():
    """The cycloidal's ratio, pin loading, pin-bending stiffness, backlash,
    Hertz limit and reflected inertia must match the hand formulas."""
    print("\n9. Cycloidal drive matches its closed-form solutions")
    from drivelab.drives import CycloidalDrive

    c = CycloidalDrive(z_pins=31, r_pitch=0.030, r_pin=2.9e-3,
                       disc_width=0.010, eccentricity=0.0015,
                       n_out_pins=8, r_out_pin=2.5e-3, r_out_pitch=0.0165,
                       out_pin_length=0.018, clearance=6.0e-6)

    check("N = z_pins - 1 = 30", c.N == 30.0, "N = %g" % c.N)
    check("F_max = 4 tau / (z_p r_p)",
          abs(c.pin_force_max(30.0) - 4.0 * 30.0 / (31.0 * 0.030)) < 1e-12,
          "%.1f N" % c.pin_force_max(30.0))

    i_pin = math.pi * 2.5e-3 ** 4 / 4.0
    k_pins_hand = 8.0 * 3.0 * c.pin_material.E * i_pin / 0.018 ** 3 * 0.0165 ** 2
    check("K_pins = n 3 E I / L^3 * R^2",
          abs(c.k_output_pins - k_pins_hand) / k_pins_hand < 1e-12,
          "%.1f N*m/rad" % c.k_output_pins)

    f_allow = c._hertz_allowable()
    k_h = c._hertz_secant(f_allow)
    k_contact_hand = k_h * 0.030 ** 2 * (31.0 / 4.0)
    check("K_contact = k_hertz r_p^2 z_p / 4",
          abs(c.k_contact - k_contact_hand) / k_contact_hand < 1e-12,
          "%.3g N*m/rad" % c.k_contact)

    k_out = 1.0 / (1.0 / k_pins_hand + 1.0 / k_contact_hand)
    check("pins and contact in series",
          abs(c.stiffness_out(0.0) - k_out) / k_out < 1e-12,
          "%.1f N*m/rad" % c.stiffness_out(0.0))

    check("backlash = 2 c / R_out",
          abs(c.backlash - 2.0 * 6.0e-6 / 0.0165) < 1e-15,
          "%.2f arcmin" % (math.degrees(c.backlash) * 60.0))
    cp = CycloidalDrive(z_pins=31, r_pitch=0.030, r_pin=2.9e-3,
                        disc_width=0.010, eccentricity=0.0015,
                        n_out_pins=8, r_out_pin=2.5e-3, r_out_pitch=0.0165,
                        out_pin_length=0.018, clearance=6.0e-6,
                        preloaded=True)
    check("preloaded output pins delete the backlash",
          cp.backlash == 0.0 and cp.spring_torque(1e-6) > 0.0)

    f_pin = (math.pi * 2.5e-3 ** 3 * c.pin_material.sigma_allow
             / (4.0 * 0.018 * 3.0))
    tau_pin_hand = f_pin * 8.0 * 0.0165
    check("tau_max,pins = pi r^3 sigma / (4 L sf) * n R",
          abs(c.tau_max_pin_bending() - tau_pin_hand) / tau_pin_hand < 1e-12,
          "%.1f N*m" % c.tau_max_pin_bending())

    # Reflected inertia: disc orbit m e^2 rides N^2; disc spin and pins are
    # output-side. Check the composition by hand.
    m_disc = c.disc_mass
    j_hand = (c.N ** 2 * (m_disc * c.eccentricity ** 2
                          + 0.5 * c.cam_mass * c.r_ecc_bearing ** 2
                          + c.cam_mass * c.eccentricity ** 2)
              + 0.5 * m_disc * c.disc_radius ** 2
              + 8.0 * (c.pin_material.density * math.pi * 2.5e-3 ** 2
                       * 0.018) * 0.0165 ** 2)
    check("reflected inertia = N^2 (m_d e^2 + J_cam) + J_disc + pins",
          abs(c.reflected_inertia_extra() - j_hand) / j_hand < 1e-12,
          "%.3g kg*m^2" % j_hand)

    # Derived loss terms.
    c_roll = c.mu_roll * (4.0 / math.pi) * (1.0 + c.r_pin / c.r_pitch)
    c_bear = c.mu_bearing * (4.0 / math.pi) * c.r_ecc_bearing * c.N / c.r_pitch
    c_out = c.mu_outpin * c.eccentricity / c.r_out_pitch
    check("eta_f = 1 / (1 + c_roll + c_bear + c_outpin)",
          abs(c.eta_forward - 1.0 / (1.0 + c_roll + c_bear + c_out)) < 1e-12,
          "eta_f = %.4f" % c.eta_forward)


def test_derived_drives_conserve_energy():
    """Both derived drives must conserve energy exactly in their lossless
    limit -- the same check the ideal drives pass, now with real geometry and
    kinematic error in the loop."""
    print("\n10. Derived drives conserve energy (lossless limit)")
    from drivelab.drives import HarmonicDrive, CycloidalDrive

    arm = LeverArm(name="v", length=0.3, payload_mass=0.5, damping=0.0)
    mot = Motor(name="v", inertia=1e-4, tau_cont=1e3, tau_peak=1e3,
                kt=1.0, resistance=1.0, v_bus=1e6, damping=0.0)

    drives = [
        HarmonicDrive(z_flex=200, r_pitch=0.022, rim_width=0.008,
                      cup_wall=0.5e-3, cup_length=0.018, mu_wg=0.0,
                      mu_tooth=0.0, seal_drag=0.0,
                      friction_model="regularized"),
        CycloidalDrive(z_pins=31, r_pitch=0.030, r_pin=2.9e-3,
                       disc_width=0.010, eccentricity=0.0015,
                       clearance=0.0, mu_roll=0.0, mu_bearing=0.0,
                       mu_outpin=0.0, seal_drag=0.0,
                       friction_model="regularized"),
    ]
    for drv in drives:
        cfg = Config(mot, drv, arm)
        x0 = quasi_static_state(cfg, theta_l=-math.pi / 2 + 0.25)
        r = simulate(cfg, FreeMotor(), t_span=(0, 1.5), x0=x0, n_eval=3000,
                     apply_motor_limit=False, rtol=1e-11, atol=1e-13)
        K = drv.stiffness_out(0.0)
        E = (0.5 * cfg.plant.inertia * r.omega_l ** 2
             + 0.5 * cfg.reflected_inertia * r.omega_m ** 2
             + arm.gravity_moment * np.sin(r.theta_l)
             + 0.5 * K * r.windup ** 2)
        drift = (E.max() - E.min()) / max(abs(E).max(), 1e-12)
        check("%s energy drift over 1.5 s" % drv.name, drift < 1e-6,
              "%.3e relative" % drift)


def main():
    print("=" * 72)
    print("drivelab validation -- known-answer checks")
    print("=" * 72)
    test_pendulum_period()
    test_energy_conservation()
    test_two_inertia_resonance()
    test_efficiency_coupling()
    test_capstan_closed_forms()
    test_wrap_saturation()
    test_shorted_motor_matters()
    test_harmonic_closed_forms()
    test_cycloidal_closed_forms()
    test_derived_drives_conserve_energy()

    n_pass = sum(1 for _, ok, _ in RESULTS if ok)
    print("\n" + "=" * 72)
    print("%d/%d checks passed" % (n_pass, len(RESULTS)))
    print("=" * 72)
    return 0 if n_pass == len(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
