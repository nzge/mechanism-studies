"""Metric extraction -- turning each test's time series into comparable numbers.

Every metric is reported in dimensionless form alongside its SI value. The
dimensionless column is the one to compare across drives; the SI column is the
one to check against a datasheet or a bench measurement.
"""

import math
import numpy as np


def _arcmin(rad):
    return math.degrees(rad) * 60.0


# ---------------------------------------------------------------------------

def t1_metrics(r):
    """Lost motion, stiffness and dissipation from the hysteresis loop."""
    tau = r.tau_ext
    th = r.theta_l - r.meta["theta_0"]
    amp = r.meta["tau_amp"]

    # Lost motion: the angular width of the loop where it crosses zero torque.
    crossings = []
    s = np.sign(tau)
    idx = np.where(np.diff(s) != 0)[0]
    for i in idx:
        denom = tau[i + 1] - tau[i]
        if denom == 0:
            continue
        f = -tau[i] / denom
        crossings.append(th[i] + f * (th[i + 1] - th[i]))
    lost_motion = (max(crossings) - min(crossings)) if len(crossings) >= 2 else 0.0

    # Secant stiffness over the upper half of the loading range, taken on the
    # loading branch only so hysteresis does not corrupt the slope.
    hi = tau > 0.5 * amp
    lo = tau < -0.5 * amp
    if hi.any() and lo.any():
        stiffness = ((np.mean(tau[hi]) - np.mean(tau[lo]))
                     / (np.mean(th[hi]) - np.mean(th[lo])))
    else:
        stiffness = float("nan")

    # Loop area = energy dissipated per cycle (shoelace on the tau-theta loop).
    area = abs(0.5 * np.sum((th[:-1] + th[1:]) * np.diff(tau)))

    s_ = r.cfg.scales
    return {
        "lost_motion_rad": lost_motion,
        "lost_motion_arcmin": _arcmin(lost_motion),
        "stiffness_Nm_rad": stiffness,
        "stiffness_hat": stiffness / s_.tau,
        "hysteresis_loss_J": area,
        "hysteresis_loss_hat": area / s_.energy,
        "peak_windup_arcmin": _arcmin(float(np.max(np.abs(r.windup)))),
        "slipped": r.did_slip(),
    }


def t2_metrics(r):
    """Tracking accuracy, effort and energy over the last full period."""
    ref = r.meta["ref"]
    th_ref = np.array([ref(t)[0] for t in r.t])
    err = r.theta_l - th_ref

    # Judge on the last period only -- the first is startup.
    T = 2 * math.pi / r.meta["w_ref"]
    mask = r.t >= (r.t[-1] - T)
    e = err[mask]

    s_ = r.cfg.scales
    tau_pk = float(np.max(np.abs(r.tau_m_out)))
    return {
        "rms_error_rad": float(np.sqrt(np.mean(e ** 2))),
        "rms_error_arcmin": _arcmin(float(np.sqrt(np.mean(e ** 2)))),
        "peak_error_arcmin": _arcmin(float(np.max(np.abs(e)))),
        "peak_motor_torque_Nm": tau_pk,
        "peak_motor_torque_hat": tau_pk / s_.tau,
        "motor_torque_utilization": tau_pk / max(r.cfg.tau_motor_max_out, 1e-12),
        "energy_in_J": r.energy_in(),
        "energy_lost_J": r.energy_lost(),
        "copper_loss_J": r.copper_loss(),
        # No simulated efficiency number is reported here, deliberately.
        # Every obvious formulation is broken for this test: energy_out/
        # energy_in is ~0/~0 over a closed cycle; 1 - loss/in explodes when
        # gravity feedforward drives net motor work toward zero; and comparing
        # |tau*omega| throughput counts a saturated drive's arm sloshing back
        # and forth on its own compliance as useful work, which flatters
        # precisely the drives that are failing the test.
        #
        # What is reported instead: the raw energies, and the quasi-static
        # efficiency at this test's peak load, which is well defined and comes
        # straight from the loss model rather than from an integral that can be
        # gamed by oscillation.
        "loss_per_cycle_hat": r.energy_lost() / s_.energy,
        "eta_at_peak_load": r.cfg.drive.efficiency_at(
            float(np.max(np.abs(r.tau_j)))),
        "saturated": _saturated(r),
        "tracking_ok": bool(_arcmin(float(np.max(np.abs(e)))) < 120.0),
        "slipped": r.did_slip(),
    }


def _saturated(r, tol=0.98):
    """Did the motor hit its torque ceiling? If so, most of the other metrics
    describe the motor rather than the drive."""
    lim = r.cfg.tau_motor_max_out
    return bool(np.max(np.abs(r.tau_m_out)) >= tol * lim) if lim > 0 else False


def t3_metrics(r, threshold_rad=None):
    """Breakaway torque -- the external torque at which the output first moves
    appreciably. Reported as a fraction of the drive's own rating, which is the
    form that transfers between drives of different size."""
    th0 = r.meta["theta_0"]
    # The run is terminated by an event at exactly the breakaway threshold, so
    # the last recorded sample sits just *below* it. Detect slightly under the
    # event level or every backdrivable drive reports as non-backdrivable.
    if threshold_rad is None:
        threshold_rad = 0.9 * r.meta.get("threshold", 0.02)
    moved = np.abs(r.theta_l - th0) > threshold_rad
    if moved.any():
        i = int(np.argmax(moved))
        breakaway = float(r.tau_ext[i])
        t_break = float(r.t[i])
    else:
        breakaway = float("inf")
        t_break = float("nan")

    s_ = r.cfg.scales
    tau_max = r.cfg.drive.tau_max()
    return {
        "backdrive_torque_Nm": breakaway,
        "backdrive_torque_hat": breakaway / s_.tau,
        "backdrive_pct_rated": (100.0 * breakaway / tau_max
                                if np.isfinite(tau_max) and tau_max > 0
                                else float("nan")),
        "backdrive_time_s": t_break,
        "backdrivable": np.isfinite(breakaway),
        "total_travel_rad": float(np.max(np.abs(r.theta_l - th0))),
        "exceeded_stroke": _stroke_exceeded(r),
        "shorted": r.meta["shorted"],
    }


def _stroke_exceeded(r):
    """A capstan has a hard travel limit; a gearbox does not. Silently running
    a drive past its stroke is an easy way to produce a flattering number."""
    lim = r.cfg.drive.stroke_limit()
    if lim is None:
        return False
    return bool(np.max(np.abs(r.theta_l - r.theta_l[0])) > lim)


def t4_metrics(r):
    """Peak transmitted torque on impact, and the impulse behind it."""
    tau = np.abs(r.tau_j)
    i = int(np.argmax(tau))
    s_ = r.cfg.scales

    contact = np.abs(r.tau_ext) > 1e-9
    impulse = (float(np.trapz(np.abs(r.tau_ext[contact]), r.t[contact]))
               if contact.any() else 0.0)

    # Momentum arriving at the wall, split into its two sources. The point of
    # the test is usually visible right here, before any plotting.
    p_link = r.cfg.plant.inertia * r.meta["omega_0"]
    p_refl = r.cfg.reflected_inertia * r.meta["omega_0"]

    return {
        "peak_transmitted_Nm": float(tau[i]),
        "peak_transmitted_hat": float(tau[i]) / s_.tau,
        "peak_over_rating": (float(tau[i]) / r.cfg.drive.tau_max()
                             if np.isfinite(r.cfg.drive.tau_max()) else
                             float("nan")),
        "peak_windup_arcmin": _arcmin(float(np.max(np.abs(r.windup)))),
        "contact_impulse_Nms": impulse,
        "momentum_link": p_link,
        "momentum_reflected": p_refl,
        "reflected_momentum_share": p_refl / (p_link + p_refl),
        "slipped": r.did_slip(),
        "slip_arcmin": _arcmin(float(np.max(np.abs(r.theta_slip)))),
    }


def t5_metrics(r, nperseg=8192):
    """Empirical FRF plus the analytic resonance pair."""
    from .tests import resonance_pair

    u = r.channels["tau_m"] * r.cfg.drive.N
    y = r.theta_l - np.mean(r.theta_l)
    dt = r.t[1] - r.t[0]

    win = np.hanning(len(u))
    U = np.fft.rfft(u * win)
    Y = np.fft.rfft(y * win)
    f = np.fft.rfftfreq(len(u), dt)
    with np.errstate(divide="ignore", invalid="ignore"):
        H = Y / U
    band = (f >= r.meta["f0"]) & (f <= r.meta["f1"])

    f_anti, f_res = resonance_pair(r.cfg, r.meta["theta_0"])
    ts = r.cfg.scales.time
    return {
        "f_antiresonance_Hz": f_anti,
        "f_resonance_Hz": f_res,
        "f_antiresonance_hat": f_anti * ts * 2 * math.pi,
        "f_resonance_hat": f_res * ts * 2 * math.pi,
        "separation": f_res / f_anti if f_anti > 0 else float("nan"),
        "_freqs": f[band],
        "_H": H[band],
    }


# ---------------------------------------------------------------------------

METRIC_FNS = {"T1": t1_metrics, "T2": t2_metrics, "T3": t3_metrics,
              "T4": t4_metrics, "T5": t5_metrics}


def run_suite(cfg, tests=("T1", "T2", "T3", "T4", "T5"), verbose=False,
              **overrides):
    """Run the standard suite on one configuration.

    Returns {test_id: (SimResult, metrics_dict)}. T3 is run twice, open-circuit
    and shorted, because reporting only one of them is how backdrivability
    claims get overstated.
    """
    from .tests import SUITE

    out = {}
    for tid in tests:
        kw = overrides.get(tid, {})
        if tid == "T3":
            for shorted, key in ((False, "T3"), (True, "T3s")):
                r = SUITE[tid](cfg, shorted=shorted, **kw)
                out[key] = (r, METRIC_FNS[tid](r))
                if verbose:
                    print("  %-4s %s" % (key, "ok" if r.success else r.message))
        else:
            r = SUITE[tid](cfg, **kw)
            out[tid] = (r, METRIC_FNS[tid](r))
            if verbose:
                print("  %-4s %s" % (tid, "ok" if r.success else r.message))
    return out


def summary_row(cfg, results):
    """Flatten a suite run into one row of the comparison table."""
    row = {
        "config": cfg.name,
        "derived": cfg.drive.physics_derived,
        "N": cfg.drive.N,
        "Pi_J": cfg.pi.Pi_J,
        "Omega_n": cfg.pi.Omega_n,
        "eta_f": cfg.pi.eta_f,
        "eta_b": cfg.pi.eta_b,
    }
    keep = {
        "T1": ["lost_motion_arcmin", "stiffness_hat", "hysteresis_loss_hat"],
        "T2": ["rms_error_arcmin", "peak_motor_torque_hat",
               "eta_at_peak_load", "loss_per_cycle_hat", "saturated",
               "tracking_ok"],
        "T3": ["backdrive_torque_hat", "backdrive_pct_rated"],
        "T3s": ["backdrive_torque_hat"],
        "T4": ["peak_transmitted_hat", "reflected_momentum_share",
               "peak_over_rating"],
        "T5": ["f_antiresonance_Hz", "f_resonance_Hz", "separation"],
    }
    for tid, keys in keep.items():
        if tid not in results:
            continue
        _, m = results[tid]
        for k in keys:
            row["%s.%s" % (tid, k)] = m.get(k)
    row["feasibility"] = "; ".join(cfg.feasibility()) or "ok"
    return row
