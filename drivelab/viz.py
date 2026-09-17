"""Plotting and animation.

Chart conventions, applied consistently so that figures from different tests
read as one system:

  * Colour identifies the *drive*, in a fixed order, never cycled and never
    reassigned when a drive is filtered out. The four-hue categorical palette
    was checked with a CVD validator: worst adjacent-pair separation is
    dE 17.0 under deuteranopia against a target of 8, so the series stay
    distinguishable without relying on the legend.
  * Two series or more always carry both a legend and, where there is room,
    a direct label -- identity is never colour alone.
  * Grid and axes are recessive; marks are thin; no value labels on every
    point.
  * Never two y-axes. Two quantities of different scale get two panels.
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

# Fixed categorical order. Colour follows the entity, not its rank.
PALETTE = {
    "capstan":   "#2563eb",
    "harmonic":  "#d97706",
    "cycloidal": "#9333ea",
    "ideal":     "#0d9488",
}
SERIES_ORDER = ["capstan", "harmonic", "cycloidal", "ideal"]

INK = "#1c1c1a"
INK_MUTED = "#6b6b66"
GRID = "#e4e4e1"
SURFACE = "#fcfcfb"


def drive_color(name):
    """Map a config or drive name onto its fixed hue."""
    low = str(name).lower()
    for key in SERIES_ORDER:
        if key in low:
            return PALETTE[key]
    return INK_MUTED


def style_axes(ax, xlabel=None, ylabel=None, title=None):
    """Recessive grid and axes; text in ink tokens, never in a series colour."""
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8, zorder=0)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK_MUTED, labelsize=9, length=0)
    if xlabel:
        ax.set_xlabel(xlabel, color=INK_MUTED, fontsize=9)
    if ylabel:
        ax.set_ylabel(ylabel, color=INK_MUTED, fontsize=9)
    if title:
        ax.set_title(title, color=INK, fontsize=11, loc="left", pad=10)
    return ax


def new_fig(nrows=1, ncols=1, figsize=(9, 4.2), **kw):
    fig, axes = plt.subplots(nrows, ncols, figsize=figsize,
                             facecolor=SURFACE, **kw)
    return fig, axes


# ---------------------------------------------------------------------------
# Per-test figures
# ---------------------------------------------------------------------------

def plot_hysteresis(results, ax=None, normalize=True):
    """T1: the torque-deflection loop. A loop is the right form here -- the
    enclosed area is the quantity of interest, and no bar chart shows it."""
    if ax is None:
        _, ax = new_fig(figsize=(5.4, 4.6))
    for label, r in results.items():
        th = (r.theta_l - r.meta["theta_0"])
        tau = r.tau_ext
        if normalize:
            tau = tau / r.cfg.scales.tau
        ax.plot(np.degrees(th) * 60, tau, lw=1.6, color=drive_color(label),
                label=label, zorder=3)
    ax.axhline(0, color=GRID, lw=1, zorder=1)
    ax.axvline(0, color=GRID, lw=1, zorder=1)
    style_axes(ax, "output deflection [arcmin]",
               "external torque / mgL" if normalize else "external torque [N*m]",
               "T1  lost motion and hysteresis")
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_MUTED, loc="best")
    return ax


def plot_tracking(results, axes=None, normalize=True):
    """T2: reference vs response, and the error beneath it. Two panels rather
    than two y-scales."""
    if axes is None:
        _, axes = new_fig(2, 1, figsize=(9, 5.6), sharex=True,
                          gridspec_kw={"height_ratios": [2, 1]})
    top, bot = axes
    first = True
    for label, r in results.items():
        t = r.t_hat if normalize else r.t
        ref = np.array([r.meta["ref"](tt)[0] for tt in r.t])
        if first:
            top.plot(t, np.degrees(ref), lw=1.4, color=INK_MUTED, ls="--",
                     label="reference", zorder=2)
            first = False
        c = drive_color(label)
        top.plot(t, np.degrees(r.theta_l), lw=1.6, color=c, label=label,
                 zorder=3)
        bot.plot(t, np.degrees(r.theta_l - ref) * 60, lw=1.4, color=c,
                 zorder=3)
    bot.axhline(0, color=GRID, lw=1, zorder=1)
    style_axes(top, None, "link angle [deg]", "T2  cyclic tracking")
    style_axes(bot, "time / t*" if normalize else "time [s]",
               "error [arcmin]")
    top.legend(frameon=False, fontsize=9, labelcolor=INK_MUTED, ncol=2)
    return axes


def plot_backdrive(results, ax=None):
    """T3: breakaway torque. Magnitude comparison across a handful of named
    things -- a bar chart, sorted, with direct value labels."""
    if ax is None:
        _, ax = new_fig(figsize=(7.2, 3.6))
    labels, vals, colors = [], [], []
    for label, (r, m) in results.items():
        v = m["backdrive_torque_hat"]
        labels.append(label)
        vals.append(v if np.isfinite(v) else 0.0)
        colors.append(drive_color(label))
    order = np.argsort(vals)
    y = np.arange(len(order))
    ax.barh(y, [vals[i] for i in order], color=[colors[i] for i in order],
            height=0.6, zorder=3)
    ax.set_yticks(y)
    ax.set_yticklabels([labels[i] for i in order], fontsize=9, color=INK)
    for yi, i in enumerate(order):
        ax.text(vals[i], yi, "  %.2f" % vals[i], va="center", fontsize=9,
                color=INK_MUTED)
    style_axes(ax, "breakaway torque / mgL", None,
               "T3  backdrive threshold  (lower is more backdrivable)")
    return ax


def plot_impact(results, axes=None):
    """T4: transmitted torque through the contact, plus where the momentum
    came from."""
    if axes is None:
        _, axes = new_fig(1, 2, figsize=(10, 3.8),
                          gridspec_kw={"width_ratios": [2, 1]})
    left, right = axes
    # Log magnitude, not signed linear. The finding is a ratio spanning two
    # orders of magnitude, and on a shared linear axis the best-performing
    # drive is a flat line indistinguishable from zero -- the reader would see
    # the worst case in detail and the interesting case not at all.
    for label, (r, m) in results.items():
        c = drive_color(label)
        left.semilogy(r.t_hat, np.maximum(np.abs(r.hat(r.tau_j)), 1e-3),
                      lw=1.6, color=c, label=label, zorder=3)
    style_axes(left, "time / t*", "|transmitted torque| / mgL",
               "T4  impact: torque through the drive")
    left.legend(frameon=False, fontsize=9, labelcolor=INK_MUTED)

    labels = list(results.keys())
    share = [results[k][1]["reflected_momentum_share"] * 100 for k in labels]
    y = np.arange(len(labels))
    right.barh(y, share, color=[drive_color(k) for k in labels], height=0.6,
               zorder=3)
    right.set_yticks(y)
    right.set_yticklabels(labels, fontsize=9, color=INK)
    for yi, v in enumerate(share):
        right.text(v, yi, "  %.0f%%" % v, va="center", fontsize=9,
                   color=INK_MUTED)
    right.set_xlim(0, 105)
    style_axes(right, "% of momentum from the motor", None,
               "where the impact energy lives")
    return axes


def plot_bode(configs, freqs=None, ax=None, mark_peaks=True):
    """T5: analytic FRF magnitude, with the resonance/antiresonance pair
    marked. Log-log; one axis, magnitude only."""
    from .bench.tests import analytic_frf, resonance_pair
    if ax is None:
        _, ax = new_fig(figsize=(8.4, 4.4))
    if freqs is None:
        freqs = np.logspace(-1, 3, 1200)
    for label, cfg in configs.items():
        c = drive_color(label)
        H = analytic_frf(cfg, freqs)
        ax.loglog(freqs, np.abs(H), lw=1.7, color=c, label=label, zorder=3)
        if mark_peaks:
            fa, fr = resonance_pair(cfg)
            for f, marker in ((fa, "v"), (fr, "^")):
                ax.plot([f], [np.abs(analytic_frf(cfg, [f]))[0]], marker,
                        ms=8, color=c, mec=SURFACE, mew=1.5, zorder=4)
    style_axes(ax, "frequency [Hz]", "|theta_link / tau_motor|  [rad/(N*m)]",
               "T5  frequency response   (v antiresonance,  ^ resonance)")
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_MUTED)
    return ax


# ---------------------------------------------------------------------------
# Capstan-specific figures
# ---------------------------------------------------------------------------

def plot_wrap_saturation(mu_values=(0.1, 0.15, 0.2, 0.3), n_max=6, ax=None):
    """The tanh, which is the single most useful capstan plot: it shows that
    wraps past about three buy nothing."""
    if ax is None:
        _, ax = new_fig(figsize=(6.4, 4.0))
    wraps = np.linspace(0.25, n_max, 300)
    cmap = plt.get_cmap("Blues")
    for i, mu in enumerate(mu_values):
        grip = np.tanh(mu * 2 * np.pi * wraps / 2)
        shade = cmap(0.35 + 0.6 * i / max(len(mu_values) - 1, 1))
        ax.plot(wraps, grip, lw=1.8, color=shade, zorder=3)
        ax.annotate("mu = %.2f" % mu, xy=(wraps[-1], grip[-1]),
                    xytext=(4, 0), textcoords="offset points",
                    fontsize=9, color=INK_MUTED, va="center")
    ax.axvline(3.0, color=INK_MUTED, lw=1, ls=":", zorder=2)
    ax.text(3.05, 0.08, "3 wraps", fontsize=9, color=INK_MUTED)
    ax.set_ylim(0, 1.05)
    ax.set_xlim(0, n_max * 1.22)
    style_axes(ax, "wraps on the capstan", "grip factor  tanh(mu*beta/2)",
               "Torque capacity saturates with wrap angle")
    return ax


def plot_ratio_tradeoff(R_output=0.090, N_range=(2, 30), arm=None,
                        cable=None, axes=None):
    """The capstan's central design conflict, in three panels.

    Raising N shrinks the capstan, which (a) collapses input-referred
    stiffness as r^2, (b) forces higher pretension for the same torque, and
    (c) drives the cable bend ratio below what the cable tolerates. Three
    panels rather than three lines on one axis, because the quantities have
    nothing to do with each other dimensionally.
    """
    from .drives.capstan import CapstanDrive
    from .catalog import CABLES, ARMS
    arm = arm or ARMS["desktop-300"]
    cable = cable or CABLES["steel-7x19"]
    if axes is None:
        _, axes = new_fig(1, 3, figsize=(11.5, 3.6))

    Ns = np.linspace(N_range[0], N_range[1], 60)
    K_in, K_out, bend, pret = [], [], [], []
    for N in Ns:
        d = CapstanDrive.sized_for(2.0 * arm.gravity_moment, R_output, N,
                                   cable, inertia_ref=arm.inertia)
        K_out.append(d.stiffness_out(0.0))
        K_in.append(d.stiffness_out(0.0) / d.N ** 2)
        bend.append(d.bend_ratio)
        pret.append(d.T_p)

    c = PALETTE["capstan"]
    axes[0].semilogy(Ns, K_out, lw=1.8, color=c, label="output-referred")
    axes[0].semilogy(Ns, K_in, lw=1.8, color=c, ls="--",
                     label="input-referred")
    axes[0].legend(frameon=False, fontsize=9, labelcolor=INK_MUTED)
    style_axes(axes[0], "reduction ratio N", "stiffness [N*m/rad]",
               "Stiffness splits with ratio")

    axes[1].plot(Ns, pret, lw=1.8, color=c)
    style_axes(axes[1], "reduction ratio N", "required pretension [N]",
               "Pretension for 2x gravity torque")

    axes[2].plot(Ns, bend, lw=1.8, color=c, zorder=3)
    axes[2].axhline(cable.min_bend_ratio, color="#b91c1c", lw=1.4, ls=":",
                    zorder=2)
    axes[2].set_ylim(0, min(max(bend), 6.0 * cable.min_bend_ratio))
    axes[2].text(Ns[-1], cable.min_bend_ratio * 1.12,
                 "%s minimum" % cable.name, fontsize=9, color="#b91c1c",
                 ha="right")
    style_axes(axes[2], "reduction ratio N", "capstan D / cable d",
               "Bend ratio -- the binding constraint")
    return axes


def plot_efficiency_curves(drives, ax=None, tau_max=None):
    """Efficiency against load. The shape matters more than any single number:
    a preload-dominated drive climbs steeply, a gear-dominated one is flat."""
    if ax is None:
        _, ax = new_fig(figsize=(6.8, 4.0))
    for label, d in drives.items():
        top = tau_max or (d.tau_max() if np.isfinite(d.tau_max()) else 10.0)
        taus = np.linspace(0.01 * top, top, 200)
        eta = [d.efficiency_at(t) for t in taus]
        ax.plot(taus / top * 100, eta, lw=1.8, color=drive_color(label),
                label=label, zorder=3)
    ax.set_ylim(0, 1.02)
    style_axes(ax, "output torque [% of rating]", "efficiency",
               "Efficiency is a curve, not a number")
    ax.legend(frameon=False, fontsize=9, labelcolor=INK_MUTED, loc="lower right")
    return ax


# ---------------------------------------------------------------------------
# Animation
# ---------------------------------------------------------------------------

def animate_capstan(r, stride=8, figsize=(9.5, 4.6), trail=60, dpi=72):
    """Side-by-side animation: the capstan turning, and the arm it drives.

    Returns a matplotlib FuncAnimation. In a notebook:
        from IPython.display import HTML
        HTML(animate_capstan(r).to_jshtml())

    The windup is drawn exaggerated -- a real capstan deflects by arcminutes,
    which is invisible at this scale. The exaggeration factor is printed on
    the figure so the picture cannot be mistaken for a measurement.

    `stride` and `dpi` govern the size of the embedded result, which matters
    more than it looks: to_jshtml() base64-encodes every frame into the
    notebook, so a committed notebook carries the whole animation as a blob and
    a fresh copy lands in git history on every re-run. Defaults here are chosen
    to keep that in the low megabytes rather than the low tens.
    """
    from matplotlib.animation import FuncAnimation
    from matplotlib.patches import Circle

    cfg = r.cfg
    drive = cfg.drive
    R = getattr(drive, "R_output", 0.06)
    rc = getattr(drive, "r_capstan", R / drive.N)
    L = cfg.plant.length

    idx = np.arange(0, len(r.t), stride)
    windup = r.windup
    w_scale = 0.25 / max(np.max(np.abs(windup)), 1e-9)   # exaggeration

    fig, (axm, axa) = plt.subplots(1, 2, figsize=figsize, facecolor=SURFACE,
                                   dpi=dpi,
                                   gridspec_kw={"width_ratios": [1, 1.3]})

    # ---- left: the mechanism
    span = R * 2.6
    axm.set_xlim(-span, span)
    axm.set_ylim(-span * 0.75, span * 0.75)
    axm.set_aspect("equal")
    axm.axis("off")
    axm.set_title("capstan  N = %.1f : 1" % drive.N, color=INK, fontsize=11,
                  loc="left")

    cx = -R - rc * 1.5
    axm.add_patch(Circle((0, 0), R, fill=False, ec=GRID, lw=2))
    axm.add_patch(Circle((cx, 0), rc, fill=False, ec=GRID, lw=2))
    sector_line, = axm.plot([], [], lw=2.5, color=PALETTE["capstan"])
    capstan_line, = axm.plot([], [], lw=2.0, color=INK_MUTED)
    cable_top, = axm.plot([], [], lw=1.2, color=INK_MUTED)
    cable_bot, = axm.plot([], [], lw=1.2, color=INK_MUTED)
    slip_txt = axm.text(0.02, 0.02, "", transform=axm.transAxes, fontsize=9,
                        color="#b91c1c")
    axm.text(0.02, 0.93, "windup drawn x%.0f" % w_scale,
             transform=axm.transAxes, fontsize=8, color=INK_MUTED)

    # ---- right: the arm
    axa.set_xlim(-L * 1.25, L * 1.25)
    axa.set_ylim(-L * 1.25, L * 1.25)
    axa.set_aspect("equal")
    axa.axis("off")
    axa.set_title("lever arm", color=INK, fontsize=11, loc="left")
    axa.plot([0], [0], "o", ms=7, color=GRID)
    arm_line, = axa.plot([], [], lw=3, color=PALETTE["capstan"],
                         solid_capstyle="round")
    tip, = axa.plot([], [], "o", ms=10, color=PALETTE["capstan"])
    trail_line, = axa.plot([], [], lw=1, color=PALETTE["capstan"], alpha=0.25)
    time_txt = axa.text(0.02, 0.02, "", transform=axa.transAxes, fontsize=9,
                        color=INK_MUTED)

    def frame(k):
        i = idx[k]
        th_l = r.theta_l[i]
        th_c = r.theta_m[i] * drive.N          # true capstan rotation
        wind = windup[i] * w_scale

        a = th_l
        sector_line.set_data([0, R * math.cos(a)], [0, R * math.sin(a)])
        b = th_c
        capstan_line.set_data([cx, cx + rc * math.cos(b)],
                              [0, rc * math.sin(b)])
        cable_top.set_data([cx, 0], [rc, R])
        cable_bot.set_data([cx, 0], [-rc, -R])
        slip_txt.set_text("SLIPPING" if abs(r.theta_slip[i]) > 1e-5 else "")

        arm_line.set_data([0, L * math.cos(th_l)], [0, L * math.sin(th_l)])
        tip.set_data([L * math.cos(th_l)], [L * math.sin(th_l)])
        lo = max(0, i - trail * stride)
        trail_line.set_data(L * np.cos(r.theta_l[lo:i + 1]),
                            L * np.sin(r.theta_l[lo:i + 1]))
        time_txt.set_text("t / t* = %.1f    wind = %.2f arcmin"
                          % (r.t_hat[i], math.degrees(windup[i]) * 60))
        return (sector_line, capstan_line, cable_top, cable_bot, arm_line,
                tip, trail_line, time_txt, slip_txt)

    plt.close(fig)
    return FuncAnimation(fig, frame, frames=len(idx), interval=40, blit=True)
