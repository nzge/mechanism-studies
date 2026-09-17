"""Friction models for the transmission.

Two are provided, with the same signature, because the tests want different
things from them:

  LuGre        -- a bristle model with a real internal state. Reproduces
                  *presliding* displacement, so a slow torque reversal traces a
                  genuine hysteresis loop instead of a corner. This is what
                  makes the lost-motion test (T1) meaningful rather than a
                  rendering of whatever deadband you typed in.

  Regularized  -- tanh-smoothed Coulomb + viscous. Stateless and cheap. Use it
                  for gross-motion tests where presliding is irrelevant.

Both take a *load-dependent* Coulomb level:

    tau_c = tau_c0 + c * |tau_j|

`tau_c0` is the no-load drag (preload, seals, bearings). The `c * |tau_j|` term
is the load-proportional loss, and it is the whole story of backdrivability:
with loss proportional to transmitted torque, forward and backward efficiency
are locked together by

    c = 1/eta_f - 1        ==>       eta_b = 2 - 1/eta_f

so eta_f = 0.8 gives eta_b = 0.75, eta_f = 0.6 gives 0.33, and eta_f = 0.5 is
exactly self-locking. Backdrivability is therefore *predicted* by the model
rather than assumed -- one number in, both directions out.
"""

import math

_EPS = 1e-12


def loss_coefficient(eta_forward):
    """c such that eta_b = 2 - 1/eta_f falls out of the loss model."""
    if eta_forward <= 0.0 or eta_forward > 1.0:
        raise ValueError("eta_forward must be in (0, 1]")
    return 1.0 / eta_forward - 1.0


def eta_backward(eta_forward):
    """Implied backdriving efficiency. <= 0 means self-locking."""
    if eta_forward <= 0.0:
        return float("-inf")
    return 2.0 - 1.0 / eta_forward


class RegularizedFriction(object):
    """tanh-smoothed Coulomb + Stribeck + viscous. Stateless."""

    n_states = 0

    def __init__(self, tau_c0, eta_forward, viscous=0.0,
                 stiction_ratio=1.4, omega_stribeck=0.01, omega_eps=1e-3):
        self.tau_c0 = tau_c0
        self.c = loss_coefficient(eta_forward)
        self.viscous = viscous
        self.stiction_ratio = stiction_ratio
        self.omega_stribeck = omega_stribeck
        self.omega_eps = omega_eps

    def coulomb_level(self, tau_j):
        return self.tau_c0 + self.c * abs(tau_j)

    def torque(self, omega, tau_j, z=0.0):
        tc = self.coulomb_level(tau_j)
        ts = self.stiction_ratio * tc
        stribeck = tc + (ts - tc) * math.exp(-(omega / self.omega_stribeck) ** 2)
        return stribeck * math.tanh(omega / self.omega_eps) + self.viscous * omega

    def dz(self, omega, tau_j, z):
        return 0.0

    def breakaway(self, tau_j):
        return self.stiction_ratio * self.coulomb_level(tau_j)


class LuGreFriction(object):
    """LuGre bristle model in the torque domain.

    State `z` is the mean bristle deflection [rad]. Bristle stiffness is set
    from a presliding displacement so the model auto-scales with drive size
    instead of needing a hand-tuned sigma0.
    """

    n_states = 1

    def __init__(self, tau_c0, eta_forward, viscous=0.0,
                 stiction_ratio=1.4, omega_stribeck=0.01,
                 presliding=1.0e-4, tau_ref=None, zeta_bristle=1.0,
                 inertia_ref=1.0):
        self.tau_c0 = tau_c0
        self.c = loss_coefficient(eta_forward)
        self.viscous = viscous
        self.stiction_ratio = stiction_ratio
        self.omega_stribeck = omega_stribeck
        self.presliding = presliding
        # Bristle stiffness from a reference drag level, so that breakaway
        # occurs after roughly `presliding` radians of micro-motion.
        ref = tau_ref if tau_ref is not None else max(tau_c0, _EPS)
        self.sigma0 = max(ref, _EPS) / presliding
        # sigma1 is *micro*-damping: it must matter during presliding and be
        # negligible in gross sliding. Sizing it as 2*sqrt(sigma0*J) with the
        # load inertia -- the form quoted for bristle critical damping -- is a
        # trap here: it produces a coefficient far above the Coulomb level, so
        # every motion reversal dissipates as though through a heavy dashpot
        # and the drive reports an efficiency it does not have.
        #
        # Tie it to the friction level and the Stribeck velocity instead, so
        # its contribution stays a fixed small fraction of tau_c at the speed
        # where presliding gives way to sliding.
        self.sigma1 = (zeta_bristle * 0.1 * max(ref, _EPS)
                       / max(omega_stribeck, _EPS))
        self.sigma2 = viscous

    def coulomb_level(self, tau_j):
        return self.tau_c0 + self.c * abs(tau_j)

    def _g(self, omega, tau_j):
        tc = self.coulomb_level(tau_j)
        ts = self.stiction_ratio * tc
        g = (tc + (ts - tc) * math.exp(-(omega / self.omega_stribeck) ** 2))
        return max(g / self.sigma0, _EPS)

    def dz(self, omega, tau_j, z):
        return omega - abs(omega) * z / self._g(omega, tau_j)

    def torque(self, omega, tau_j, z):
        dz = self.dz(omega, tau_j, z)
        return self.sigma0 * z + self.sigma1 * dz + self.sigma2 * omega

    def breakaway(self, tau_j):
        return self.stiction_ratio * self.coulomb_level(tau_j)


def make_friction(kind, **kw):
    if kind == "lugre":
        return LuGreFriction(**kw)
    if kind == "regularized":
        return RegularizedFriction(
            **{k: v for k, v in kw.items()
               if k in ("tau_c0", "eta_forward", "viscous",
                        "stiction_ratio", "omega_stribeck")})
    raise ValueError("unknown friction model: %r" % (kind,))
