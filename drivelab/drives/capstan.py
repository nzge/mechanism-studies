"""Cable capstan drive -- derived from geometry and material.

Almost nothing here is a datasheet number. Ratio, stiffness, torque capacity,
stroke and no-load drag all fall out of the cable, the two radii and the
pretension. That is the point: when the comparison later says the capstan wins
on backdrivability, it is a *prediction* from the geometry, not a constant that
was typed in to make it win.

The two results worth knowing before reading the code
-----------------------------------------------------

Torque capacity, from Euler-Eytelwein with a symmetric pretension T_p. The
cable grips while T_tight/T_slack <= exp(mu*beta); writing T_tight = T_p + dT
and T_slack = T_p - dT and solving for the largest dT gives

    tau_max = 2 * T_p * r_c * tanh(mu * beta / 2)

The tanh saturates hard. At mu = 0.2, three wraps (beta = 18.8 rad) already
reaches tanh(1.88) = 0.955; a fourth wrap buys 2%. Wraps are nearly free but
nearly useless past three, so the only real levers are pretension and radius --
and pretension is paid for twice, in bearing load and in drag.

Stiffness, from two cable runs of stiffness EA/L acting at radius R:

    K_out = 2 (EA/L) R^2        K_in = K_out / N^2 = 2 (EA/L) r_c^2

Output stiffness grows with R^2 while input-referred stiffness falls with
r_c^2. Raising the ratio by shrinking the capstan therefore guts the stiffness
the *motor* sees, quadratically, even as the number on the output side improves.
That is the trade this simulation exists to make visible.
"""

import math
from .base import DriveModel
from ..friction import make_friction


class CableMaterial(object):
    """Cable properties. E_eff is the *effective* modulus of the construction,
    not the bulk modulus of the wire: a 7x19 steel rope runs near 100-130 GPa
    against 200 GPa for solid wire, because the strands take up before the
    metal does. Fill factor is the metallic area fraction of the circumscribed
    circle -- about 0.55-0.60 for 7x19."""

    def __init__(self, name, E_eff, strength_mpa, fill_factor=0.58,
                 density=7850.0, min_bend_ratio=20.0, creep_rate=0.0):
        self.name = name
        self.E_eff = E_eff                    # Pa
        self.strength_mpa = strength_mpa      # MPa on metallic area
        self.fill_factor = fill_factor
        self.density = density                # kg/m^3
        self.min_bend_ratio = min_bend_ratio  # minimum sheave D / cable d
        self.creep_rate = creep_rate          # fractional tension loss/decade

    def area(self, diameter):
        """Load-bearing metallic area [m^2]."""
        return self.fill_factor * math.pi * diameter ** 2 / 4.0

    def breaking_load(self, diameter):
        return self.strength_mpa * 1e6 * self.area(diameter)

    def __repr__(self):
        return "<CableMaterial %s E=%.0fGPa>" % (self.name, self.E_eff / 1e9)


class CapstanDrive(DriveModel):
    """A cable wrapped n times around a driven capstan, terminating on an
    output sector. Single stage."""

    physics_derived = True

    def __init__(self, r_capstan, R_output, cable, cable_diameter,
                 n_wraps=3.0, mu=0.2, pretension=None, pretension_sf=0.25,
                 free_length=None, capstan_width=None,
                 mu_bend=0.02, bearing_drag=0.0,
                 eta_load_proportional=0.985,
                 structural_damping_ratio=0.02,
                 friction_model="lugre", safety_factor=3.0,
                 name="capstan", inertia_ref=None):
        self.r_capstan = float(r_capstan)
        self.R_output = float(R_output)
        self.cable = cable
        self.d = float(cable_diameter)
        self.n_wraps = float(n_wraps)
        self.mu = float(mu)
        self.mu_bend = float(mu_bend)
        self.safety_factor = float(safety_factor)
        self.name = name

        # The cable neutral axis sits half a diameter off the groove floor.
        # Forgetting this is a classic capstan sizing error -- on a 3 mm
        # capstan with 1 mm cable it is a 33% ratio error.
        self.r_eff = self.r_capstan + self.d / 2.0
        self.R_eff = self.R_output + self.d / 2.0

        # Pretension defaults to a fraction of breaking load.
        self.T_break = cable.breaking_load(self.d)
        if pretension is None:
            pretension = pretension_sf * self.T_break / self.safety_factor
        self.T_p = float(pretension)

        self._inertia_ref = float(inertia_ref) if inertia_ref else 1.0e-4
        self.free_length = (float(free_length) if free_length is not None
                            else 2.0 * self.R_output)
        self.capstan_width = (float(capstan_width) if capstan_width is not None
                              else 20.0 * self.d)

        N = self.R_eff / self.r_eff
        self._structural_damping_ratio = structural_damping_ratio
        self._slip_tau = 1.0e-3   # slip relaxation time constant [s]

        # No-load drag is *derived*: cable bending hysteresis over the capstan,
        # proportional to the tension running through the bend and to d/D.
        # With symmetric pretension the two runs carry 2*T_p in total, so the
        # geometry collapses neatly to tau_c0 = N * mu_bend * d * T_p.
        tau_c0 = N * self.mu_bend * self.d * self.T_p + bearing_drag
        self.bearing_drag = bearing_drag

        DriveModel.__init__(
            self, N=N, eta_forward=eta_load_proportional,
            friction=make_friction(
                friction_model,
                tau_c0=tau_c0,
                eta_forward=eta_load_proportional,
                viscous=0.0,
                stiction_ratio=1.15,   # cable grip: very little stiction
                omega_stribeck=0.02,
                presliding=2.0e-4,
                tau_ref=max(tau_c0, 1e-6),
                inertia_ref=(inertia_ref if inertia_ref else 1.0e-4)))

    # ---- sizing ----------------------------------------------------------

    @classmethod
    def sized_for(cls, tau_required, R_output, N, cable, n_wraps=3.0, mu=0.2,
                  safety_factor=3.0, d_min=0.3e-3, d_max=6.0e-3, **kw):
        """Size a capstan to a torque requirement, the way you would on paper.

        Pretension is *derived* rather than guessed: invert
        tau = 2 T_p r tanh(mu beta / 2) for the T_p that just meets
        `tau_required`, then pick the smallest standard-ish cable whose tight
        side still clears the allowable stress.

        This ordering is what exposes the capstan's real constraint. Torque
        wants a small capstan (high N) and high pretension; high pretension
        wants a thick cable; a thick cable wants a *large* capstan to keep the
        bend ratio D/d healthy. Those three pull against each other, and for
        compact high-ratio designs the bend ratio usually loses -- which is
        why real capstan drives are physically larger than their torque
        rating suggests they should be. The returned drive reports
        `bend_ratio_ok()` so the conflict stays visible rather than silently
        becoming a fatigue problem.
        """
        r_capstan = R_output / N
        grip = math.tanh(mu * 2.0 * math.pi * n_wraps / 2.0)

        # Solve on effective radii, which depend on d -- iterate a few times.
        d = d_min
        T_p = 0.0
        for _ in range(60):
            r_eff = r_capstan + d / 2.0
            T_p = tau_required / (2.0 * r_eff * grip)
            T_tight = T_p * (1.0 + grip)
            need_break = safety_factor * T_tight
            area = need_break / (cable.strength_mpa * 1e6)
            d_new = math.sqrt(4.0 * area / (cable.fill_factor * math.pi))
            if abs(d_new - d) < 1e-7:
                d = d_new
                break
            d = 0.5 * (d + d_new) if d_new > d else d_new
            if d > d_max:
                d = d_max
                break
        d = max(d_min, min(d, d_max))
        r_eff = r_capstan + d / 2.0
        T_p = tau_required / (2.0 * r_eff * grip)

        return cls(r_capstan=r_capstan, R_output=R_output, cable=cable,
                   cable_diameter=d, n_wraps=n_wraps, mu=mu,
                   pretension=T_p, safety_factor=safety_factor, **kw)

    # ---- geometry-derived quantities -------------------------------------

    @property
    def wrap_angle(self):
        """beta [rad] on the capstan."""
        return 2.0 * math.pi * self.n_wraps

    @property
    def grip_factor(self):
        """tanh(mu*beta/2) -- the fraction of 2*T_p*r that is usable."""
        return math.tanh(self.mu * self.wrap_angle / 2.0)

    @property
    def cable_stiffness(self):
        """Axial stiffness of one free run, EA/L [N/m]."""
        return self.cable.E_eff * self.cable.area(self.d) / self.free_length

    @property
    def tension_tight_max(self):
        return self.T_p * (1.0 + self.grip_factor)

    @property
    def bend_ratio(self):
        """Capstan diameter / cable diameter. Below the cable minimum this
        wrecks fatigue life -- reported, not enforced."""
        return 2.0 * self.r_capstan / self.d

    def bend_ratio_ok(self):
        return self.bend_ratio >= self.cable.min_bend_ratio

    # ---- interface: compliance -------------------------------------------

    def stiffness_out(self, delta=0.0):
        """K_out = 2 (EA/L) R^2 while both runs are in tension.

        Past |dT| > T_p the slack run goes slack and only one carries, halving
        the stiffness. In practice this never happens: the slack-out torque is
        2*R*T_p while the slip torque is 2*r*T_p*tanh(...) < 2*r*T_p, and
        r < R always. A capstan therefore *always slips before it goes slack* --
        the bilinear stiffness is modelled here for completeness but is not a
        regime a real drive reaches.
        """
        k_full = 2.0 * self.cable_stiffness * self.R_eff ** 2
        dT = self.cable_stiffness * self.R_eff * abs(delta)
        return k_full if dT < self.T_p else 0.5 * k_full

    def damping_out(self):
        # Referred to the same inertia that damping_ratio() normalizes by, so
        # the zeta asked for is the zeta reported. Getting this inconsistent is
        # an easy way to hand one drive a quiet advantage in every test.
        return (self._structural_damping_ratio * 2.0
                * math.sqrt(self.stiffness_out(0.0) * self._inertia_ref))

    # ---- interface: limits -----------------------------------------------

    def tau_max_slip(self):
        """2 T_p r tanh(mu beta / 2) -- the Euler-Eytelwein limit."""
        return 2.0 * self.T_p * self.r_eff * self.grip_factor

    def tau_max_strength(self):
        """Torque at which the tight side reaches the allowable stress."""
        allow = self.T_break / self.safety_factor
        dT = max(allow - self.T_p, 0.0)
        return 2.0 * dT * self.r_eff

    def tau_max(self):
        return min(self.tau_max_slip(), self.tau_max_strength())

    def limiting_mode(self):
        return ("slip" if self.tau_max_slip() <= self.tau_max_strength()
                else "cable strength")

    def stroke_limit(self):
        """Output half-travel [rad], set by how much cable the capstan can
        spool alongside its grip wraps: N*theta/(2pi) + n_wraps <= W/d."""
        turns_available = self.capstan_width / self.d - self.n_wraps
        if turns_available <= 0.0:
            return 0.0
        full_range = 2.0 * math.pi * turns_available / self.N
        return full_range / 2.0

    def slip_rate(self, tau_spring, omega_rel):
        excess = abs(tau_spring) - self.tau_max()
        if excess <= 0.0:
            return 0.0
        # Windup beyond the grip limit relaxes with time constant _slip_tau.
        rate = excess / (self.stiffness_out(0.0) * self._slip_tau)
        return math.copysign(rate, tau_spring)

    # ---- interface: inertia ----------------------------------------------

    def reflected_inertia_extra(self, capstan_density=2700.0,
                                sector_density=2700.0,
                                sector_thickness=0.006):
        """Capstan (reflected by N^2) plus output sector, output-referred."""
        m_cap = (capstan_density * math.pi * self.r_capstan ** 2
                 * self.capstan_width)
        J_cap = 0.5 * m_cap * self.r_capstan ** 2
        m_sec = sector_density * math.pi * self.R_output ** 2 * sector_thickness
        J_sec = 0.5 * m_sec * self.R_output ** 2
        return self.N ** 2 * J_cap + J_sec

    # ---- reporting -------------------------------------------------------

    def efficiency_at(self, tau_out):
        """Efficiency at a given output torque. Unlike a gearbox, the dominant
        capstan loss is preload-driven and load-*independent*, so efficiency
        rises with load rather than staying flat."""
        if abs(tau_out) <= 0.0:
            return 0.0
        loss = self.friction.tau_c0 + self.friction.c * abs(tau_out)
        return abs(tau_out) / (abs(tau_out) + loss)

    def summary(self):
        d = DriveModel.summary(self)
        d.update({
            "r_capstan_mm": self.r_capstan * 1e3,
            "R_output_mm": self.R_output * 1e3,
            "cable": "%s d=%.2fmm" % (self.cable.name, self.d * 1e3),
            "n_wraps": self.n_wraps,
            "pretension_N": self.T_p,
            "pretension_pct_break": 100.0 * self.T_p / self.T_break,
            "grip_factor": self.grip_factor,
            "tau_max_slip": self.tau_max_slip(),
            "tau_max_strength": self.tau_max_strength(),
            "limiting_mode": self.limiting_mode(),
            "bend_ratio": self.bend_ratio,
            "bend_ratio_ok": self.bend_ratio_ok(),
            "K_in": self.stiffness_out(0.0) / self.N ** 2,
            "eta_at_tau_max": self.efficiency_at(self.tau_max()),
            "eta_at_10pct": self.efficiency_at(0.1 * self.tau_max()),
        })
        return d
