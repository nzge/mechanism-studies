"""Cycloidal drive -- derived from geometry, material and tolerances.

Like the capstan and the harmonic, almost nothing here is a datasheet number.
The ratio is a counting argument (pins minus one); the stiffness falls out of
the output pins in bending plus Hertzian contact at the pin ring; the backlash
falls out of the output-pin clearance; the ripple falls out of two tolerances;
and the load-proportional loss falls out of the rolling of the disc over the
ring pins, the eccentric bearing, and the sliding of the output pins.

The closed forms worth knowing before reading the code
--------------------------------------------------------

Ratio. A disc with z_d lobes rolls inside a ring of z_p = z_d + 1 pins while
its centre orbits at eccentricity e on the input cam. Every cam revolution
advances the disc by one pin pitch, so

    N = z_d = z_p - 1

a 30:1 drive is a 30-lobe disc in 31 pins. Exact, by counting.

Pin loading. Half the ring pins are engaged and share the torque with a
sinusoidal distribution, so sum(F_i sin phi_i) = tau / r_p and

    F_max = 4 tau / (z_p r_p)

-- the single number every stress and stiffness estimate below hangs on.

Stiffness. The soft element is the output pins: cantilevers in bending,
n of them in parallel at the output pitch radius,

    K_out ~ n (3 E I / L^3) R_out^2          I = pi r^4 / 4

in series with the Hertzian line contact at the pin ring. The disc body and
the ring are treated as rigid -- they are, comparatively, by an order of
magnitude or more.

Backlash. The disc drives the output flange through pins that run in holes
with diametral clearance c. Before the pins engage, the disc can rotate
c / R_out each way:

    backlash = 2 c / R_out

This is the cycloidal's distinguishing cost, and it is *derived* here from a
fit tolerance rather than assumed. Preloaded (tapered) output pins delete it
and are modelled with the same flag.

Ripple. The ideal cycloidal profile is conjugate -- zero transmission error.
What is measured enters through manufacture: eccentricity error on the cam
(repeating once per input revolution) and pin pitch error (repeating at the
pin-passing frequency, z_p cycles per output revolution). Both amplitudes are
the tolerance over the pitch radius.

Torque capacity. The output pins bend: F_allow = pi r^3 sigma_allow / (4 L sf)
per pin, and the pin-ring contact must survive Hertzian line loading:
F_allow = 2 pi L R* (sigma_allow / sf)^2 / E*. Both are derived; the smaller
wins.

Loss. The disc *rolls* over the ring pins (rolling resistance), the eccentric
bearing carries the mesh force, and the output pins *slide* in their holes by
the eccentricity per output revolution. All three scale with load, so they
enter as c = 1/eta_f - 1 and the backward efficiency follows as 2 - 1/eta_f --
one knob, as everywhere else in this bench.
"""

import math

from .base import DriveModel
from ..friction import make_friction
from ..materials import IsotropicMaterial, STEEL_HARD


class CycloidalDrive(DriveModel):
    """A single-disc cycloidal reducer, derived from geometry.

    Identity: the pin ring (count and pitch radius), the disc, the cam
    eccentricity, and the output pin set. Everything reported by the bench
    follows from those plus material and two tolerances.
    """

    physics_derived = True

    def __init__(self, z_pins, r_pitch, r_pin, disc_width, eccentricity,
                 n_out_pins=8, r_out_pin=3.0e-3, r_out_pitch=None,
                 out_pin_length=0.02, clearance=10.0e-6, preloaded=False,
                 material=STEEL_HARD, pin_material=None,
                 eccentricity_tol=5.0e-6, pin_pitch_tol=10.0e-6,
                 mu_roll=0.005, mu_bearing=0.002, mu_outpin=0.08,
                 r_ecc_bearing=None, seal_drag=2.0e-3,
                 safety_factor=3.0, structural_damping_ratio=0.02,
                 friction_model="lugre", name="cycloidal", inertia_ref=None):
        self.z_pins = int(z_pins)
        if self.z_pins < 3:
            raise ValueError("a cycloidal ring needs at least 3 pins")
        self.r_pitch = float(r_pitch)        # pin-ring pitch radius [m]
        self.r_pin = float(r_pin)            # ring pin (roller) radius [m]
        r_pin_max = self.r_pitch * math.sin(math.pi / self.z_pins)
        if self.r_pin > 0.98 * r_pin_max:
            raise ValueError(
                "ring pins overlap: r_pin %.2f mm exceeds %.2f mm spacing"
                % (self.r_pin * 1e3, r_pin_max * 1e3))
        self.disc_width = float(disc_width)  # disc face width [m]
        self.eccentricity = float(eccentricity)  # cam eccentricity e [m]
        self.n_out_pins = int(n_out_pins)
        self.r_out_pin = float(r_out_pin)
        self.r_out_pitch = (float(r_out_pitch) if r_out_pitch is not None
                            else 0.55 * self.r_pitch)
        self.out_pin_length = float(out_pin_length)
        self.clearance = float(clearance)    # diametral hole-pin clearance [m]
        self.preloaded = bool(preloaded)
        self.material = material
        self.pin_material = pin_material if pin_material is not None else material
        self.eccentricity_tol = float(eccentricity_tol)
        self.pin_pitch_tol = float(pin_pitch_tol)
        self.mu_roll = float(mu_roll)
        self.mu_bearing = float(mu_bearing)
        self.mu_outpin = float(mu_outpin)
        self.r_ecc_bearing = (float(r_ecc_bearing) if r_ecc_bearing is not None
                              else 0.40 * self.r_pitch)
        self.seal_drag = float(seal_drag)
        self.safety_factor = float(safety_factor)
        self.name = name
        self._structural_damping_ratio = structural_damping_ratio
        self._inertia_ref = float(inertia_ref) if inertia_ref else 1.0e-4

        N = float(self.z_pins - 1)
        self.N = N   # needed by _derive_losses; re-set by the base class
        self._disc_radius = self.r_pitch - self.r_pin - self.eccentricity
        if self._disc_radius <= 0.0:
            raise ValueError("disc does not fit inside the pin ring: "
                             "r_pitch - r_pin - e must be positive")
        eta_f, tau_c0, c_total = self._derive_losses()
        self._tau_c0 = tau_c0
        self._c_total = c_total

        DriveModel.__init__(
            self, N=N, eta_forward=eta_f,
            friction=make_friction(
                friction_model,
                tau_c0=tau_c0,
                eta_forward=eta_f,
                viscous=0.0,
                stiction_ratio=1.2,    # rolling contacts: little stiction
                omega_stribeck=0.02,
                presliding=5.0e-5,
                tau_ref=max(tau_c0, 1e-6),
                inertia_ref=(inertia_ref if inertia_ref else 1.0e-4)))

    # ---- sizing ----------------------------------------------------------

    @classmethod
    def sized_for(cls, tau_required, N, r_pitch, r_pin=6.0e-3,
                  disc_width=10.0e-3, eccentricity=None, out_pin_length=None,
                  material=STEEL_HARD, safety_factor=3.0, min_stiffness=0.0,
                  **kw):
        """Size the output pin set to a torque requirement: invert the pin
        bending limit for the pin radius, then check the disc still fits the
        ring. This ordering is what exposes the cycloidal's conflict -- a big
        ratio wants a big pin ring, but a big ring forces a large eccentricity
        to keep the contact geometry healthy, and the eccentricity is what the
        output pins must absorb every revolution.

        `min_stiffness` adds the second, control-side requirement: the pins
        must reach an output stiffness even if strength does not demand it.
        This is not a refinement, it is how real cycloidals are sized --
        a strength-only sizing leaves the drive an order of magnitude too
        compliant to control, which is exactly what their catalog ratings
        (always far above the joint load) are quietly paying for."""
        if N < 2 or N % 1 != 0:
            raise ValueError("N must be a positive integer >= 2")
        z_pins = int(N) + 1
        e = eccentricity if eccentricity is not None else 0.05 * r_pitch
        # Ring pins cannot overlap: at most r_pitch * sin(pi / z_pins) each.
        r_pin = min(r_pin, 0.95 * r_pitch * math.sin(math.pi / z_pins))
        L = out_pin_length if out_pin_length is not None else 0.5 * r_pitch
        n_out = kw.pop("n_out_pins", 8)
        r_out_pitch = kw.pop("r_out_pitch", 0.55 * r_pitch)
        clearance = kw.get("clearance", 10.0e-6)
        # Strength: F_allow = pi r^3 sigma / (4 L sf); tau = n F R.
        r_strength = (4.0 * tau_required * L * safety_factor
                      / (math.pi * material.sigma_allow
                         * n_out * r_out_pitch)) ** (1.0 / 3.0)
        # Control: K = n 3 E I / L^3 * R^2 with I = pi r^4 / 4.
        if min_stiffness > 0.0:
            r_control = (4.0 * min_stiffness * L ** 3
                         / (3.0 * material.E * n_out * r_out_pitch ** 2
                            * math.pi)) ** 0.25
        else:
            r_control = 0.0
        r_pin_out = max(r_strength, r_control)

        # The output holes must sit inside the disc, and the disc inside the
        # pin ring: r_disc = r_pitch - r_pin - e must clear r_out_pitch + the
        # hole radius. If it does not, grow the ring until the pack closes --
        # the same honest iteration as the capstan's cable sizing, and the
        # source of the cycloidal's bulk.
        r_hole = r_pin_out + clearance / 2.0 + e
        r_pitch_need = (r_pin + 2.0 * e + r_pin_out
                        + clearance / 2.0 + 1.0e-3) / 0.45
        r_pitch = max(r_pitch, r_pitch_need)
        r_out_pitch = 0.55 * r_pitch
        # Re-solve: the pin radius was sized for the smaller output pitch.
        r_strength = (4.0 * tau_required * L * safety_factor
                      / (math.pi * material.sigma_allow
                         * n_out * r_out_pitch)) ** (1.0 / 3.0)
        if min_stiffness > 0.0:
            r_control = (4.0 * min_stiffness * L ** 3
                         / (3.0 * material.E * n_out * r_out_pitch ** 2
                            * math.pi)) ** 0.25
        r_pin_out = max(r_strength, r_control)
        return cls(z_pins=z_pins, r_pitch=r_pitch, r_pin=r_pin,
                   disc_width=disc_width, eccentricity=e,
                   n_out_pins=n_out, r_out_pin=r_pin_out,
                   r_out_pitch=r_out_pitch, out_pin_length=L,
                   material=material, safety_factor=safety_factor, **kw)

    # ---- geometry-derived quantities -------------------------------------

    @property
    def disc_radius(self):
        """Largest disc that fits inside the pin ring while orbiting."""
        return self._disc_radius

    def pin_force_max(self, tau):
        """Most-loaded ring pin: F_max = 4 tau / (z_p r_p)."""
        return 4.0 * abs(tau) / (self.z_pins * self.r_pitch)

    def _hertz_allowable(self):
        """Per-pin normal force at which the line contact reaches the
        allowable stress: F = 2 pi L R* (sigma/sf)^2 / E*."""
        r_star = self.r_pin   # pin on near-flat lobe flank
        return (2.0 * math.pi * self.disc_width * r_star
                * (self.material.sigma_allow / self.safety_factor) ** 2
                / self.material.E_plane)

    def _hertz_secant(self, force):
        """Secant line-contact stiffness at force `force` [N/m] for a cylinder
        on a flat. Stiffness is weakly (logarithmically) load-dependent, so the
        secant at the design load is the honest single number."""
        e_star = self.material.E_plane
        L = self.disc_width
        r_star = self.r_pin
        b = math.sqrt(4.0 * force * r_star / (math.pi * L * e_star))
        delta = (2.0 * force / (math.pi * L * e_star)
                 * (math.log(4.0 * r_star / b) - 0.5))
        return force / delta if delta > 0.0 else float("inf")

    @property
    def k_output_pins(self):
        """Output pins in bending: n * 3 E I / L^3 * R^2 [N*m/rad]."""
        i_pin = math.pi * self.r_out_pin ** 4 / 4.0
        k_each = 3.0 * self.pin_material.E * i_pin / self.out_pin_length ** 3
        return self.n_out_pins * k_each * self.r_out_pitch ** 2

    @property
    def k_contact(self):
        """Pin-ring Hertzian contact referred to the output: engaged pins
        share load sinusoidally, so the effective parallel count is z_p/4."""
        f_ref = self._hertz_allowable()
        k_h = self._hertz_secant(f_ref)
        return k_h * self.r_pitch ** 2 * (self.z_pins / 4.0)

    @property
    def backlash(self):
        """Deadband from output-pin clearance: 2 c / R_out. Zero if the pins
        are preloaded (tapered)."""
        if self.preloaded:
            return 0.0
        return 2.0 * self.clearance / self.r_out_pitch

    def fit_ok(self):
        """The output holes must fit inside the disc, and the disc inside the
        ring -- a pure geometry check the sizing routine can violate if asked
        for too much torque from too small a package."""
        r_hole = self.r_out_pin + self.clearance / 2.0 + self.eccentricity
        inside_disc = (self.r_out_pitch + r_hole
                       < self.disc_radius - 1.0e-3)
        return inside_disc

    def _derive_losses(self):
        """(eta_f, tau_c0, c_total) -- all derived.

        Load-proportional, each scaled by its own geometry:
            c_roll   = mu_roll (4/pi) (1 + r_pin/r_p)     disc over rollers
            c_bear   = mu_bear (4/pi) r_b N / r_p         eccentric bearing
            c_outpin = mu_outpin e / R_out                pins sliding in holes
        No-load: seal drag plus the unbalanced disc mass orbiting at nominal
        speed through the eccentric bearing, referred to the output.
        """
        c_roll = self.mu_roll * (4.0 / math.pi) * (1.0 + self.r_pin / self.r_pitch)
        c_bear = (self.mu_bearing * (4.0 / math.pi) * self.r_ecc_bearing
                  * self.N / self.r_pitch)
        c_outpin = self.mu_outpin * self.eccentricity / self.r_out_pitch
        c_total = c_roll + c_bear + c_outpin
        eta_f = 1.0 / (1.0 + c_total)

        m_disc = self.disc_mass
        f_unb = m_disc * self.eccentricity * 100.0 ** 2   # at nominal 100 rad/s
        tau_c0 = self.N * (self.seal_drag
                           + self.mu_bearing * f_unb * self.r_ecc_bearing)
        return eta_f, tau_c0, c_total

    # ---- interface: kinematics -------------------------------------------

    def kinematic_error(self, theta_m_out):
        """Two derived components: eccentricity error, once per input
        revolution (N cycles per output revolution), plus pin-pitch error at
        the pin-passing frequency (z_p cycles per output revolution)."""
        e = 0.0
        if self.eccentricity_tol > 0.0:
            e += (self.eccentricity_tol / self.r_pitch
                  * math.sin(self.N * theta_m_out))
        if self.pin_pitch_tol > 0.0:
            e += (self.pin_pitch_tol / (2.0 * self.r_pitch)
                  * math.sin(self.z_pins * theta_m_out))
        return e

    def d_kinematic_error(self, theta_m_out):
        de = 0.0
        if self.eccentricity_tol > 0.0:
            de += (self.eccentricity_tol / self.r_pitch
                   * self.N * math.cos(self.N * theta_m_out))
        if self.pin_pitch_tol > 0.0:
            de += (self.pin_pitch_tol / (2.0 * self.r_pitch)
                   * self.z_pins * math.cos(self.z_pins * theta_m_out))
        return de

    def error_amplitude(self):
        return (self.eccentricity_tol / self.r_pitch
                + self.pin_pitch_tol / (2.0 * self.r_pitch))

    # ---- interface: compliance -------------------------------------------

    def stiffness_out(self, delta=0.0):
        """Output pins and pin-ring contact in series; pins dominate."""
        return 1.0 / (1.0 / self.k_output_pins + 1.0 / self.k_contact)

    def damping_out(self):
        return (self._structural_damping_ratio * 2.0
                * math.sqrt(self.stiffness_out(0.0) * self._inertia_ref))

    def spring_torque(self, delta):
        """Deadband backlash at the output pins -- no torque until the pins
        take up the clearance."""
        half = self.backlash / 2.0
        if delta > half:
            engaged = delta - half
        elif delta < -half:
            engaged = delta + half
        else:
            return 0.0
        return self.stiffness_out(engaged) * engaged

    # ---- interface: limits -----------------------------------------------

    def tau_max_pin_bending(self):
        f_allow = (math.pi * self.r_out_pin ** 3
                   * self.pin_material.sigma_allow
                   / (4.0 * self.out_pin_length * self.safety_factor))
        return f_allow * self.n_out_pins * self.r_out_pitch

    def tau_max_contact(self):
        f_allow = self._hertz_allowable()
        return f_allow * self.z_pins * self.r_pitch / 4.0

    def tau_max(self):
        return min(self.tau_max_pin_bending(), self.tau_max_contact())

    def stroke_limit(self):
        return None   # continuous rotation

    # ---- interface: inertia ----------------------------------------------

    @property
    def disc_mass(self):
        return (self.material.density * math.pi * self.disc_radius ** 2
                * self.disc_width)

    @property
    def cam_mass(self):
        return (self.material.density * math.pi * self.r_ecc_bearing ** 2
                * 1.5 * self.disc_width)

    def reflected_inertia_extra(self):
        """Disc and cam inertias. The signature term is the disc's orbit: its
        centre circles at N times the output speed, so m_disc e^2 rides N^2.
        It turns out to be small -- the reflected *motor* inertia dominates --
        but it is derived and reported, not assumed away."""
        m_disc = self.disc_mass
        j_disc = 0.5 * m_disc * self.disc_radius ** 2
        j_cam = 0.5 * self.cam_mass * self.r_ecc_bearing ** 2 \
            + self.cam_mass * self.eccentricity ** 2
        m_pin = (self.pin_material.density * math.pi * self.r_out_pin ** 2
                 * self.out_pin_length)
        j_pins = self.n_out_pins * m_pin * self.r_out_pitch ** 2
        return (self.N ** 2 * (m_disc * self.eccentricity ** 2 + j_cam)
                + j_disc + j_pins)

    # ---- reporting -------------------------------------------------------

    def efficiency_at(self, tau_out):
        """Load-proportional loss dominates once moving -- a flat curve like
        any gearbox, but starting from a much lower base than the harmonic's
        preload drag."""
        if abs(tau_out) <= 0.0:
            return 0.0
        loss = self.friction.tau_c0 + self.friction.c * abs(tau_out)
        return abs(tau_out) / (abs(tau_out) + loss)

    def summary(self):
        d = DriveModel.summary(self)
        d.update({
            "z_pins": self.z_pins,
            "r_pitch_mm": self.r_pitch * 1e3,
            "eccentricity_mm": self.eccentricity * 1e3,
            "disc_radius_mm": self.disc_radius * 1e3,
            "k_output_pins": self.k_output_pins,
            "k_contact": self.k_contact,
            "backlash_arcmin": math.degrees(self.backlash) * 60.0,
            "preloaded": self.preloaded,
            "fits": self.fit_ok(),
            "pin_force_at_rating": self.pin_force_max(self.tau_max()),
            "load_loss_coefficient": self._c_total,
            "tau_max_pin_bending": self.tau_max_pin_bending(),
            "tau_max_contact": self.tau_max_contact(),
            "eta_at_tau_max": self.efficiency_at(self.tau_max()),
            "eta_at_10pct": self.efficiency_at(0.1 * self.tau_max()),
        })
        return d
