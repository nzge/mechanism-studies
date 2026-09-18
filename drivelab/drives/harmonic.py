"""Harmonic drive -- derived from geometry, material and tolerances.

Almost nothing here is a datasheet number. Ratio follows from two tooth
counts; stiffness from the flexspline cup as a thin-walled shell; the no-load
drag from the wave-generator preload; the 2-per-revolution ripple from the
cam's runout tolerance; and the load-proportional loss from the friction of
the wave-generator bearing and the sliding of the teeth under the preload.

The closed forms worth knowing before reading the code
--------------------------------------------------------

Ratio. The circular spline has z_c teeth and the flexspline z_f, with
z_c - z_f = 2 (one tooth per wave). Per wave-generator turn the flexspline
falls behind the circular spline by z_c - z_f teeth, so

    N = z_f / (z_c - z_f)      (= z_f / 2 for the two-lobe wave generator)

a 100:1 drive is a 200-tooth flexspline inside a 202-tooth ring. The ratio is
*exact* -- it is a counting argument, not a friction argument.

Stiffness. Torque enters through the toothed rim, crosses the shallow cone,
and winds the thin cup. Each segment is a closed thin-walled tube, so Bredt's
formula applies to every one of them exactly:

    cup:  K = 2 pi G r^3 t / L
    cone: K = 4 pi G t sin(alpha) / (1/a^2 - 1/b^2)      a, b = cone end radii

The segments carry the same torque in series, so K_out = 1 / sum(1 / K_i).
The one compliance this deliberately does not capture is radial flexure of
the toothed rim under load -- a shell problem with no useful closed form.
The notebook compares the derived number against published figures for the
same size class and reports the gap instead of hiding it.

No-load drag. The wave generator is an interference fit inside the flexspline:
it must deflect the rim by w0 to stay engaged, and that preload costs drag.
The radial 2-lobe stiffness of the rim ring is 9 E I / r^4 per unit
circumference (extensionless in-plane bending), so the preload force is

    F_p = 9 pi E I_rim w0 / (2 r^4)        I_rim = b t_rim^3 / 12

and the no-load drag follows from the wave-generator bearing friction.

Ripple. An ideal harmonic drive has zero transmission error -- the ratio is a
counting argument. The 2-cycles-per-input-revolution ripple everyone measures
therefore *enters through manufacturing*: the cam's runout tolerance moves the
two engagement zones, and the first-order angular error is the runout over the
pitch radius. That is derived here, from one tolerance, not typed in.

Torque capacity. The flexspline cup fails in shear where it meets the cone,
so tau_max = 2 pi r_cup^2 t_cup * tau_allow with tau_allow the von Mises
shear of the material allowable divided by the safety factor. The allowable
is a fatigue surrogate -- see `materials.py` -- because real flexsplines die
of crack growth, and this bench has no fatigue model.
"""

import math

from .base import DriveModel
from ..friction import make_friction
from ..materials import IsotropicMaterial, STEEL_GEAR


class HarmonicDrive(DriveModel):
    """A two-lobe (cup-type) harmonic drive, derived from geometry.

    The flexspline geometry is the identity of the drive: pitch radius, cup
    wall, cup length, and the cone that joins rim to cup. The wave generator
    contributes the preload, the drag and the ripple.
    """

    physics_derived = True

    def __init__(self, z_flex, r_pitch, rim_width, cup_wall, cup_length,
                 rim_wall=0.4e-3, w0=0.4e-3, cone_wall=None, cone_length=None,
                 cone_radii=None, z_ring=None, material=STEEL_GEAR,
                 cam_runout=5.0e-6, r_wg_bearing=None, mu_wg=0.0015,
                 mu_tooth=0.05, pressure_angle_deg=20.0, seal_drag=1.0e-3,
                 safety_factor=3.0, structural_damping_ratio=0.02,
                 friction_model="lugre", name="harmonic", inertia_ref=None):
        if z_flex % 2 != 0:
            raise ValueError("flexspline tooth count must be even (two waves)")
        z_ring = int(z_ring) if z_ring is not None else int(z_flex) + 2
        if z_ring - z_flex <= 0:
            raise ValueError("circular spline must have more teeth than the "
                             "flexspline")
        self.z_flex = int(z_flex)
        self.z_ring = int(z_ring)
        self.r_pitch = float(r_pitch)         # flexspline pitch radius [m]
        self.rim_width = float(rim_width)     # tooth face width b [m]
        self.rim_wall = float(rim_wall)       # toothed rim band thickness [m]
        self.w0 = float(w0)                   # radial deflection amplitude [m]
        self.cup_wall = float(cup_wall)       # cup wall thickness t [m]
        self.cup_length = float(cup_length)   # rim-to-hub cup length L [m]
        self.cup_radius = float(r_pitch * 0.73)
        self.cone_wall = (float(cone_wall) if cone_wall is not None
                          else self.cup_wall)
        self.cone_length = (float(cone_length) if cone_length is not None
                            else 0.45 * float(cup_length))
        a, b = (cone_radii if cone_radii is not None
                else (self.cup_radius, float(r_pitch)))
        self.cone_a, self.cone_b = float(a), float(b)
        if not (0.0 < self.cone_a < self.cone_b):
            raise ValueError("cone radii must satisfy 0 < a < b")
        self.material = material
        self.cam_runout = float(cam_runout)   # WG cam runout tolerance [m]
        self.r_wg_bearing = (float(r_wg_bearing) if r_wg_bearing is not None
                             else float(r_pitch) * 0.62)
        self.mu_wg = float(mu_wg)
        self.mu_tooth = float(mu_tooth)
        self.alpha = math.radians(float(pressure_angle_deg))
        self.seal_drag = float(seal_drag)
        self.safety_factor = float(safety_factor)
        self.name = name
        self._structural_damping_ratio = structural_damping_ratio
        self._inertia_ref = float(inertia_ref) if inertia_ref else 1.0e-4

        N = float(self.z_flex) / float(self.z_ring - self.z_flex)
        self.N = N   # needed by _derive_losses; re-set by the base class
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
                stiction_ratio=1.5,    # preloaded wave generator: some stiction
                omega_stribeck=0.02,
                presliding=5.0e-5,
                tau_ref=max(tau_c0, 1e-6),
                inertia_ref=(inertia_ref if inertia_ref else 1.0e-4)))

    # ---- sizing ----------------------------------------------------------

    @classmethod
    def sized_for(cls, tau_required, N, r_pitch, rim_width=8.0e-3,
                  cup_length=None, material=STEEL_GEAR, safety_factor=3.0,
                  **kw):
        """Size a flexspline cup to a torque requirement, the way you would on
        paper: invert the shear limit of the cup for the wall thickness.

        The result is the *thinnest* cup that clears the allowable stress --
        which is also the most compliant. Everything else (stiffness, preload
        drag, ripple) then follows from that one decision, exactly as with
        `CapstanDrive.sized_for`."""
        if N < 2 or N % 1 != 0:
            raise ValueError("N must be a positive integer >= 2")
        z_flex = int(2 * N)   # two-lobe wave generator: z_c - z_f = 2
        tau_allow = 0.577 * material.sigma_allow / safety_factor
        r_cup = 0.73 * r_pitch
        t_cup = tau_required / (2.0 * math.pi * r_cup ** 2 * tau_allow)
        t_cup = max(t_cup, 0.2e-3)
        L = cup_length if cup_length is not None else 0.9 * r_pitch
        return cls(z_flex=z_flex, r_pitch=r_pitch, rim_width=rim_width,
                   cup_wall=t_cup, cup_length=L, material=material,
                   safety_factor=safety_factor, **kw)

    # ---- geometry-derived quantities -------------------------------------

    @property
    def cone_sin_alpha(self):
        """sin of the cone half-angle, from its radii and axial length."""
        return ((self.cone_b - self.cone_a)
                / math.hypot(self.cone_b - self.cone_a, self.cone_length))

    @property
    def k_cup(self):
        """Bredt torsion of the straight cup: 2 pi G r^3 t / L."""
        return (2.0 * math.pi * self.material.G * self.cup_radius ** 3
                * self.cup_wall / self.cup_length)

    @property
    def k_cone(self):
        """Bredt torsion of the conical section: 4 pi G t sin(a) / (1/a^2 -
        1/b^2). Degenerates to the straight-tube formula as a -> b."""
        return (4.0 * math.pi * self.material.G * self.cone_wall
                * self.cone_sin_alpha
                / (1.0 / self.cone_a ** 2 - 1.0 / self.cone_b ** 2))

    @property
    def k_rim_radial(self):
        """Radial 2-lobe stiffness of the toothed rim ring, 9 E I / r^4 --
        extensionless in-plane bending of a thin ring, the mode the wave
        generator excites."""
        i_rim = self.rim_width * self.rim_wall ** 3 / 12.0
        return 9.0 * self.material.E * i_rim / self.r_pitch ** 4

    @property
    def preload_force(self):
        """Radial force the wave generator must apply per lobe to hold the
        flexspline in its elliptical shape: F_p = 9 pi E I w0 / (2 r^4)."""
        return 9.0 * math.pi * self.material.E * (
            self.rim_width * self.rim_wall ** 3 / 12.0) * self.w0 \
            / (2.0 * self.r_pitch ** 4)

    @property
    def ripple_amplitude(self):
        """Peak kinematic error [rad at output] from cam runout -- the runout
        over the pitch radius, to first order."""
        return self.cam_runout / self.r_pitch

    def _derive_losses(self):
        """(eta_f, tau_c0, c_total) -- every loss term derived from geometry.

        No-load: the preloaded wave generator rolls inside the flexspline, two
        lobes, all the time, regardless of load. Output-referred drag:

            tau_c0 = N * (2 mu_wg F_p r_b + seal)

        Load-proportional: the wave-generator bearing carries the mesh force,
        and the teeth slide radially through 4 w0 per input revolution while
        loaded. Both scale with transmitted torque, so they enter as the loss
        coefficient c = 1/eta_f - 1, and the backward efficiency eta_b =
        2 - 1/eta_f follows with no second knob.
        """
        tau_c0 = self.N * (2.0 * self.mu_wg * self.preload_force
                           * self.r_wg_bearing + self.seal_drag)

        # WG bearing under mesh load: bearing carries ~ the mesh force at each
        # lobe; referred to the output the lever arm is r_b * N / r_pitch.
        c_wg = (self.mu_wg * self.r_wg_bearing * self.N / self.r_pitch
                * (1.0 + math.tan(self.alpha)))

        # Tooth sliding: per input revolution each engaged tooth pumps
        # radially through 4 w0 against the tangential mesh force (pressure
        # angle converts). Sliding work per rev / (2 pi) is the loss torque.
        c_tooth = (self.N * self.mu_tooth * 4.0 * self.w0
                   / (2.0 * math.pi * self.r_pitch * math.tan(self.alpha)))

        c_total = c_wg + c_tooth
        eta_f = 1.0 / (1.0 + c_total)
        return eta_f, tau_c0, c_total

    # ---- interface: kinematics -------------------------------------------

    def kinematic_error(self, theta_m_out):
        """2 cycles per input revolution -- the harmonic signature.

        theta_m_out is the motor angle referred to the output, so the input
        angle is N * theta_m_out and the ripple is sin(2 N theta_m_out)."""
        if self.ripple_amplitude == 0.0:
            return 0.0
        return self.ripple_amplitude * math.sin(2.0 * self.N * theta_m_out)

    def d_kinematic_error(self, theta_m_out):
        if self.ripple_amplitude == 0.0:
            return 0.0
        w = 2.0 * self.N
        return self.ripple_amplitude * w * math.cos(w * theta_m_out)

    def error_amplitude(self):
        return self.ripple_amplitude

    # ---- interface: compliance -------------------------------------------

    def stiffness_out(self, delta=0.0):
        """Cup and cone in series (same torque, added windup).

        Constant with load here; real harmonic drives soften near zero torque
        (their hysteresis has a K1 < K3 slope). That near-zero softening is
        deliberately not modelled -- it is a measured hysteresis property, not
        a geometric one -- and its absence is reported as a limitation."""
        return 1.0 / (1.0 / self.k_cup + 1.0 / self.k_cone)

    def damping_out(self):
        # Same reference-inertia convention as every other drive, so the zeta
        # asked for is the zeta reported.
        return (self._structural_damping_ratio * 2.0
                * math.sqrt(self.stiffness_out(0.0) * self._inertia_ref))

    # ---- interface: limits -----------------------------------------------

    def tau_max(self):
        """Cup shear at the cone junction -- the flexspline's classic failure
        site. tau_allow = 0.577 sigma_allow / sf (von Mises)."""
        tau_allow = 0.577 * self.material.sigma_allow / self.safety_factor
        return 2.0 * math.pi * self.cup_radius ** 2 * self.cup_wall * tau_allow

    def stroke_limit(self):
        return None   # continuous rotation

    # ---- interface: inertia ----------------------------------------------

    def reflected_inertia_extra(self):
        """Flexspline (output side) plus wave generator reflected by N^2.

        Thin-shell approximation: each shell of the flexspline is a ring of
        mass at its radius. The wave generator is a solid cam on the input
        side, so its inertia rides N^2."""
        rho = self.material.density
        m_cup = rho * 2.0 * math.pi * self.cup_radius * self.cup_wall \
            * self.cup_length
        s_cone = math.hypot(self.cone_b - self.cone_a, self.cone_length)
        m_cone = rho * math.pi * (self.cone_a + self.cone_b) * s_cone \
            * self.cone_wall
        m_rim = rho * 2.0 * math.pi * self.r_pitch * self.rim_width \
            * 2.0 * self.rim_wall
        j_fs = (m_cup * self.cup_radius ** 2
                + m_cone * ((self.cone_a + self.cone_b) / 2.0) ** 2
                + m_rim * self.r_pitch ** 2)

        # The wave-generator cam is an elliptical plug, not a solid cylinder:
        # solidity factor 0.4 over a width of one rim face, documented rather
        # than hidden because the number is an estimate.
        m_wg = (rho * math.pi * self.r_wg_bearing ** 2
                * self.rim_width * 0.4)
        j_wg = 0.5 * m_wg * (self.r_wg_bearing ** 2 + self.w0 ** 2 / 2.0)
        return j_fs + self.N ** 2 * j_wg

    # ---- reporting -------------------------------------------------------

    def efficiency_at(self, tau_out):
        """Preload-dominated below the mesh load: the no-load drag is large
        (a preloaded wave generator turns constantly), so efficiency climbs
        with load before flattening -- between the capstan's steep curve and
        a plain gearbox's flat one."""
        if abs(tau_out) <= 0.0:
            return 0.0
        loss = self.friction.tau_c0 + self.friction.c * abs(tau_out)
        return abs(tau_out) / (abs(tau_out) + loss)

    def summary(self):
        d = DriveModel.summary(self)
        d.update({
            "z_flex": self.z_flex,
            "z_ring": self.z_ring,
            "r_pitch_mm": self.r_pitch * 1e3,
            "cup_wall_mm": self.cup_wall * 1e3,
            "cup_length_mm": self.cup_length * 1e3,
            "k_cup": self.k_cup,
            "k_cone": self.k_cone,
            "preload_N": self.preload_force,
            "wg_bearing_drag": self.N * 2.0 * self.mu_wg
            * self.preload_force * self.r_wg_bearing,
            "load_loss_coefficient": self._c_total,
            "ripple_arcmin": math.degrees(self.ripple_amplitude) * 60.0,
            "tau_max_shear": self.tau_max(),
            "eta_at_tau_max": self.efficiency_at(self.tau_max()),
            "eta_at_10pct": self.efficiency_at(0.1 * self.tau_max()),
        })
        return d
