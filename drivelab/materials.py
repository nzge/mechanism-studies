"""Isotropic material properties shared by the geared drives.

The capstan keeps its cable properties in `CableMaterial` because a wire rope
is a *construction*, not a bulk solid. The harmonic and cycloidal drives are
machined from stock, so a plain isotropic solid is the right description.

One field deserves comment: `sigma_allow` is a design allowable, not the
material yield. For the geared drives it stands in for the *fatigue* limit,
which this bench explicitly does not model -- the flexspline of a harmonic
drive fails by crack growth at a stress far below yield. Using a reduced
allowable with a safety factor is an honest surrogate, and it is documented at
every point of use.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class IsotropicMaterial:
    name: str
    E: float            # Young's modulus        [Pa]
    nu: float = 0.30    # Poisson's ratio        [-]
    density: float = 7850.0   # [kg/m^3]
    sigma_allow: float = 250e6  # design allowable [Pa]

    @property
    def G(self):
        """Shear modulus [Pa]."""
        return self.E / (2.0 * (1.0 + self.nu))

    @property
    def E_plane(self):
        """Plane-strain modulus E/(1-nu^2) [Pa] -- the modulus that appears in
        line-contact (Hertzian) formulas."""
        return self.E / (1.0 - self.nu ** 2)

    def __repr__(self):
        return "<IsotropicMaterial %s E=%.0fGPa sigma_allow=%.0fMPa>" % (
            self.name, self.E / 1e9, self.sigma_allow / 1e6)


# Typical values, kept deliberately generic -- swap for a real alloy before
# quoting results about specific hardware (same rule as the rest of the
# catalog).
STEEL_GEAR = IsotropicMaterial(
    name="steel-gear",
    E=200e9,
    nu=0.30,
    density=7850.0,
    sigma_allow=250e6,   # fatigue surrogate for a flexspline / hardened pins
)

STEEL_HARD = IsotropicMaterial(
    name="steel-hardened",
    E=200e9,
    nu=0.30,
    density=7850.0,
    sigma_allow=800e6,   # case-hardened contact surfaces, still a surrogate
)
