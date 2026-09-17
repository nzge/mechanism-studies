"""Cable materials for capstan drives.

The field that trips people up is `E_eff`. A wire rope is not a solid bar: the
strands bed in and take up before the metal carries, so the construction's
effective modulus runs roughly half the bulk value. Using 200 GPa for 7x19
steel will overstate drive stiffness by about 2x, which is exactly the quantity
the resonance and bandwidth results hinge on.

`min_bend_ratio` (sheave diameter / cable diameter) does not limit torque -- it
limits *life*. The bench reports when it is violated rather than enforcing it,
because the honest answer is "this configuration works and will fail early",
not "this configuration is impossible".
"""

from ..drives.capstan import CableMaterial

CABLES = {}


def _add(c):
    CABLES[c.name] = c
    return c


# Galvanized/stainless 7x19 -- the default capstan cable. Flexible enough for
# small capstans, well-characterized, cheap.
_add(CableMaterial(
    name="steel-7x19",
    E_eff=100e9,          # Pa, construction effective modulus
    strength_mpa=1770.0,  # on metallic area
    fill_factor=0.58,
    density=7850.0,
    min_bend_ratio=20.0,
))

# 1x19 -- stiffer and stronger, but wants a much larger capstan.
_add(CableMaterial(
    name="steel-1x19",
    E_eff=140e9,
    strength_mpa=1860.0,
    fill_factor=0.72,
    density=7850.0,
    min_bend_ratio=40.0,
))

# Single tungsten wire: the stiffness champion, and the reason some precision
# capstans use it. Brittle, so it needs a large bend radius.
_add(CableMaterial(
    name="tungsten-single",
    E_eff=400e9,
    strength_mpa=3000.0,
    fill_factor=1.0,
    density=19300.0,
    min_bend_ratio=60.0,
))

# UHMWPE (Dyneema-class). Light, kind to small capstans, strong -- but it
# creeps, and creep in a capstan drive is pretension loss, which is torque
# capacity loss. The creep_rate field exists to study that.
_add(CableMaterial(
    name="uhmwpe-sk78",
    E_eff=60e9,
    strength_mpa=2500.0,
    fill_factor=0.65,
    density=970.0,
    min_bend_ratio=8.0,
    creep_rate=0.02,      # fractional tension loss per decade of time
))

# LCP (Vectran-class): most of the UHMWPE benefits with far less creep.
_add(CableMaterial(
    name="lcp-vectran",
    E_eff=70e9,
    strength_mpa=2900.0,
    fill_factor=0.65,
    density=1400.0,
    min_bend_ratio=10.0,
    creep_rate=0.002,
))
