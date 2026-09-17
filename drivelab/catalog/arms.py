"""Standard lever arms -- the test loads.

The arm is the reference for the entire bench: every characteristic scale is
taken from it, so choosing an arm is choosing the units the comparison is
reported in. Keep this list short and stable. A comparison is only meaningful
between drives tested on the *same* arm.
"""

from ..plant import LeverArm

ARMS = {}


def _add(a):
    ARMS[a.name] = a
    return a


# Desktop scale: the regime where a capstan is genuinely competitive.
_add(LeverArm(
    name="desktop-300",
    length=0.30,          # m
    payload_mass=0.5,     # kg at the tip
    link_mass=0.15,
    damping=1.0e-3,       # N*m/(rad/s)
))

# Same arm, unloaded -- isolates the drive from the payload.
_add(LeverArm(
    name="desktop-300-bare",
    length=0.30,
    payload_mass=0.0,
    link_mass=0.15,
    damping=1.0e-3,
))

# Light and fast: low inertia, so reflected motor inertia dominates and the
# ratio penalty is at its most visible.
_add(LeverArm(
    name="light-200",
    length=0.20,
    payload_mass=0.1,
    link_mass=0.05,
    damping=5.0e-4,
))

# Collaborative-arm scale: harmonic-drive territory.
_add(LeverArm(
    name="cobot-500",
    length=0.50,
    payload_mass=3.0,
    link_mass=1.2,
    damping=1.0e-2,
))
