"""Catalog of real-world parts, kept strictly separate from the physics.

Nothing in `drivelab` outside this package knows that these exist. The models
are dimensionless at heart; the catalog is the adapter that turns "a 4008
gimbal motor on a 300 mm arm" into numbers the bench can chew on.

A warning about provenance, because it matters for how much weight the
conclusions can carry: these entries are *representative archetypes*, sized
from typical published figures for their class, not transcriptions of any
particular manufacturer's datasheet. They are here so the bench produces
plausibly-scaled results out of the box. Before quoting a result about real
hardware, replace the entry with the actual datasheet values -- every field is
a documented SI quantity, so that is a five-minute job.

    from drivelab.catalog import MOTORS, CABLES, ARMS
    MOTORS["gimbal-4008"]
"""

from .motors import MOTORS
from .cables import CABLES
from .arms import ARMS
from .configs import reference_capstan, REFERENCE_CONFIGS

__all__ = ["MOTORS", "CABLES", "ARMS", "reference_capstan",
           "REFERENCE_CONFIGS"]


def describe(registry, name):
    """Pretty-print one catalog entry."""
    item = registry[name]
    return "\n".join("  %-22s %s" % (k, v)
                     for k, v in sorted(vars(item).items()))
