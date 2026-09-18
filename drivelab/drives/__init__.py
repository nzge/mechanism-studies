"""Transmission models, all implementing the `DriveModel` two-port interface.

Provenance matters and is tracked per model:

  physics_derived = True   parameters follow from geometry + material
  physics_derived = False  parameters come from a datasheet

`CapstanDrive`, `HarmonicDrive` and `CycloidalDrive` are derived -- their
parameters follow from geometry and material, with the one place each of them
leans on a tolerance (rather than pure geometry) documented in the module
docstring. `GenericGearedDrive` is not derived: it is a faithful *container*
for published figures, useful for comparison but proving nothing about
mechanism. The interface is deliberately small: adding a drive means
implementing five methods and nothing else.
"""

from .base import DriveModel, SLIP, BRISTLE, N_EXTRA
from .capstan import CapstanDrive, CableMaterial
from .harmonic import HarmonicDrive
from .cycloidal import CycloidalDrive
from .ideal import RigidIdealDrive, CompliantIdealDrive, GenericGearedDrive

__all__ = [
    "DriveModel", "SLIP", "BRISTLE", "N_EXTRA",
    "CapstanDrive", "CableMaterial",
    "HarmonicDrive", "CycloidalDrive",
    "RigidIdealDrive", "CompliantIdealDrive", "GenericGearedDrive",
]
