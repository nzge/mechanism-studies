"""Transmission models, all implementing the `DriveModel` two-port interface.

Provenance matters and is tracked per model:

  physics_derived = True   parameters follow from geometry + material
  physics_derived = False  parameters come from a datasheet

`CapstanDrive` is derived. `GenericGearedDrive` is not -- it is a faithful
*container* for published harmonic/cycloidal figures, which is useful for
comparison but proves nothing about mechanism. Dedicated derived models for
those two are the next things to write; the interface is deliberately small so
that adding one means implementing five methods and nothing else.
"""

from .base import DriveModel, SLIP, BRISTLE, N_EXTRA
from .capstan import CapstanDrive, CableMaterial
from .ideal import RigidIdealDrive, CompliantIdealDrive, GenericGearedDrive

__all__ = [
    "DriveModel", "SLIP", "BRISTLE", "N_EXTRA",
    "CapstanDrive", "CableMaterial",
    "RigidIdealDrive", "CompliantIdealDrive", "GenericGearedDrive",
]
