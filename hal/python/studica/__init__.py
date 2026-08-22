"""Studica VMX vendor bindings over the stable native C ABI.

This package intentionally contains no hardware implementation.  It loads the
same ``wpiHal``/vendor native library used by the Java and C++ bindings and
keeps handles lifetime-safe and close-idempotent for RobotPy applications.
Set ``STUDICA_NATIVE_LIBRARY`` when the native library is not on the system
loader path.
"""

from ._native import Cobra, Colore, LightTower, Parsec, TitanQuad, VMXIMU

__all__ = ["VMXIMU", "TitanQuad", "Cobra", "Parsec", "Colore", "LightTower"]
