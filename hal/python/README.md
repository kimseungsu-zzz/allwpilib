# Studica VMX Python ABI package

`studica` is a thin `ctypes` binding over the fixed-width
`StudicaVMXIMU_*`, `StudicaTitan_*`, `StudicaCobra_*`, `StudicaParsec_*`, and
`StudicaColore_*` C ABI exported by the same native VMX runtime used by C++ and
Java. It contains no second hardware implementation. Set
`STUDICA_NATIVE_LIBRARY` to the staged AArch64 `wpiHal` library when it is not
on the dynamic loader path.

```python
from studica import VMXIMU, TitanQuad, Cobra, Parsec, Colore, LightTower

with VMXIMU() as imu:
    print(imu.snapshot()["yaw"])
```

Construction errors become `RuntimeError` with the native status code;
invalid constructor arguments raise `ValueError`; `close()` is idempotent and
all wrappers support context-manager use. The package is host-importable
without a native library so RobotPy packaging can select the target-matched
library at deployment time.
