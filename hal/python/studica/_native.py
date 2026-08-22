"""ctypes adapters for the versioned Studica VMX C ABI."""

from __future__ import annotations

import ctypes
import ctypes.util
import os
from pathlib import Path


class NativeUnavailable(RuntimeError):
    """Raised when the VMX native HAL library is not installed."""


def _load_library() -> ctypes.CDLL | None:
    candidates = []
    configured = os.environ.get("STUDICA_NATIVE_LIBRARY")
    if configured:
        candidates.append(configured)
    candidates += ["wpiHal", "libwpiHal.so", "wpiHalJNI", "libwpiHalJNI.so"]
    for candidate in candidates:
        try:
            return ctypes.CDLL(candidate)
        except OSError:
            continue
    for name in ("wpiHal", "wpiHalJNI"):
        found = ctypes.util.find_library(name)
        if found:
            try:
                return ctypes.CDLL(found)
            except OSError:
                pass
    return None


_LIB = _load_library()


def _function(name: str):
    if _LIB is None:
        raise NativeUnavailable(
            "Studica native library is unavailable; set STUDICA_NATIVE_LIBRARY"
        )
    try:
        return getattr(_LIB, name)
    except AttributeError as exc:
        raise NativeUnavailable(f"native ABI symbol {name} is unavailable") from exc


def _status(code: int, operation: str) -> None:
    if code != 0:
        raise RuntimeError(f"{operation} failed with native status {code}")


class _Snapshot(ctypes.Structure):
    _fields_ = [
        ("structSize", ctypes.c_uint32),
        ("abiVersion", ctypes.c_uint32),
        ("yaw", ctypes.c_double),
        ("pitch", ctypes.c_double),
        ("roll", ctypes.c_double),
        ("accumulatedAngle", ctypes.c_double),
        ("yawRate", ctypes.c_double),
        ("quaternionW", ctypes.c_double),
        ("quaternionX", ctypes.c_double),
        ("quaternionY", ctypes.c_double),
        ("quaternionZ", ctypes.c_double),
        ("rawGyroX", ctypes.c_double),
        ("rawGyroY", ctypes.c_double),
        ("rawGyroZ", ctypes.c_double),
        ("rawAccelX", ctypes.c_double),
        ("rawAccelY", ctypes.c_double),
        ("rawAccelZ", ctypes.c_double),
        ("rawMagX", ctypes.c_double),
        ("rawMagY", ctypes.c_double),
        ("rawMagZ", ctypes.c_double),
        ("worldLinearAccelX", ctypes.c_double),
        ("worldLinearAccelY", ctypes.c_double),
        ("worldLinearAccelZ", ctypes.c_double),
        ("compassHeading", ctypes.c_double),
        ("fusedHeading", ctypes.c_double),
        ("temperatureC", ctypes.c_double),
        ("pressure", ctypes.c_double),
        ("altitude", ctypes.c_double),
        ("sensorTimestamp", ctypes.c_int64),
        ("moving", ctypes.c_uint8),
        ("rotating", ctypes.c_uint8),
        ("calibrating", ctypes.c_uint8),
        ("connected", ctypes.c_uint8),
        ("altitudeValid", ctypes.c_uint8),
        ("reserved", ctypes.c_uint8 * 3),
        ("firmwareVersion", ctypes.c_char * 96),
    ]


class VMXIMU:
    """Shared-runtime VMX onboard IMU vendor view."""

    def __init__(self):
        self._handle = ctypes.c_uint32()
        fn = _function("StudicaVMXIMU_Create")
        fn.argtypes = [ctypes.POINTER(ctypes.c_uint32)]
        fn.restype = ctypes.c_int32
        _status(fn(ctypes.byref(self._handle)), "VMXIMU_Create")

    def close(self):
        if self._handle.value:
            fn = _function("StudicaVMXIMU_Destroy")
            fn.argtypes = [ctypes.c_uint32]
            fn.restype = None
            fn(self._handle)
            self._handle.value = 0

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def _read(self):
        if not self._handle.value:
            raise RuntimeError("VMXIMU is closed")
        snapshot = _Snapshot()
        snapshot.structSize = ctypes.sizeof(snapshot)
        snapshot.abiVersion = 1
        fn = _function("StudicaVMXIMU_ReadSnapshot")
        fn.argtypes = [ctypes.c_uint32, ctypes.POINTER(_Snapshot)]
        fn.restype = ctypes.c_int32
        _status(fn(self._handle, ctypes.byref(snapshot)), "VMXIMU_ReadSnapshot")
        return snapshot

    def snapshot(self) -> dict[str, object]:
        value = self._read()
        return {name: getattr(value, name) for name, _ in value._fields_}

    def __getattr__(self, name):
        if name.startswith("get_"):
            field = name[4:]
            if field in {field_name for field_name, _ in _Snapshot._fields_}:
                return lambda: self.snapshot()[field]
        raise AttributeError(name)

    def zero_yaw(self):
        self._command("StudicaVMXIMU_ZeroYaw")

    def reset(self):
        self._command("StudicaVMXIMU_Reset")

    def _command(self, symbol):
        if not self._handle.value:
            raise RuntimeError("VMXIMU is closed")
        fn = _function(symbol)
        fn.argtypes = [ctypes.c_uint32]
        fn.restype = ctypes.c_int32
        _status(fn(self._handle), symbol)


class _Handle:
    _destroy_symbol = ""

    def __init__(self):
        self._handle = ctypes.c_uint32(0)

    def close(self):
        if self._handle.value:
            fn = _function(self._destroy_symbol)
            fn.argtypes = [ctypes.c_uint32]
            fn.restype = None
            fn(self._handle)
            self._handle.value = 0

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def __del__(self):
        try:
            self.close()
        except (NativeUnavailable, OSError):
            pass


class TitanQuad(_Handle):
    _destroy_symbol = "StudicaTitan_Destroy"

    def __init__(self, can_id=42, motor_port=0, motor_frequency_hz=15600, distance_per_tick=0.0):
        super().__init__()
        fn = _function("StudicaTitan_Create")
        fn.argtypes = [ctypes.c_uint8, ctypes.c_uint8, ctypes.c_uint16, ctypes.c_double, ctypes.POINTER(ctypes.c_uint32)]
        fn.restype = ctypes.c_int32
        _status(fn(can_id, motor_port, motor_frequency_hz, distance_per_tick, ctypes.byref(self._handle)), "Titan_Create")

    def set(self, speed):
        self._command("StudicaTitan_Set", ctypes.c_double(speed))

    def enable(self):
        self._command("StudicaTitan_Enable")

    def disable(self):
        self._command("StudicaTitan_Disable")

    def stop(self):
        self._command("StudicaTitan_StopMotor")

    def _command(self, symbol, *args):
        fn = _function(symbol)
        fn.argtypes = [ctypes.c_uint32] + [type(arg) for arg in args]
        fn.restype = ctypes.c_int32
        _status(fn(self._handle, *args), symbol)


class Cobra(_Handle):
    _destroy_symbol = "StudicaCobra_Destroy"

    def __init__(self, reference_voltage=5.0):
        super().__init__()
        fn = _function("StudicaCobra_Create")
        fn.argtypes = [ctypes.c_double, ctypes.POINTER(ctypes.c_uint32)]
        fn.restype = ctypes.c_int32
        _status(fn(reference_voltage, ctypes.byref(self._handle)), "Cobra_Create")

    def raw(self, channel):
        out = ctypes.c_int32()
        fn = _function("StudicaCobra_GetRaw")
        fn.argtypes = [ctypes.c_uint32, ctypes.c_uint8, ctypes.POINTER(ctypes.c_int32)]
        fn.restype = ctypes.c_int32
        _status(fn(self._handle, channel, ctypes.byref(out)), "Cobra_GetRaw")
        return out.value

    def voltage(self, channel):
        out = ctypes.c_double()
        fn = _function("StudicaCobra_GetVoltage")
        fn.argtypes = [ctypes.c_uint32, ctypes.c_uint8, ctypes.POINTER(ctypes.c_double)]
        fn.restype = ctypes.c_int32
        _status(fn(self._handle, channel, ctypes.byref(out)), "Cobra_GetVoltage")
        return out.value


class Parsec(_Handle):
    _destroy_symbol = "StudicaParsec_Destroy"

    def __init__(self, can_id=None, usb_path=None):
        super().__init__()
        if (can_id is None) == (usb_path is None):
            raise ValueError("provide exactly one of can_id or usb_path")
        symbol = "StudicaParsec_CreateCAN" if can_id is not None else "StudicaParsec_CreateUSB"
        fn = _function(symbol)
        fn.restype = ctypes.c_int32
        if can_id is not None:
            fn.argtypes = [ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint32)]
            result = fn(can_id, ctypes.byref(self._handle))
        else:
            fn.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_uint32)]
            result = fn(os.fsencode(Path(usb_path)), ctypes.byref(self._handle))
        _status(result, symbol)


class Colore(_Handle):
    _destroy_symbol = "StudicaColore_Destroy"

    def __init__(self, can_id=None, usb_path=None):
        super().__init__()
        if (can_id is None) == (usb_path is None):
            raise ValueError("provide exactly one of can_id or usb_path")
        symbol = "StudicaColore_CreateCAN" if can_id is not None else "StudicaColore_CreateUSB"
        fn = _function(symbol)
        fn.restype = ctypes.c_int32
        if can_id is not None:
            fn.argtypes = [ctypes.c_uint8, ctypes.POINTER(ctypes.c_uint32)]
            result = fn(can_id, ctypes.byref(self._handle))
        else:
            fn.argtypes = [ctypes.c_char_p, ctypes.POINTER(ctypes.c_uint32)]
            result = fn(os.fsencode(Path(usb_path)), ctypes.byref(self._handle))
        _status(result, symbol)


class LightTower(_Handle):
    _destroy_symbol = "StudicaLightTower_Destroy"

    def __init__(self, continuous, red, green, yellow, buzzer):
        super().__init__()
        fn = _function("StudicaLightTower_Create")
        fn.argtypes = [
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.c_uint8,
            ctypes.POINTER(ctypes.c_uint32),
        ]
        fn.restype = ctypes.c_int32
        _status(
            fn(continuous, red, green, yellow, buzzer, ctypes.byref(self._handle)),
            "LightTower_Create",
        )

    def _output(self, name, enabled):
        fn = _function(name)
        fn.argtypes = [ctypes.c_uint32, ctypes.c_uint8]
        fn.restype = ctypes.c_int32
        _status(fn(self._handle, bool(enabled)), name)

    def set_red(self, enabled):
        self._output("StudicaLightTower_SetRed", enabled)

    def set_yellow(self, enabled):
        self._output("StudicaLightTower_SetYellow", enabled)

    def set_green(self, enabled):
        self._output("StudicaLightTower_SetGreen", enabled)

    def set_buzzer(self, enabled):
        self._output("StudicaLightTower_SetBuzzer", enabled)

    def solid(self):
        self._command("StudicaLightTower_SetSolid")

    def blink(self):
        self._command("StudicaLightTower_SetBlink")

    def off(self):
        self._command("StudicaLightTower_Off")

    def _command(self, name):
        fn = _function(name)
        fn.argtypes = [ctypes.c_uint32]
        fn.restype = ctypes.c_int32
        _status(fn(self._handle), name)
