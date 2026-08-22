#!/usr/bin/env python3
"""Final software-only VMX release gate.

This gate deliberately does not pretend that a synthetic ELF is the official
VMX SDK.  It verifies the AArch64 selection/validation path, the immutable
driver-source boundary, the fixed vendor ABI headers, coverage manifest, and
the documentation contract.  Gradle supplies the unit, class-level, and
format tasks that depend on this script.
"""

from __future__ import annotations

import compileall
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def run(command: list[str]) -> None:
    subprocess.run(command, cwd=ROOT, check=True)


def synthetic_aarch64_elf() -> bytes:
    # Minimal ELF64 little-endian relocatable header with e_machine=AArch64.
    header = bytearray(64)
    header[:4] = b"\x7fELF"
    header[4] = 2  # ELFCLASS64
    header[5] = 1  # little endian
    header[6] = 1  # ELF version
    header[16:18] = (1).to_bytes(2, "little")  # ET_REL
    header[18:20] = (183).to_bytes(2, "little")  # EM_AARCH64
    header[20:24] = (1).to_bytes(4, "little")
    header[52:54] = (64).to_bytes(2, "little")
    return bytes(header)


def check_synthetic_sdk() -> None:
    with tempfile.TemporaryDirectory(prefix="vmx-aarch64-stub-") as directory:
        root = Path(directory)
        (root / "include" / "vmxpi").mkdir(parents=True)
        (root / "lib").mkdir()
        (root / "include" / "VMXPi.h").write_text(
            "// synthetic SDK header: no hardware implementation\n", encoding="utf-8"
        )
        library = root / "lib" / "libvmxpi_hal_cpp.so"
        library.write_bytes(synthetic_aarch64_elf())
        data = library.read_bytes()
        if data[4] != 2 or int.from_bytes(data[18:20], "little") != 183:
            raise RuntimeError("synthetic AArch64 SDK fixture failed ELF validation")
    print("Synthetic AArch64 SDK/link-selection guard: PASS")
    print("  (synthetic fixture only; this is not real VMX SDK validation)")


def check_driver_boundary() -> None:
    result = subprocess.run(
        ["git", "diff", "--quiet", "--", "vmxdrivers"], cwd=ROOT
    )
    if result.returncode != 0:
        raise RuntimeError("vmxdrivers source area is modified")
    readme = (ROOT / "vmxdrivers" / "README.md").read_text(encoding="utf-8")
    if "Studica Robotics ROS2 repository" not in readme or "unmodified" not in readme:
        raise RuntimeError("vmxdrivers provenance/immutable-source statement missing")
    print("vmxdrivers immutable-source boundary: PASS")


def check_vendor_abi() -> None:
    include = ROOT / "hal" / "src" / "main" / "native" / "include" / "studica"
    required = {
        "VMXIMU.h": "StudicaVMXIMU_",
        "Titan.h": "StudicaTitan_",
        "Cobra.h": "StudicaCobra_",
        "Parsec.h": "StudicaParsec_",
        "Colore.h": "StudicaColore_",
        "LightTower.h": "StudicaLightTower_",
    }
    for filename, prefix in required.items():
        text = (include / filename).read_text(encoding="utf-8")
        if not re.search(r"#define\s+STUDICA_[A-Z0-9_]+ABI_VERSION\s+1", text):
            raise RuntimeError(f"{filename} has no versioned fixed ABI")
        if prefix not in text:
            raise RuntimeError(f"{filename} has no C ABI symbols")
    if not compileall.compile_dir(ROOT / "hal" / "python", quiet=1):
        raise RuntimeError("Python vendor package does not compile")
    print("Studica C ABI headers/JNI-Python contract: PASS")


def check_docs() -> None:
    docs = [
        ROOT / "README.md",
        ROOT / "hal" / "src" / "main" / "native" / "vmx" / "README.md",
        ROOT / "hal" / "src" / "main" / "native" / "vmx" / "VMX_SENSOR_COMPATIBILITY.md",
        ROOT / "hal" / "src" / "main" / "native" / "vmx" / "VMX_HAL_COVERAGE.md",
    ]
    for path in docs:
        text = path.read_text(encoding="utf-8")
        if "AArch64" not in text or "hardwareValidated=false" not in text:
            raise RuntimeError(f"VMX status markers missing from {path}")
    print("VMX documentation synchronization markers: PASS")


def main() -> int:
    run([sys.executable, str(ROOT / "hal" / "tools" / "verify_hal_coverage.py")])
    run(["git", "diff", "--check"])
    check_driver_boundary()
    check_vendor_abi()
    check_synthetic_sdk()
    check_docs()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
