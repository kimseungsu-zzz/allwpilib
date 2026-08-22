#!/usr/bin/env python3
"""Final software-only VMX release gate.

This gate deliberately does not pretend that a synthetic ELF is the official
VMX SDK.  It verifies the AArch64 selection/validation path, the immutable
driver-source boundary, the fixed vendor ABI headers, coverage manifest, and
the documentation contract.  Gradle supplies the unit, class-level, and
format tasks that depend on this script.
"""

from __future__ import annotations

import ast
import compileall
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
STUDICA_INCLUDE = ROOT / "hal" / "src" / "main" / "native" / "include" / "studica"
PYTHON_NATIVE = ROOT / "hal" / "python" / "studica" / "_native.py"

# ctypes spellings mapped onto the fixed-width C spellings the ABI headers use.
CTYPES_TO_C = {
    "c_uint8": "uint8_t",
    "c_uint16": "uint16_t",
    "c_uint32": "uint32_t",
    "c_uint64": "uint64_t",
    "c_int8": "int8_t",
    "c_int16": "int16_t",
    "c_int32": "int32_t",
    "c_int64": "int64_t",
    "c_double": "double",
    "c_float": "float",
    "c_char": "char",
    "c_char_p": "char*",
    "c_void_p": "void*",
}

# ctypes.Structure subclasses in _native.py and the C struct each one mirrors.
PY_STRUCT_TO_C = {"_Snapshot": "StudicaVMXIMUSnapshot"}

SYMBOL_PATTERN = re.compile(r"^Studica[A-Za-z0-9]+_[A-Za-z0-9]+$")


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


def _normalize_c_type(text: str, typedefs: dict[str, str]) -> str:
    text = re.sub(r"\bconst\b", "", text)
    text = re.sub(r"\s*\*\s*", "*", text)
    text = re.sub(r"\s+", " ", text).strip()
    base = text.rstrip("*")
    stars = text[len(base):]
    return typedefs.get(base.strip(), base.strip()) + stars


def _split_params(text: str) -> list[str]:
    parts = [p.strip() for p in text.split(",")]
    return [p for p in parts if p]


def parse_c_abi() -> tuple[dict[str, tuple[str, list[str]]], dict[str, list[tuple[str, str]]]]:
    """Read the fixed C ABI: free functions and versioned snapshot structs."""
    typedefs: dict[str, str] = {}
    macros: dict[str, int] = {}
    functions: dict[str, tuple[str, list[str]]] = {}
    structs: dict[str, list[tuple[str, str]]] = {}

    texts = {p.name: p.read_text(encoding="utf-8") for p in sorted(STUDICA_INCLUDE.glob("*.h"))}

    for text in texts.values():
        for name, value in re.findall(r"#define\s+(STUDICA_[A-Z0-9_]+)\s+(\d+)u?\b", text):
            macros[name] = int(value)
        for underlying, alias in re.findall(
            r"typedef\s+(uint\d+_t|int\d+_t)\s+(Studica\w+)\s*;", text
        ):
            typedefs[alias] = underlying

    for filename, text in texts.items():
        for ret, name, params in re.findall(
            r"\n\s*(int32_t|void)\s+(Studica\w+)\s*\(([^;]*?)\)\s*;", text, re.DOTALL
        ):
            entries = _split_params(params)
            if entries == ["void"]:
                entries = []
            types = []
            for entry in entries:
                match = re.match(r"^(.*?)(\w+)$", entry.strip(), re.DOTALL)
                if not match:
                    raise RuntimeError(f"{filename}: cannot parse parameter {entry!r}")
                types.append(_normalize_c_type(match.group(1), typedefs))
            functions[name] = (ret, types)

        for body, name in re.findall(
            r"typedef\s+struct\s+\w+\s*\{(.*?)\}\s*(Studica\w+)\s*;", text, re.DOTALL
        ):
            fields: list[tuple[str, str]] = []
            for line in body.split(";"):
                line = re.sub(r"//.*", "", line).strip()
                if not line:
                    continue
                match = re.match(r"^(.*?)(\w+)\s*(?:\[\s*(\w+)\s*\])?$", line, re.DOTALL)
                if not match:
                    raise RuntimeError(f"{filename}: cannot parse field {line!r}")
                ctype = _normalize_c_type(match.group(1), typedefs)
                extent = match.group(3)
                if extent is not None:
                    count = macros.get(extent, extent)
                    ctype = f"{ctype}[{count}]"
                fields.append((match.group(2), ctype))
            structs[name] = fields

    return functions, structs


def _ctype_from_ast(node: ast.expr) -> str | None:
    """Render a ctypes annotation as the C spelling it claims to represent."""
    if isinstance(node, ast.Constant) and node.value is None:
        return "void"
    if isinstance(node, ast.Attribute) and node.attr in CTYPES_TO_C:
        return CTYPES_TO_C[node.attr]
    if isinstance(node, ast.Name) and node.id in PY_STRUCT_TO_C:
        return PY_STRUCT_TO_C[node.id]
    if isinstance(node, ast.Call):
        func = node.func
        if isinstance(func, ast.Attribute) and func.attr == "POINTER" and node.args:
            inner = _ctype_from_ast(node.args[0])
            return None if inner is None else f"{inner}*"
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Mult):
        inner = _ctype_from_ast(node.left)
        if inner is not None and isinstance(node.right, ast.Constant):
            return f"{inner}[{node.right.value}]"
    return None


def _collect_bound_signatures(tree: ast.AST) -> dict[str, dict[str, list[str] | str]]:
    """Pair `fn = _function("Name")` with the argtypes/restype set on `fn`."""
    bound: dict[str, dict[str, list[str] | str]] = {}
    for scope in ast.walk(tree):
        body = getattr(scope, "body", None)
        if not isinstance(body, list):
            continue
        current: dict[str, str] = {}
        for statement in body:
            if not isinstance(statement, ast.Assign) or len(statement.targets) != 1:
                continue
            target = statement.targets[0]
            value = statement.value
            if (
                isinstance(target, ast.Name)
                and isinstance(value, ast.Call)
                and isinstance(value.func, ast.Name)
                and value.func.id == "_function"
                and value.args
                and isinstance(value.args[0], ast.Constant)
                and isinstance(value.args[0].value, str)
            ):
                current[target.id] = value.args[0].value
                continue
            if (
                isinstance(target, ast.Attribute)
                and isinstance(target.value, ast.Name)
                and target.value.id in current
            ):
                symbol = current[target.value.id]
                record = bound.setdefault(symbol, {})
                if target.attr == "argtypes" and isinstance(value, ast.List):
                    rendered = [_ctype_from_ast(element) for element in value.elts]
                    if all(item is not None for item in rendered):
                        record["argtypes"] = rendered
                elif target.attr == "restype":
                    rendered = _ctype_from_ast(value)
                    if rendered is not None:
                        record["restype"] = rendered
    return bound


def check_python_abi_contract() -> None:
    """The ctypes layer must track the fixed C ABI, not merely byte-compile."""
    functions, structs = parse_c_abi()
    tree = ast.parse(PYTHON_NATIVE.read_text(encoding="utf-8"))

    referenced = {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant)
        and isinstance(node.value, str)
        and SYMBOL_PATTERN.match(node.value)
    }
    missing = sorted(referenced - functions.keys())
    if missing:
        raise RuntimeError(
            "ctypes layer binds symbols absent from the C ABI headers: "
            + ", ".join(missing)
        )

    for symbol, record in sorted(_collect_bound_signatures(tree).items()):
        expected_ret, expected_args = functions[symbol]
        actual_args = record.get("argtypes")
        if actual_args is not None and actual_args != expected_args:
            raise RuntimeError(
                f"{symbol} argtypes {actual_args} do not match C ABI {expected_args}"
            )
        actual_ret = record.get("restype")
        if actual_ret is not None and actual_ret != expected_ret:
            raise RuntimeError(
                f"{symbol} restype {actual_ret!r} does not match C ABI {expected_ret!r}"
            )

    for python_name, c_name in PY_STRUCT_TO_C.items():
        node = next(
            (
                item
                for item in ast.walk(tree)
                if isinstance(item, ast.ClassDef) and item.name == python_name
            ),
            None,
        )
        if node is None:
            raise RuntimeError(f"{python_name} is missing from the ctypes layer")
        fields_assign = next(
            (
                statement
                for statement in node.body
                if isinstance(statement, ast.Assign)
                and any(
                    isinstance(target, ast.Name) and target.id == "_fields_"
                    for target in statement.targets
                )
            ),
            None,
        )
        if fields_assign is None or not isinstance(fields_assign.value, ast.List):
            raise RuntimeError(f"{python_name} has no literal _fields_ list")
        actual = []
        for element in fields_assign.value.elts:
            if not isinstance(element, ast.Tuple) or len(element.elts) != 2:
                raise RuntimeError(f"{python_name} has a non-literal _fields_ entry")
            name_node, type_node = element.elts
            actual.append((name_node.value, _ctype_from_ast(type_node)))
        if actual != structs[c_name]:
            differences = [
                f"{index}: python={got} c={want}"
                for index, (got, want) in enumerate(zip(actual, structs[c_name]))
                if got != want
            ]
            raise RuntimeError(
                f"{python_name} does not mirror {c_name}: "
                + ("; ".join(differences) or "field count differs")
            )

    print(
        f"Python ctypes/C ABI signature contract: PASS "
        f"({len(referenced)} symbols, {len(PY_STRUCT_TO_C)} snapshot structs)"
    )


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
    check_python_abi_contract()
    check_synthetic_sdk()
    check_docs()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
