#!/usr/bin/env python3
"""Verify that every public WPILib HAL declaration has a VMX status.

The VMX coverage document intentionally uses header defaults plus explicit
symbol overrides.  This keeps the manifest reviewable while still making the
check mechanical: adding a public HAL function without adding a manifest
row (or an override for a mixed header) fails the build.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


STATUSES = {
    "IMPLEMENTED",
    "PARTIAL",
    "MISSING_FEASIBLE",
    "UNSUPPORTED_HARDWARE",
    "NOT_APPLICABLE",
    "HARDWARE_VALIDATION_REQUIRED",
}

# Orthogonal to status: whether the VMX wpiHal defines the header's symbols at
# all. An UNSUPPORTED_HARDWARE symbol still has to resolve if anything links
# against it, so the two axes cannot be collapsed. The cross-compilation gate
# checks these against real object symbol tables; here they are only validated
# structurally, because a text search cannot distinguish a definition from a
# forward declaration.
LINKAGES = {"PROVIDED", "ABSENT"}

ROOT = Path(__file__).resolve().parents[2]
HAL_INCLUDE = ROOT / "hal" / "src" / "main" / "native" / "include" / "hal"
GENERATED_INCLUDE = ROOT / "hal" / "src" / "generated" / "main" / "native" / "include" / "hal"
MANIFEST = ROOT / "hal" / "src" / "main" / "native" / "vmx" / "VMX_HAL_COVERAGE.md"
VMX_SOURCES = (
    ROOT / "hal" / "src" / "main" / "native" / "vmx",
    ROOT / "hal" / "src" / "main" / "native" / "shared",
)

SYMBOL_RE = re.compile(r"\b(HAL_[A-Za-z0-9_]+)\s*\(")
COMMENT_RE = re.compile(r"//[^\n]*|/\*.*?\*/", re.DOTALL)


def strip_comments(text: str) -> str:
    return COMMENT_RE.sub("", text)


def public_headers() -> dict[str, Path]:
    headers: dict[str, Path] = {}
    for root in (HAL_INCLUDE, GENERATED_INCLUDE):
        if not root.exists():
            continue
        for path in root.glob("*.h"):
            # Simulation/type plumbing is not a VMX production HAL contract.
            if path.name in {"ChipObject.h", "Errors.h", "Types.h"}:
                continue
            if path.name.endswith("Types.h"):
                continue
            headers[path.name] = path
    return headers


STATEMENT_KEYWORDS = {"return", "case", "else", "do", "co_return"}

PROTOTYPE_RE = re.compile(
    r"(?:^|\n)[ \t]*((?:const[ \t]+)?[A-Za-z_][\w\s*&]*?)\b"
    r"(HAL_[A-Za-z0-9_]+)[ \t]*\([^;{]*?\)[ \t]*;",
    re.DOTALL,
)


def declared_symbols(path: Path) -> set[str]:
    """Symbols the ABI must carry: out-of-line prototypes and nothing else.

    Matching every `HAL_Foo(` in a header also picks up the inline helpers it
    defines and the typedefs it names, and no backend defines either out of
    line. That inflated the surface -- SimDevice.h counted thirty symbols
    where nine are real -- and would have demanded stub definitions for
    functions whose bodies sit a few lines above the call.
    """
    text = strip_comments(path.read_text(encoding="utf-8"))
    symbols: set[str] = set()
    for match in PROTOTYPE_RE.finditer(text):
        leading, symbol = match.group(1), match.group(2)
        if symbol == "HAL_ENUM" or "inline" in leading:
            continue
        # `return HAL_Foo(a, b);` inside a header's own inline wrapper reads
        # exactly like a prototype otherwise, and would demand an out-of-line
        # definition for a function defined a few lines above.
        if leading.split() and leading.split()[-1] in STATEMENT_KEYWORDS:
            continue
        symbols.add(symbol)
    return symbols


def backend_text() -> str:
    chunks: list[str] = []
    for root in VMX_SOURCES:
        if root.exists():
            chunks.extend(
                path.read_text(encoding="utf-8")
                for path in root.rglob("*.cpp")
            )
    return "\n".join(chunks)


def parse_manifest() -> dict[str, tuple[str, str, dict[str, str]]]:
    rows: dict[str, tuple[str, str, dict[str, str]]] = {}
    for line in MANIFEST.read_text(encoding="utf-8").splitlines():
        if not line.startswith("|") or line.startswith("| ---"):
            continue
        columns = [column.strip() for column in line.strip().strip("|").split("|")]
        if len(columns) < 5 or columns[0] in {"Header", "Status", "Linkage"}:
            continue
        header = columns[0].strip("`")
        default = columns[1].strip("`")
        linkage = columns[2].strip("`")
        override_text = columns[3]
        if default not in STATUSES:
            raise ValueError(f"{header}: invalid default status {default!r}")
        if linkage not in LINKAGES:
            raise ValueError(
                f"{header}: invalid linkage {linkage!r}; expected one of "
                + ", ".join(sorted(LINKAGES))
            )
        if header in rows:
            raise ValueError(f"duplicate manifest row for {header}")
        overrides: dict[str, str] = {}
        if override_text not in {"", "-", "—"}:
            for item in override_text.split(";"):
                item = item.strip()
                if not item:
                    continue
                symbol, separator, status = item.partition("=")
                if not separator or status not in STATUSES:
                    raise ValueError(f"{header}: invalid override {item!r}")
                if symbol in overrides:
                    raise ValueError(f"{header}: duplicate override for {symbol}")
                overrides[symbol] = status
        rows[header] = (default, linkage, overrides)
    return rows


def main() -> int:
    try:
        headers = public_headers()
        manifest = parse_manifest()
        missing_rows = sorted(set(headers) - set(manifest))
        extra_rows = sorted(
            name
            for name in set(manifest) - set(headers)
            # These headers are generated during a normal HAL build. They may
            # not exist yet when this standalone verification task is invoked
            # from a clean checkout.
            if name not in {"FRCUsageReporting.h", "UsageReporting.h"}
        )
        if missing_rows or extra_rows:
            if missing_rows:
                print("HAL coverage: headers without a manifest row:", file=sys.stderr)
                print("  " + " ".join(missing_rows), file=sys.stderr)
            if extra_rows:
                print("HAL coverage: stale manifest rows:", file=sys.stderr)
                print("  " + " ".join(extra_rows), file=sys.stderr)
            return 1

        backend = backend_text()
        counts = {status: 0 for status in sorted(STATUSES)}
        symbol_count = 0
        absent_implemented: list[str] = []
        for header, path in sorted(headers.items()):
            default, _linkage, overrides = manifest[header]
            symbols = declared_symbols(path)
            unknown_overrides = sorted(set(overrides) - symbols)
            if unknown_overrides:
                print(
                    f"HAL coverage: {header} overrides unknown symbols: "
                    + " ".join(unknown_overrides),
                    file=sys.stderr,
                )
                return 1
            for symbol in symbols:
                status = overrides.get(symbol, default)
                counts[status] += 1
                symbol_count += 1
                if status == "IMPLEMENTED" and not re.search(
                    rf"\b{re.escape(symbol)}\s*\(", backend
                ):
                    absent_implemented.append(f"{header}:{symbol}")

        if absent_implemented:
            print(
                "HAL coverage: IMPLEMENTED symbols missing from VMX/shared sources:",
                file=sys.stderr,
            )
            print("  " + " ".join(absent_implemented), file=sys.stderr)
            return 1

        print(
            f"HAL coverage OK: {len(headers)} headers, {symbol_count} symbols; "
            + ", ".join(f"{status}={counts[status]}" for status in sorted(counts))
        )
        return 0
    except (OSError, ValueError) as error:
        print(f"HAL coverage: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
