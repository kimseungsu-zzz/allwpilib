#!/usr/bin/env python3
"""AArch64 cross-compilation gate for the VMX HAL backend.

Nothing in normal CI compiles hal/src/main/native/vmx.  The Gradle VMX source
set is opt-in (-PvmxBuild) and Bazel selects the sim backend off the roboRIO,
so roughly ten thousand lines of backend translation units never reach a
compiler until somebody builds for real hardware.  This gate closes that hole
by compiling every VMX and shared translation unit with the cross compiler,
then reading the resulting object symbol tables for references that nothing
defines.  A real link needs the vendor library, which is not redistributable,
but the symbol tables catch the same class of error.

It deliberately does not stub the VMX SDK.  A stub synthesised from our own
call sites would only prove that we agree with ourselves; it cannot catch a
method that does not exist or takes different arguments, which is exactly the
breakage a first hardware build hits.  Instead it prefers an installed SDK,
which is what a real build links against, and otherwise uses the genuine
MIT-licensed vendor headers vendored in thirdparty/vmxpi.  Only if neither is
usable does it fall back to the subset of translation units that never reach
VMXPi.h, and it reports PARTIAL rather than pretending to pass.  Pass
--require to make anything short of full coverage a failure.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

SOURCE_DIRS = [
    ROOT / "hal" / "src" / "main" / "native" / "vmx",
    ROOT / "hal" / "src" / "main" / "native" / "shared",
]

# Mirrors the vmxCpp exportedHeaders block in hal/build.gradle.
INCLUDE_DIRS = [
    ROOT / "hal" / "src" / "main" / "native" / "include",
    ROOT / "hal" / "src" / "generated" / "main" / "native" / "include",
    ROOT / "hal" / "src" / "main" / "native" / "shared",
    ROOT / "vmxdrivers",
    ROOT / "wpiutil" / "src" / "main" / "native" / "include",
    ROOT / "wpiutil" / "src" / "main" / "native" / "thirdparty" / "fmtlib" / "include",
    ROOT / "wpiutil" / "src" / "main" / "native" / "thirdparty" / "llvm" / "include",
    ROOT / "wpiutil" / "src" / "main" / "native" / "thirdparty" / "sigslot" / "include",
]

# Compiled only so their symbols resolve; not themselves gated.
RESOLUTION_DIRS = [
    ROOT / "hal" / "src" / "main" / "native" / "cpp",
    ROOT / "hal" / "src" / "main" / "native" / "cpp" / "handles",
]

DEFAULT_STANDARD = "c++20"
AARCH64_TRIPLE = "aarch64-linux-gnu"
VENDORED_SDK = ROOT / "thirdparty" / "vmxpi" / "include"


class Unavailable(Exception):
    """A prerequisite the host does not provide."""


def resolve_compiler() -> list[str]:
    """Return the cross-compiler command, honouring VMX_CROSS_CXX first."""
    configured = os.environ.get("VMX_CROSS_CXX")
    if configured:
        parts = configured.split()
        if not shutil.which(parts[0]):
            raise Unavailable(f"VMX_CROSS_CXX={configured!r} is not executable")
        return parts

    gnu = f"{AARCH64_TRIPLE}-g++"
    if shutil.which(gnu):
        return [gnu]

    # Cross containers spell the prefix after their own distribution, for
    # example aarch64-bookworm-linux-gnu-g++, so discover it rather than
    # pinning a name that changes with the image.
    for entry in os.environ.get("PATH", "").split(os.pathsep):
        if not entry:
            continue
        try:
            matches = sorted(Path(entry).glob("aarch64-*-linux-gnu-g++*"))
        except OSError:
            continue
        for match in matches:
            if match.is_file() and os.access(match, os.X_OK):
                return [str(match)]

    # clang is a cross compiler by construction; it only needs the triple.
    for candidate in ("clang++", "clang"):
        if shutil.which(candidate):
            return [candidate, f"--target={AARCH64_TRIPLE}"]

    raise Unavailable(
        f"no AArch64 C++ cross compiler found (looked for $VMX_CROSS_CXX, "
        f"{gnu}, aarch64-*-linux-gnu-g++ on PATH, clang++)"
    )


def probe_toolchain(compiler: list[str]) -> None:
    """Reject a compiler that cannot actually target Linux AArch64.

    Finding a compiler on PATH does not mean it can build for this target.
    A Windows host, for instance, commonly ships clang++, which accepts
    --target=aarch64-linux-gnu and then fails on the first POSIX header
    because it has no Linux sysroot. That is an unusable toolchain, not a
    broken backend, so it must degrade to SKIP rather than fail the build of
    everyone running :hal:check on a developer machine.
    """
    probe = "#include <unistd.h>\n#include <termios.h>\nint main() { return 0; }\n"
    with tempfile.TemporaryDirectory(prefix="vmx-probe-") as directory:
        source = Path(directory) / "probe.cpp"
        source.write_text(probe, encoding="utf-8")
        try:
            result = subprocess.run(
                [*compiler, "-fsyntax-only", str(source)],
                capture_output=True,
                text=True,
            )
        except OSError as error:
            raise Unavailable(f"{compiler[0]!r} will not run: {error}") from error
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip().splitlines()
        raise Unavailable(
            f"{' '.join(compiler)} cannot target Linux AArch64"
            + (f": {detail[0]}" if detail else "")
        )


def vendored_sdk_includes() -> list[Path]:
    """The MIT-licensed VMXPi.h include closure vendored into thirdparty.

    These headers exist so the backend compiles with no SDK installed. They
    must stay byte-identical to what the gate was verified against, so a local
    edit -- which would quietly change what the gate proves -- is a failure
    rather than a silent pass. Digests live beside them in README.md.
    """
    if not (VENDORED_SDK / "VMXPi.h").is_file():
        raise Unavailable(f"vendored VMX headers are missing from {VENDORED_SDK}")

    readme = (VENDORED_SDK.parent / "README.md").read_text(encoding="utf-8")
    expected = {
        name: digest
        for name, digest in re.findall(
            r"\|\s*`([^`]+\.h)`\s*\|\s*`([0-9a-f]{32})", readme
        )
    }
    if not expected:
        raise Unavailable("thirdparty/vmxpi/README.md records no digests")

    modified = []
    for name, prefix in sorted(expected.items()):
        path = VENDORED_SDK / name
        if not path.is_file():
            modified.append(f"{name} (missing)")
            continue
        actual = hashlib.sha256(path.read_bytes()).hexdigest()
        if not actual.startswith(prefix):
            modified.append(name)
    if modified:
        raise Unavailable(
            "vendored VMX headers were modified: " + ", ".join(modified)
        )
    return [VENDORED_SDK]


def resolve_sdk_includes() -> list[Path]:
    """Resolve the SDK include layout documented in vmxdrivers/README.md."""
    configured = os.environ.get("VMX_SDK_ROOT")
    if not configured:
        raise Unavailable("VMX_SDK_ROOT is not set")
    root = Path(configured)
    if not root.is_dir():
        raise Unavailable(f"VMX_SDK_ROOT={configured!r} is not a directory")

    candidates = [root / "include", root / "include" / "vmxpi"]
    includes = [path for path in candidates if path.is_dir()]
    if not any((path / "VMXPi.h").is_file() for path in includes):
        raise Unavailable(
            f"VMXPi.h not found under {root / 'include'} or "
            f"{root / 'include' / 'vmxpi'}"
        )
    return includes


def needs_sdk(output: str) -> bool:
    """Did this translation unit fail only because the SDK header is absent?

    Roughly a third of the backend never includes VMXPi.h, directly or through
    a vmxdrivers header, so those translation units cross-compile with no SDK
    at all.  Rather than maintain a hand-written list of which ones -- which
    would rot the moment an include changes -- ask the compiler and read the
    diagnostic.  GCC reports `VMXPi.h: No such file or directory`; clang
    reports `'VMXPi.h' file not found`.
    """
    return "VMXPi.h" in output and (
        "No such file or directory" in output or "file not found" in output
    )


def collect_sources() -> list[Path]:
    sources: list[Path] = []
    for directory in SOURCE_DIRS:
        sources.extend(sorted(directory.rglob("*.cpp")))
    if not sources:
        raise RuntimeError("no VMX or shared translation units found")
    return sources


def collect_resolution_sources() -> list[Path]:
    """Platform-independent HAL sources, compiled only to resolve symbols.

    The backend legitimately calls into these -- hal::createHandle and
    hal::SetLastError, for instance -- so without them every such reference
    would look like an unresolved symbol.
    """
    sources: list[Path] = []
    for directory in RESOLUTION_DIRS:
        if directory.is_dir():
            sources.extend(sorted(directory.glob("*.cpp")))
    return sources


def resolve_nm(compiler: list[str]) -> list[str] | None:
    """Find the nm matching the compiler, falling back to the host's."""
    binary = Path(compiler[0]).name
    if binary.endswith("g++"):
        candidate = Path(compiler[0]).with_name(binary[: -len("g++")] + "nm")
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return [str(candidate)]
        if shutil.which(binary[: -len("g++")] + "nm"):
            return [binary[: -len("g++")] + "nm"]
    return ["nm"] if shutil.which("nm") else None


def object_symbols(
    nm: list[str], objects: list[Path]
) -> tuple[set[str], set[str]] | None:
    """Return (defined, undefined) symbol names across the object files."""
    if not objects:
        return None
    result = subprocess.run(
        [*nm, *[str(path) for path in objects]],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        return None
    defined: set[str] = set()
    undefined: set[str] = set()
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) >= 2 and fields[-2] == "U":
            undefined.add(fields[-1])
        elif len(fields) >= 3 and fields[-2] not in {"U", "w"}:
            defined.add(fields[-1])
    return defined, undefined


def linkage_violations(defined: set[str]) -> list[str]:
    """Check the manifest's Linkage column against what really got defined.

    Status says what the hardware can do; Linkage says whether the symbol
    exists at all. Only this gate can enforce the second, because it has real
    symbol tables -- a text search cannot tell a definition from a forward
    declaration, which is how several absences went unnoticed.
    """
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    try:
        import verify_hal_coverage as coverage
    except ImportError:
        return []

    try:
        headers = coverage.public_headers()
        manifest = coverage.parse_manifest()
    except (OSError, ValueError):
        return []

    problems: list[str] = []
    for header, path in sorted(headers.items()):
        row = manifest.get(header)
        if row is None:
            continue
        linkage = row[1]
        symbols = coverage.declared_symbols(path)
        if linkage == "PROVIDED":
            for symbol in sorted(symbols - defined):
                problems.append(
                    f"{header}: {symbol} is marked PROVIDED but nothing defines it"
                )
        elif linkage == "ABSENT":
            for symbol in sorted(symbols & defined):
                problems.append(
                    f"{header}: {symbol} is marked ABSENT but is defined; "
                    f"the manifest row is stale"
                )
    return problems


def unresolved_backend_symbols(
    nm: list[str], objects: list[Path]
) -> list[str] | None:
    """Symbols the backend needs that nothing in the HAL defines.

    -fsyntax-only accepts a declaration that no translation unit ever defines,
    and an anonymous-namespace definition of a function declared with external
    linkage. Both link-fail. Reading the object symbol table catches them; a
    real link cannot run here because the vendor library is not redistributed.
    """
    if not objects:
        return None
    result = subprocess.run(
        [*nm, *[str(path) for path in objects]],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        return None

    defined: set[str] = set()
    undefined: set[str] = set()
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) >= 2 and fields[-2] == "U":
            undefined.add(fields[-1])
        elif len(fields) >= 3 and fields[-2] not in {"U", "w"}:
            defined.add(fields[-1])

    # hal::vmx is the backend's own namespace, and a HAL_/HALSIM_ entry point
    # is either implemented here or by the shared HAL sources compiled above.
    # Anything of either kind still missing would be an undefined reference.
    external = undefined - defined
    return sorted(
        symbol
        for symbol in external
        if symbol.startswith(("HAL_", "HALSIM_")) or "3hal3vmx" in symbol
    )


def build_command(
    compiler: list[str], sdk_includes: list[Path], standard: str
) -> list[str]:
    # -c rather than -fsyntax-only: it covers everything syntax checking does,
    # and leaves object files whose symbol tables reveal link errors.
    command = [*compiler, "-c", f"-std={standard}"]
    for directory in [*INCLUDE_DIRS, *sdk_includes]:
        command.extend(["-I", str(directory)])
    command.extend(os.environ.get("VMX_CROSS_CXXFLAGS", "").split())
    return command


def compile_one(
    command: list[str], source: Path, objects: Path
) -> tuple[Path, int, str]:
    obj = objects / (source.relative_to(ROOT).as_posix().replace("/", "_") + ".o")
    try:
        result = subprocess.run(
            [*command, str(source), "-o", str(obj)],
            cwd=ROOT,
            capture_output=True,
            text=True,
        )
    except OSError as error:
        # A compiler that resolved on PATH but will not execute -- a broken
        # symlink, or a binary for the wrong host -- is a gate failure, not a
        # traceback.
        return source, 1, f"could not run {command[0]!r}: {error}"
    return source, result.returncode, (result.stderr or result.stdout).strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--require",
        action="store_true",
        help="treat a missing cross compiler or SDK as a failure, not a skip",
    )
    parser.add_argument(
        "--std",
        default=DEFAULT_STANDARD,
        help=f"C++ standard to compile against (default: {DEFAULT_STANDARD})",
    )
    args = parser.parse_args()

    def unavailable(reason: Unavailable, what: str) -> int:
        if args.require:
            print(
                f"VMX AArch64 cross-compilation gate: FAIL ({reason})",
                file=sys.stderr,
            )
            return 1
        print(f"VMX AArch64 cross-compilation gate: SKIP ({reason})")
        print(f"  {what} This is not a pass.")
        return 0

    try:
        compiler = resolve_compiler()
        probe_toolchain(compiler)
    except Unavailable as reason:
        return unavailable(
            reason,
            "No part of the VMX backend was compiled. Install an AArch64 "
            "cross compiler, or run with --require to make this a failure.",
        )

    # An installed SDK wins, since it is what a real build links against. With
    # none configured, fall back to the vendored MIT headers so every
    # translation unit still compiles. Only if both are unusable does the run
    # degrade to the subset that never reaches VMXPi.h.
    sdk_includes: list[Path] = []
    sdk_reason: Unavailable | None = None
    try:
        sdk_includes = resolve_sdk_includes()
    except Unavailable as installed_reason:
        try:
            sdk_includes = vendored_sdk_includes()
        except Unavailable as vendored_reason:
            if args.require:
                print(
                    f"VMX AArch64 cross-compilation gate: FAIL "
                    f"({installed_reason}; {vendored_reason})",
                    file=sys.stderr,
                )
                return 1
            sdk_reason = vendored_reason

    missing = [path for path in INCLUDE_DIRS if not path.is_dir()]
    if missing:
        for path in missing:
            print(
                f"VMX cross-compilation gate: include directory is absent: {path}",
                file=sys.stderr,
            )
        return 1

    sources = collect_sources()
    command = build_command(compiler, sdk_includes, args.std)

    failures: list[tuple[Path, str]] = []
    deferred: list[Path] = []
    unresolved: list[str] | None = None
    stale_linkage: list[str] = []
    with tempfile.TemporaryDirectory(prefix="vmx-objects-") as objdir:
        objects = Path(objdir)
        with concurrent.futures.ThreadPoolExecutor() as pool:
            futures = [
                pool.submit(compile_one, command, source, objects)
                for source in sources
            ]
            for future in concurrent.futures.as_completed(futures):
                source, code, output = future.result()
                if code == 0:
                    continue
                if sdk_includes or not needs_sdk(output):
                    failures.append((source, output))
                else:
                    deferred.append(source)

        if not failures and not deferred:
            # Only meaningful once every unit compiled: a partial object set
            # would report absences that are really just missing objects.
            for source in collect_resolution_sources():
                compile_one(command, source, objects)
            nm = resolve_nm(compiler)
            if nm:
                symbols = object_symbols(nm, sorted(objects.glob("*.o")))
                if symbols is not None:
                    defined, _undefined = symbols
                    stale_linkage = linkage_violations(defined)
                unresolved = unresolved_backend_symbols(
                    nm, sorted(objects.glob("*.o"))
                )

    if stale_linkage:
        print(
            f"VMX AArch64 cross-compilation gate: FAIL "
            f"({len(stale_linkage)} manifest Linkage rows disagree with reality)",
            file=sys.stderr,
        )
        for problem in stale_linkage:
            print(f"  {problem}", file=sys.stderr)
        print(
            "Linkage records whether the VMX wpiHal defines a symbol, "
            "which is separate from whether the hardware supports it. "
            "See VMX_HAL_COVERAGE.md.",
            file=sys.stderr,
        )
        return 1

    if unresolved:
        print(
            f"VMX AArch64 cross-compilation gate: FAIL "
            f"({len(unresolved)} symbols the backend needs are defined nowhere)",
            file=sys.stderr,
        )
        for symbol in unresolved:
            print(f"  {symbol}", file=sys.stderr)
        print(
            "\nThese compile but would not link. A function declared in a "
            "header\nand defined only inside an anonymous namespace looks like "
            "this.",
            file=sys.stderr,
        )
        return 1

    if failures:
        print(
            f"VMX AArch64 cross-compilation gate: FAIL "
            f"({len(failures)} of {len(sources)} translation units)",
            file=sys.stderr,
        )
        for source, output in sorted(failures):
            print(f"\n--- {source.relative_to(ROOT).as_posix()}", file=sys.stderr)
            print(output, file=sys.stderr)
        return 1

    compiled = len(sources) - len(deferred)
    if deferred:
        print(
            f"VMX AArch64 cross-compilation gate: PARTIAL "
            f"({compiled} of {len(sources)} translation units, "
            f"{' '.join(compiler)})"
        )
        print(f"  {len(deferred)} require the VMX SDK, which is unavailable: "
              f"{sdk_reason}")
        print("  Set VMX_SDK_ROOT to cover them, or run with --require to "
              "make the gap a failure.")
        return 0

    print(
        f"VMX AArch64 cross-compilation gate: PASS "
        f"({len(sources)} translation units, {' '.join(compiler)})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
