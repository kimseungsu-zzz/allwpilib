#!/usr/bin/env python3
"""AArch64 cross-compilation gate for the VMX HAL backend.

Nothing in normal CI compiles hal/src/main/native/vmx.  The Gradle VMX source
set is opt-in (-PvmxBuild) and Bazel selects the sim backend off the roboRIO,
so roughly ten thousand lines of backend translation units never reach a
compiler until somebody builds for real hardware.  This gate closes that hole
by running the cross compiler in -fsyntax-only mode over every VMX and shared
translation unit.

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


def build_command(
    compiler: list[str], sdk_includes: list[Path], standard: str
) -> list[str]:
    command = [*compiler, "-fsyntax-only", f"-std={standard}"]
    for directory in [*INCLUDE_DIRS, *sdk_includes]:
        command.extend(["-I", str(directory)])
    command.extend(os.environ.get("VMX_CROSS_CXXFLAGS", "").split())
    return command


def compile_one(command: list[str], source: Path) -> tuple[Path, int, str]:
    try:
        result = subprocess.run(
            [*command, str(source)],
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
    with concurrent.futures.ThreadPoolExecutor() as pool:
        futures = [pool.submit(compile_one, command, source) for source in sources]
        for future in concurrent.futures.as_completed(futures):
            source, code, output = future.result()
            if code == 0:
                continue
            if sdk_includes or not needs_sdk(output):
                failures.append((source, output))
            else:
                deferred.append(source)

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
