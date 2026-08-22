# VMX-pi HAL SDK headers

These are the public headers of the VMX-pi HAL SDK by Kauai Labs, copied here
verbatim under the MIT license each file carries inline. Nothing else from the
SDK is vendored: no library, no `studica_drivers` headers, and no unrelated
third-party components that ship in the same SDK bundle.

## Why they are here

The VMX HAL backend in `hal/src/main/native/vmx` is not compiled by an
ordinary build: the Gradle source set is opt-in and every non-roboRIO platform
selects the sim backend. Before these headers were vendored, only 15 of its
translation units could be compiled without a proprietary SDK installed, so CI
could never see a break in the other 27 -- which is exactly where the five
defects fixed in commit `70be48f7e` were hiding.

With these headers present, `hal/tools/verify_vmx_crosscompile.py` compiles
all 44 on any machine, with no SDK.

## What this does not give you

Headers only. Linking and running still require the real SDK library, which is
not redistributed here. This buys compile-time verification everywhere, not a
runnable VMX binary.

## Provenance and immutability

These come from the ELF64 AArch64 VMX-pi SDK -- the same SDK a real build
links against. That matters: an earlier revision of this directory was taken
from a legacy ELF32 ARM SDK whose headers were an older revision, so seven of
the eighteen differed from the ones the shipped library was built from, and
`IMURegisters.h` had since been renamed `IMURegisters_VMX.h`. Compiling
against headers that do not match the library proves very little, so always
re-vendor from the SDK that is actually deployed.

This is the transitive include closure of `VMXPi.h` and nothing more.

These files must remain unmodified so a future SDK refresh can replace them
cleanly, exactly as `vmxdrivers/` is treated. Adaptation belongs in
`hal/src/main/native/vmx`, never here. `hal/tools/verify_vmx_crosscompile.py`
checks these SHA-256 digests and fails if a file is edited:

| File | SHA-256 |
| --- | --- |
| `AHRS.h` | `04aa66a171cba73a74fe6f221396bdc2…` |
| `AHRSProtocol.h` | `beb82855bcc9350bb2d088fd11eaefcc…` |
| `IMUProtocol.h` | `e3b442b5f7f3aac1640498faff006842…` |
| `IMURegisters_VMX.h` | `89b548fe4f0c04e5662f106306a28b4c…` |
| `IVMXTimestampedAHRSDataSubscriber.h` | `6d7ea92efac95a95952813c32d77cd6d…` |
| `VMXCAN.h` | `9db010ba580289ab990a2520deb059f0…` |
| `VMXChannel.h` | `79e5627674cb54d1a8f3ae82b4da3a1d…` |
| `VMXErrors.h` | `710760819e4644783b27d377939a4a42…` |
| `VMXHandlers.h` | `cfc703f3a4c431c14f3854f93c07828a…` |
| `VMXIO.h` | `760f2acdb744e3340ff1e4df5aa6a7cc…` |
| `VMXPi.h` | `b4aef48e001b1639c1a9b84dbac24644…` |
| `VMXPiConstants.h` | `f0edaeb3aeefe0d81587e451376c8054…` |
| `VMXPower.h` | `3e0c8aba8d271f586671ac57a639604b…` |
| `VMXResource.h` | `93f47dc9c874888742a43574468e9f48…` |
| `VMXResourceConfig.h` | `888d3c24dd3d202aa71cb76475c73bad…` |
| `VMXThread.h` | `60c184acbfb551f501c380eb947011e0…` |
| `VMXTime.h` | `a4b997102cb4aaf4d65fdaff646fdbd0…` |
| `VMXVersion.h` | `d6aa250d97e0313d00387baf6f27b2c2…` |

## License

MIT, Kauai Labs (2013-2017). The full notice is inline at the top of every
file and is reproduced in the repository's `ThirdPartyNotices.txt`.
