# VMX-pi HAL SDK headers

These are the public headers of the VMX-pi HAL SDK by Kauai Labs, copied here
verbatim under the MIT license each file carries inline. Nothing else from the
SDK is vendored: no library, no `studica_drivers` headers, and no unrelated
third-party components that ship in the same SDK bundle.

## Why they are here

The VMX HAL backend in `hal/src/main/native/vmx` is not compiled by an
ordinary build: the Gradle source set is opt-in and every non-roboRIO platform
selects the sim backend. Before these headers were vendored, only 15 of its 42
translation units could be compiled without a proprietary SDK installed, so CI
could never see a break in the other 27 -- which is exactly where the five
defects fixed in commit `70be48f7e` were hiding.

With these headers present, `hal/tools/verify_vmx_crosscompile.py` compiles
all 42 on any machine, with no SDK.

## What this does not give you

Headers only. Linking and running still require the real SDK library, which is
not redistributed here. This buys compile-time verification everywhere, not a
runnable VMX binary.

## Provenance and immutability

This is the exact header set that the cross-compilation gate was first
verified against, at 42 of 42 translation units. It is the transitive include
closure of `VMXPi.h` and nothing more.

These files must remain unmodified so a future SDK refresh can replace them
cleanly, exactly as `vmxdrivers/` is treated. Adaptation belongs in
`hal/src/main/native/vmx`, never here. `hal/tools/verify_vmx_crosscompile.py`
checks these SHA-256 digests and fails if a file is edited:

| File | SHA-256 |
| --- | --- |
| `AHRS.h` | `89d7fb41c1a46143a58bd86e4580372f…` |
| `AHRSProtocol.h` | `e89020924de584e3c2b52165f2891b04…` |
| `IMUProtocol.h` | `e3b442b5f7f3aac1640498faff006842…` |
| `IMURegisters.h` | `4fa383f0958997465a284cff2b345565…` |
| `IVMXTimestampedAHRSDataSubscriber.h` | `6d7ea92efac95a95952813c32d77cd6d…` |
| `VMXCAN.h` | `9db010ba580289ab990a2520deb059f0…` |
| `VMXChannel.h` | `79e5627674cb54d1a8f3ae82b4da3a1d…` |
| `VMXErrors.h` | `710760819e4644783b27d377939a4a42…` |
| `VMXHandlers.h` | `cfc703f3a4c431c14f3854f93c07828a…` |
| `VMXIO.h` | `25428ca0d3786fe1eccd8acbcfa1b95e…` |
| `VMXPi.h` | `b4aef48e001b1639c1a9b84dbac24644…` |
| `VMXPiConstants.h` | `f0edaeb3aeefe0d81587e451376c8054…` |
| `VMXPower.h` | `43c95d617e002e409611cc5679eed754…` |
| `VMXResource.h` | `93f47dc9c874888742a43574468e9f48…` |
| `VMXResourceConfig.h` | `888d3c24dd3d202aa71cb76475c73bad…` |
| `VMXThread.h` | `ec74248fae62c1d58660eaa6f8cc8b4b…` |
| `VMXTime.h` | `48e2006e43d13aa58e5d62f86ed694c5…` |
| `VMXVersion.h` | `6b1372bab6be5046266b4959e3b428d9…` |

## License

MIT, Kauai Labs (2013-2017). The full notice is inline at the top of every
file and is reproduced in the repository's `ThirdPartyNotices.txt`.
