# Studica vendor adapters

The Parsec and Colore protocol implementations consumed by this directory are
the pinned, standalone `drivers` sources imported from the Studica Robotics
ROS2 repository:

* Source: [Studica Robotics ROS2](https://github.com/Studica-Robotics/ROS2)
* Pinned source snapshot: `1a52cfbf3c3eab51807b9d2d7fa42dd6143882df`
* Upstream license: Apache-2.0 (see the immutable `vmxdrivers/LICENSE`)

`vmxdrivers/` is kept byte-for-byte unchanged. The files in this directory
are WPILib-project-owned adapters only: they translate the imported driver's
CAN/USB lifecycle and data into the fixed-width `StudicaParsec_*` and
`StudicaColore_*` contracts, share `VMXRuntime`, enforce the physical resource
registry, and expose status/lifecycle semantics needed by the C ABI, C++
facades, Java JNI, and Python ctypes callers. No WPILib types or HAL status
codes are added to the upstream driver sources.

`transport/` contains the reusable lifecycle/ownership seam. The pinned
drivers remain the protocol/parser source because their constructors own the
device-specific wire details; adapters must not duplicate or patch those
protocols in `vmxdrivers`.
