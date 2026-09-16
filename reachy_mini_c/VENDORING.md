# Vendored code

`rust/vendor/` contains code from Pollen Robotics, copied from the source
distributions published on PyPI, so that this package builds without network
access to GitHub and without Python:

| directory | origin | version | what was changed |
|---|---|---|---|
| `reachy_mini_motor_controller` | PyPI sdist `reachy_mini_motor_controller` | 1.5.6 | kept `src/controller.rs` verbatim; dropped the pyo3 bindings (`bindings.rs`), the async control loop (`control_loop.rs`) and the tokio/pyo3 dependencies; new 3-line `lib.rs` |
| `reachy_mini_kinematics` | PyPI sdist `reachy_mini_rust_kinematics` | 1.0.3 | kept the pure-Rust `Kinematics` struct and its `impl` verbatim; dropped the pyo3 wrapper class, the module init and the file-based tests |

`rust/reachy_mini_c/kinematics_data.json` is `reachy_mini/assets/kinematics_data.json`
from the `reachy_mini` Python package 1.10.0 (Apache-2.0).

The `reachy_mini` SDK is Apache-2.0; the two Rust source distributions do not
ship a licence file but are published by the same authors for the same
project. Ask Pollen Robotics before redistributing this package outside the lab.

To upgrade: download the new sdists (`pip download --no-deps --no-binary :all: <pkg>==<ver>`),
re-apply the cuts above, and run `../tools/check_reachy_mini_c_against_sdk.py` against the SDK of
the same version.
