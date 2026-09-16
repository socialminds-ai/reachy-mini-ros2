"""Developer check: compare reachy_mini_c with the Python SDK kinematics.

Usage (needs the reachy_mini SDK installed, e.g. in its venv):
    python3 tools/check_reachy_mini_c_against_sdk.py build/reachy_mini_c/cargo/release/libreachy_mini_c.so
"""
import ctypes, numpy as np, sys
from reachy_mini.kinematics.analytical_kinematics import AnalyticalKinematics
from reachy_mini.utils import create_head_pose

lib = ctypes.CDLL(sys.argv[1])
lib.rmc_kinematics_new.restype = ctypes.c_void_p
lib.rmc_last_error.restype = ctypes.c_char_p
lib.rmc_version.restype = ctypes.c_char_p
D = ctypes.POINTER(ctypes.c_double)
for f in (lib.rmc_kinematics_ik, lib.rmc_kinematics_ik_fixed_yaw):
    f.argtypes = [ctypes.c_void_p, D, ctypes.c_double, D]; f.restype = ctypes.c_int
lib.rmc_kinematics_reset_fk.argtypes = [ctypes.c_void_p, D]; lib.rmc_kinematics_reset_fk.restype = ctypes.c_int
lib.rmc_kinematics_fk.argtypes = [ctypes.c_void_p, D, ctypes.c_int, D]; lib.rmc_kinematics_fk.restype = ctypes.c_int
print("version", lib.rmc_version())

k = lib.rmc_kinematics_new(); assert k, lib.rmc_last_error()
sdk = AnalyticalKinematics(automatic_body_yaw=True)
sdk_fixed = AnalyticalKinematics(automatic_body_yaw=False)
rng = np.random.default_rng(0)
max_err = 0.0; max_err_fixed = 0.0; max_fk = 0.0; unreachable = 0
for i in range(200):
    pose = create_head_pose(x=rng.uniform(-0.02,0.02), y=rng.uniform(-0.02,0.02), z=rng.uniform(-0.02,0.02),
                            roll=rng.uniform(-20,20), pitch=rng.uniform(-20,20), yaw=rng.uniform(-40,40), degrees=True)
    body_yaw = rng.uniform(-1.0, 1.0)
    ref = sdk.ik(pose, body_yaw=body_yaw)
    out = (ctypes.c_double*7)()
    r = lib.rmc_kinematics_ik(k, np.ascontiguousarray(pose).ctypes.data_as(D), body_yaw, out)
    if np.any(np.isnan(ref)):
        assert r != 0, "SDK says unreachable but shim succeeded"; unreachable += 1; continue
    assert r == 0, lib.rmc_last_error()
    max_err = max(max_err, np.abs(np.array(out) - ref).max())
    ref2 = sdk_fixed.ik(pose, body_yaw=body_yaw)
    r = lib.rmc_kinematics_ik_fixed_yaw(k, np.ascontiguousarray(pose).ctypes.data_as(D), body_yaw, out)
    if np.any(np.isnan(ref2)):
        assert r != 0; continue
    assert r == 0, lib.rmc_last_error()
    max_err_fixed = max(max_err_fixed, np.abs(np.array(out) - ref2).max())
    # FK roundtrip: reset the warm start to a neutral pose, then FK(IK(pose)) must give pose back
    fk = (ctypes.c_double*16)()
    eye = np.eye(4)
    assert lib.rmc_kinematics_reset_fk(k, eye.ctypes.data_as(D)) == 0
    r = lib.rmc_kinematics_fk(k, out, 20, fk); assert r == 0, lib.rmc_last_error()
    fk = np.array(fk).reshape(4,4)
    max_fk = max(max_fk, np.abs(fk - pose).max())
print(f"max |ik_auto - sdk| = {max_err:.2e} rad")
print(f"max |ik_fixed - sdk| = {max_err_fixed:.2e} rad")
print(f"max |fk(ik(pose)) - pose| = {max_fk:.2e}  (unreachable poses, both agree: {unreachable}/200)")
print("SHIM_CHECK:", "PASS" if max_err < 1e-9 and max_err_fixed < 1e-9 and max_fk < 1e-4 else "FAIL")
