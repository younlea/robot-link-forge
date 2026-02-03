#!/usr/bin/env python3
"""
Quick test to verify validation script generation
"""
import sys

sys.path.insert(0, "src/backend")

from exporters.motion_exporter import generate_validation_script

# Mock data
test_model = "test_robot.xml"
test_recording = {
    "duration": 5000,  # 5 seconds in ms
    "joints_info": [
        {"name": "joint_1"},
        {"name": "joint_2"},
    ],
    "keyframes": [
        {"time": 0, "joints": {"joint_1": 0.0, "joint_2": 0.0}},
        {"time": 1000, "joints": {"joint_1": 0.5, "joint_2": 0.3}},
        {"time": 2000, "joints": {"joint_1": 1.0, "joint_2": 0.6}},
        {"time": 3000, "joints": {"joint_1": 0.5, "joint_2": 0.3}},
        {"time": 4000, "joints": {"joint_1": 0.0, "joint_2": 0.0}},
    ],
}

print("Testing validation script generation...")
print("=" * 60)

try:
    script = generate_validation_script(test_model, test_recording)

    # Verify script has essential components
    checks = [
        ("#!/usr/bin/env python3" in script, "Has shebang"),
        ("import mujoco" in script, "Imports MuJoCo"),
        ("MAX_ACCEPTABLE_ERROR" in script, "Defines thresholds"),
        ("def validate_motor_parameters" in script, "Has validation function"),
        ("VALIDATION PASSED" in script, "Has success message"),
        ("VALIDATION FAILED" in script, "Has failure message"),
        ("sys.exit(0 if success else 1)" in script, "Has exit codes"),
    ]

    all_passed = True
    for check, description in checks:
        status = "✅" if check else "❌"
        print(f"{status} {description}")
        if not check:
            all_passed = False

    print("=" * 60)
    if all_passed:
        print("✅ ALL CHECKS PASSED")
        print(f"\nGenerated script length: {len(script)} characters")
        print(f"Generated script lines: {len(script.split(chr(10)))} lines")
    else:
        print("❌ SOME CHECKS FAILED")
        sys.exit(1)

except Exception as e:
    print(f"❌ ERROR: {e}")
    import traceback

    traceback.print_exc()
    sys.exit(1)
