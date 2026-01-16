
import sys
import os

# Adjust path
sys.path.append("/home/younlea/source-code/robot-link-forge/src/backend")

from exporters.mjcf_exporter import generate_mjcf_xml
from robot_models import RobotData, RobotLink, RobotJoint, Visual
from utils import generate_unique_names

def test_mjcf_export():
    robot = RobotData(
        baseLinkId="base",
        links={
            "base": RobotLink(id="base", name="base", visual=Visual(type="box", dimensions=[0.1, 0.1, 0.1]), childJoints=["j1", "j_prox"]),
            "index_intermediate": RobotLink(id="index_intermediate", name="IndexFinger_Proximal", visual=Visual(type="box", dimensions=[0.02, 0.02, 0.02]), childJoints=["j_dist"]),
            "index_tip": RobotLink(id="index_tip", name="index_tip", visual=Visual(type="box", dimensions=[0.02, 0.02, 0.02]), childJoints=[]),
            "little_tip": RobotLink(id="little_tip", name="LittleFinger_tip", visual=Visual(type="box", dimensions=[0.02, 0.02, 0.02]), childJoints=[]),
        },
        joints={
            "j_prox": RobotJoint(id="j_prox", name="j_prox", type="fixed", parentLinkId="base", childLinkId="index_intermediate", dof={"roll":False,"pitch":False,"yaw":False}, limits={}, origin={"xyz":[0,0,0], "rpy":[0,0,0]}),
            "j_dist": RobotJoint(id="j_dist", name="j_dist", type="fixed", parentLinkId="index_intermediate", childLinkId="index_tip", dof={"roll":False,"pitch":False,"yaw":False}, limits={}, origin={"xyz":[0,0,0], "rpy":[0,0,0]}),
            "j1": RobotJoint(id="j1", name="j1", type="fixed", parentLinkId="base", childLinkId="little_tip", dof={"roll":False,"pitch":False,"yaw":False}, limits={}, origin={"xyz":[0,0,0], "rpy":[0,0,0]}),
        }
    )

    unique_names = generate_unique_names(robot)
    
    print("\nTesting direct_hand=True (Refined Logic)...")
    xml_true, _ = generate_mjcf_xml(robot, "test_robot", {}, unique_names, direct_hand=True)
    
    print("FULL XML OUTPUT DEBUG:")
    print(xml_true)
    print("END FULL XML DEBUG")

    # DEBUG: Print XML snippet for Little tip
    print("--- XML Generation Snippet ---")
    if "LittleFinger_tip" in xml_true:
        print("Found 'LittleFinger_tip' in XML. Searching for sensors nearby...")
        # Crude print nearby lines
        lines = xml_true.split('\n')
        for i, line in enumerate(lines):
            if "LittleFinger_tip" in line:
                print(f"Line {i}: {line}")
                for j in range(1, 25): # Print next few lines
                    if i+j < len(lines):
                        print(f"  {lines[i+j]}")

    if 'index_tip_sensor_0_1' in xml_true:
        print("\n[PASS] Index Tip grid sensor found.")
    else:
        print("\n[FAIL] Index Tip grid sensor missing.")

    if 'index_intermediate_sensor' in xml_true:
        print("[FAIL] Sensors found on restricted intermediate link!")
    else:
        print("[PASS] Intermediate link clean.")

    # Utils converts LittleFinger_tip -> little_finger_tip
    expected_little_name = 'little_finger_tip_sensor_0_1'
    if expected_little_name in xml_true:
        print("[PASS] Little Tip grid sensor found.")
    else:
        print("[FAIL] Little Tip grid sensor missing.")
        print(f"Expected: '{expected_little_name}'")

if __name__ == "__main__":
    test_mjcf_export()
