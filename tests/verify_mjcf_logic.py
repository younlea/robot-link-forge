
class MockLink:
    def __init__(self, name):
        self.name = name
        self.childJoints = []

class MockRobot:
    def __init__(self):
        self.joints = {}

def check_logic(body_name, direct_hand=True):
    name_lower = body_name.lower()
    is_leaf = True # Assume leaf for now since user names imply it
    
    apply_direct_sensor = False
    
    if direct_hand:
        if is_leaf or any(k in name_lower for k in ["tip", "distal", "end", "3rd"]):
            apply_direct_sensor = True
            
    found_config = None
    finger_prefix = "none"
    
    if apply_direct_sensor:
         nm = body_name.lower()
         if 'little' in nm or 'pinky' in nm: finger_prefix = 'Little'
         elif 'ring' in nm: finger_prefix = 'Ring'
         elif 'middle' in nm: finger_prefix = 'Middle'
         elif 'index' in nm: finger_prefix = 'Index'
         elif 'thumb' in nm: finger_prefix = 'Thumb'
    
    return apply_direct_sensor, finger_prefix

test_names = [
    "index_finger-3rd-end",
    "little_finger-3rd-end",
    "middle_finger-3rd-end",
    "ring_finger-3rd-end",
    "thumb-3rd-end",
]

print("--- JS Logic verification ---")
for name in test_names:
    apply, prefix = check_logic(name)
    print(f"Name: {name: <25} | Apply: {str(apply): <5} | Prefix: {prefix}")
