from urdfpy import URDF
import numpy as np
from scipy.spatial.transform import Rotation as R
import re
import yaml
import os
# === 1. Strip visual/collision from URDF ===
urdf_path = "/home/juli/coding/ROS-projekts/safety_demo/src/safety_shield_node/urdf/ur_robot/ur3.urdf"
with open(urdf_path) as f:
    urdf_text = f.read()
print(urdf_path)
# Remove full <visual> and <collision> blocks
urdf_text = re.sub(r'<visual>.*?</visual>', '', urdf_text, flags=re.DOTALL)
urdf_text = re.sub(r'<collision>.*?</collision>', '', urdf_text, flags=re.DOTALL)

temp_urdf_path = "/tmp/ur3_temp.urdf"
if os.path.exists(temp_urdf_path):
    os.remove(temp_urdf_path)
with open(temp_urdf_path, "w") as f:
    f.write(urdf_text)
print(f"Temporary URDF saved to {temp_urdf_path}")
robot = URDF.load(temp_urdf_path)

# === 2. Define joints and zero angles ===
# ur robots
joint_names = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]
# robco
joint_names = [f"joint{i}_joint" for i in range(6)]
# schunk
joint_names = [
    "arm_1_joint",
    "arm_2_joint",
    "arm_3_joint",
    "arm_4_joint",
    "arm_5_joint",
    "arm_6_joint",
]
joint_angles = {name: 0.0 for name in joint_names}

# === 3. Compute transforms manually ===
def get_transform(joint, angle_rad):
    T = joint.origin.copy()
    if joint.joint_type == 'revolute':
        axis = np.array(joint.axis)
        R_joint = R.from_rotvec(angle_rad * axis).as_matrix()
        T[:3, :3] = T[:3, :3] @ R_joint
    return T

def find_joint_path(robot, from_link, to_joint_name):
    """Finds a joint path from from_link to the joint with name `to_joint_name`."""
    path = []
    visited = set()
    stack = [(from_link, [])]

    while stack:
        current_link, current_path = stack.pop()
        for joint in robot.joints:
            if joint.parent == current_link and joint.name not in visited:
                new_path = current_path + [joint]
                if joint.name == to_joint_name:
                    return new_path
                visited.add(joint.name)
                stack.append((joint.child, new_path))
    raise ValueError(f"Joint path from {from_link} to {to_joint_name} not found.")

# === 4. Build FK chain per joint ===
flat_matrices = []
base_link = "world"

for joint_name in joint_names:
    path_joints = find_joint_path(robot, base_link, joint_name)
    T = np.eye(4)
    for joint in path_joints:
        angle = joint_angles.get(joint.name, 0.0)
        T = T @ get_transform(joint, angle)

    flat_matrices.extend([float(f"{v:.3f}") for row in T for v in row])
    base_link = robot.joint_map[joint_name].child  # next base

# === 5. Dump to YAML ===
# Load the flattened matrix list
with open("transformation_matrices.yaml", "w") as f:
    yaml.dump({"transformation_matrices": flat_matrices}, f)
print("✅ Transformations saved to transformation_matrices.yaml")

# Optional: Pretty-print
matrices = [flat_matrices[i:i + 16] for i in range(0, len(flat_matrices), 16)]
print("transformation_matrices: [")
for idx, mat in enumerate(matrices):
    for row in range(4):
        values = mat[row * 4:(row + 1) * 4]
        print("    " + ", ".join(f"{v:.6g}" for v in values) + ",")
    if idx < len(matrices) - 1:
        print("    ###")
print("]")
