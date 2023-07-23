from utils import Bestman, Pose, Kitchen, RRT_joint
import math
import numpy as np

# load robot
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose)
demo.enable_vertical_view(4.0, [1.0, 1.0, 0])

# reset arm joint position
joint_angles = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(joint_angles)

# # load table
# table_id = demo.load_object(
#     "./models/furniture_table_rectangle_high/table.urdf",
#     [1.0, 1.0, 0.0],
#     [0.0, 0.0, 0.0],
#     "table",
# )

# # load bowl
# bowl_id = demo.load_object(
#     "./models/utensil_bowl_red/model.urdf", [0.6, 0.6, 0.85], [0.0, 0.0, 0.0], "bowl"
# )

# start configuration
joint_angles = [-0.1] * 6
demo.set_arm_to_joint_angles(joint_angles)
start_configuration = np.array(joint_angles)
demo.wait(3)
print("-" * 30 + "\n" + "start_configuration:{}".format(start_configuration))

# goal configuration (option 1)
# joint_angles = [-0.5] * 6
# demo.set_arm_to_joint_angles(joint_angles)
# goal_configuration = np.array(joint_angles)
# print("-" * 30 + "\n" + "goal_configuration:{}".format(goal_configuration))

# goal configuration (option 2)
joint_angles = demo.cartesian_to_joints([0.6, 0.6, 1.2], [0.0, math.pi / 2.0, 0.0])
demo.set_arm_to_joint_angles(joint_angles)
goal_configuration = np.array(joint_angles)
print("-" * 30 + "\n" + "goal_configuration:{}".format(goal_configuration))

print('demo.obstacle_manipulation_ids:{}'.format(demo.obstacle_manipulation_ids))

# trajectory planning
rrt = RRT_joint(
        start=start_configuration,
        goal=goal_configuration,
        obstacle_list=demo.obstacle_manipulation_ids,
        arm_id=demo.arm_id,
        base_id=demo.base_id
    )
path = rrt.plan()
if path is None:
    print("Cannot find path")
else:
    print("-" * 30 + "\n" + "found path")
    # print("path: {}".format(path))
    demo.visualize_path(path)

demo.wait(10)

