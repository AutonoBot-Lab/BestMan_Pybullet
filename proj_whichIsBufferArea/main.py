import os
import pickle
import numpy as np
import torch
import torchvision.transforms as transforms
from torch import nn
import torch.nn.functional as F
from utils_control import Bestman, Pose
from utils_envs import Kitchen
import math

def generate_random_position_within_table():
    # table boundary
    table_min_x, table_max_x = 0.249, 1.751
    table_min_y, table_max_y = 0.499, 1.501
    table_min_z, table_max_z = -0.186, 0.826

    # Convert polar coordinates to Cartesian coordinates
    x = np.random.uniform(table_min_x, table_max_x)
    y = np.random.uniform(table_min_y, table_max_y)
    z = table_height

    return [x, y, z]

def world_to_image(physicsClient, viewMatrix, projectionMatrix, world_point):
    """
    Function to transform world coordinates to image coordinates
    """

    def reshape_matrix(matrix):
        """
        Reshape a 1D matrix of length 16 (list or numpy array) to a 4x4 2D matrix
        """
        return np.reshape(matrix, (4, 4))

    # Reshape the matrices
    viewMatrix = reshape_matrix(viewMatrix)
    projectionMatrix = reshape_matrix(projectionMatrix)

    # Project world to camera
    world_point = np.append(world_point, 1)  # Add homogenous coordinate
    view_projection_matrix = np.matmul(projectionMatrix, viewMatrix)
    camera_point = np.matmul(view_projection_matrix, world_point)

    # Normalize homogenous coordinates
    camera_point = camera_point[:3] / camera_point[3]

    # Convert camera coordinates to image coordinates
    img_width = 640
    img_height = 480
    img_point = np.zeros(2, dtype=int)
    img_point[0] = int((camera_point[0] + 1.0) / 2.0 * img_width)  # x
    img_point[1] = int((1.0 - camera_point[1]) / 2.0 * img_height)  # y

    return img_point


def crop_image(image, center, size):
    """
    Function to crop a square image at "center" with side length "size".
    "center" is a 2-element list or tuple or ndarray [x, y]
    "size" is an int
    """
    image_height, image_width = image.shape
    top = max(0, int(center[1] - size / 2))
    bottom = min(image_height, int(center[1] + size / 2))
    left = max(0, int(center[0] - size / 2))
    right = min(image_width, int(center[0] + size / 2))
    return image[top:bottom, left:right]

def get_depth_image():
    img_arr = p.getCameraImage(
        640, 480, viewMatrix, projectionMatrix, shadow=1, lightDirection=[1, 1, 1]
    )
    depth_buffer = img_arr[3]  # depth buffer
    near = 0.01  # Near plane distance
    far = 100  # Far plane distance
    depth = far * near / (far - (far - near) * depth_buffer)
    # personalized colormap
    cdict = {
        "red": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
        "green": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
        "blue": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
    }
    custom_cmap = LinearSegmentedColormap("custom_cmap", cdict)
    # plt.imshow(depth, cmap=custom_cmap)
    # plt.colorbar()
    # plt.show()

    # image_point = world_to_image(
    #     physicsClient, viewMatrix, projectionMatrix, [0, 0, 0.05]
    # )
    image_point = [320, 240]

    crop_size = 200
    depth_image = crop_image(depth, image_point, crop_size)
    # print("depth size:{} * {}".format(len(depth_image), len(depth_image[0])))
    # print("image_point:{}".format(image_point))
    # plt.imshow(depth_image, cmap=custom_cmap)
    # plt.imsave(name + '.png', depth_image, cmap=custom_cmap)
    # plt.colorbar()
    # plt.show()
    return depth_image


# ----------------------------------------------------------------
#  Check collision
# ----------------------------------------------------------------
def inflate_aabb(aabb, inflation):
    return (
        [coord - inflation for coord in aabb[0]],  # Lower limits
        [coord + inflation for coord in aabb[1]],  # Upper limits
    )

def collision_exists(object_ids, new_object_id, collision_distance=-0.01):
    # print('enter exist_collision, object_ids:{}'.format(object_ids))
    if len(object_ids) == 0:
        return False
    # Loop over all pairs of objects
    for object_x in object_ids:
        # Check for collision between object i and object j
        print('object_ids:{}, Check for collision between new object {} and object {}'.format(object_ids, new_object_id, object_x))
        aabb_x = inflate_aabb(p.getAABB(object_x), collision_distance)
        aabb_y = inflate_aabb(p.getAABB(new_object_id), collision_distance)
        if (
            aabb_x[0][0] <= aabb_y[1][0]
            and aabb_x[1][0] >= aabb_y[0][0]
            and aabb_x[0][1] <= aabb_y[1][1]
            and aabb_x[1][1] >= aabb_y[0][1]
            # and aabb_x[0][2] <= aabb_y[1][2]
            # and aabb_x[1][2] >= aabb_y[0][2]
        ):
            # print("Collision detected")
            return True
    return False


# ----------------------------------------------------------------
# Generate random positions for objects within the specified radius and get images
# ----------------------------------------------------------------
# load robot
init_pose = Pose([1, 0, 0], [0.0, 0.0, math.pi / 2])
demo = Bestman(init_pose)
demo.enable_vertical_view(4.0, [1.0, 1.0, 0])

# reset arm joint position
pose1 = [0, -1.57, 2.0, -1.57, -1.57, 0]
demo.move_arm_to_joint_angles(pose1)

# ----------------------------------------------------------------
# Load plan and table
# ----------------------------------------------------------------
# load table
table_id = demo.load_object(
    "./URDF_models/furniture_table_rectangle_high/table.urdf",
    [1.0, 1.0, 0.0],
    [0.0, 0.0, 0.0],
    1.0, 
    "table",
)
(_, _, _), (_, _, table_height) = p.getAABB(table_id)
table_height += 0.08

# table_boundary = demo.get_bounding_box(table_id)
# table_min_x, table_min_y, table_min_z = table_boundary[0]
# table_max_x, table_max_y, table_max_z = table_boundary[1]
# print('table_min_x:{}, table_min_y:{}, table_min_z:{}'.format(table_min_x, table_min_y, table_min_z))
# print('table_max_x:{}, table_max_y:{}, table_max_z:{}'.format(table_max_x, table_max_y, table_max_z))

# ----------------------------------------------------------------
# Set camera parameters
# ----------------------------------------------------------------
tablePos, _ = p.getBasePositionAndOrientation(table_id)
cameraPos = [tablePos[0], tablePos[1], table_height + 0.7]
cameraUp = [0, 1, 0]  # bird-view
viewMatrix = p.computeViewMatrix(cameraPos, tablePos, cameraUp)
projectionMatrix = p.computeProjectionMatrixFOV(
    fov=60, aspect=1.0, nearVal=0.01, farVal=100.0
)

# ----------------------------------------------------------------
# Objects
# ----------------------------------------------------------------
object_names = ["blue_cup", "blue_plate", "plastic_apple", "plastic_peach"]
models = {"blue_cup": "./URDF_models/blue_cup/table.urdf",
        "blue_plate": "./URDF_models/blue_plate/table.urdf",
        "plastic_apple": "./URDF_models/plastic_apple/table.urdf",
        "plastic_peach": "./URDF_models/plastic_peach/table.urdf"}

# ----------------------------------------------------------------
# Generate random positions for objects within table
# ----------------------------------------------------------------
n_objects = 10 # number of objects on table
for _ in range(n_objects):
    object_name = random.choice(object_names) # Choose a random object
    num_c = 0
    while True:
        position_other = generate_random_position_within_table()
        orientation_other = p.getQuaternionFromEuler([0, 0, np.random.uniform(0, 2 * np.pi)])
        # orientation_other = p.getQuaternionFromEuler([0, 0, 0])
        object_id = p.loadURDF(models[object_name], position_other, orientation_other)
        # simulate(1)
        demo.draw_aabb(object_id)
        print('-' * 20 + '\n' + 'object_id:{} is waitting for to be added.'.format(object_id))

        # ----------------------------------------------------------------
        # Ensure that the position is free of other objects
        # ----------------------------------------------------------------
        if not collision_exists(object_ids, object_id):
            # No collision detected, add the object and exit the loop
            positions_orientations.append([object_name, position_other, orientation_other])
            object_ids.append(object_id)
            print('object_id:{} has been added.'.format(object_id))
            print("object_name:{} position:{}".format(object_name, position_other))
            break
        else:
            # Collision detected, remove the object and try again
            p.removeBody(object_id)
            simulate(2)
            print('-' * 20 + '\n' + 'object_id:{} has been removed.'.format(object_id))
        num_c += 1
        if num_c >= 1000:
            print("Try many times, exit!")
            break
# Get depth image A
simulate(2)
print("-" * 20 + "\n" + "Image A")
depth_image_A = get_depth_image("depth_image_A")

# # ----------------------------------------------------------------
# # predict buffer area
# # ----------------------------------------------------------------

# # define a deeper model
# device = torch.device("cpu")
# model = torch.hub.load("zhanghang1989/ResNeSt", "resnest50", pretrained=False).to(
#     device
# )
# model.fc = nn.Linear(2048, 2, bias=True).to(device)
# model.conv1 = nn.Conv2d(
#     2, 64, kernel_size=(7, 7), stride=(2, 2), padding=(3, 3), bias=False
# ).to(device)
# model.load_state_dict(torch.load("best_model.pth", map_location=device))

# # define the transform
# val_tf = transforms.Compose(
#     [
#         transforms.ToTensor(),
#     ]
# )

# # path to your test folder
# data_dir = "test" # store images
# data_files = os.listdir(data_dir)

# preds = []  # predict results
# trues = []  # real results

# model.eval() # Set the model to evaluation mode

# # loop through each file in the directory
# for file_name in data_files:
#     with open(f"{data_dir}/{file_name}", "rb") as f:
#         data = pickle.load(f)

#     depth_image_A = data["depth_image_A"]
#     depth_image_B = data["depth_image_B"]
#     success_rate = data["success_rate"]

#     # Convert depth images to tensor and stack
#     depth_image = np.stack((depth_image_A, depth_image_B), axis=2)
#     inputs = val_tf(depth_image)
#     inputs = inputs.unsqueeze(0).to(device)  # add batch dimension

#     success_rate = torch.tensor(success_rate).long().unsqueeze(0).to(device)  # add batch dimension

#     with torch.no_grad():
#         outputs = model(inputs).squeeze()
#         probs = F.softmax(outputs, dim=0)
#         probs = probs.tolist()
#         pred = torch.argmax(outputs).item()
#         print(f"File: {file_name}, Probs:{probs}, Predicted: {pred}, Actual: {success_rate.item()}")
#         preds.append(pred)
#         trues.append(success_rate.item())