import pickle
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import LinearSegmentedColormap
import os
import sys
import torch
import torch.nn.functional as F

# ----------------------------------------------------------------
#  convert score to probability
# ----------------------------------------------------------------

logits = torch.tensor([1.9075618, -1.6673121])
probs = F.softmax(logits, dim=0)
print(probs) 

# ----------------------------------------------------------------
#  dispaly image A and image B
# ----------------------------------------------------------------

# Your model name
model_name = "laptop_2023-08-12_04-16-50"

# Filename
filename = f"{model_name}.pkl"

# Read data from a pickle file
with open("./test/" + filename, "rb") as f:
    data = pickle.load(f)

# Extract the "depth" information from the data
depth_image_A = data["depth_image_A"]
depth_image_B = data["depth_image_B"]
success_rate = data["success_rate"]

print(
    "depth_image_A max:{} min:{}".format(np.amax(depth_image_A), np.amin(depth_image_A))
)
print(
    "depth_image_B max:{} min:{}".format(np.amax(depth_image_B), np.amin(depth_image_B))
)
print("success_rate:{}".format(success_rate))

# Save to a text file
# np.savetxt(f'depth_{model_name}.txt', depth, fmt='%.3f')

# Visualize the "depth" information
# plt.figure(figsize=(10, 6))

# Personalized colormap
cdict = {
    "red": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
    "green": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
    "blue": [[0.0, 0.0, 0.0], [1.0, 1.0, 1.0]],
}
custom_cmap = LinearSegmentedColormap("custom_cmap", cdict)

# plt.imshow(depth_image_A, cmap=custom_cmap)
# plt.colorbar()
# plt.show()

# plt.imshow(depth_image_B, cmap=custom_cmap)
# plt.colorbar()
# plt.show()

# Create a 1x2 subplot layout
fig, axes = plt.subplots(1, 2, figsize=(10, 5))

# Display Depth Image A
im_A = axes[0].imshow(depth_image_A, cmap=custom_cmap)
axes[0].set_title("Depth Image A")
plt.colorbar(im_A, ax=axes[0])

# Display Depth Image B
im_B = axes[1].imshow(depth_image_B, cmap=custom_cmap)
axes[1].set_title("Depth Image B")
plt.colorbar(im_B, ax=axes[1])

plt.show()

# sys.exit()
# ----------------------------------------------------------------
#  save image B
# ----------------------------------------------------------------
# object_names = ["blue_cup", "blue_plate", "plastic_apple", "plastic_peach"]

# Filename
imageB_filename = "./image_object/plastic_peach.pkl"

# Save depth_image_B to a pickle file
with open(imageB_filename, "wb") as file:
    pickle.dump(depth_image_B, file)

print(f"{imageB_filename} has been saved successfully.")


# ----------------------------------------------------------------
#  do statisitic
# ----------------------------------------------------------------
# Specify the folder path
directory = "train"

# Iterate through each .pkl file in the folder
success_rates = []
for filename in os.listdir(directory):
    if filename.endswith(".pkl"):
        filepath = os.path.join(directory, filename)
        with open(filepath, "rb") as file:
            data = pickle.load(file)
            depth_image_A = data["depth_image_A"]
            depth_image_B = data["depth_image_B"]
            success_rate = data["success_rate"]
            success_rates.append(success_rate)

# Use numpy for some basic statistics
success_rates_array = np.array(success_rates)
mean_success_rate = np.mean(success_rates_array)
std_dev_success_rate = np.std(success_rates_array)
min_success_rate = np.min(success_rates_array)
max_success_rate = np.max(success_rates_array)

print(f"Mean Success Rate: {mean_success_rate}")
print(f"Standard Deviation of Success Rate: {std_dev_success_rate}")
print(f"Minimum Success Rate: {min_success_rate}")
print(f"Maximum Success Rate: {max_success_rate}")

# Plot a histogram
plt.hist(success_rates_array, bins=100, edgecolor="black")
plt.title("Distribution of Success Rates")
plt.xlabel("Success Rate")
plt.ylabel("Frequency")
plt.show()
