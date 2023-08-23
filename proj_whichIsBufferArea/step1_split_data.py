import os
import shutil
from random import shuffle

# Specify the source folder
source_folder = 'data'

# Create target folders
train_folder = 'train'
test_folder = 'test'
validation_folder = 'validation'

os.makedirs(train_folder, exist_ok=True)
os.makedirs(test_folder, exist_ok=True)
os.makedirs(validation_folder, exist_ok=True)

# Get all pkl files
pkl_files = [f for f in os.listdir(source_folder) if f.endswith('.pkl')]

# Randomly shuffle the files
shuffle(pkl_files)

# Allocation ratios
train_ratio = 0.7
test_ratio = 0.2
# The remaining part is used for validation

# Calculate the size of each part
num_total = len(pkl_files)
num_train = int(train_ratio * num_total)
num_test = int(test_ratio * num_total)

# Distribute to each folder
for i, file_name in enumerate(pkl_files):
    source = os.path.join(source_folder, file_name)
    if i < num_train:
        shutil.move(source, os.path.join(train_folder, file_name))
    elif i < (num_train + num_test):
        shutil.move(source, os.path.join(test_folder, file_name))
    else:
        shutil.move(source, os.path.join(validation_folder, file_name))

print('-'*20 + '\n' + "Distribution completed!")

