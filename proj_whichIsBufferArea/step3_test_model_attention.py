import os
import sys
import pickle
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.optim.lr_scheduler import StepLR
from torch.utils.data import Dataset, DataLoader
from sklearn.model_selection import train_test_split
import numpy as np
from torchvision.models import resnet50
import torchvision.transforms as transforms
import matplotlib.pyplot as plt

"""
Split-Attention Network, A New ResNet Variant. 
It significantly boosts the performance of downstream models such as Mask R-CNN, Cascade R-CNN and DeepLabV3.
"""
torch.hub.list("zhanghang1989/ResNeSt", force_reload=True)


"""
define a dataset
"""

class CustomDataset(Dataset):
    def __init__(self, data_dir, transform=None):
        super().__init__()
        self.data_dir = data_dir
        self.transform = transform
        self.data_files = os.listdir(data_dir)

    def __getitem__(self, index):
        file_name = self.data_files[index]

        with open(f"{self.data_dir}/{file_name}", "rb") as f:
            data = pickle.load(f)

        depth_image_A = data["depth_image_A"]
        depth_image_B = data["depth_image_B"]
        success_rate = data["success_rate"]

        # Convert depth images to tensor and stack
        depth_image = np.stack((depth_image_A, depth_image_B), axis=2)
        inputs = self.transform(depth_image)

        # Make sure that success_rate is a float
        # success_rate = 1 if success_rate >= 0.9 else 0
        success_rate = torch.tensor(success_rate).long()

        return inputs, success_rate

    def __len__(self):
        return len(self.data_files)


# convert the image into a tensor
val_tf = transforms.Compose(
    [
        transforms.ToTensor(),
    ]
)
# create a dataset instance
dataset = CustomDataset("test", val_tf)
print('-'*20 + '\n' + 'dataloader is constructed!')

loader = DataLoader(dataset, batch_size=32, shuffle=True)

# define a deeper model
# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
device = torch.device("cpu")
model = torch.hub.load("zhanghang1989/ResNeSt", "resnest50", pretrained=False).to(
    device
)
model.fc = nn.Linear(2048, 2, bias=True).to(device)
model.conv1 = nn.Conv2d(
    2, 64, kernel_size=(7, 7), stride=(2, 2), padding=(3, 3), bias=False
).to(device)
model.load_state_dict(torch.load("best_model.pth", map_location=device))

# define loss function
criterion = nn.CrossEntropyLoss()
total_loss = 0

preds = []  # predict results
trues = []  # real results
acc = 0

print('-'*20 + '\n' + 'start testing')
with torch.no_grad():
    for data in loader:
        images, labels = data[0].to(device), data[1].to(device)
        outputs = model(images).squeeze()
        loss = criterion(outputs, labels)
        total_loss += loss.item()
        acc += sum(torch.argmax(outputs, dim=1) == labels).item()
        preds.append(outputs.cpu().detach().numpy())
        trues.append(labels.cpu().detach().numpy())
        # for pred, true in zip(preds, trues):
        #     print(f"Predicted: {pred}, Actual: {true}")
    total_loss /= len(loader)  # Calculate average validation loss
    total_acc = acc / len(dataset)
print(f"Total acc: {total_acc}")