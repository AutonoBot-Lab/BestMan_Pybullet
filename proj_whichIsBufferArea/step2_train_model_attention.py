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
import torch
# get list of models
torch.hub.list('zhanghang1989/ResNeSt', force_reload=True)
# load pretrained models, using ResNeSt-50 as an example

import torchvision.transforms as transforms

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
        success_rate = 1 if success_rate >= 0.9 else 0
        success_rate = torch.tensor(success_rate).long()

        return inputs, success_rate

    def __len__(self):
        return len(self.data_files)

train_tf = transforms.Compose([
    transforms.ToTensor(),
    transforms.RandomHorizontalFlip(),
    transforms.RandomVerticalFlip()
])
val_tf = transforms.Compose([
    transforms.ToTensor(),
])
# create a dataloader
train_dataset = CustomDataset(
    "train",
    train_tf
)
val_dataset = CustomDataset(
    "validation",
    val_tf
)
print('-'*20 + '\n' + 'dataloader is constructed!')

train_loader = DataLoader(train_dataset, batch_size=32, shuffle=True)
val_loader = DataLoader(val_dataset, batch_size=32, shuffle=False)
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = torch.hub.load('zhanghang1989/ResNeSt', 'resnest50', pretrained=True).to(device)


model.fc = nn.Linear(2048, 2, bias=True).to(device)
model.conv1 = nn.Conv2d(2, 64, kernel_size=(7, 7), stride=(2, 2), padding=(3, 3), bias=False).to(device)
# Define a learning rate adjustment strategy
optimizer = optim.Adam(model.parameters(), lr=0.00001)
# scheduler = StepLR(optimizer, step_size=10, gamma=0.1)
criterion = nn.CrossEntropyLoss()

# Monitor the training loss and validation loss
best_val_loss = np.inf

print('-'*20 + '\n' + 'start training')
for epoch in range(100):
    model.train()
    running_loss = 0.0
    for i, data in enumerate(train_loader):
        inputs, labels = data[0].to(device), data[1].to(device)

        optimizer.zero_grad()

        outputs = model(inputs).squeeze()
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()

        running_loss += loss.item()

    running_loss /= len(train_loader)  # Calculate average training loss

    # validation
    model.eval()
    val_loss = 0.0
    with torch.no_grad():
        for data in val_loader:
            images, labels = data[0].to(device), data[1].to(device)
            outputs = model(images).squeeze()
            loss = criterion(outputs, labels)
            val_loss += loss.item()

        val_loss /= len(val_loader)  # Calculate average validation loss

    print(f'Epoch {epoch+1}, Training Loss: {running_loss}, Validation Loss: {val_loss}')

    if val_loss < best_val_loss:
        best_val_loss = val_loss
        torch.save(model.state_dict(), 'best_model.pth')

# scheduler.step()
