import torch
import numpy as np
import torchvision
import torchvision.transforms as transforms
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torchvision import models
import shutil
import logging
from tqdm import tqdm
from data_loader import DataLoader

class Trainer:
    def __init__(self):
        self.terminal_width = shutil.get_terminal_size((80, 20)).columns

        logging.info(f' Loading Data '.center(self.terminal_width, '*'))
        data_loading = DataLoader()

        self.train_loader = data_loading.create_train_loader()
        self.val_loader = data_loading.create_val_loader()

        # Model
        logging.info(f' Model: {"MobileNet"} '.center(self.terminal_width, '*'))
        model = models.mobilenet_v2(pretrained=False)
        num_ftrs = model.classifier[1].in_features
        model.features[0][0] = nn.Conv2d(3, 32, kernel_size=(3, 3), stride=(2, 2), padding=(1, 1), bias=False)
        model.classifier[1] = nn.Linear(num_ftrs, 5)
        self.model = model
        logging.info(f'{self.model}\n')
        # Loss, Optimizer and LRScheduler
        self.criterion_class = nn.CrossEntropyLoss()
        # self.criterion = self.criterion_class + self.criterion_energy
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=0.001)

        self.scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(self.optimizer, factor=0.5,
                                                                    patience=11,
                                                                    verbose=True)
        self.start_epoch, self.min_val_loss = 1, None
        # Load checkpoint if training is to be resumed
    def _get_val_loss_and_err(self):
        self.model.eval()
        progbar = tqdm(self.val_loader)
        progbar.set_description("             ")
        losses_class, losses_energy = [], []
        labels, predictions = [], []
        for idx, item in enumerate(progbar):
            img = item['img']
            lab = item['label']
            lab_class = lab
            preds = self.model(img)
            loss_class = self.criterion_class(preds, torch.tensor(lab_class,dtype=torch.long))
            losses_class.append(loss_class.data.cpu().numpy())
        return np.mean(losses_class)
    def train(self):
        logging.info(f' Training '.center(self.terminal_width, '*'))
        for epoch in range(self.start_epoch, 100 + 1):
            logging.info(f'\n Epoch [{epoch}/{100}] '.center(self.terminal_width, 'x'))
            self.model.train()
            progbar = tqdm(self.train_loader)
            losses_class, losses_energy = [], []
            labels, predictions = [], []

            for idx, item in enumerate(progbar):
                img = item['img']
                lab = item['label']
                lab_class = lab
                # Forward + Backward + Optimize
                self.optimizer.zero_grad()
                preds = self.model(img)
                loss_class = self.criterion_class(preds, torch.tensor(lab_class,dtype=torch.long))
                labels.append(lab.data.cpu().numpy())
                predictions.append(preds.data.cpu().numpy())
                losses_class.append(loss_class.data.cpu().numpy())
                progbar.set_description(f"loss = {np.mean(losses_class):0.3f} ")

                loss = loss_class
                loss.backward()
                self.optimizer.step()
            val_loss= self._get_val_loss_and_err()
            train_loss = np.mean(losses_class)
            print(f'Training Loss: {train_loss:.4f}, ' f'Validation Loss: {val_loss:.4f}')
            self.scheduler.step(val_loss)
        self.writer.close()
if __name__ == "__main__":
    terminal_width = shutil.get_terminal_size((80, 20)).columns
    trainer = Trainer()
    trainer.train()
