import torch
import shutil
import os
import numpy as np
from torchvision import models
from PIL import Image
import torch
import numpy as np
import torchvision
import torchvision.transforms as transforms
import torch.nn as nn
import torch.nn.functional as F
import shutil
import os

class ModelCheckpoint:
    def __init__(self, weight_dir='./weights'):
        self.weight_dir = weight_dir
        self.filename = os.path.join(os.path.dirname(__file__), 'model_best.pth.tar')
        self.mfilename ='mmodel_best.pth.tar'
        self.qfilename = 'qmobilenet_model_best.pth.tar'
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    def save(self, model):
        save_dict = {'model': model.state_dict()}
        torch.save(save_dict, self.filename)

    def load(self, model, mobile=False, quantized=False):
        load_filename = self.mfilename if mobile else self.filename
        load_filename = self.qfilename if quantized else load_filename
        if os.path.isfile(load_filename):
            if mobile:
                model= torch.jit.load(load_filename)
            else:
                checkpoint = torch.load(load_filename, map_location=self.device)
                model.load_state_dict(checkpoint['model'])
        else:
            raise FileNotFoundError(f'No checkpoint found at {load_filename}')

        return model
class PredictModel:
    def __init__(self):
        model = models.densenet121(pretrained=False)
        num_ftrs = model.classifier.in_features
        model.classifier = nn.Linear(num_ftrs, 4)
        self.model = model
        model_checkpoint = ModelCheckpoint()
        self.model = model_checkpoint.load(self.model)
        self.soft = nn.Softmax(dim=1)
    def predictor(self, img):
        self.transforms = transforms.Compose([
            transforms.ToTensor()])
        img = self.transforms(img)
        pred = self.model(img.unsqueeze(0))
        _, pred = torch.max(pred,1)
        return pred
if __name__ == "__main__":
    predict = PredictModel()
    img = np.array(Image.open('./Images_robot_2/Image_4676.png'))
    print(predict.predictor(img))
