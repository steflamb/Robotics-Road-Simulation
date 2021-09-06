import sys
sys.path.extend(['..'])
import torch.utils.data
import torch.utils.data as data_utils
import torchvision.transforms as transforms
from sklearn.model_selection import train_test_split
from prepare_data import read_data
class CustomDataset(data_utils.Dataset):
    """
    Custom dataset
    Arguments:
    Returns:
    """

    def __init__(self, X, y=None, is_training=True, test_ids=None):
        self.is_training = is_training
        self.X = X
        self.y = y
        self.test_ids = test_ids

        self.transforms_train = transforms.Compose([
            transforms.ToTensor()])

        self.transforms_test = transforms.Compose([
            transforms.ToTensor()])

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        item = {}

        # Get image and label
        img = self.X[idx]

        if self.is_training:
            img = self.transforms_train(img)
        else:
            img = self.transforms_test(img)

        item['img'] = img
        if self.y is not None:
            lab = self.y[idx]
            item['label'] = torch.tensor(lab).float()

        if self.test_ids is not None:
            im_id = self.test_ids[idx]
            item['id'] = im_id
        return item


class DataLoader:
    def __init__(self):
        
        train_X, train_y =  read_data()

        # split into training and validation
        self.train_X, self.val_X, self.train_y, self.val_y = train_test_split(train_X, train_y,
                                                                              test_size=0.1,
                                                                              random_state=42)

    def create_train_loader(self):
        self.dataset = CustomDataset(self.train_X, self.train_y)
        return torch.utils.data.DataLoader(
            self.dataset, batch_size=2, shuffle=True,
            num_workers=2, pin_memory=True)

    def create_val_loader(self):
        self.dataset = CustomDataset(self.val_X, self.val_y, False)
        return torch.utils.data.DataLoader(
            self.dataset, batch_size=2, shuffle=False,
            num_workers=2, pin_memory=True)
