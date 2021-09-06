import sys
sys.path.extend(['..'])
import numpy as np
from PIL import Image
from tqdm import tqdm
import glob



def read_data():
    """
    Saves dictionary of preprocessed images and labels for the required partition
    """

    # read data and get labels
    print('Training Data:')
    train_X, train_y = [], []
    for im_path in glob.glob('./Images_robot/*.*'):
        img = np.array(Image.open(im_path))
        # img = crop_center(img, img_w, img_h)
        # img = cv2.resize(img, (img_w, img_h))
        class_str = im_path[im_path.find('/Images_robot/') + 14: im_path.find('/Images_robot/') +16]
        label_class = 0 if class_str == 'sl' else 1 if class_str == 'st' else 2 if class_str == 'nl' else 3
        train_X.append(img)
        train_y.append(label_class)

    train_X = np.array(train_X)
    train_y = np.array(train_y)
    return train_X, train_y
if __name__ == '__main__':
    print('Processing Data:\n')
    print('\nData processing completed')
