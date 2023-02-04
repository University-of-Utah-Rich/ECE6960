# Author: Rich Baird
# Date: 2020-05-01
# Description: This file contains the functions used to open and manipulate images


import os
from PIL import Image
from math import floor

# img_iter returns an iterator that can be used to iterate over images in a directory
def img_iter(img_dir):
    return __Iterator(img_dir)
# open_image opens an image from a path
def open_image(img_path):
    return Image.open(img_path)

# resize_image resizes an image to a given size
# The default size is 32x32
def resize_image(img, size=(32, 32)):
    img.thumbnail(size)
    return img

# scale_intensity scales the intensity of an image
# The default intensity is 0.25
def scale_intensity(img, intensity = 0.25):
    if intensity >= 1 or intensity < 0:
        return img
    return img.point(lambda x: floor(x * 256 * intensity))


# __Iterator is a private class that is used to iterate over images in a directory
# It is used by img_iter
# It is not intended to be used directly
class __Iterator:
    # __open_image is a private function that is used to open an image
    def __open_image(self, img_path):
        img = open_image(img_path)
        return img
    # __init__ initializes the iterator
    def __init__(self, img_dir):
        self.img_formats = (".jpg", ".png", ".jpeg", ".bmp", ".gif")
        self.img_dir = img_dir
        # Get a list of all images in the directory
        # Only include images that have a valid extension
        self.imgs = [img for img in os.listdir(img_dir) if img.endswith(self.img_formats) ]
        self.index = 0
    # __iter__ returns the iterator
    def __iter__(self):
        return self
    # __next__ returns the next image in the directory
    def __next__(self):
        if self.index >= len(self.imgs):
            raise StopIteration
        img = self.imgs[self.index]
        try:
            img = self.__open_image(os.path.join(self.img_dir, img))
        except:
            # If the image fails to open, skip it and try the next one
            print(f"Failed to open {img}")
            self.index += 1
            return self.__next__()
        self.index += 1
        return img
    # __len__ returns the number of images in the directory
    def __len__(self):
        return len(self.imgs)

