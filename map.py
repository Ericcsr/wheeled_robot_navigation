import numpy as np
from a_star import Point
from cv2 import imread
import cv2

class RandomMap:
    def __init__(self, img):
        self.size  = size
        self.img   = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.sizeX = len(self.img[0])
        self.sizeY = len(self.img)

    def IsObstacle(self, x ,y):
        return self.img[y][x] != 255