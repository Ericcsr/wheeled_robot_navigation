import numpy as np
from cv2 import imread
import cv2

class Point:
    def __init__(self, x, y):
        self.x      = x
        self.y      = y
        self.cost   = float('inf')
        self.parent = None

class RandomMap:
    def __init__(self, img):
        self.img   = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.sizeX = len(self.img[0])
        self.sizeY = len(self.img)

    def IsObstacle(self, x ,y):
        return self.img[y][x] != 255