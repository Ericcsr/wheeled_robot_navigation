import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import sys

if len(sys.argv) == 2:
    fname = sys.argv[1]
else:
    print('Usage:python3 extract_prob.py <filename>')

image = cv.imread('maps/'+fname+'.bmp')
image = cv.cvtColor(image,cv.COLOR_BGR2GRAY)
thresh = cv.inRange(image,50,240)
thresh1 = np.clip(thresh.astype('int32')+50,0,255)
cv.imshow('image',thresh)
cv.waitKey(0)
xP = thresh1.flatten()/np.sum(thresh1)
mapp = np.zeros(image.shape,dtype=np.int)
x = np.arange(image.shape[0]*image.shape[1])
blacks_x = np.random.choice(x,size=4000,replace=True,p=xP)
for x in blacks_x:
    mapp[x//image.shape[1],x%image.shape[1]] = 255
plt.imshow(mapp)
plt.show()
np.save('distributions/'+fname,thresh1)
