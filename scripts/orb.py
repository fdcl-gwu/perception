import numpy as np
import cv2
from matplotlib import pyplot as plt
import pdb

img = cv2.imread('template2.png',0)
# cv2.imshow('test',img)
# Initiate STAR detector
orb = cv2.ORB_create()

# find the keypoints with ORB
kp = orb.detect(img,None)

# pdb.set_trace()
# compute the descriptors with ORB
kp, des = orb.compute(img, kp)

draw_params = dict(matchColor = (0,255,0),
                   singlePointColor = (255,0,0),
                   flags = 0)


# img2 = np.zeros((height,width,3), np.uint8)
# draw only keypoints location,not size and orientation

img2 = cv2.drawKeypoints(img,kp,None,color=(0,255,0),flags=0)
plt.imshow(img2),plt.show()
