# import the necessary packages
import numpy as np
import argparse
import glob
import cv2
import pdb
from matplotlib import pyplot as plt
cap = cv2.VideoCapture('test1.mp4')
img1 = cv2.imread('template.png',0)
# img1 = cv2.Canny(img1,80,200)
# Initiate SIFT detector
sift = cv2.ORB_create()


while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # Our operations on the frame come here
    # rgb_im = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)
    rgb_im = frame
    rgb_im = cv2.resize(rgb_im,None,fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
    gray_im = cv2.cvtColor(rgb_im,cv2.COLOR_BGR2GRAY)
    gray_im = cv2.resize(gray_im,None,fx=0.7, fy=0.7, interpolation = cv2.INTER_CUBIC)
    gray = np.float32(gray_im)
    dim = gray.shape
    rgb = cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)

    img = np.zeros((dim[0],dim[1],3), np.uint8)
    dst = cv2.cornerHarris(gray,2,3,0.04)
    dst = cv2.dilate(dst,None)
    rgb[dst>0.01*dst.max()]=[0,0,255]
    # cv2.imshow('dst',img)
    # Display the resulting frame
    # sift = cv2.SIFT()
    # kp = sift.detect(gray,None)
    # Match descriptors.
    # gray_im = cv2.Canny(gray_im,80,200)

    kp2 = sift.detect(gray_im,None)
    if len(kp2)>10:
        kp2, des2 = sift.compute(gray_im,kp2)
        pdb.set_trace()
    if len(kp2)<0:
        kp1, des1 = sift.compute(img1,None)
        # kp2, des2 = sift.detectAndCompute(gray_im,None)

        # FLANN parameters
        FLANN_INDEX_KDTREE = 0
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=50)   # or pass empty dictionary

        flann = cv2.FlannBasedMatcher(index_params,search_params)

        matches = flann.knnMatch(des1,des2,k=2)

        # Need to draw only good matches, so create a mask
        matchesMask = [[0,0] for i in xrange(len(matches))]

        # ratio test as per Lowe's paper
        for i,(m,n) in enumerate(matches):
            if m.distance < 0.7*n.distance:
                matchesMask[i]=[1,0]

        draw_params = dict(matchColor = (0,255,0),
                           singlePointColor = (255,0,0),
                           matchesMask = matchesMask,
                           flags = 0)

        img3 = cv2.drawMatchesKnn(img1,kp1,gray_im,kp2,matches,None,**draw_params)
        cv2.imshow('test',img3)

    # edges = cv2.Canny(rgb,80,200)
    # cv2.drawKeypoints(gray_im,kps,rgb)
    # cv2.imshow('edge',gray_im)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
