#-------------------------------------------------------------------------------
# Name:        main
# Purpose:     Testing the package pySaliencyMap
#
# Author:      Akisato Kimura <akisato@ieee.org>
#
# Created:     May 4, 2014
# Copyright:   (c) Akisato Kimura 2014-
# Licence:     All rights reserved
#-------------------------------------------------------------------------------

import cv2
import matplotlib.pyplot as plt
import pySaliencyMap
import numpy as np

def return_signal(img):
    # read


    img = cv2.GaussianBlur(img,(3,3),0)
    # initialize
    imgsize = img.shape
    img_width  = imgsize[1]
    img_height = imgsize[0]
    sm = pySaliencyMap.pySaliencyMap(img_width, img_height)
    # computation
    saliency_map = sm.SMGetSM(img)


    binarized_map = sm.SMGetBinarizedSM(img)


    B, G, R = cv2.split(img)
    b = (B*(binarized_map/255)).astype(np.uint8)
    g = (G*(binarized_map/255)).astype(np.uint8)
    r = (R*(binarized_map/255)).astype(np.uint8)
    T = cv2.merge((b,g,r))


    xor = cv2.bitwise_xor(b, r)
    _, thr1 = cv2.threshold(xor, 60, 255, cv2.THRESH_BINARY)
    b_a = (B*(thr1/255)).astype(np.uint8)
    g_a = (G*(thr1/255)).astype(np.uint8)
    r_a = (R*(thr1/255)).astype(np.uint8)

    _, r_t = cv2.threshold( r_a, 100, 255, cv2.THRESH_TOZERO)
    _, g_t = cv2.threshold( g_a, 100, 255, cv2.THRESH_TOZERO)
    _, b_t = cv2.threshold( b_a, 100, 255, cv2.THRESH_TOZERO)

    sd_r = cv2.GaussianBlur(r_t,(11,11),0)
    _, thrsd_r = cv2.threshold(sd_r, 60, 255, cv2.THRESH_BINARY)

    sd_b = cv2.GaussianBlur(b_t,(11,11),0)
    _, thrsd_b = cv2.threshold(sd_b, 60, 255, cv2.THRESH_BINARY)

    contours_r, _ = cv2.findContours(thrsd_r, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_b, _ = cv2.findContours(thrsd_b, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    S_2 = 0
    w_r = 0
    h_r = 0
    location_r=[]
    if len(contours_r) != 0:
        c_r = max(contours_r, key = cv2.contourArea)
        x_r,y_r,w_r,h_r = cv2.boundingRect(c_r)
        location_r = [(2*x_r+w_r)/2, (2*y_r+h_r)/2]
        #cv2.rectangle(img,(x_r,y_r),(x_r+w_r,y_r+h_r),(255,255,255),2)



    S_1 = 0
    w_b = 0
    h_b = 0
    location_b=[]
    if len(contours_b) != 0:
        c_b = max(contours_b, key = cv2.contourArea)
        x_b,y_b,w_b,h_b = cv2.boundingRect(c_b)
        location_b = [(2*x_b+w_b)/2, (2*y_b+h_b)/2]
        cv2.rectangle(img,(x_b,y_b),(x_b+w_b,y_b+h_b),(255,255,255),2)




        # visualize
    # plt.subplot(2,2,1), plt.imshow(img, 'gray')
    # plt.subplot(2,2,1), plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    # # plt.title('Input image')
    # # cv2.imshow("input",  img)
    # plt.subplot(2,2,2), plt.imshow(b_a, 'gray')
    # plt.title('Saliency map')
    # # cv2.imshow("output", map)
    # plt.subplot(2,2,3), plt.imshow(cv2.cvtColor(cv2.merge((b_t, g_t, r_t)), cv2.COLOR_BGR2RGB))
    # plt.title('Binarized saliency map')
    # # cv2.imshow("Binarized", binarized_map)
    # plt.subplot(2,2,4), plt.imshow(cv2.cvtColor(salient_region, cv2.COLOR_BGR2RGB))
    # plt.title('Salient region')
    # # cv2.imshow("Segmented", segmented_map)

    # plt.show()

    return (w_b, h_b, w_r, h_r, location_b, location_r, sd_b)
