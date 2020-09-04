#!/usr/bin/env python
from __future__ import print_function

import roslib
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2 as cv
print(cv.__version__)

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_bgr8",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/xtion/color/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      img = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = img.shape
    if cols > 60 and rows > 60 :
      cv.circle(img, (50,50), 10, 0,0,255)

    
    blue = np.mat(img[:, :, 0])
    green = np.mat(img[:, :, 1])
    red = np.mat(img[:, :, 2])

    blue_only = np.int16(blue) - np.int16(red) - np.int16(green)

    blue_only[blue_only < 0] = 0
    blue_only[blue_only >= 255] = 255

    blue_only = np.uint8(blue_only)

    kernel = np.ones((5, 5), np.uint8)

    imgCanny = cv.Canny(blue_only, 100, 150)  # edge detection

    imgDilation = cv.dilate(imgCanny, kernel, iterations=1)
    imgEroded = cv.erode(imgDilation, kernel, iterations=1)

    im2,contours, hierarchy = cv.findContours(imgEroded, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    cnt = max(contours, key=len)

    x, y, w, h = cv.boundingRect(cnt)

    rect = cv.minAreaRect(cnt)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    ang = rect[2]
    ang = np.abs(ang)
    print("ang_before", ang)
    leftmost = tuple(cnt[cnt[:, :, 0].argmin()][0])
    rightmost = tuple(cnt[cnt[:, :, 0].argmax()][0])
    topmost = tuple(cnt[cnt[:, :, 1].argmin()][0])
    bottommost = tuple(cnt[cnt[:, :, 1].argmax()][0])

    if ang < 8 or ang > 82:
        if w > h:
            x1 = x
            y1 = y + np.uint8(h / 2)
            x2 = x + w
            y2 = y1
            print("horizontal")
        else:
            y1 = y
            x1 = x + np.uint8(w / 2)
            x2 = x1
            y2 = y + h
            print("vertical")
    else:       

        if ang > 10 or ang < 80:
            if rightmost[1] - leftmost[1] >= 20:
                x1 = np.int0((leftmost[0] + topmost[0]) / 2)
                y1 = np.int0((leftmost[1] + topmost[1]) / 2)
                x2 = np.int0((rightmost[0] + bottommost[0]) / 2)
                y2 = np.int0((rightmost[1] + bottommost[1]) / 2)
                print("left up")
            else:
                if rightmost[0] > bottommost[0]:
                    x2 = np.int0((rightmost[0] + bottommost[0]) / 2)
                    y2 = np.int0((rightmost[1] + bottommost[1]) / 2)
                    x1 = np.int0((leftmost[0] + topmost[0]) / 2)
                    y1 = np.int0((leftmost[1] + topmost[1]) / 2)
                    print("right up 1") 

                else:
                    x1 = np.int0((rightmost[0] + topmost[0]) / 2)
                    y1 = np.int0((rightmost[1] + topmost[1]) / 2)
                    x2 = np.int0((leftmost[0] + bottommost[0]) / 2)
                    y2 = np.int0((leftmost[1] + bottommost[1]) / 2)
                    print("right up 2") 
                         
            
        else:             
            if w > h:
                x1 = x
                y1 = y + np.uint8(h / 2)
                x2 = x + w
                y2 = y1
                print("horizontal 2")
            else:
                y1 = y
                x1 = x + np.uint8(w / 2)
                x2 = x1
                y2 = y + h
                print("vertical 2")           

    print("ang", ang)
    print("leftmost:", leftmost, "rightmost:", rightmost, "topmost:", topmost, "bottommost:", bottommost, "\n")
    print("x1, y1",x1,y1 ,"x2,y2", x2,y2)
    
    print("box", box)

    # cv.drawContours(imgEroded, [box], 0, (255, 0, 255), 2)
    # print("x:", x, "y:", y, "w:", w, "h:", h, "\n")
    #
    #

    cv.circle(imgEroded, (x1, y1), 10, (255, 0, 0), 2)
    cv.circle(imgEroded, (x2, y2), 10, (255, 0, 0), 2)

    # cv.drawContours(img, contours, 0, (255, 255, 0), 5)
    cv.drawContours(img, cnt, -1, (0, 255, 255), 5)

    # cv.imshow("Orig", img)
    cv.imshow("Eroded Image", imgEroded)    
    cv.waitKey(1)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)