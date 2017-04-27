#!/usr/bin/env python
import rospy
import cv2
import geometry_msgs.msg
from apriltags.msg import AprilTagDetections
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from math import atan2

import pdb

class AprilTagMarker:

  def __init__(self):
    
    # Initialize image converter
    self.bridge = CvBridge()
    
    self.counter = 0
    self.old_x = 0 
    self.old_y = 0

    # Initialize raw image subscriber
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.imageCb)
    self.image = []

    # Initialize april tag publisher
    self.marker_pub = rospy.Publisher("/marked_april_tag", Image, queue_size = 1)
    self.marked_image = []
    self.pub_image = 0

    self.angle_pub = rospy.Publisher("/angle", Float32, queue_size = 1)

    # Initialize detection subscriber
    self.detection_sub = rospy.Subscriber("/apriltags/detections",AprilTagDetections,self.detectionCb)
    self.detection_result = AprilTagDetections()
    rospy.loginfo("Yay I am started")

  # Grab detection result, published from apriltag identifier
  def detectionCb(self,data):
    self.detection_result = data

  # Grab image and convert from Image message to openCV image
  def imageCb(self,data):
    try:
      self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

  # Publish modified ariltag image
  def publishImage(self):
    try:
      self.marker_pub.publish(self.bridge.cv2_to_imgmsg(self.image, "bgr8"))
    except CvBridgeError as e:
      print(e)    

  # Draw marker on the center of the apriltag
  def drawMarker(self):
    if len(self.detection_result.detections) == 0:
      return -1
    ########################
    ### INSERT CODE HERE ###
    ########################
    corners=self.detection_result.detections[0] 
    centerx=(corners.corners2d[0].x+corners.corners2d[1].x)/2
    centery=(corners.corners2d[0].y+corners.corners2d[3].y)/2
    centery=(corners.corners2d[0].y+corners.corners2d[3].y)/2
    centerz=(corners.corners2d[0].z+corners.corners2d[2].z)/2
    x = int(centerx/centerz)
    y = int(centery/centerz)
    cv2.circle(self.image,(x,y),10,(0,0,255),-1)
    self.angle_motion(x,y)
    self.publishImage()

  def angle_motion(self,x,y):
    self.counter += 1
    if(self.counter%10): 
      diff_x = x-self.old_x
      diff_y = y-self.old_y
      val = atan2(diff_y,diff_x)
      self.old_x = x
      self.old_y = y
      self.angle_pub.publish(val)

if __name__ == '__main__':

  # Init Node
  rospy.init_node('AprilTagMarker')
  # Initialize april tag class
  marker = AprilTagMarker()
  
  # Spin ROS
  rate = rospy.Rate(120) #hz
  while not rospy.is_shutdown(): 
    marker.drawMarker()
    rate.sleep()