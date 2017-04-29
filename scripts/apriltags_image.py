#!/usr/bin/env python
import rospy
import cv2
import geometry_msgs.msg
from apriltags.msg import AprilTagDetections
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from math import atan2
from math import sqrt
from april_tracker.msg import pose

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

    self.pose_pub = rospy.Publisher("/pose", pose, queue_size = 1)

    # Initialize detection subscriber
    self.detection_sub = rospy.Subscriber("/apriltags/detections",AprilTagDetections,self.detectionCb)
    self.detection_result = AprilTagDetections()
    rospy.loginfo("Yay I am started")

  # Grab detection result, published from apriltag identifier
  def detectionCb(self,data):
    self.detection_result = data
    self.drawMarker(data)
    self.drawArrow(data)

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
  def drawMarker(self, data):
    if len(self.detection_result.detections) == 0:
      return -1
    ########################
    ### INSERT CODE HERE ###
    ########################
    for i in range(len(self.detection_result.detections)):
      print i
      corners=self.detection_result.detections[i] 
      centerx=(corners.corners2d[0].x+corners.corners2d[1].x)/2
      centery=(corners.corners2d[0].y+corners.corners2d[3].y)/2
      centery=(corners.corners2d[0].y+corners.corners2d[3].y)/2
      centerz=(corners.corners2d[0].z+corners.corners2d[2].z)/2
      x = int(centerx/centerz)
      y = int(centery/centerz)
      cv2.circle(self.image,(x,y),10,(0,0,255),-1)
    self.publishImage()

  def drawArrow(self,data):
    if len(data.detections) == 0:
      return -1
    tag0 = ()
    tag1 = ()
    tag0_set = False
    tag1_set = False
    for i in range(len(data.detections)):
      tag = data.detections[i]
      if(tag.id == 2):
        tag0_set = True
        centerx=(tag.corners2d[0].x+tag.corners2d[1].x)/2
        centery=(tag.corners2d[0].y+tag.corners2d[3].y)/2
        centery=(tag.corners2d[0].y+tag.corners2d[3].y)/2
        centerz=(tag.corners2d[0].z+tag.corners2d[2].z)/2
        # Transform to arm base frame
        centerx = centerx - 100
        centery = centery - 86
        tag0x = int(centerx/centerz)
        tag0y = int(centery/centerz)
        tag0 = (tag0x,tag0y)
      if(tag.id == 1):
        tag1_set = True
        centerx=(tag.corners2d[0].x+tag.corners2d[1].x)/2
        centery=(tag.corners2d[0].y+tag.corners2d[3].y)/2
        centery=(tag.corners2d[0].y+tag.corners2d[3].y)/2
        centerz=(tag.corners2d[0].z+tag.corners2d[2].z)/2
        tag1x = int(centerx/centerz)
        tag1y = int(centery/centerz)
        tag1 = (tag1x,tag1y)
    if tag0_set and tag1_set:
      #cv2.circle(self.image,tag1,20,(0,255,0),-1)
      cv2.line(self.image, tag0, tag1,(255,0,0),5)
      self.angle_motion(tag0, tag1)
    self.publishImage()

  def angle_motion(self,arm,anki):
    self.counter += 1
    if(self.counter): 
      diff_x = anki[0] - arm[0]#x-self.old_x
      diff_y = anki[1] - arm[1]#y-self.old_y
      val = atan2(diff_x,diff_y)
      dist = sqrt((diff_x*diff_x)+(diff_y*diff_y))
      msg = pose()
      msg.angle = val
      msg.dist = dist
      self.pose_pub.publish(msg)

if __name__ == '__main__':

  # Init Node
  rospy.init_node('AprilTagMarker')
  # Initialize april tag class
  marker = AprilTagMarker()
  
  # Spin ROS
  rate = rospy.Rate(10) #hz
  while not rospy.is_shutdown(): 
    
    rate.sleep()