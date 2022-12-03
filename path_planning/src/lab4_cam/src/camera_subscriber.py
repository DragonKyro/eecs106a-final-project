#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Image, PointCloud2
from lab4_cam.srv import ImageSrv, ImageSrvResponse



pub = rospy.Publisher('publisher_data', Image, queue_size=10)


def logitechRecevied(message):
  #Save the image in the instance variable
  logitechRGB = message
  print("received logitech")
  #Print an alert to the console
  #print(rospy.get_name() + ":Image received!")

def realsenseRGBReceived(message):
  realsenseRGB = message
  print("received realsenseRGB")
  pub.publish(realsenseRGB)

def realsenseDepthReceived(message):
  realsenseDepth = message
  print("received realsenseDepth")

def pointcloudReceived(message):
  realsensePointCloud = message
  print("received realsensePointCloud")

#When another node calls the service, return the last image
def getCameraData(request):
  #Print an alert to the console
  #print("Image requested!")

  #Return the last image
  return [logitechRGB, realsenseRGB, realsenseDepth, realsensePointCloud]


#INSERT ROSPY.PUBLISHER CODE HERE TO TRY AND SEE IF YOU CAN SUBSCRIBE AND PUBLISH AT THE SAME TIME

  # def __init__(self):
  #   #Create an instance variable to store the last image received
  #   self.logitechRGB = None;
  #   self.realsenseRGB = None;
  #   self.realsenseDepth = None;
  #   self.realsensePointCloud = None;

  #   #Initialize the node
  #   rospy.init_node('cam_subscriber')
  #   listener()


    #Create the service
    #rospy.Service('images_and_pointcloud', ImageSrv, self.getCameraData)

def listener():
  #rgb values from logitech webcam
  rospy.Subscriber("/usb_cam/image_raw", Image, logitechRecevied)

  #rgb values from realsense
  rospy.Subscriber("/camera/color/image_raw", Image, realsenseRGBReceived)

  #depth values from realsense
  rospy.Subscriber("/camera/depth/image_raw", Image, realsenseDepthReceived)

  #pointcloud from realsense
  rospy.Subscriber("/camera/depth/color/points", PointCloud2, pointcloudReceived)

  rospy.spin()

  def run(self):
    rospy.spin()

#Python's syntax for a main() method
if __name__ == '__main__':
  logitechRGB = None
  realsenseRGB = None
  realsenseDepth = None
  realsensePointCloud = None

  rospy.init_node('cam_subscriber')
  listener()

  r = rospy.Rate(10)
  while not rospy.is_shutdown():
    cameraData = getCameraData()
    pub.publish(cameraData[2])
    r.sleep()
