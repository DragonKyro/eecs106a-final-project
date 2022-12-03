#!/usr/bin/env python
import rospy

from sensor_msgs.msg import Image, PointCloud2
from lab4_cam.srv import ImageSrv, ImageSrvResponse


class ImgService:
  #Callback for when an image is received
  def logitechRecevied(self, message):
    #Save the image in the instance variable
    self.logitechRGB = message
    print("received logitech")
    #Print an alert to the console
    #print(rospy.get_name() + ":Image received!")

  def realsenseRGBReceived(self, message):
    self.realsenseRGB = message
    print("received realsenseRGB")
  
  def realsenseDepthReceived(self, message):
    self.realsenseDepth = message
    print("received realsenseDepth")
  
  def pointcloudReceived(self,message):
    self.realsensePointCloud = message
    print("received realsensePointCloud")

  #When another node calls the service, return the last image
  def getCameraData(self, request):
    #Print an alert to the console
    #print("Image requested!")

    #Return the last image
    return ImageSrvResponse(self.logitechRGB, self.realsenseRGB, self.realsenseDepth, self.realsensePointCloud) 
    #WHAT IS THIS RESPONSE THING??????????



  def __init__(self):
    #Create an instance variable to store the last image received
    self.logitechRGB = None;
    self.realsenseRGB = None;
    self.realsenseDepth = None;
    self.realsensePointCloud = None;

    #Initialize the node
    rospy.init_node('cam_listener2')

    #rgb values from logitech webcam
    rospy.Subscriber("/usb_cam/image_raw", Image, self.logitechRecevied)

    #rgb values from realsense
    rospy.Subscriber("/camera/color/image_raw", Image, self.realsenseRGBReceived)

    #depth values from realsense
    rospy.Subscriber("/camera/depth/image_raw", Image, self.realsenseDepthReceived)

    #pointcloud from realsense
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.pointcloudReceived)

    #Create the service
    rospy.Service('images_and_pointcloud', ImageSrv, self.getCameraData)

  def run(self):
    rospy.spin()

#Python's syntax for a main() method
if __name__ == '__main__':
  node = ImgService()
  node.run()
