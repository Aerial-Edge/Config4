#!/usr/bin/etv python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import cv2
from cv_bridge import CvBridge
import cvzone
from cvzone.ColorModule import ColorFinder
import numpy as np
import time


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection') # call the constructor of the parent class
        self.subscription = self.create_subscription(Image, 'image_data', self.process_image, 10) # create a subscriber
        self.bridge = CvBridge() # create a bridge between OpenCV and ROS
        self.distance_and_position_publisher = self.create_publisher(Int32MultiArray, 'distance_and_pos', 10)

    def process_image(self, msg): # callback function main processing function
        start_time = time.time()

        distance = 0 # create a message
        position = 0 # create a message

        dist_and_pos =[0, 0, 0]  #Distance on[0] and x pos on[1] and y pos on[2]    
        
        myColorFinder = ColorFinder(False) # create a color finder object
        hsvVals = {'hmin': 72, 'smin': 25, 'vmin': 4, 'hmax': 144, 'smax': 170, 'vmax': 81}
        opencv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # convert the ROS message to an OpenCV image

        _, mask = myColorFinder.update(opencv_image, hsvVals)
        _, contours = cvzone.findContours(opencv_image, mask)

        if contours:
            f = 474
            W = 6.2
            w = np.sqrt(contours[0]['area']/np.pi) * 2
            dist_and_pos[0] = (W * f) / w
            #log distance
            self.get_logger().info(f"Distance: {distance:.3f}cm")

        processing_time = time.time() - start_time
        desired_fps = 1/30

        if processing_time > desired_fps:
            self.get_logger().warning(f"Processing time exceeded desired time: {processing_time:.3f}s > {desired_fps:.3f}s")

        self.publish_object_distance(distance) # Publish the distance
        self.publish_object_position(position) # Publish the position
    

    def publish_dist_and_pos(self, data):
        msg = Int32MultiArray()
        msg.data = data
        self.distance_and_position_publisher.publish(msg)

def main(args=None): # args is a list of strings
    rclpy.init(args=args) # initialize the ROS client library

    node = ObjectDetection() # create a node

    rclpy.spin(node) #  wait for messages
    node.destroy_node() # destroy the node explicitly

    rclpy.shutdown() # shutdown the ROS client library

if __name__ == '__main__':
    main()
