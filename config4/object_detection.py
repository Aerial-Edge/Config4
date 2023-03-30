#!/usr/bin/etv python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import cv2
from cv_bridge import CvBridge
import cvzone
from cvzone.ColorModule import ColorFinder
import numpy as np
import time


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection') # call the constructor of the parent class
        self.subscription = self.create_subscription(Image, 'image_data', self.process_image, 1) # create a subscriber
        self.bridge = CvBridge() # create a bridge between OpenCV and ROS
        self.distance_publisher = self.create_publisher(Float64, 'distance', 1) # create a publisher
        self.position_publisher = self.create_publisher(Float64, 'position', 1) # create a publisher

    def process_image(self, msg): # callback function main processing function
        start_time = time.time()

        distance = 0.0 # create a message
        position = 0.0 # create a message
        
        myColorFinder = ColorFinder(False) # create a color finder object
        hsvVals = {'hmin': 104, 'smin': 162, 'vmin': 60, 'hmax': 115, 'smax': 255, 'vmax': 255}

        opencv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # convert the ROS message to an OpenCV image

        _, mask = myColorFinder.update(opencv_image, hsvVals)
        _, contours = cvzone.findContours(opencv_image, mask)

        if contours:
            f = 535
            W = 6.5
            w = np.sqrt(contours[0]['area']/np.pi) * 2
            distance = (W * f) / w
            #log distance
            self.get_logger().info(f"Distance: {distance:.3f}cm")

        processing_time = time.time() - start_time
        desired_fps = 1/30

        if processing_time > desired_fps:
            self.get_logger().warning(f"Processing time exceeded desired time: {processing_time:.3f}s > {desired_fps:.3f}s")

        self.publish_object_distance(distance) # Publish the distance
        self.publish_object_position(position) # Publish the position
    
    def publish_object_distance(self, distance): # callback function
        msg = Float64() # create a message
        msg.data = distance # fill in the message
        self.distance_publisher.publish(msg) # publish the message

    def publish_object_position(self, position): # callback function
        msg = Float64() # create a message
        msg.data = position # fill in the message
        self.position_publisher.publish(msg) # publish the message

def main(args=None): # args is a list of strings
    rclpy.init(args=args) # initialize the ROS client library

    node = ObjectDetection() # create a node

    rclpy.spin(node) #  wait for messages
    node.destroy_node() # destroy the node explicitly

    rclpy.shutdown() # shutdown the ROS client library

if __name__ == '__main__':
    main()
