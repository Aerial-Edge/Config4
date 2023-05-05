#!/usr/bin/etv python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import cv2
from cv_bridge import CvBridge
import cvzone
from cvzone.FPS import FPS
from cvzone.ColorModule import ColorFinder
import numpy as np
import time
import math

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection') # call the constructor of the parent class
        self.subscription = self.create_subscription(Image, 'image_data', self.process_image, 10) # create a subscriber
        self.bridge = CvBridge() # create a bridge between OpenCV and ROS
        self.fpsreader = FPS() # Initialize FPS reader
        self.distance_and_position_publisher = self.create_publisher(Int32MultiArray, 'distance_and_pos', 10)

    def is_circle(self, cnt, threshold=0.6):
            area = cv2.contourArea(cnt)
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:
                return False
            circularity = 4 * np.pi * area / (perimeter * perimeter)
            return circularity >= threshold


    def process_image(self, msg): # callback function main processing function

        start_time = time.time()
        opencv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        

        h, w, _ = opencv_image.shape

        myColorFinder: ColorFinder = ColorFinder(False)
        hsvValsBlue = {'hmin': 105, 'smin': 168, 'vmin': 119, 'hmax': 111, 'smax': 255, 'vmax': 255} #blue
        hsvValsGreen = {'hmin': 76, 'smin': 29, 'vmin': 132, 'hmax': 97, 'smax': 124, 'vmax': 255} #grønn
        hsvValsOrange = {'hmin': 0, 'smin': 120, 'vmin': 120, 'hmax': 20, 'smax': 255, 'vmax': 255} #orange

        fps, img = self.fpsreader.update(opencv_image)

        _, mask = myColorFinder.update(img, hsvValsBlue)
        _, maskRed = myColorFinder.update(img, hsvValsGreen)
        _, maskOrange = myColorFinder.update(img, hsvValsOrange)


        _, contours = cvzone.findContours(img, mask)
        _, contoursRed = cvzone.findContours(img, maskRed)
        _, contoursOrange = cvzone.findContours(img, maskOrange)

        # Filter contours that are circles
        circular_contours_blue = [cnt for cnt in contours if self.is_circle(cnt['cnt'])]
        circular_contours_green = [cnt for cnt in contoursRed if self.is_circle(cnt['cnt'])]
        circular_contours_orange = [cnt for cnt in contoursOrange if self.is_circle(cnt['cnt'])]

        # Process and display depth, x, and y position for each ball
        for color, circular_contours_list in zip(['blue', 'green', 'orange'],
                                                [circular_contours_blue, circular_contours_green, circular_contours_orange]):
            if circular_contours_list:
                cnt = circular_contours_list[0]
                #data = cnt['center'][0], h - cnt['center'][1], int(cnt['area'])
                x, y = cnt['center']

                f = 474
                W = 6.5
                w = np.sqrt(cnt['area'] / np.pi) * 2
                d = (W * f) / w
                self.get_logger().info(f"{color}: {d}")
                self.get_logger().info(f" fps: {fps}")

                self.publish_dist_and_pos(x , y, d)
        
        
        computaiton_time = time.time() - start_time
        self.get_logger().info("Time to compute: " + str(computaiton_time))

    def publish_dist_and_pos(self, x, y, distance):
        msg = Int32MultiArray()
        msg.data = [int(x), int(y), int(distance)]
        self.distance_and_position_publisher.publish(msg)

def main(args=None): # args is a list of strings
    rclpy.init(args=args) # initialize the ROS client library

    node = ObjectDetection() # create a node

    rclpy.spin(node) #  wait for messages
    node.destroy_node() # destroy the node explicitly

    rclpy.shutdown() # shutdown the ROS client library

if __name__ == '__main__':
    main()
