#!/usr/bin/etv python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
import cv2 as cv
from cv_bridge import CvBridge
import cvzone
from cvzone.FPS import FPS
import numpy as np
import time
import math
import imutils


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection') # call the constructor of the parent class
        self.subscription = self.create_subscription(Image, 'image_data', self.process_image, 10) # create a subscriber
        self.bridge = CvBridge() # create a bridge between OpenCV and ROS
        self.distance_and_position_publisher = self.create_publisher(Int32MultiArray, 'distance_and_pos', 10)



    def process_image(self, msg): # callback function main processing function
        
        start_time = time.time()
   
        # Function to calculate distance from the object based on its radius in pixels
        def calculateDistance(ballRadius_px):
            return int(faktor / ballRadius_px)

        # Function to detect a colored object within a given color range and size
        def detect_colored_object(colorLower, colorUpper, min_radius, max_radius):
            
            gray = cv.cvtColor(opencv_image, cv.COLOR_BGR2GRAY)
            gray = cv.medianBlur(gray, 5)
            mask = cv.erode(gray, None, iterations=2)
            mask = cv.dilate(mask, None, iterations=2)
            
            # Detect circles using the Hough transform
            circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, 20, param1=100, param2=30,
                                    minRadius=min_radius, maxRadius=max_radius)

            # Check if any circles were detected
            if circles is not None:
                circles = np.uint16(np.around(circles))

                for circle in circles[0, :]:
                    x, y, radius = circle

                    # Check if the circle's center is within the mask's boundaries
                    if 0 <= x < mask.shape[1] and 0 <= y < mask.shape[0] and mask[y, x] > 0:
                        return (x, y, radius)
            
            
            return None
        
        # Distance measurement parameters:
        ballRadius = 3.25   # cm (radius of the ball)
        cameraFOV = 62.2    # degrees (field of view of the camera)
        faktor = (640 / 2) * (ballRadius / math.tan(math.radians(cameraFOV / 2)))

        # Color detection settings:
        colors = {
             'green': {
                'lower': (57, 45, 18), #(L-H, L-S, L-V)
                'upper': (113, 191, 125), #(U-H,  U-S, u-V)
                'min_radius': 0, #ex between 20 
                'max_radius': 0, #to 60 pixels
                'color': (0, 255, 0), #Color of the circle around object
                'text_offset': 0, #Distance text position under FPS
            },
            'orange': {
                'lower': (0, 115, 99), #(L-H, L-S, L-V)
                'upper': (18, 255, 255), #(U-H,  U-S, u-V)
                'min_radius': 0,
                'max_radius': 0,
                'color': (0, 102, 255), #Color of the circle around object
                'text_offset': 20, #Distance text position under FPS
            },
            'red': {
                'lower': (119, 37, 0), #(L-H, L-S, L-V)
                'upper': (179, 179, 147), #(U-H,  U-S, u-V)
                'min_radius': 0,
                'max_radius': 0,
                'color': (0, 0, 255), #Color of the circle around object
                'text_offset': 40,  #Distance text position under FPS
            },
        }

        #fpsreader = FPS() # Initialize FPS reader
        
        # Main loop
       
        #(grabbed, frame) = videoCap.read() # Read a frame from the video capture
        opencv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # convert the ROS message to an OpenCV image
        #fps, opencv_image = fpsreader.update(opencv_image, color=(255, 0, 0)) # Update the FPS overlay on the frame
        #frame = imutils.resize(frame, width=1280) # Resize the frame
        hsv = cv.cvtColor(opencv_image, cv.COLOR_BGR2HSV) # Convert the frame to HSV format

        # Initialize x, y, and ballRadius_px values for each color
        for color_info in colors.values():
            color_info['x'] = None
            color_info['y'] = None
            color_info['ballRadius_px'] = None

        # Iterate through the defined colors and detect objects
        for idx, (color_name, color_info) in enumerate(colors.items()):
            # Call the detect_colored_object function to find objects in the frame
            obj = detect_colored_object(color_info['lower'], color_info['upper'], color_info['min_radius'],
                                        color_info['max_radius'])

            if obj: # If an object is detected
                x, y, ballRadius_px = obj # Get the coordinates and radius of the object
                dist = calculateDistance(ballRadius_px) # Calculate the distance to the object
                #self.get_logger().info(f" FPS : {fps}")

                self.publish_dist_and_pos(x, y, dist)
                self.get_logger().info(f" X = {x}, Y = {y}, Distance = {dist}")

                # Update the color_info dictionary with the new values
                color_info['x'] = x
                color_info['y'] = y
                color_info['ballRadius_px'] = ballRadius_px
                color_info['distance'] = dist
                
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
