# MIT License
# Copyright (c) 2019-2022 JetsonHacks

# A simple code snippet
# Using two  CSI cameras (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit with two CSI ports (Jetson Nano, Jetson Xavier NX) via OpenCV
# Drivers for the camera and OpenCV are included in the base image in JetPack 4.3+

# This script will open a window and place the camera stream from each camera in a window
# arranged horizontally.
# The camera streams are each read in their own thread, as when done sequentially there
# is a noticeable lag

import cv2
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge

class CSI_Camera:

    def __init__(self):
        # Initialize instance variables
        # OpenCV video capture element
        self.video_capture = None
        # The last captured image from the camera
        self.frame = None
        self.grabbed = False
        # The thread where the video capture runs
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False

    def open(self, gstreamer_pipeline_string):
        try:
            self.video_capture = cv2.VideoCapture(
                gstreamer_pipeline_string, cv2.CAP_GSTREAMER
            )
            # Grab the first frame to start the video capturing
            self.grabbed, self.frame = self.video_capture.read()

        except RuntimeError:
            self.video_capture = None
            print("Unable to open camera")
            print("Pipeline: " + gstreamer_pipeline_string)


    def start(self):
        if self.running:
            print('Video capturing is already running')
            return None
        # create a thread to read the camera image
        if self.video_capture != None:
            self.running = True
            self.read_thread = threading.Thread(target=self.updateCamera)
            self.read_thread.start()
        return self

    def stop(self):
        self.running = False
        # Kill the thread
        self.read_thread.join()
        self.read_thread = None

    def updateCamera(self):
        # This is the thread to read images from the camera
        while self.running:
            try:
                grabbed, frame = self.video_capture.read()
                with self.read_lock:
                    self.grabbed = grabbed
                    self.frame = frame
            except RuntimeError:
                print("Could not read image from camera")
        # FIX ME - stop and cleanup thread
        # Something bad happened

    def read(self):
        with self.read_lock:
            frame = self.frame.copy()
            grabbed = self.grabbed
        return grabbed, frame

    def release(self):
        if self.video_capture != None:
            self.video_capture.release()
            self.video_capture = None
        # Now kill the thread
        if self.read_thread != None:
            self.read_thread.join()


class CameraNode(Node):
    def __init__(self, **kwargs):
        super().__init__("stereocamera_node")
        left_image_topic_ = self.declare_parameter(
            "left_image_topic", "/image/left/image_compressed").value
        right_image_topic_ = self.declare_parameter(
            "right_image_topic", "/image/right/image_compressed").value
        self.left_frame_id_ = self.declare_parameter("left_frame_id", "left_camera").value
        self.right_frame_id_ = self.declare_parameter("right_frame_id", "right_camera").value
        self.left_image_publisher_ = self.create_publisher(CompressedImage, left_image_topic_, 1)
        self.right_image_publisher_ = self.create_publisher(CompressedImage, right_image_topic_, 1)
        self.br = CvBridge()

        self.get_logger().info(
            f"Starting publishers")
        
        self.start_cameras()

        self.timer = self.create_timer(1.0/20, self.image_callback)

    def close_videocapture(self):
        self.left_camera.stop()
        self.left_camera.release()
        self.right_camera.stop()
        self.right_camera.release()

    def image_callback(self):
        try:
            time_msg = self.get_time_msg()
            if self.left_camera.video_capture.isOpened():
                _, left_image = self.left_camera.read()
                left_img_msg = self.get_image_msg(left_image, time_msg)
                self.left_image_publisher_.publish(left_img_msg)

            if self.right_camera.video_capture.isOpened():
                _, right_image = self.right_camera.read()
                right_img_msg = self.get_image_msg(right_image, time_msg, False)
                self.right_image_publisher_.publish(right_img_msg)

        except Exception as e:
            print(e)

    def get_time_msg(self):
        time_msg = Time()
        msg_time = self.get_clock().now().seconds_nanoseconds()

        time_msg.sec = int(msg_time[0])
        time_msg.nanosec = int(msg_time[1])
        return time_msg

    def get_image_msg(self, image, time,left=True):
        """
        Get image message, takes image as input and returns CvBridge image message
        :param image: cv2 image
        :return: sensor_msgs/Imag
        """
        img_msg = self.br.cv2_to_compressed_imgmsg(image)  # , encoding="bgr8")
        #print(img_msg)
        img_msg.header.stamp = time
        img_msg.header.frame_id = self.left_frame_id_ if left else self.right_frame_id_
        return img_msg

    def start_cameras(self):
    
        self.left_camera = CSI_Camera()
        self.left_camera.open(
            gstreamer_pipeline(
                sensor_id=0,
                capture_width=1920,
                capture_height=1080,
                flip_method=1,
                display_width=960,
                display_height=540,
            )
        )
        self.left_camera.start()

        self.right_camera = CSI_Camera()
        self.right_camera.open(
            gstreamer_pipeline(
                sensor_id=1,
                capture_width=1920,
                capture_height=1080,
                flip_method=2,
                display_width=960,
                display_height=540,
            )
        )
        self.right_camera.start()


""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080
"""


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=1920,
    display_height=1080,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def run_cameras():
    window_title = "Dual CSI Cameras"
    left_camera = CSI_Camera()
    left_camera.open(
        gstreamer_pipeline(
            sensor_id=0,
            capture_width=1920,
            capture_height=1080,
            flip_method=0,
            display_width=960,
            display_height=540,
        )
    )
    left_camera.start()

    right_camera = CSI_Camera()
    right_camera.open(
        gstreamer_pipeline(
            sensor_id=1,
            capture_width=1920,
            capture_height=1080,
            flip_method=0,
            display_width=960,
            display_height=540,
        )
    )
    right_camera.start()

    if left_camera.video_capture.isOpened() and right_camera.video_capture.isOpened():

        cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)

        try:
            while True:
                _, left_image = left_camera.read()
                _, right_image = right_camera.read()
                # Use numpy to place images next to each other
                camera_images = np.hstack((left_image, right_image)) 
                # Check to see if the user closed the window
                # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
                # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow(window_title, camera_images)
                else:
                    break

                # This also acts as
                keyCode = cv2.waitKey(30) & 0xFF
                # Stop the program on the ESC key
                if keyCode == 27:
                    break
        finally:

            left_camera.stop()
            left_camera.release()
            right_camera.stop()
            right_camera.release()
        cv2.destroyAllWindows()
    else:
        print("Error: Unable to open both cameras")
        left_camera.stop()
        left_camera.release()
        right_camera.stop()
        right_camera.release()



if __name__ == "__main__":
    rclpy.init()
    camera_node = CameraNode()
    rclpy.spin(camera_node)

    camera_node.close_videocapture()
    camera_node.destroy_node()
    rclpy.shutdown()
