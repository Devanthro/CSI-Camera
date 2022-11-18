# MIT License
# Copyright (c) 2019-2022 JetsonHacks

# Using a CSI camera (such as the Raspberry Pi Version 2) connected to a
# NVIDIA Jetson Nano Developer Kit using OpenCV
# Drivers for the camera and OpenCV are included in the base image

import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge


""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080 displayd in a 1/4 size window
"""


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1960,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=20,
    flip_method=1,
):
    return (
        "nvarguscamerasrc sensor-id=%d !"
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


class CameraNode(Node):
    def __init__(self, **kwargs):
        super().__init__("camera_node")
        image_topic_ = self.declare_parameter(
            "image_topic", "/image/image_compressed").value
        self.frame_id_ = self.declare_parameter("frame_id", "camera").value
        self.image_publisher_ = self.create_publisher(CompressedImage, image_topic_, 1)
        self.br = CvBridge()

        self.get_logger().info(
            f"Starting publisher:\n {gstreamer_pipeline(flip_method=0)}")
        self.video_capture = cv2.VideoCapture(
            gstreamer_pipeline(flip_method=1,framerate=20), cv2.CAP_GSTREAMER)

        self.timer = self.create_timer(1.0/20, self.image_callback)

    def close_videocapture(self):
        self.video_capture.release()

    def image_callback(self):
        if self.video_capture.isOpened():
            try:
                ret_val, image = self.video_capture.read()
                #print(image)
                time_msg = self.get_time_msg()
                img_msg = self.get_image_msg(image, time_msg)
                self.image_publisher_.publish(img_msg)
            except Exception as e:
                print(e)

    def get_time_msg(self):
        time_msg = Time()
        msg_time = self.get_clock().now().seconds_nanoseconds()

        time_msg.sec = int(msg_time[0])
        time_msg.nanosec = int(msg_time[1])
        return time_msg

    def get_image_msg(self, image, time):
        """
        Get image message, takes image as input and returns CvBridge image message
        :param image: cv2 image
        :return: sensor_msgs/Imag
        """
        img_msg = self.br.cv2_to_compressed_imgmsg(image)  # , encoding="bgr8")
        #print(img_msg)
        img_msg.header.stamp = time
        img_msg.header.frame_id = self.frame_id_
        return img_msg


def show_camera():
    window_title = "CSI Camera"

    # To flip the image, modify the flip_method parameter (0 and 2 are the most common)
    print(gstreamer_pipeline(flip_method=0))
    video_capture = cv2.VideoCapture(
        gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if video_capture.isOpened():
        try:
            #window_handle = cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            while True:
                ret_val, frame = video_capture.read()
                print(frame)
                # Check to see if the user closed the window
                # Under GTK+ (Jetson Default), WND_PROP_VISIBLE does not work correctly. Under Qt it does
                # GTK - Substitute WND_PROP_AUTOSIZE to detect if window has been closed by user
                # if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                #    cv2.imshow(window_title, frame)
                # else:
                #    break
                keyCode = cv2.waitKey(10) & 0xFF
                # Stop the program on the ESC key or 'q'
                if keyCode == 27 or keyCode == ord('q'):
                    break
        finally:
            video_capture.release()
            # cv2.destroyAllWindows()
    else:
        print("Error: Unable to open camera")


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)

    camera_node.close_videocapture()
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    # show_camera()
