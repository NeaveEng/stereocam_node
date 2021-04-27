#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import time
import rospy
from sensor_msgs.msg import CameraInfo, Image
import yaml
import io
import signal # for ctrl-C handling
import sys
from threading import Thread
import numpy as np 

class VideoStreamWidget(object):
    def __init__(self, src=0):
        # Create a VideoCapture object
        self.capture = cv2.VideoCapture(src)

        # Start the thread to read frames from the video stream
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()

    def update(self):
        # Read the next frame from the stream in a different thread
        while True:
            if self.capture.isOpened():
                (self.status, self.frame) = self.capture.read()

    def crop(self, image):
        height, width, channels = image.shape
        if(height == 480):
            # croppedImage = image[0:240, 0:320] #this line crops
            croppedImage = image[240:480, 0:320] #this line crops
            return croppedImage
        else:
            return None

    def show_frame(self):
        # Display frames in main program
        if self.status:
            #self.frame = self.maintain_aspect_ratio_resize(self.frame, width=600)
            image = self.crop(self.frame)

            if(image is not None):
                self.frame = image
                cv2.imshow('IP Camera Video Streaming', self.frame)

        # Press Q on keyboard to stop recording
        key = cv2.waitKey(1)
        if key == ord('q'):
            self.capture.release()
            cv2.destroyAllWindows()
            exit(1)

    def get_frame(self):
        if self.status:
            return self.frame            
        else:
            return None

    # Resizes a image and maintains aspect ratio
    def maintain_aspect_ratio_resize(self, image, width=None, height=None, inter=cv2.INTER_AREA):
        # Grab the image size and initialize dimensions
        dim = None
        (h, w) = image.shape[:2]

        # Return original image if no need to resize
        if width is None and height is None:
            return image

        # We are resizing height if width is none
        if width is None:
            # Calculate the ratio of the height and construct the dimensions
            r = height / float(h)
            dim = (int(w * r), height)
        # We are resizing width if height is none
        else:
            # Calculate the ratio of the 0idth and construct the dimensions
            r = width / float(w)
            dim = (width, int(h * r))

        # Return the resized image
        return cv2.resize(image, dim, interpolation=inter)

def parse_calibration_yaml(calib_file):
    with open(calib_file, 'r') as f:
        params = yaml.load(f)

    cam_info = CameraInfo()
    cam_info.height = params['image_height']
    cam_info.width = params['image_width']
    cam_info.distortion_model = params['distortion_model']
    cam_info.K = params['camera_matrix']['data']
    cam_info.D = params['distortion_coefficients']['data']
    cam_info.R = params['rectification_matrix']['data']
    cam_info.P = params['projection_matrix']['data']

    return cam_info

def create_message(frame_id):
    img_msg = Image()
    img_msg.height = res_y
    img_msg.width = res_x
    img_msg.step = res_x*3 # bytes per row: pixels * channels * bytes per channel (1 normally)
    img_msg.encoding = 'rgb8'
    img_msg.header.frame_id = frame_id # TF frame

    return img_msg

# cam resolution
res_x = 320 #320 # per camera
res_y = 240 #240 
requested_fps = 18
requested_rate = 23

print("Init camera...")

# ----------------------------------------------------------
#setup the publishers
print("init publishers")

# queue_size should be roughly equal to FPS or that causes lag?
left_img_pub = rospy.Publisher('left/image_raw', Image, queue_size=1)
right_img_pub = rospy.Publisher('right/image_raw', Image, queue_size=1)
left_cam_pub = rospy.Publisher('left/camera_info', CameraInfo, queue_size=1)
right_cam_pub = rospy.Publisher('right/camera_info', CameraInfo, queue_size=1)

rospy.init_node('stereo_pub')

# init messages
left_img_msg = create_message("left")
right_img_msg = create_message("right")

imageBytes = res_x*res_y*3

# setup the CV bridge
bridge = CvBridge()

# parse the left and right camera calibration yaml files
left_cam_info = parse_calibration_yaml('left.yaml')
right_cam_info = parse_calibration_yaml('right.yaml')

# ---------------------------------------------------------------
# this is supposed to shut down gracefully on CTRL-C but doesn't quite work:
def signal_handler(signal, frame):
    print('CTRL-C caught')
    print('closing camera')
    capture.release()
    time.sleep(1)
    print('camera closed')    
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#-----------------------------------------------------------

print("Setup done, entering main loop")
framecount=0
frametimer=time.time()
toggle = True

rate = rospy.Rate(requested_rate)

stream_link = 'http://turtlebot:8080'
video_stream_widget = VideoStreamWidget(stream_link)

# capture frames from the camera
while True:
    try:
        image = video_stream_widget.get_frame()
    except:
        continue

    if(image is None):
        print("no image")

    height, width, channels = image.shape
    if(height == 480):
        left_frame = image[0:240, 0:320] #this line crops
        right_frame = image[240:480, 0:320] #this line crops
    else:
        print("image is " + height)
        continue

    framecount +=1
    left_img_msg = bridge.cv2_to_imgmsg(left_frame, encoding="passthrough")
    right_img_msg = bridge.cv2_to_imgmsg(right_frame, encoding="passthrough")

    stamp = rospy.Time.now()
    left_cam_info.header.stamp = stamp    
    right_cam_info.header.stamp = stamp    

    left_cam_pub.publish(left_cam_info)
    left_img_pub.publish(left_img_msg)

    right_cam_pub.publish(right_cam_info)
    right_img_pub.publish(right_img_msg)

    rate.sleep()

    # console info
    if time.time() > frametimer +1.0:
        if toggle: 
            indicator = '  o' # just so it's obviously alive if values aren't changing
        else:
            indicator = '  -'
        toggle = not toggle        
        print('approx publish rate:', framecount, indicator)
        frametimer=time.time()
        framecount=0
    else:
        print("not open")
            

