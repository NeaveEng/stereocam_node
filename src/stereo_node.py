#!/usr/bin/env python3

# picamera stereo ROS node using dual CSI Pi CS3 board
# Wes Freeman 2018
# modified from code by Adrian Rosebrock, pyimagesearch.com
# and jensenb, https://gist.github.com/jensenb/7303362

from cv_bridge import CvBridge
import cv2
import time
import rospy
from sensor_msgs.msg import CameraInfo, Image
import yaml
import io
import signal  # for ctrl-C handling
import sys
import threading


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


# cam resolution
res_x = 320  # 320 # per camera
res_y = 240  # 240
target_FPS = 15

# initialize the camera
print("Init capture...")
cap_right = cv2.VideoCapture(0)
cap_left = cv2.VideoCapture(1)

cap_left.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap_left.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

cap_right.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap_right.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

stream = io.BytesIO()

# ----------------------------------------------------------
# setup the publishers
print("Init publishers")

# queue_size should be roughly equal to FPS or that causes lag?
left_img_pub = rospy.Publisher('stereo/left/image_raw', Image, queue_size=1)
right_img_pub = rospy.Publisher('stereo/right/image_raw', Image, queue_size=1)

left_cam_pub = rospy.Publisher('stereo/left/camera_info', CameraInfo, queue_size=1)
right_cam_pub = rospy.Publisher('stereo/right/camera_info', CameraInfo, queue_size=1)

rospy.init_node('stereo_pub')

# init messages
left_img_msg = Image()
left_img_msg.height = res_y
left_img_msg.width = res_x
left_img_msg.step = res_x * 3  # bytes per row: pixels * channels * bytes per channel (1 normally)
left_img_msg.encoding = 'rgb8'
left_img_msg.header.frame_id = 'left_camera'  # TF frame

right_img_msg = Image()
right_img_msg.height = res_y
right_img_msg.width = res_x
right_img_msg.step = res_x * 3
right_img_msg.encoding = 'rgb8'
right_img_msg.header.frame_id = 'right_camera'

imageBytes = res_x * res_y * 3

# parse the left and right camera calibration yaml files
left_cam_info = parse_calibration_yaml('left.yaml')
right_cam_info = parse_calibration_yaml('right.yaml')


# ---------------------------------------------------------------
# this is supposed to shut down gracefully on CTRL-C but doesn't quite work:
def signal_handler(signal, frame):
    print('CTRL-C caught')
    print('closing cameras')
    cap_left.release()
    cap_right.release()
    time.sleep(1)
    print('cameras closed')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

# -----------------------------------------------------------

print("Setup done, entering main loop")

# capture frames from the camera
br = CvBridge()

debug_view = False

def Publish(cap, left):
    global left_img_msg, left_cam_info, left_cam_pub, right_img_msg, right_cam_info, right_cam_pub, br
    try:
        while(cap.isOpened()):
            # Capture frame-by-frame
            ret, frame = cap.read()
            if ret == True:
                if(debug_view == True):
                    # Display the resulting frame
                    cv2.imshow('Source',frame)

                # Press Q on keyboard to  exit
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break

                stamp = rospy.Time.now()

                if(left):
                    left_img_msg.header.stamp = stamp
                    left_cam_info.header.stamp = stamp
                    left_cam_pub.publish(left_cam_info)
                    left_img_pub.publish(br.cv2_to_imgmsg(frame, encoding="bgr8"))
                else:
                    right_img_msg.header.stamp = stamp
                    right_cam_info.header.stamp = stamp            
                    right_cam_pub.publish(right_cam_info)
                    right_img_pub.publish(br.cv2_to_imgmsg(frame, encoding="bgr8"))
        
    except:
        print(sys.exc_info()[0])

def PublishLeft():
    global cap_left
    Publish(cap_left, True)

def PublishRight():
    global cap_right
    Publish(cap_right, False)
    
trdLeft = threading.Thread(target=PublishLeft)
trdRight = threading.Thread(target=PublishRight)

trdLeft.start()
trdRight.start()

trdLeft.join()
trdRight.join()