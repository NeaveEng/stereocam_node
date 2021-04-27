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
import FreshestFrame as ff

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

print("Init camera...")

left_cam = cv2.VideoCapture(1)
left_cam.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
left_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
left_ff = ff.FreshestFrame(left_cam)

right_cam = cv2.VideoCapture(0)
right_cam.set(cv2.CAP_PROP_FRAME_WIDTH, res_x)
right_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, res_y)
right_ff = ff.FreshestFrame(right_cam)

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
    left_cam.release()
    right_cam.release()
    time.sleep(1)
    print('camera closed')    
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

#-----------------------------------------------------------

print("Setup done, entering main loop")
framecount=0
frametimer=time.time()
toggle = True

rate = rospy.Rate(10) # 15hz

# capture frames from the camera
while True:
    left_ret, left_frame = left_cam.read()
    if not left_ret:
        print("failed to grab left frame")
        break
    
    right_ret, right_frame = right_cam.read()
    if not right_ret:
        print("failed to grab right frame")
        break
    
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
