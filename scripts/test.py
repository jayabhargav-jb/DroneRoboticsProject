
import cv2
import rospy
import numpy as np
import math
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, PoseStamped
from time import sleep


def position_bots(pose_data):
        print(pose_data.pose)
        gx, gy, gz = pose_data.pose[2].position.x, pose_data.pose[2].position.y, pose_data.pose[2].position.z
        ux, uy, uz = pose_data.pose[1].position.x, pose_data.pose[1].position.y, pose_data.pose[1].position.z

coor_topic = "/gazebo/model_states"
rospy.init_node("hey")
vehicle_position = rospy.Subscriber(coor_topic, ModelStates, position_bots)

rospy.spin()
