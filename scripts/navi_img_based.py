import cv2
import rospy
import numpy as np
import math
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from time import sleep

class DroneLanding:

    def __init__(self, img_topic = "/first/front_cam/camera/image", vel_topic = "/first/cmd_vel", vehicle_coor_topic = "/atom/odom", uav_coor_topic = "/gazebo/model_states/pose"):
        # initialze the node and topic publish and subscribe
        rospy.init_node('Landing_node', anonymous=True)
        self.vel_pub = rospy.Publisher(vel_topic, Twist, queue_size=1)
        self.image_sub = rospy.Subscriber(img_topic,Image, self.imageCallback)
        self.vehicle_position = rospy.Subscriber(vehicle_coor_topic, Odometry, self.position_wheeled_bot)
        self.move_uav_position = rospy.Subscriber(uav_coor_topic, Odometry, self.position_move_uav_bot)
        self.bridge = CvBridge()

        # track status
        self.detected = 0
        self.count_detected = 0
        self.launched = 0

        # continuous execution
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Exiting")
        cv2.destroyAllWindows()
        print("Exited")

    # the next four functions describe the quadcopter movement
    def hover(self,vx=0.0, vy=0.0, vz=0.0, vaz=0.0):
        self.vel_msg = Twist()
        self.vel_msg.linear.x = float(vx)
        self.vel_msg.linear.y = float(vy)
        self.vel_msg.linear.z = float(vz)
        self.vel_msg.angular.z = float(vaz)
        self.vel_msg.angular.x = float(0.0)
        self.vel_msg.angular.y = float(0.0)
        self.vel_pub.publish(self.vel_msg)

    def up(self):
        self.vel_msg = Twist()
        self.vel_msg.linear.z = float(1.0)
        self.vel_pub.publish(self.vel_msg)

    def forward(self):
        self.vel_msg = Twist()
        self.vel_msg.linear.x = float(1.0)
        self.vel_pub.publish(self.vel_msg)

    def ccw(self):
        self.vel_msg = Twist()
        self.vel_msg.angular.z = float(1.0)
        self.vel_pub.publish(self.vel_msg)

    # image processing
    def imageCallback(self, ros_image):
        try:
            self.img = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
                print(e)

        # image processing
        self.imgContour = self.img.copy()
        # cv2.imshow("window", self.imgContour)
        # cv2.waitKey(1)
        self.imgGray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        self.imgCanny = cv2.Canny(self.imgGray,50,50)
        self.getContours(self.imgCanny, self.imgContour)
        imgStacked = self.stackImages(0.5, ([self.img, self.imgContour]))
        cv2.imshow("Image", imgStacked)
        cv2.waitKey(0.001)

    def getContours(self, img, imgContour) :
        global count_detected
        # global ccw_start
        contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) # image, retrieval method (here, for outermost contours), approximation
        global launched
        go_down = 0
        for cnt in contours : # contours - array of contours detected in image

            area = cv2.contourArea(cnt) # finds area of selected contour
            # print(area)
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 0), 3) # image copy, selected contour, (-1 to draw all contours), color, thickness
            if area > 500 : # selects only contours without too much noise (contours with area > 500 units)
                cv2.drawContours(imgContour, cnt, -1, (255, 0, 0),3)  # image copy, selected contour, (-1 to draw all contours), color, thickness
                perimeter = cv2.arcLength(cnt, True) # contour, is closed(?)
                # print(perimeter)
                approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True) # contour, resolution, is closed(?)
                # print(approx) # gives corner points for each contour (shape)
                print(len(approx)) # prints number of vertices
                objCor = len(approx) # number of corners (higher the number, more likely to be a circle)
                x, y, w, h = cv2.boundingRect(approx) # coordinates of each shape

                if objCor == 4 : # evaluating number of corners to determine shape
                    aspRatio = w / float(h)
                    if aspRatio > 0.95 and aspRatio < 1.05 : # if width = height within error margin, then square
                        objType = 'Square'
                    else :
                        objType = 'Quadrilateral'
                elif objCor == 5 :
                    objType = 'Pentagon'
                elif objCor > 5 : # circle if large number of corners are detected (large being greater than 5 here)
                    objType = 'Conic'
                else :
                    objType = 'None'
                
                global detected
                

                cv2.rectangle(imgContour, (x,y), (x+w, y+h), (0, 255, 0), 2) # bounding rectangle (green for each detected shape)
                cv2.putText(imgContour, objType, (x + (w//2) - 10 , y + (h//2) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
        #         if (objType == 'Square' or objType == 'Quadrilateral' or objType == 'Pentagon' or objType == 'Conic') and area>1000:
        #             print(objType)
        #             self.detected = 1
        #             count_detected+=1
        #             self.forward()
        #             sleep(0.1)
        #             # hover()
        #             sleep(0.1)
        #             # sleep(0.1)
        #             # move(x)
        #             self.hover()
        #             print(count_detected)

        #         elif detected == 0:
        #             self.ccw()
        #             sleep(0.1)
                
        #         if count_detected > 10 and not (objType == 'Square' or objType == 'Quadrilateral' or objType == 'Pentagon' or objType == 'Conic'):
        #             self.down()
        #             go_down = 1

        # if count_detected>20:
        #     self.down()
        
        # if detected==0:
        #     self.ccw()
        #     sleep(0.01)

        # IMPORTANT UNCOMMENT LATER
        while self.launched!= 50000:
            self.up()
            # sleep(5)
            # self.hover()
            self.launched += 1

    
    def stackImages(self, scale,imgArray):
        rows = len(imgArray)
        cols = len(imgArray[0])
        rowsAvailable = isinstance(imgArray[0], list)
        width = imgArray[0][0].shape[1]
        height = imgArray[0][0].shape[0]
        if rowsAvailable:
            for x in range ( 0, rows):
                for y in range(0, cols):
                    if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                        imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                    else:
                        imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                    if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
            imageBlank = np.zeros((height, width, 3), np.uint8)
            hor = [imageBlank]*rows
            hor_con = [imageBlank]*rows
            for x in range(0, rows):
                hor[x] = np.hstack(imgArray[x])
            ver = np.vstack(hor)
        else:
            for x in range(0, rows):
                if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                    imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
                else:
                    imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
                if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
            hor= np.hstack(imgArray)
            ver = hor
        return ver
    
    def position_wheeled_bot(self, pos_data):
        self.gx, self.gy, self.gz = pos_data.pose.pose.position.x, pos_data.pose.pose.position.y, pos_data.pose.pose.position.z

    def position_move_uav_bot(self, pose_data):


        # x, y, z = pos_data.pose.position.x, pos_data.pose.pose.position.y, pos_data.pose.pose.position.z
        # qx, qy, qz, qw = pos_data.pose.orientation.x, pos_data.pose.pose.orientation.y, pos_data.pose.pose.orientation.z
        # position_difference = (x-self.gx, y-self.gy, z-self.gz)

        if position_difference[0]>0:
            pass
    
    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z 
            


        



# def main():
#     global vel_pub
#     vel_pub = rospy.Publisher('/first/cmd_vel', Twist, queue_size=1)
#     vel_msg = Twist()
#     vel_msg.linear.z = float(1.0)
#     vel_pub.publish(vel_msg)
#     sleep(3)

#     rospy.spin()
#     cv2.destroyAllWindows()

if __name__ == "__main__":
    drone_class = DroneLanding()

