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

class DroneLanding:

    def __init__(self, img_topic = "/first/front_cam/camera/image", vel_topic = "/first/cmd_vel", coor_topic = "/gazebo/model_states"):
        # initialze the node and topic publish and subscribe
        rospy.init_node('Landing_node')
        self.vel_pub = rospy.Publisher(vel_topic, Twist, queue_size=1)
        self.image_sub = rospy.Subscriber(img_topic,Image, self.imageCallback)
        # self.vehicle_position = rospy.Subscriber(vehicle_coor_topic, Odometry, self.position_wheeled_bot)
        self.vehicle_position = rospy.Subscriber(coor_topic, ModelStates, self.position_bots, queue_size=1)
        self.bridge = CvBridge()

        # track status
        self.detected = 0
        self.count_detected = 0
        self.launched = 0 #500001 #

        # initial coordinate status
        self.gx, self.gy, self.gz, self.ux, self.uy, self.uz = 0, 0, 0, 0, 0, 0

        while self.launched < 50000:
            self.up()
            # sleep(5)
            self.hover()
            self.launched += 1

        # continuous execution
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Exiting")
        cv2.destroyAllWindows()
        print("Exited")

    # the next four functions describe the quadcopter movement
    def initMessage(self, vx, vy, vz, vaz):
        """
        Initializes and sends a velocity message to the drone.
        
        :param vx: Linear velocity in the x direction (forward/backward)
        :param vy: Linear velocity in the y direction (left/right)
        :param vz: Linear velocity in the z direction (up/down)
        :param vaz: Angular velocity around the z axis (yaw)
        """
        self.vel_msg = Twist()  # Create a new Twist message object
        self.vel_msg.linear.x = float(vx)  # Set linear velocity in x direction
        self.vel_msg.linear.y = float(vy)  # Set linear velocity in y direction
        self.vel_msg.linear.z = float(vz)  # Set linear velocity in z direction
        self.vel_msg.angular.z = float(vaz)  # Set angular velocity around z axis (yaw)
        self.vel_msg.angular.x = float(0.0)  # Set angular velocity around x axis (not used)
        self.vel_msg.angular.y = float(0.0)  # Set angular velocity around y axis (not used)
        self.vel_pub.publish(self.vel_msg)  # Publish the velocity message

    def hover(self):
        """
        Commands the drone to hover in place by setting all velocities to 0.
        """
        self.initMessage(0.0, 0.0, 0.0, 0.0)  # Send a velocity message with all components set to 0

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
        cv2.waitKey(1)

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
                # print(len(approx)) # prints number of vertices
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
    
    # def position_wheeled_bot(self, pose_data):
    #     self.gx, self.gy, self.gz = pose_data.pose[2].position.x, pose_data.pose[2].position.y, pose_data.pose[2].position.z
    #     # print(self.gx)

    def position_bots(self, pose_data):
        # print(pose_data.pose)
        self.gx, self.gy, self.gz = pose_data.pose[2].position.x, pose_data.pose[2].position.y, pose_data.pose[2].position.z
        self.ux, self.uy, self.uz = pose_data.pose[1].position.x, pose_data.pose[1].position.y, pose_data.pose[1].position.z
        # print(self.gx, self.gy)
        # print(self.ux, self.uy)
        self.move_to_coordinates()

        # print(self.ux)
    
    # def euler_from_quaternion(self, x, y, z, w):
    #     """
    #     Convert a quaternion into euler angles (roll, pitch, yaw)
    #     roll is rotation around x in radians (counterclockwise)
    #     pitch is rotation around y in radians (counterclockwise)
    #     yaw is rotation around z in radians (counterclockwise)
    #     """
    #     t0 = +2.0 * (w * x + y * z)
    #     t1 = +1.0 - 2.0 * (x * x + y * y)
    #     roll_x = math.atan2(t0, t1)
     
    #     t2 = +2.0 * (w * y - z * x)
    #     t2 = +1.0 if t2 > +1.0 else t2
    #     t2 = -1.0 if t2 < -1.0 else t2
    #     pitch_y = math.asin(t2)
     
    #     t3 = +2.0 * (w * z + x * y)
    #     t4 = +1.0 - 2.0 * (y * y + z * z)
    #     yaw_z = math.atan2(t3, t4)
     
    #     return roll_x, pitch_y, yaw_z 

    ########################################################################################################################################
    ########################################################################################################################################
    ########################################################################################################################################

    def move_to_coordinates(self):
        """
        Move the drone to the specified coordinates (target_x, target_y, target_z).

        """
        
        dx = self.gx - self.ux  # Calculate the difference between target x and current x
        dy = self.gy - self.uy  # Calculate the difference between target y and current y
        
        # dz = self.gz - self.uz  # Calculate the difference between target z and current z
        
        distance = (dx**2 + dy**2)**0.5  # Calculate the Euclidean distance to the target # + dz**2
        # print(dx, dy)
        if distance < 0.1 and distance > 0.03:  # Check if the drone is close enough to the target coordinates
            self.initMessage(0.0, 0.0, -0.01, 0)  # Hover if the target coordinates are reached
            sleep(0.1)
            self.hover()

        if distance < 0.03:
            self.hover()
            rospy.signal_shutdown("Done")

        print("distance remaining: ", distance)
        
        vx = dx * 0.1  # Calculate forward/backward velocity based on x difference (proportional control)
        vy = dy * 0.1  # Calculate left/right velocity based on y difference (proportional control)
        # vz = dz * 0.1  # Calculate vertical velocity based on z difference (proportional control)
        
        self.initMessage(vx, vy, 0.0, 0.0)  # Send the velocity command
        sleep(0.1)
        self.hover()


if __name__ == "__main__":
    drone_class = DroneLanding()

