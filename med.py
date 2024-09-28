import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import cv2
from cv2 import aruco
import time

rospy.init_node("along_wall")

timer = 0
cl = False
first = True
aruco_marker = True
cmd = Twist()
cmd.linear.x = 20
target = 0.3
target_aruco = 0  # Здесь будем сохранять ID первого маркера

Kp = 325
Kd = 325

last_dist = 0
move = False
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
aruco_params = aruco.DetectorParameters_create()


def callback_scan(msg):
    global timer
    global cl
    global move
    global target_aruco
    global target
    global first

    if move:
        global last_dist
        min_ranges = min(msg.ranges)
        dX = min_ranges - last_dist
        error = target - min_ranges
        cmd.angular.z = Kp * error - Kd * dX
        if not aruco_marker and not first:
            pub.publish(cmd)
        last_dist = min_ranges
        print(cmd.angular.z)


def callback_img(msg):
    global timer
    global cl
    global move
    global target_aruco
    global target
    global first
    global aruco_marker

    try:
        frame = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected_img_points = aruco.detectMarkers(
            gray, aruco_dict, parameters=aruco_params
        )
        print(ids)

        if len(corners) > 0 and not aruco_marker:
            if first:
                target_aruco = ids[0]
                aruco_marker = True
                first = False
                move = True
                print("target:", target_aruco)
            else:
                for i in range(len(ids)):
                    if ids[i] == target_aruco:
                        c = corners[i]
                        M = cv2.moments(c)
                        cX = int(M["m10"] / M["m00"])
                        # cY = int(M["m01"] / M["m00"])

                        cmd.angular.z = -(320 - cX) / 20
                        pub.publish(cmd)

        if len(corners) >= 3:
            timer = time.time()
            aruco_marker = True
            move = False

        if time.time() - timer > 10 and aruco_marker is True:
            move = True
            aruco_marker = False
            cmd.angular.z = 0
            cmd.linear.x = -20
            pub.publish(cmd)
            time.sleep(0.5)
            pub.publish(cmd)
            time.sleep(0.5)
            cmd.linear.x = 20

    except CvBridgeError as e:
        print(e)


rospy.Subscriber("/scan", LaserScan, callback_scan)
bridge = CvBridge()
rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, callback_img)
rospy.spin()
