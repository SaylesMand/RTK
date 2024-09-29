from tkinter.tix import Tree
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import cv2
import time

rospy.init_node("med")

move = False
first_aruco = False
target_aruco = None
need_med = False

# add flag for first marker
flag = True

kP = 500 # обычный поворот was 450
kD = 150 # быстрый поворот was 150

target = 0.305
last_error = 0

timer = 0 # для проверки сколько времени у лекарств

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
aruco_params = cv2.aruco.DetectorParameters_create()

def callback_scan(msg):
    global image
    global first_aruco
    global target_aruco
    global move
    global last_error
    global pub
    global need_med    

    # медленно ищем первый маркер
    # if not first_aruco:
    #     cmd = Twist()
    #     cmd.angular.z = 30
    #     pub.publish(cmd)

    if move:
        min_ranges = min(msg.ranges)
        
        error = target - min_ranges
        delta_error = last_error - error
        print(f"error = {error}")
        print(f"delta_error = {delta_error}")

        cmd = Twist()        
        
        cmd.angular.z = kP * error + kD * delta_error
        if abs(cmd.angular.z) > 25:
            cmd.linear.x = 0
        else:
            cmd.linear.x = 20 # was 20


        pub.publish(cmd)
        last_error = error
        print(cmd.angular.z)
    


def callback_camera(msg):
    global image
    global first_aruco
    global target_aruco
    global move
    global need_med
    global timer
    global flag

    try:
        ids = []
        frame = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        
        if corners:
            print("Нашел маркеры:", ids)

        if corners and not first_aruco:
            first_aruco = True
            target_aruco = ids[0]
            move = True

        if len(corners) >= 3 and flag:
            timer = time.time()
            need_med = True
            move = False

    except CvBridgeError as e:
        print(e)

    if need_med:        
        for i in range(len(corners)):
            if ids[i] == target_aruco and first_aruco:                
                c = corners[i]
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                # cY = int(M["m01"] / M["m00"])

                cmd = Twist()
                cmd.angular.z = (320-cX)/20 
                cmd.linear.x = 40
                pub.publish(cmd)
                print(cmd)      
                print("Я увидел и забрал лекарство!!!")          
                
    
        if time.time() - timer > 7 and first_aruco:            
            cmd = Twist()            
            first_aruco = False
            need_med = False
            flag = False
            # cmd.angular.z = 0
            # скорость хода назад
            cmd.linear.x = -100
            pub.publish(cmd)
            time.sleep(1)            
            move = True

    
    


        
print("Жду маркер")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
rospy.Subscriber("/scan", LaserScan, callback_scan)
rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, callback_camera)
bridge = CvBridge()

rospy.spin()