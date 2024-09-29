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
aruco_marker = False  # флаг - обнаружен ли маркер ArUco | changed True to False
cmd = Twist()  # управление лин и угл скоростью робота
cmd.linear.x = 20
target = 0.3  # расстояние от стены
target_aruco = 0

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
    global first  # обозначение первый ли цикл

    if move:
        global last_dist
        min_ranges = min(msg.ranges)  # мин расстояние до препятствий
        dX = min_ranges - last_dist
        error = target - min_ranges  #
        cmd.angular.z = Kp * error - Kd * dX  # изменение угла поворота
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
                print(f"target: {target_aruco}")

        elif len(corners) > 2 and aruco_marker:
            timer = time.time()
            move = False

            # Как будет происходить процесс выравнивания не понятно
            # Ясно как он выравнивается, но для этого он должен занять нужное положение (что-то типа центра)
            for i in range(len(ids)):
                if ids[i] == target_aruco:
                    c = corners[i]
                    M = cv2.moments(c)
                    cX = int(M["m10"] / M["m00"])
                    # cY = int(M["m01"] / M["m00"])

                    cmd.angular.z = -(320 - cX) / 20
                    pub.publish(cmd)                        

        if time.time() - timer > 10 and aruco_marker:
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

"""
Пропорциональная составляющая (Kp * error): Чем больше ошибка, тем больше будет управляющее воздействие, направленное на уменьшение этой ошибки.
Дифференциальная составляющая (Kd * dX): Учитывает скорость изменения ошибки. Если ошибка быстро возрастает, то дифференциальная составляющая будет стремиться уменьшить это возрастание, а если ошибка быстро уменьшается, то дифференциальная составляющая будет стремиться поддержать это уменьшение.

"""
