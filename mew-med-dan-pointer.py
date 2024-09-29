from tkinter.tix import Tree
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage
import cv2
import time
import numpy as np

rospy.init_node("med")

# Переменные для стрелок
stop = False

# Списки для площадей
square_red = []
square_blue = []

# Отслеживание цвета треугольника
color = ""



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

        # Запускаем функцию для отслеживания треугольника и определения его цвета
        triangle_detected = detect_pointer(frame)
        # Условие, которое срабатывает, если найден треугольник
        if triangle_detected:
            print("Треугольник найден!")                

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

    

def detect_pointer(frame):
        global square_red
        global square_blue
        global color        

        # Преобразуем изображение в HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Определяем диапазоны HSV для красного и синего цвета
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])

        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # contours_red = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        # contours_blue = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        # In OpenCV 3.x, findContours returns 3 values: image, contours, and hierarchy

        _, contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        _, contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        # cv2.imshow("red mask", mask_red)
        # cv2.imshow("blue mask", mask_blue)

        triangle_found = check_triangles(mask_red, contours_red, "red")
        square_red.append(triangle_found[-1])

        if triangle_found[-1] > 90000:            
            print("Робот остановился")            
            return triangle_found

        triangle_found = check_triangles(mask_blue, contours_blue, "blue")
        square_blue.append(triangle_found[-1])

        if triangle_found[-1] > 90000:            
            print("Робот остановился")
            return triangle_found

        return triangle_found

def calculate_triangle_area(contour):
    moments = cv2.moments(contour)
    if moments["m00"] != 0:
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
    else:
        return 0
    area = cv2.contourArea(contour)
    return area

def check_triangles(frame, contours, color_name):
    global color
    triangle_found = False
    total_area = 0

    for contour in contours:
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)

        if len(approx) == 3:
            # Координаты вершин треугольника
            pts = approx.reshape(3, 2)
            pts = sorted(pts, key=lambda x: x[1])  # Сортируем по оси Y (по высоте)

            # Определение направления:
            top_vertex = pts[0]  # Вершина треугольника сверху
            left_vertex = pts[1] if pts[1][0] < pts[2][0] else pts[2]
            right_vertex = pts[2] if pts[1][0] < pts[2][0] else pts[1]

            if top_vertex[1] < left_vertex[1] and top_vertex[1] < right_vertex[1]:
                pass
            elif top_vertex[1] > left_vertex[1] and top_vertex[1] > right_vertex[1]:
                pass
            elif left_vertex[0] < top_vertex[0] and right_vertex[0] > top_vertex[0]:
                print("Треугольник направлен вправо")
            else:
                print("Треугольник направлен влево")

            triangle_found = True
            area = calculate_triangle_area(approx)
            total_area += area

            cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)

            color = color_name
            print(f"{color_name} треугольник найден!")
            print(f"Площадь равна: {total_area}")

    return triangle_found, total_area




        
print("Жду маркер")
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
rospy.Subscriber("/scan", LaserScan, callback_scan)
rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, callback_camera)
bridge = CvBridge()

rospy.spin()