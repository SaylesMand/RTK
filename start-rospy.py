import rospy
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge
import numpy as np
import move


class CameraSubscriber:
    def __init__(self):
        # Инициализация узла ROS
        print("check.init")
        rospy.init_node("camera_subscriber", anonymous=True)
        self.subscription = rospy.Subscriber(
            "/usb_cam/image_raw/compressed", CompressedImage, self.listener_callback
        )

        self.br = CvBridge()

        # Настройки для ArUco
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.parameters = cv2.aruco.DetectorParameters_create()
        # Use the old way to detect markers
        self.detector = cv2.aruco

        self.gray = None
        self.id_markers = []  # ids для считывания маркера

        # Списки для площадей
        self.square_red = []
        self.square_blue = []

        # Свойства для движения
        self.stop = False
        self.RobotMovings = move.RobotMovings()

        # Отслеживание цвета треугольника
        self.color = ""

    def listener_callback(self, msg):
        print("check.listener")
        # Convert CompressedImage to an OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Переводим изображение в оттенки серого для обработки ArUco маркеров
        self.gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Обнаружение ArUco маркеров
        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
            self.gray, self.dictionary, parameters=self.parameters
        )

        # Если маркеры найдены, отобразить их
        if len(corners) > 0:
            print("Marker detected!")
            self.draw_detected_markers(frame, corners, ids)            

            # rotate the robot when the marker is detected
            self.RobotMovings.stop_move()

            if len(self.id_markers) < 2:
                self.RobotMovings.rotate_by_90_to_left()
                self.RobotMovings.move()

            # rotate by 180 the robot if it took the med
            elif len(self.id_markers) > 1:
                self.RobotMovings.rotate_by_180_to_left()
                self.RobotMovings.move()

        # If no marker is detected, keep the robot moving
        # if not self.stop:
        #     self.RobotMovings.move()

        # Запускаем функцию для отслеживания треугольника и определения его цвета
        triangle_detected = self.detect_pointer(frame)

        # Условие, которое срабатывает, если найден треугольник
        if triangle_detected:
            print("Треугольник найден!")
            if self.stop:
                print("STOOOOOOOP")
                self.RobotMovings.stop_move()

                if self.color == "red":
                    self.RobotMovings.rotate_by_90_to_left()
                    self.RobotMovings.move()
                elif self.color == "blue":
                    self.RobotMovings.rotate_by_90_to_right()
                    self.RobotMovings.move()

        # Показываем изображение с помощью OpenCV
        # cv2.imshow("Camera Frame with ArUco Detection", frame)

        # Ждем 1 миллисекунду и обрабатываем нажатие клавиши 'q' для выхода
        if cv2.waitKey(1) & 0xFF == ord("q"):
            rospy.signal_shutdown("Shutdown requested")

        # print(self.square_blue)
        # print(self.square_red)

    def draw_detected_markers(self, frame, corners, ids=None):
        # Перебираем каждый обнаруженный маркер
        for i, corner in enumerate(corners):
            # Преобразуем массив углов в формат numpy и получаем координаты каждого угла
            corner = np.int32(corner)

            # Рисуем контур маркера, соединяя его углы
            cv2.polylines(
                frame, [corner], isClosed=True, color=(0, 255, 0), thickness=2
            )            

            # Если есть список идентификаторов, то подпишем маркеры
            if ids is not None:
                if ids[i][0] not in self.id_markers:
                    self.id_markers.append(ids[i][0])

                print(f"Detected marker ID: {ids[i][0]}")

                center = self.detect_center_marker(corner)
                cv2.putText(
                    frame,
                    str(ids[i][0]),
                    (center[0], center[1]),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 0, 0),
                    2,
                    cv2.LINE_AA,
                )

        return frame

    def detect_center_marker(self, corner):
        # Рассчитаем центр маркера для отрисовки текста
        return np.mean(corner, axis=1).astype(int)[0]

    def detect_pointer(self, frame):
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

        triangle_found = self.check_triangles(mask_red, contours_red, "red")
        self.square_red.append(triangle_found[-1])

        if triangle_found[-1] > 90000:
            self.stop = True
            self.RobotMovings.stop_move()
            return triangle_found

        triangle_found = self.check_triangles(mask_blue, contours_blue, "blue")
        self.square_blue.append(triangle_found[-1])

        if triangle_found[-1] > 90000:
            self.stop = True
            self.RobotMovings.stop_move()
            return triangle_found

        return triangle_found

    def calculate_triangle_area(self, contour):
        moments = cv2.moments(contour)
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"])
        else:
            return 0
        area = cv2.contourArea(contour)
        return area

    def check_triangles(self, frame, contours, color_name):
        triangle_found = False
        total_area = 0

        for contour in contours:
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)

            if len(approx) == 3:
                triangle_found = True
                area = self.calculate_triangle_area(approx)
                total_area += area

                cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)

                self.color = color_name
                print(f"{color_name} треугольник найден!")
                print(f"Площадь равна: {total_area}")

        return triangle_found, total_area


if __name__ == "__main__":
    camera_subscriber = CameraSubscriber()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
