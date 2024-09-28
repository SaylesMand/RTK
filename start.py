import cv2
import numpy as np

class CameraSubscriber:
    def __init__(self):
        # Открываем доступ к веб-камере (0 означает первую камеру по умолчанию)
        self.cap = cv2.VideoCapture(0)

        # Настройки для ArUco
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.gray = None

        self.square_red = []
        self.square_blue = []
        self.stop = False

        # self.list_red = []
        # self.list_blue = []

    # this instead launch...
    def run(self):
        while True:
            # Захватываем кадр с веб-камеры
            ret, frame = self.cap.read()

            if not ret:
                print("Не удалось получить кадр с камеры")
                break

            # Переводим изображение в оттенки серого для обработки ArUco маркеров
            self.gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Обнаружение ArUco маркеров
            corners, ids, rejected_img_points = self.detector.detectMarkers(self.gray)


            # Если маркеры найдены, отобразить их
            if len(corners) > 0:
                self.draw_detected_markers(frame, corners, ids)
            
             # Запускаем функцию для отслеживания треугольника и определения его цвета
            triangle_detected = self.detect_pointer(frame)
            
            # Условие, которое срабатывает, если найден треугольник
            if triangle_detected:
                print("Треугольник найден!")
                if self.stop:
                    print("STOOOOOOOP")
                    break

            # Показываем изображение с помощью OpenCV
            cv2.imshow("Camera Frame with ArUco Detection", frame)

            # Ждем 1 миллисекунду и обрабатываем нажатие клавиши 'q' для выхода
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # Освобождаем ресурсы камеры
        self.cap.release()
        cv2.destroyAllWindows()


        # print(self.square_blue)
        # print(self.square_red)
        # print(f"max is {max(self.square_red)}")
        # print(f"min is {min(self.square_red)}")
        # print(f"max is {max(self.square_blue)}")
        # print(f"min is {min(self.square_blue)}")

        # print(f"self.list_red = {self.list_red}")
        # print(f"self.list_blue = {self.list_blue}")

    def draw_detected_markers(self, frame, corners, ids=None):
        # Перебираем каждый обнаруженный маркер
        for i, corner in enumerate(corners):
            # Преобразуем массив углов в формат numpy и получаем координаты каждого угла
            corner = np.int32(corner)            

            # Рисуем контур маркера, соединяя его углы
            cv2.polylines(frame, [corner], isClosed=True, color=(0, 255, 0), thickness=2)

            # Если есть список идентификаторов, то подпишем маркеры
            if ids is not None:                

                print(f"ids = {ids}")
                print(f"corners = {corners}")            
                # print coordinates (x;y)
                center = self.detect_center_marker(corner)
                print(center)

                # Добавим текст с идентификатором маркера на изображение
                cv2.putText(frame, str(ids[i][0]), (center[0], center[1]), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.5, (255, 0, 0), 2, cv2.LINE_AA)

        return frame
    
    def detect_center_marker(self, corner):
        # Рассчитаем центр маркера для отрисовки текста
        return np.mean(corner, axis=1).astype(int)[0]

    def detect_pointer(self, frame):
        # Преобразуем изображение в HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Определяем диапазоны HSV для красного и синего цвета
        # Красный цвет может быть представлен в двух диапазонах
        lower_red1 = np.array([0, 70, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 70, 50])
        upper_red2 = np.array([180, 255, 255])

        # Диапазон синего цвета
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])

        # Создаем маски для красного и синего цветов
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)  # Объединяем две красные маски

        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)        


        # Ищем контуры в масках
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cv2.imshow("red mask", mask_red)
        cv2.imshow("blue mask", mask_blue)

        # Проверяем наличие треугольников в красных контурах
        triangle_found = self.check_triangles(mask_red, contours_red, "red")        
        self.square_red.append(triangle_found[-1])
        
        # if triangle_found:
            # self.list_red.append(triangle_found)

        if triangle_found[-1] > 90000:
            self.stop = True
            return triangle_found
        
        # Если не найден красный треугольник, проверяем синий        
        triangle_found = self.check_triangles(mask_blue, contours_blue, "blue")        
        self.square_blue.append(triangle_found[-1])
            # self.list_blue.append(triangle_found)
        
        if triangle_found[-1] > 90000:
            self.stop = True
            return triangle_found
        

        return triangle_found

    def calculate_triangle_area(self, contour):
        # Вычисляем моменты для контура
        moments = cv2.moments(contour)
        if moments['m00'] != 0:
            # Вычисляем координаты центра масс
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
        else:
            # Если не удается вычислить, то площадь равна нулю
            return 0

        # Вычисляем площадь треугольника
        area = cv2.contourArea(contour)
        return area
    
    def check_triangles(self, frame, contours, color_name):
        triangle_found = False
        total_area = 0

        for contour in contours:
            # Вычисляем периметр контура
            perimeter = cv2.arcLength(contour, True)

            # Аппроксимируем контур (упрощаем его)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)

            # Если контур имеет 3 угла, это треугольник
            if len(approx) == 3:
                triangle_found = True

                # Вычисляем площадь треугольника
                area = self.calculate_triangle_area(approx)
                total_area += area

                # Рисуем контур треугольника
                cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)

                # Печатаем цвет треугольника
                print(f"{color_name} треугольник найден!")
                print(f"Площадь равна: {total_area}")

        return triangle_found, total_area

    



if __name__ == '__main__':
    camera_subscriber = CameraSubscriber()
    camera_subscriber.run()
