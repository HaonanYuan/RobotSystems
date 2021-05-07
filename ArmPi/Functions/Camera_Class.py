#!/usr/bin/python3
# coding=utf8
import sys

sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


class Camera_class:

    def __init__(self):
        self.AK = ArmIK()

        # set the color which can be identify
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255),
        }

        self.__target_color = 'red'
        # The angle of gripper in closed state
        self.servo1 = 500
        # Initial parameter
        self.count = 0
        self.track = False
        self._stop = False
        self.get_roi = False
        self.center_list = []
        self.first_move = True
        self.__isRunning = False
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True
        # Initial move parameter
        self.rect = None
        # the size of window shown the camera content
        self.size = (640, 480)
        self.rotation_angle = 0
        self.unreachable = False
        self.world_X, self.world_Y = 0, 0
        self.world_x, self.world_y = 0, 0
        # Running the move function
        self.th = threading.Thread(target=self.move)
        self.th.setDaemon(True)
        self.th.start()

        self.t1 = 0
        self.roi = ()
        self.last_x, self.last_y = 0, 0

    # set the identify color
    def setTargetColor(self, target_color):
        self.__target_color = target_color
        return True, ()

    # Find the contour with the largest area.
    # The input is the list of contour
    def getAreaMaxContour(self, contours):
        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # Traverse all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # get the area of contour
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # the max area of contour need larger than 300
                    area_max_contour = c
        # return the index of contour and the area of contour
        return area_max_contour, contour_area_max

        # The initial state

    def initMove(self):
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)

    def setBuzzer(self, timer):
        Board.setBuzzer(0)
        Board.setBuzzer(1)
        time.sleep(timer)
        Board.setBuzzer(0)

    # Set the color of LED in board the same as the identify color
    def set_rgb(self, color):
        if color == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif color == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()
        elif color == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()

    # Reset the parameters
    def reset(self):
        self.count = 0
        self._stop = False
        self.track = False
        self.get_roi = False
        self.center_list = []
        self.first_move = True
        self.__target_color = ()
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True

    # Run the application and set the arm to initial state
    def init(self):
        print("ColorTracking Init")
        self.initMove()

    # Run the application and start to do the color tracking
    def start(self):
        self.reset()
        self.__isRunning = True
        print("ColorTracking Start")

    # Run the application and stop doing the color tracking
    def stop(self):
        self._stop = True
        self.__isRunning = False
        print("ColorTracking Stop")

    # Exit the color tracking function
    def exit(self):
        self._stop = True
        self.__isRunning = False
        print("ColorTracking Exit")

    # The function that move the robot arm
    def move(self):

        # The location of each color object in initial state(in each initial place)
        coordinate = {
            'red': (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5, 1.5),
            'blue': (-15 + 0.5, 0 - 0.5, 1.5),
        }
        while True:
            if self.__isRunning:
                if self.first_move and self.start_pick_up:  # First time to identify the object
                    self.action_finish = False
                    self.set_rgb(self.detect_color)  # set the board LED color
                    self.setBuzzer(0.1)  # set the buzzer working time
                    # if there is no working time, set the time automatically
                    result = self.AK.setPitchRangeMoving((self.world_X, self.world_Y - 2, 5), -90, -90, 0)
                    if result == False:
                        self.unreachable = True
                    else:
                        self.unreachable = False
                    time.sleep(result[2] / 1000)  # The third item of the return parameter is time
                    self.start_pick_up = False
                    self.first_move = False
                    self.action_finish = True
                elif not self.first_move and not self.unreachable:  # not the first time to identify the object
                    self.set_rgb(self.detect_color)
                    if self.track:  # If in the tracking stage
                        if not self.__isRunning:  # stop and exit the flag detection
                            continue
                        self.AK.setPitchRangeMoving((self.world_x, self.world_y - 2, 5), -90, -90, 0, 20)
                        time.sleep(0.02)
                        self.track = False
                    if self.start_pick_up:  # if the object do not move for a while, start to pick
                        self.action_finish = False
                        if not self.__isRunning:  # stop and exit the flag detection
                            continue
                        Board.setBusServoPulse(1, self.servo1 - 280, 500)  # open the gripper
                        # calculate the angle that gripper need to rotate
                        servo2_angle = getAngle(self.world_X, self.world_Y, self.rotation_angle)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.8)

                        if not self.__isRunning:
                            continue
                        self.AK.setPitchRangeMoving((self.world_X, self.world_Y, 2), -90, -90, 0,
                                                    1000)  # lower the height
                        time.sleep(2)

                        if not self.__isRunning:
                            continue
                        Board.setBusServoPulse(1, self.servo1, 500)  # close the gripper
                        time.sleep(1)

                        if not self.__isRunning:
                            continue
                        Board.setBusServoPulse(2, 500, 500)
                        self.AK.setPitchRangeMoving((self.world_X, self.world_Y, 12), -90, -90, 0,
                                                    1000)  # Raise the arm
                        time.sleep(1)

                        if not self.__isRunning:
                            continue
                        # 对不同颜色方块进行分类放置
                        # Sort and place different colored squares
                        result = self.AK.setPitchRangeMoving(
                            (coordinate[self.detect_color][0], coordinate[self.detect_color][1], 12), -90,
                            -90, 0)
                        time.sleep(result[2] / 1000)

                        if not self.__isRunning:
                            continue
                        servo2_angle = getAngle(coordinate[self.detect_color][0], coordinate[self.detect_color][1], -90)
                        Board.setBusServoPulse(2, servo2_angle, 500)
                        time.sleep(0.5)

                        if not self.__isRunning:
                            continue
                        self.AK.setPitchRangeMoving(
                            (coordinate[self.detect_color][0], coordinate[self.detect_color][1],
                             coordinate[self.detect_color][2] + 3),
                            -90, -90, 0, 500)
                        time.sleep(0.5)

                        if not self.__isRunning:
                            continue
                        self.AK.setPitchRangeMoving((coordinate[self.detect_color]), -90, -90, 0, 1000)
                        time.sleep(0.8)

                        if not self.__isRunning:
                            continue
                        Board.setBusServoPulse(1, self.servo1 - 200, 500)  # open the gripper, release the object
                        time.sleep(0.8)

                        if not self.__isRunning:
                            continue
                        self.AK.setPitchRangeMoving(
                            (coordinate[self.detect_color][0], coordinate[self.detect_color][1], 12),
                            -90, -90, 0,
                            800)
                        time.sleep(0.8)

                        self.initMove()  # back to initial pose
                        time.sleep(1.5)

                        self.detect_color = 'None'
                        self.first_move = True
                        self.get_roi = False
                        self.action_finish = True
                        self.start_pick_up = False
                        self.set_rgb(self.detect_color)
                    else:
                        time.sleep(0.01)
            else:
                if self._stop:
                    self._stop = False
                    Board.setBusServoPulse(1, self.servo1 - 70, 300)
                    time.sleep(0.5)
                    Board.setBusServoPulse(2, 500, 500)
                    self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                    time.sleep(1.5)
                time.sleep(0.01)

    def run(self, img):

        img_copy = img.copy()
        img_h, img_w = img.shape[:2]
        cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
        cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

        if not self.__isRunning:
            return img

        frame_resize = cv2.resize(img_copy, self.size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        # 如果检测到某个区域有识别到的物体，则一直检测该区域直到没有为止
        # If it detects that there is an object in a certain area,
        # it will continue to detect the area until there is no object.
        if self.get_roi and self.start_pick_up:
            self.get_roi = False
            frame_gb = getMaskROI(frame_gb, self.roi, self.size)
        # Convert the image to LAB space.
        frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

        area_max = 0
        areaMaxContour = 0
        if not self.start_pick_up:
            for i in color_range:
                if i in self.__target_color:
                    self.detect_color = i
                    # Perform bit operations on the original image and the mask.
                    frame_mask = cv2.inRange(frame_lab, color_range[self.detect_color][0],
                                             color_range[self.detect_color][1])  # 对原图像和掩模进行位运算
                    # open operation
                    opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
                    # close operation
                    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算
                    # find the counter
                    contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
                    # find the largest counter
                    areaMaxContour, area_max = self.getAreaMaxContour(contours)  # 找出最大轮廓
            # if there is a largest area
            if area_max > 2500:  # 有找到最大面积
                self.rect = cv2.minAreaRect(areaMaxContour)
                box = np.int0(cv2.boxPoints(self.rect))

                # get the roi area
                self.roi = getROI(box)  # 获取roi区域
                self.get_roi = True

                # get the location of object center
                img_centerx, img_centery = getCenter(self.rect, self.roi, self.size, square_length)  # 获取木块中心坐标
                # Transform to the world frame
                self.world_x, self.world_y = convertCoordinate(img_centerx, img_centery, self.size)  # 转换为现实世界坐标

                cv2.drawContours(img, [box], -1, self.range_rgb[self.detect_color], 2)
                # draw the center point
                cv2.putText(img, '(' + str(self.world_x) + ',' + str(self.world_y) + ')',
                            (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.range_rgb[self.detect_color], 1)  # 绘制中心点
                # Compare the last coordinate to determine whether to move
                distance = math.sqrt(
                    pow(self.world_x - self.last_x, 2) + pow(self.world_y - self.last_y, 2))  # 对比上次坐标来判断是否移动
                self.last_x, self.last_y = self.world_x, self.world_y
                self.track = True
                # print(count,distance)
                # 累计判断
                # Cumulative judgment
                if self.action_finish:
                    if distance < 0.3:
                        self.center_list.extend((self.world_x, self.world_y))
                        self.count += 1
                        if self.start_count_t1:
                            self.start_count_t1 = False
                            self.t1 = time.time()
                        if time.time() - self.t1 > 1.5:
                            self.rotation_angle = self.rect[2]
                            self.start_count_t1 = True
                            self.world_X, self.world_Y = np.mean(np.array(self.center_list).reshape(self.count, 2),
                                                                 axis=0)
                            self.count = 0
                            self.center_list = []
                            self.start_pick_up = True
                    else:
                        self.t1 = time.time()
                        self.start_count_t1 = True
                        self.count = 0
                        self.center_list = []
        return img


if __name__ == '__main__':
    cam = Camera_class()
    cam.init()
    cam.start()
    cam.__target_color = ('red',)
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = cam.run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
