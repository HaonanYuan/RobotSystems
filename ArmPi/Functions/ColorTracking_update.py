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
from perception_class import perception
from Motion_class import motion

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)

AK = ArmIK()

# set the color which can be identify
range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

__target_color = 'red'


# set the identify color
def setTargetColor(target_color):
    global __target_color

    # print("COLOR", target_color)
    __target_color = target_color
    return True, ()


# Find the contour with the largest area.
# The input is the list of contour
def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # Traverse all contours
        contour_area_temp = math.fabs(cv2.contourArea(c))  # get the area of contour
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # the max area of contour need larger than 300
                area_max_contour = c

    return area_max_contour, contour_area_max  # return the index of contour and the area of contour


# The angle of gripper in closed state
servo1 = 500


# The initial state
def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)


#
def setBuzzer(timer):
    Board.setBuzzer(0)
    Board.setBuzzer(1)
    time.sleep(timer)
    Board.setBuzzer(0)


# Set the color of LED in board the same as the identify color
def set_rgb(color):
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


count = 0
track = False
_stop = False
get_roi = False
center_list = []
first_move = True
__isRunning = False
detect_color = 'None'
action_finish = True
start_pick_up = False
start_count_t1 = True


# Reset the parameters
def reset():
    global count
    global track
    global _stop
    global get_roi
    global first_move
    global center_list
    global __isRunning
    global detect_color
    global action_finish
    global start_pick_up
    global __target_color
    global start_count_t1

    count = 0
    _stop = False
    track = False
    get_roi = False
    center_list = []
    first_move = True
    __target_color = ()
    detect_color = 'None'
    action_finish = True
    start_pick_up = False
    start_count_t1 = True


# Run the application and set the arm to initial state
def init():
    print("ColorTracking Init")
    initMove()


# Run the application and start to do the color tracking
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorTracking Start")


# Run the application and stop doing the color tracking
def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorTracking Stop")


# Exit the color tracking function
def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorTracking Exit")


rect = None
# the size of window shown the camera content
size = (640, 480)
rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0
world_x, world_y = 0, 0


# The function that move the robot arm
def move():
    global rect
    global track
    global _stop
    global get_roi
    global unreachable
    global __isRunning
    global detect_color
    global action_finish
    global rotation_angle
    global world_X, world_Y
    global world_x, world_y
    global center_list, count
    global start_pick_up, first_move

    # The location of each color object in initial state(in each initial place)
    coordinate = {
        'red': (-15 + 0.5, 12 - 0.5, 1.5),
        'green': (-15 + 0.5, 6 - 0.5, 1.5),
        'blue': (-15 + 0.5, 0 - 0.5, 1.5),
    }
    mot = motion()
    while True:
        if __isRunning:
            if first_move and start_pick_up:  # First time to identify the object
                action_finish = False
                set_rgb(detect_color)  # set the board LED color
                setBuzzer(0.1)  # set the buzzer working time
                # if there is no working time, set the time automatically
                result = AK.setPitchRangeMoving((world_X, world_Y - 2, 5), -90, -90, 0)
                if result == False:
                    unreachable = True
                else:
                    unreachable = False
                time.sleep(result[2] / 1000)  # The third item of the return parameter is time
                start_pick_up = False
                first_move = False
                action_finish = True
            elif not first_move and not unreachable:  # not the first time to identify the object
                set_rgb(detect_color)
                if track:  # If in the tracking stage
                    if not __isRunning:  # stop and exit the flag detection
                        continue
                    mot.arm_controller(world_x, world_y - 2, 5, -90, -90, 0, 20)
                    # AK.setPitchRangeMoving((world_x, world_y - 2, 5), -90, -90, 0, 20)
                    # time.sleep(0.02)
                    track = False
                if start_pick_up:  # if the object do not move for a while, start to pick
                    action_finish = False
                    if not __isRunning:  # stop and exit the flag detection
                        continue
                    mot.gripper_controller(-280)  # open the gripper
                    # calculate the angle that gripper need to rotate
                    mot.palm_controller(world_X, world_Y, rotation_angle)
                    # servo2_angle = getAngle(world_X, world_Y, rotation_angle)
                    # Board.setBusServoPulse(2, servo2_angle, 500)
                    # time.sleep(0.8)

                    if not __isRunning:
                        continue
                    mot.arm_controller(world_X, world_Y, 2, -90, -90, 0, 1000)
                    # AK.setPitchRangeMoving((world_X, world_Y, 2), -90, -90, 0, 1000)  # lower the height
                    # time.sleep(2)

                    if not __isRunning:
                        continue
                    mot.gripper_controller()
                    # Board.setBusServoPulse(1, servo1, 500)  # close the gripper
                    # time.sleep(1)

                    if not __isRunning:
                        continue
                    mot.palm_controller()
                    mot.arm_controller(world_X, world_Y, 12, -90, -90, 0, 1000)
                    # Board.setBusServoPulse(2, 500, 500)
                    # AK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000)  # Raise the arm
                    # time.sleep(1)

                    if not __isRunning:
                        continue
                    # 对不同颜色方块进行分类放置
                    # Sort and place different colored squares
                    mot.arm_controller(coordinate[detect_color][0], coordinate[detect_color][1], 12, -90, -90, 0)
                    # result = AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0)
                    # time.sleep(result[2] / 1000)

                    if not __isRunning:
                        continue
                    mot.palm_controller(coordinate[detect_color][0], coordinate[detect_color][1], -90)
                    # servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
                    # Board.setBusServoPulse(2, servo2_angle, 500)
                    # time.sleep(0.5)

                    if not __isRunning:
                        continue
                    mot.arm_controller(coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3,
                        -90, -90, 0, 500)
                    """AK.setPitchRangeMoving(
                        (coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3),
                        -90, -90, 0, 500)
                    time.sleep(0.5)"""

                    if not __isRunning:
                        continue
                    mot.arm_controller(coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2],
                        -90, -90, 0, 500)
                    # AK.setPitchRangeMoving((coordinate[detect_color]), -90, -90, 0, 1000)
                    # time.sleep(0.8)

                    if not __isRunning:
                        continue
                    mot.gripper_controller(-200)
                    # Board.setBusServoPulse(1, servo1 - 200, 500)  # open the gripper, release the object
                    # time.sleep(0.8)

                    if not __isRunning:
                        continue
                    mot.arm_controller(coordinate[detect_color][0], coordinate[detect_color][1], 12, -90, -90, 0,
                                           800)
                    """AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0,
                                           800)
                    time.sleep(0.8)"""

                    initMove()  # back to initial pose
                    time.sleep(1.5)

                    detect_color = 'None'
                    first_move = True
                    get_roi = False
                    action_finish = True
                    start_pick_up = False
                    set_rgb(detect_color)
                else:
                    time.sleep(0.01)
        else:
            if _stop:
                _stop = False
                Board.setBusServoPulse(1, servo1 - 70, 300)
                time.sleep(0.5)
                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                time.sleep(1.5)
            time.sleep(0.01)


# Running the move function
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

t1 = 0
roi = ()
last_x, last_y = 0, 0


def run(img):
    global roi
    global rect
    global count
    global track
    global get_roi
    global center_list
    global __isRunning
    global unreachable
    global detect_color
    global action_finish
    global rotation_angle
    global last_x, last_y
    global world_X, world_Y
    global world_x, world_y
    global start_count_t1, t1
    global start_pick_up, first_move

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]
    cv2.line(img, (0, int(img_h / 2)), (img_w, int(img_h / 2)), (0, 0, 200), 1)
    cv2.line(img, (int(img_w / 2), 0), (int(img_w / 2), img_h), (0, 0, 200), 1)

    if not __isRunning:
        return img

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
    # 如果检测到某个区域有识别到的物体，则一直检测该区域直到没有为止
    # If it detects that there is an object in a certain area,
    # it will continue to detect the area until there is no object.
    if get_roi and start_pick_up:
        get_roi = False
        frame_gb = getMaskROI(frame_gb, roi, size)
    # Convert the image to LAB space.

    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # 将图像转换到LAB空间

    area_max = 0
    areaMaxContour = 0
    if not start_pick_up:
        for i in color_range:
            if i in __target_color:
                detect_color = i
                # Perform bit operations on the original image and the mask.
                frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0],
                                         color_range[detect_color][1])  # 对原图像和掩模进行位运算
                # open operation
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
                # close operation
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算
                # find the counter
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
                # find the largest counter
                areaMaxContour, area_max = getAreaMaxContour(contours)  # 找出最大轮廓
        # if there is a largest area
        if area_max > 2500:  # 有找到最大面积
            rect = cv2.minAreaRect(areaMaxContour)
            box = np.int0(cv2.boxPoints(rect))

            # get the roi area
            roi = getROI(box)  # 获取roi区域
            get_roi = True

            # get the location of object center
            img_centerx, img_centery = getCenter(rect, roi, size, square_length)  # 获取木块中心坐标
            # Transform to the world frame
            world_x, world_y = convertCoordinate(img_centerx, img_centery, size)  # 转换为现实世界坐标

            cv2.drawContours(img, [box], -1, range_rgb[detect_color], 2)
            # draw the center point
            cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1)  # 绘制中心点
            # Compare the last coordinate to determine whether to move
            distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2))  # 对比上次坐标来判断是否移动
            last_x, last_y = world_x, world_y
            track = True
            # print(count,distance)
            # 累计判断
            # Cumulative judgment
            if action_finish:
                if distance < 0.3:
                    center_list.extend((world_x, world_y))
                    count += 1
                    if start_count_t1:
                        start_count_t1 = False
                        t1 = time.time()
                    if time.time() - t1 > 1.5:
                        rotation_angle = rect[2]
                        start_count_t1 = True
                        world_X, world_Y = np.mean(np.array(center_list).reshape(count, 2), axis=0)
                        count = 0
                        center_list = []
                        start_pick_up = True
                else:
                    t1 = time.time()
                    start_count_t1 = True
                    count = 0
                    center_list = []
    return img


if __name__ == '__main__':
    init()
    start()
    __target_color = ('red',)
    my_camera = Camera.Camera()
    my_camera.camera_open()
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = run(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
