from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import time
import cv2
import math


class perception:

    def __init__(self):
        self.size = (640, 480)

    def calculation(self, lab_image, color, goal):
        # Perform bit operations on the original image and the mask.
        frame_mask = cv2.inRange(lab_image, color[goal][0],
                                 color[goal][1])  # 对原图像和掩模进行位运算
        # open operation
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # 开运算
        # close operation
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # 闭运算
        # find the counter
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓
        # find the largest counter
        areaMaxContour, area_max = self.getAreaMaxContour(contours)  # 找出最大轮廓
        return areaMaxContour, area_max

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

        return area_max_contour, contour_area_max  # return the index of contour and the area of contour

    def draw_block(self, rect, roi, img, box, color, goal, last_x, last_y):
        # get the location of object center
        img_centerx, img_centery = getCenter(rect, roi, self.size, square_length)  # 获取木块中心坐标
        # Transform to the world frame
        world_x, world_y = convertCoordinate(img_centerx, img_centery, self.size)  # 转换为现实世界坐标

        cv2.drawContours(img, [box], -1, color[goal], 2)
        # draw the center point
        cv2.putText(img, '(' + str(world_x) + ',' + str(world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color[goal], 1)  # 绘制中心点
        # Compare the last coordinate to determine whether to move
        distance = math.sqrt(pow(world_x - last_x, 2) + pow(world_y - last_y, 2))  # 对比上次坐标来判断是否移动
        last_x, last_y = world_x, world_y
        return distance, last_x, last_y


if __name__ == '__main__':
    pass
