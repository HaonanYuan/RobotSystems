import math
import time

import cv2
import numpy as np
from Motion_class import *


class Handpose:
    def __init__(self):
        self.mot = motion()
        self.finger_num = None
        self.finger_num_lst = []
        self.count = 0
        # The signal of choose arm or palm
        self.choose_arm_sig = False
        self.choose_palm_sig = False
        self.servo1 = 500  # The gripper
        self.servo2 = 500  # The palm
        self.servo3 = 500  #
        self.servo4 = 500
        self.servo5 = 500
        self.servo6 = 500  # The arm base

    def get_euclidean_distance(self, a, b):
        # Calculate the euclidean distance between two point
        c = (float(a[0] - b[0]), float(a[1] - b[1]))
        return np.sqrt(c[0] ** 2 + c[1] ** 2)

    def remove_background(self, frame):
        # Clean the background noise
        fgbg = cv2.createBackgroundSubtractorMOG2()
        fgmask = fgbg.apply(frame)
        kernel = np.ones((3, 3), np.uint8)
        fgmask = cv2.erode(fgmask, kernel, iterations=1)
        res = cv2.bitwise_and(frame, frame, mask=fgmask)
        return res

    def skin_color(self, img):
        # detect the human skin and return the img
        # Transform to TCrCb space
        YCrCb = cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB)
        # Split the Y, Cr, Cb value
        (y, cr, cb) = cv2.split(YCrCb)
        cr1 = cv2.GaussianBlur(cr, (5, 5), 0)
        _, skin = cv2.threshold(cr1, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        res = cv2.bitwise_and(img, img, mask=skin)
        return res

    def get_contour(self, img):
        # Get the finger contour
        h = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # find the contour

        contour = h[0]
        contour = sorted(contour, key=cv2.contourArea, reverse=True)  # sort the known contour

        bg = np.ones(self.dst.shape, np.uint8) * 255  # Create the white background
        ret = cv2.drawContours(bg, contour[0], -1, (0, 0, 0), 3)  # Draw the finger contour
        return ret, contour

    def get_defect_count(self, frame, contour, defects, verbose=False):
        # Calculate the number of finger
        ndefects = 0
        for i in range(defects.shape[0]):
            s, e, f, _ = defects[i, 0]
            beg = tuple(contour[s][0])
            end = tuple(contour[e][0])
            far = tuple(contour[f][0])
            a = self.get_euclidean_distance(beg, end)
            b = self.get_euclidean_distance(beg, far)
            c = self.get_euclidean_distance(end, far)
            angle = math.acos((b ** 2 + c ** 2 - a ** 2) / (2 * b * c))
            # if the angle is acute, it means that it is a depression between two fingers
            # if the number of acute is 0, the handpose is zero or one
            # if the number of acute is 1, the handpose is two
            # if the number of acute is 2, the handpose is three
            # if the number of acute is 3, the handpose is four
            # if the number of acute is 4, the handpose is five
            if angle <= math.pi / 2:
                ndefects = ndefects + 1
            # Draw the fingertips
            if verbose:
                cv2.circle(frame, far, 5, (0, 0, 255), -1)
            if verbose:
                cv2.line(frame, beg, end, (0, 0, 255), 1)
        return frame, ndefects

    def get_convex(self, frame, contours):
        # get the position of convex point and return the number of acute angle
        hull = cv2.convexHull(contours, returnPoints=False)
        defects = cv2.convexityDefects(contours, hull)
        if defects is not None:
            frame, number = self.get_defect_count(frame, contours, defects, verbose=True)
            return frame, number
        return None, 0

    def camera_run(self, path):
        if path is None:
            cap = cv2.VideoCapture(0)
        else:
            cap = cv2.VideoCapture(path)

        while True:
            ret, frame = cap.read()
            frame = cv2.bilateralFilter(frame, 5, 50, 100)

            # change the window size
            src = cv2.resize(frame, (800, 600), interpolation=cv2.INTER_CUBIC)  # window size
            # a rectangle which show you use to identify the hand pose
            cv2.rectangle(src, (200, 150), (600, 450), (0, 0, 255))
            cv2.imshow("the original image", src)

            roi = src[150:450, 200:600]  # the area you want to use identify the hand pose
            resb = self.remove_background(roi)
            res = self.skin_color(resb)  # detect the skin
            cv2.imshow("skin detection", res)

            gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
            self.dst = cv2.Laplacian(gray, cv2.CV_16S, ksize=3)
            Laplacian = cv2.convertScaleAbs(self.dst)

            contour, array = self.get_contour(Laplacian)  # Get contour and draw it
            cv2.imshow("draw finger contour", contour)

            largecont = max(array, key=lambda contour: cv2.contourArea(contour))
            frame_res, self.finger_num = self.get_convex(contour, largecont)
            print('The number of finger: {}'.format(self.finger_num))
            cv2.imshow("draw the point of fingertips", frame_res)
            if self.count < 100:
                self.finger_num_lst.append(self.finger_num)
                self.count = self.count + 1
            elif self.count == 100:
                # Make sure which servo you want to move
                if not self.choose_arm_sig:
                    self.choose_arm_sig = self.choose_arm(self.finger_num_lst)
                    if self.choose_arm_sig:
                        self.mot.setBuzzer(1)
                        print('You have choose servo {} to control\n'.format(self.servo_num))
                    else:
                        print('You should continue to choose servo\n')
                else:
                    # you want to control palm or gripper
                    if self.servo_num == 5 and not self.choose_palm_sig:
                        self.choose_palm_sig = self.choose_palm_gripper(self.finger_num_lst)
                        if self.choose_palm_sig:
                            self.mot.setBuzzer(1)
                            print('You have choose servo {} to control'.format(self.servo_num))
                        else:
                            print('You should continue to choose servo\n')
                    else:
                        # after make sure which servo you want to control
                        # move the servo
                        self.move_arm(self.finger_num_lst)
            else:
                self.finger_num_lst = []
                self.count = 0
            # time.sleep(0.1)
            key = cv2.waitKey(50) & 0xFF
            if key == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

    def choose_palm_gripper(self, servo_hand):
        servo_hand = int(sum(servo_hand) / 100) + 1
        if servo_hand == 1:
            self.servo_num = 2
            print('You will control the palm')
            return True
        elif servo_hand == 2:
            self.servo_num = 1
            print('You will control the gripper')
            return True
        else:
            self.servo_num = None
            return False

    def choose_arm(self, servo_arm):
        servo_arm = int(sum(servo_arm) / 100) + 1
        if servo_arm == 1:
            # control the servo 6, the arm base
            self.servo_num = 6
            print('You will control servo 6')
            return True
        elif servo_arm == 2:
            self.servo_num = 5
            print('You will control servo 5')
            return True
        elif servo_arm == 3:
            self.servo_num = 4
            print('You will control servo 4')
            return True
        elif servo_arm == 4:
            self.servo_num = 3
            print('You will control servo 3')
            return True
        elif servo_arm == 5:
            self.servo_num = 2
            print('You will control gripper or palm')
            return True
        else:
            self.servo_num = None
            return False

    def move_arm(self, finger_number):
        fig_num = int(sum(finger_number) / 100) + 1
        print('The actually number of finger: {}\n'.format(fig_num))
        if self.servo_num == 6:
            # control the servo 6, the arm base
            if fig_num == 2:
                # Turn left
                self.servo6 = self.servo6 + 1
                Board.setBusServoPulse(6, self.servo6, 500)
            elif fig_num == 3:
                # Turn right
                self.servo6 = self.servo6 - 1
                Board.setBusServoPulse(6, self.servo6, 500)
            else:
                Board.setBusServoPulse(6, self.servo6, 500)
                self.choose_arm_sig = None
                self.choose_palm_sig = None
                self.mot.setBuzzer(0.5)
                self.mot.setBuzzer(0.5)
                time.sleep(1)
        elif self.servo_num == 5:
            # control the servo 5, the second joint
            if fig_num == 2:
                # raise arm
                self.servo5 = self.servo5 + 1
                Board.setBusServoPulse(5, self.servo5, 500)
            elif fig_num == 3:
                # Lower the arm
                self.servo5 = self.servo5 - 1
                Board.setBusServoPulse(5, self.servo5, 500)
            else:
                Board.setBusServoPulse(5, self.servo5, 500)
                self.choose_arm_sig = None
                self.choose_palm_sig = None
                self.mot.setBuzzer(0.5)
                self.mot.setBuzzer(0.5)
                time.sleep(1)
        elif self.servo_num == 4:
            # control the servo 4, the third joint
            if fig_num == 2:
                # raise arm
                self.servo4 = self.servo4 + 1
                Board.setBusServoPulse(4, self.servo4, 500)
            elif fig_num == 3:
                # Lower the arm
                self.servo4 = self.servo4 - 1
                Board.setBusServoPulse(4, self.servo4, 500)
            else:
                Board.setBusServoPulse(4, self.servo4, 500)
                self.choose_arm_sig = None
                self.choose_palm_sig = None
                self.mot.setBuzzer(0.5)
                self.mot.setBuzzer(0.5)
                time.sleep(1)
        elif self.servo_num == 3:
            # control the servo 3, the forth joint
            if fig_num == 2:
                # raise arm
                self.servo3 = self.servo3 + 1
                Board.setBusServoPulse(3, self.servo3, 500)
            elif fig_num == 3:
                # Lower the arm
                self.servo3 = self.servo3 - 1
                Board.setBusServoPulse(3, self.servo3, 500)
            else:
                Board.setBusServoPulse(3, self.servo3, 500)
                self.choose_arm_sig = None
                self.choose_palm_sig = None
                self.mot.setBuzzer(0.5)
                self.mot.setBuzzer(0.5)
                time.sleep(1)
        elif self.servo_num == 2:
            # control the servo 2, the palm joint
            if fig_num == 2:
                # clockwise rotate the palm
                self.servo2 = self.servo2 + 1
                Board.setBusServoPulse(2, self.servo2, 500)
            elif fig_num == 3:
                # counter-clockwise rotate the palm
                self.servo2 = self.servo2 - 1
                Board.setBusServoPulse(2, self.servo2, 500)
            else:
                Board.setBusServoPulse(2, self.servo2, 500)
                self.choose_arm_sig = None
                self.choose_palm_sig = None
                self.mot.setBuzzer(0.5)
                self.mot.setBuzzer(0.5)
                time.sleep(1)
        elif self.servo_num == 1:
            # control the servo 1, the gripper
            if fig_num == 2:
                # close the gripper
                self.servo1 = self.servo1 + 1
                Board.setBusServoPulse(1, self.servo1, 500)
            elif fig_num == 3:
                # open the gripper
                self.servo1 = self.servo1 - 1
                Board.setBusServoPulse(1, self.servo1, 500)
            else:
                Board.setBusServoPulse(1, self.servo1, 500)
                self.choose_arm_sig = None
                self.choose_palm_sig = None
                self.mot.setBuzzer(0.5)
                self.mot.setBuzzer(0.5)
                time.sleep(1)


if __name__ == '__main__':
    path = None
    hand = Handpose()
    mot = motion()
    # move the arm to the initial position
    mot.initMove()
    hand.camera_run(path)
