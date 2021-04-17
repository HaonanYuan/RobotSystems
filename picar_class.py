import atexit

try:
    from ezblock import *
except ImportError:
    print(
        "This computer does not appear to be a PiCar - X system(/ opt / ezblock is not present ) . Shadowing hardware "
        "calls with substitute functions ")
    from sim_ezblock import *

import time
import cv2
from camera_follow import HandCodedLaneFollower
import datetime


class picar:
    def __init__(self, Servo, PWM, Pin):
        self.PERIOD = 4095
        self.PRESCALER = 10
        self.TIMEOUT = 0.02

        self.dir_servo_pin = Servo(PWM('P2'))
        self.camera_servo_pin1 = Servo(PWM('P0'))
        self.camera_servo_pin2 = Servo(PWM('P1'))
        self.left_rear_pwm_pin = PWM("P13")
        self.right_rear_pwm_pin = PWM("P12")
        self.left_rear_dir_pin = Pin("D4")
        self.right_rear_dir_pin = Pin("D5")

        self.Servo_dir_flag = 1
        self.dir_cal_value = 0
        self.cam_cal_value_1 = 0
        self.cam_cal_value_2 = 0
        self.motor_direction_pins = [self.left_rear_dir_pin, self.right_rear_dir_pin]
        self.motor_speed_pins = [self.left_rear_pwm_pin, self.right_rear_pwm_pin]
        self.cali_dir_value = [1, -1]
        self.cali_speed_value = [0, 0]

        atexit.register(self.cleanup)
        # 初始化PWM引脚
        for pin in self.motor_speed_pins:
            pin.period(self.PERIOD)
            pin.prescaler(self.PRESCALER)

    def set_motor_speed(self, motor, speed):
        motor -= 1
        if speed >= 0:
            direction = 1 * self.cali_dir_value[motor]
        elif speed < 0:
            direction = -1 * self.cali_dir_value[motor]
        speed = abs(speed)
        # if speed != 0:
        #     speed = int(speed / 2) + 50
        speed = speed - self.cali_speed_value[motor]
        if direction < 0:
            self.motor_direction_pins[motor].high()
            self.motor_speed_pins[motor].pulse_width_percent(speed)
        else:
            self.motor_direction_pins[motor].low()
            self.motor_speed_pins[motor].pulse_width_percent(speed)

    def motor_speed_calibration(self, value):
        self.cali_speed_value = value
        if value < 0:
            self.cali_speed_value[0] = 0
            self.cali_speed_value[1] = abs(self.cali_speed_value)
        else:
            self.cali_speed_value[0] = abs(self.cali_speed_value)
            self.cali_speed_value[1] = 0

    def motor_direction_calibration(self, motor, value):
        # 0: positive direction
        # 1:negative direction
        motor -= 1
        if value == 1:
            self.cali_dir_value[motor] = -1 * self.cali_dir_value[motor]

    def dir_servo_angle_calibration(self, value):
        self.dir_cal_value = value
        self.set_dir_servo_angle(self.dir_cal_value)
        # dir_servo_pin.angle(dir_cal_value)

    def set_dir_servo_angle(self, value):
        self.dir_servo_pin.angle(value + self.dir_cal_value)

    def set_power(self, speed):
        self.set_motor_speed(1, speed)
        self.set_motor_speed(2, speed)

    def backward(self, speed, angle=0):
        if angle == 0:
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, speed)
        elif angle > 0:
            # car will turn right
            # right rear wheel will slower than left one
            self.set_motor_speed(1, speed)
            self.set_motor_speed(2, (angle / 90) * speed)
        elif angle < 0:
            # car will turn left
            # right rear wheel will faster than left one
            self.set_motor_speed(1, abs((angle / 90)) * speed)
            self.set_motor_speed(2, speed)

    def forward(self, speed, angle=0):
        if angle == 0:
            self.set_motor_speed(1, -1 * speed)
            self.set_motor_speed(2, -1 * speed)
        elif angle > 0:
            # car will turn right
            # right rear wheel will slower than left one
            self.set_motor_speed(1, -1 * speed)
            self.set_motor_speed(2, -1 * (angle / 90) * speed)
        elif angle < 0:
            # car will turn left
            # right rear wheel will faster than left one
            self.set_motor_speed(1, -1 * abs((angle / 90)) * speed)
            self.set_motor_speed(2, -1 * speed)

    def stop(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)

    def Get_distance(self):
        timeout = 0.01
        trig = Pin('D8')
        echo = Pin('D9')

        trig.low()
        time.sleep(0.01)
        trig.high()
        time.sleep(0.000015)
        trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        while echo.value() == 0:
            pulse_start = time.time()
            if pulse_start - timeout_start > timeout:
                return -1
        while echo.value() == 1:
            pulse_end = time.time()
            if pulse_end - timeout_start > timeout:
                return -2
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        # print(cm)
        return cm

    def maneuvering_a(self, cmd):
        # forward
        if cmd == 'forward':
            # set_dir_servo_angle(0)
            self.forward(50)
        # backward
        elif cmd == 'backward':
            self.backward(50)
        # forward turn right
        elif cmd == 'fright':
            self.set_dir_servo_angle(30)
            self.forward(50, 30)
        # forward turn left
        elif cmd == 'fleft':
            self.set_dir_servo_angle((-30))
            self.forward(50, (-30))
        # backward turn right
        elif cmd == 'bright':
            self.set_dir_servo_angle(30)
            self.backward(50, 30)
        # backward turn left
        elif cmd == 'bleft':
            self.set_dir_servo_angle((-30))
            self.backward(50, (-30))

        # stop
        elif cmd == 'stop':
            self.stop()

    def maneuvering_b(self, cmd):
        # parallel-parking left
        if cmd == 'left':
            # initial servo angle
            self.set_dir_servo_angle(0)
            # go forward and turn left
            self.forward(50)
            self.set_dir_servo_angle((-30))
            time.sleep(1)
            # turn right
            self.set_dir_servo_angle(30)
            time.sleep(1)
            # back to initial state
            self.stop()
            self.set_dir_servo_angle(0)
        elif cmd == 'right':
            # initial servo angle
            self.set_dir_servo_angle(0)
            # go forward and turn left
            self.forward(50)
            self.set_dir_servo_angle(30)
            time.sleep(1)
            # turn right
            self.set_dir_servo_angle((-30))
            time.sleep(1)
            # back to initial state
            self.stop()
            self.set_dir_servo_angle(0)
        else:
            self.stop()
            self.set_dir_servo_angle(0)
            print('please re-type command(left/right)')

    def maneuvering_c(self, cmd):
        # k turn
        if cmd == 'left':
            # initial servo angle
            self.set_dir_servo_angle(0)
            # go first step
            self.set_dir_servo_angle((-30))
            self.forward(50)
            time.sleep(1)
            # go second step
            self.stop()
            self.set_dir_servo_angle(30)
            self.backward(50)
            time.sleep(1)
            # third step
            self.stop()
            self.set_dir_servo_angle((-10))
            self.forward(50)
            time.sleep(1)
            self.set_dir_servo_angle(0)
            time.sleep(1)
            # back to initial state
            self.stop()
            self.set_dir_servo_angle(0)

        elif cmd == 'right':
            # initial servo angle
            self.set_dir_servo_angle(0)
            # go first step
            self.set_dir_servo_angle(30)
            self.forward(50)
            time.sleep(1)
            # go second step
            self.stop()
            self.set_dir_servo_angle((-30))
            self.backward(50)
            time.sleep(1)
            # third step
            self.stop()
            self.set_dir_servo_angle(10)
            self.forward(50)
            time.sleep(1)
            self.set_dir_servo_angle(0)
            time.sleep(1)
            # back to initial state
            self.stop()
            self.set_dir_servo_angle(0)

    def maneuvering_d(self):
        # back to initial state
        self.stop()
        self.set_dir_servo_angle(0)

    def cleanup(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)


class sensor:
    def __init__(self, adc):
        """
        :param adc: The ADC class
        """
        self.adc = adc
        self.output = None

    def sensor_read(self):
        adc_A0 = self.adc("A0")
        adc_A1 = self.adc("A1")
        adc_A2 = self.adc("A2")
        self.output = [adc_A0.read(), adc_A1.read(), adc_A2.read()]

        return self.output


class interpreter:

    def __init__(self, sensitivity=1000, polarity=1):
        """
        :param sensitivity: how different “dark” and “light” readings are expected to be
        :param polarity: is the line the system is following darker or lighter than the surrounding floor
        """
        self.senstvt = int(sensitivity)
        self.plart = int(polarity)

    def main(self, data):
        """
        :param data: it is a list [float, float, float]
        :return:
        """
        left = data[0]
        center = data[1]
        right = data[2]

        if self.plart > 0:
            # darker than the surrounding floor
            if center < self.senstvt < left and self.senstvt < right:
                result = 'center'
                pos = 0
            elif right < self.senstvt < left and self.senstvt < center:
                result = 'right'
                pos = -(abs(right - self.senstvt) / self.senstvt)
            elif center > self.senstvt > left and self.senstvt < right:
                result = 'left'
                pos = abs(left - self.senstvt) / self.senstvt
            else:
                result = 'None'
                pos = 'None'
        else:
            # lighter than the surrounding floor
            if center > self.senstvt > right and self.senstvt > left:
                result = 'center'
                pos = 0
            elif right > self.senstvt > left and self.senstvt > center:
                result = 'right'
                pos = -(abs(right - self.senstvt) / self.senstvt)
            elif center < self.senstvt < left and self.senstvt > right:
                result = 'left'
                pos = abs(left - self.senstvt) / self.senstvt
            else:
                result = 'None'
                pos = 'None'

        return result, pos


class controller:

    def __init__(self, picar, scale):
        """
        :param scale: scaling factor
        """
        self.scale = scale
        self.picarx = picar

    def main(self, res, pos):
        if res == 'center':
            print('The car is running on the center line')
            self.picarx.set_dir_servo_angle(pos)
            self.picarx.forward(50)
            return pos
        elif res == 'left':
            print('The car is running offset the center, need to turn left')
            self.picarx.set_dir_servo_angle(-pos * self.scale / 90)
            self.picarx.forward(50)
            return -pos * self.scale / 90
        elif res == 'right':
            print('The car is running offset the center, need to turn right')
            self.picarx.set_dir_servo_angle(-pos * self.scale / 90)
            self.picarx.forward(50)
            return -pos * self.scale / 90
        else:
            print('The car is running in unknown environment')
            self.picarx.set_dir_servo_angle(0)
            self.picarx.forward(0)
            return 0


def Motor_commands(picar_x):
    # Assignment two
    run = True
    print('command info:\n')
    print('task a: Forward and backward in straight lines or with different steering angles\n')
    print('task b: Parallel-parking left and right\n')
    print('task c: Three-point turning (K-turning) with initial turn to the left or right\n')
    print('task d: Stop the car and back to initial state\n')
    print('task e: End the whole task\n')
    while run:
        cmd = input('Please choose a task(a/b/c/d/e): ')
        if cmd == 'a':
            d_cmd = input('Choose a command(forward/backward/fleft/fright/bleft/bright): ')
            picar_x.maneuvering_a(d_cmd)
        elif cmd == 'b':
            d_cmd = input('Choose a command(left/right): ')
            picar_x.maneuvering_b(d_cmd)
        elif cmd == 'c':
            d_cmd = input('Choose a command(left/right): ')
            picar_x.maneuvering_c(d_cmd)
        elif cmd == 'd':
            picar_x.maneuvering_d()
        elif cmd == 'e':
            picar_x.stop()
            picar_x.set_dir_servo_angle(0)
            print('Task will be end!')
            run = False
        else:
            print('Please choose a task(a/b/c/d): ')


def Sensors_and_control(sensr, interpt, cnto, run=True):
    while run:
        data = sensr.sensor_read()
        print('The value of each I/O: {}'.format(data))
        # time.sleep(1)
        text, position = interpt.main(data)
        angle = cnto.main(text, position)
        print('The angle is {}'.format(angle))
        time.sleep(1)


def Camera_based_driving(pic, cam, fou, lanefollower):
    pic.forward(40)
    cam.set(3, 320)
    cam.set(4, 240)
    # datestr = datetime.datetime.now().strftime("%y%m%d_%H%M%S")
    video_orig = cv2.VideoWriter('../data/tmp/car_video.avi', fou, 20, (320, 240))
    video_lane = cv2.VideoWriter('../data/tmp/car_video_lane.avi', fou, 20, (320, 240))
    # video_objs = cv2.VideoWriter('../data/tmp/car_video_objs.avi', fou, 20, (320, 240))
    i = 0
    while cam.isOpened():
        _, image_lane = cam.read()
        image_objs = image_lane.copy()
        i += 1
        video_orig.write(image_lane)

        """image_objs = process_objects_on_road(image_objs)
        video_objs.write(image_objs)
        show_image('Detected Objects', image_objs)"""

        image_lane = lanefollower.follow_lane(image_lane)
        video_lane.write(image_lane)
        cv2.imshow('Lane Lines', image_lane)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            """self.back_wheels.speed = 0
            self.front_wheels.turn(90)"""
            cam.release()
            video_orig.release()
            video_lane.release()
            # video_objs.release()
            cv2.destroyAllWindows()
            break


if __name__ == "__main__":
    picarx = picar(Servo, PWM, Pin)
    sensorx = sensor(ADC)
    controllerx = controller(picarx, scale=9000)
    print('Which assignment you want to test?\n')
    print('First: Motor commands\n')
    print('Second: Sensors and control\n')
    print('Third: Camera based driving\n')
    index = input('Please enter the name of assignment(full name, include space, capitalization): ')
    if index == 'Motor commands':
        Motor_commands(picarx)
    elif index == 'Sensors and control':
        sensitivity = input('Please enter the value of sensitivity: ')
        polarity = input('Please enter the value of polarity: ')
        interpreterx = interpreter(int(sensitivity), int(polarity))
        Sensors_and_control(sensorx, interpreterx, controllerx)
    elif index == 'Camera based driving':
        hand = HandCodedLaneFollower(car=picarx)
        camera = cv2.VideoCapture(-1)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        Camera_based_driving(picarx, camera, fourcc, hand)
    else:
        pass
