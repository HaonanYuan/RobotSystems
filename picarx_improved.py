try:
    from ezblock import *
except ImportError:
    print(
        "This computer does not appear to be a PiCar - X system(/ opt / ezblock is not present ) . Shadowing hardware "
        "calls with substitute functions ")
    from sim_ezblock import *

import time
import logging
from logdecorator import log_on_start, log_on_end, log_on_error

logging_format = "%(asctime)s:%(message)s"
logging.basicConfig(format=logging_format, level=logging.INFO, datefmt="%H:%M:%S")
logging.getLogger().setLevel(logging.DEBUG)

PERIOD = 4095
PRESCALER = 10
TIMEOUT = 0.02

dir_servo_pin = Servo(PWM('P2'))
camera_servo_pin1 = Servo(PWM('P0'))
camera_servo_pin2 = Servo(PWM('P1'))
left_rear_pwm_pin = PWM("P13")
right_rear_pwm_pin = PWM("P12")
left_rear_dir_pin = Pin("D4")
right_rear_dir_pin = Pin("D5")

S0 = ADC('A0')
S1 = ADC('A1')
S2 = ADC('A2')

Servo_dir_flag = 1
dir_cal_value = 0
cam_cal_value_1 = 0
cam_cal_value_2 = 0
motor_direction_pins = [left_rear_dir_pin, right_rear_dir_pin]
motor_speed_pins = [left_rear_pwm_pin, right_rear_pwm_pin]
cali_dir_value = [1, -1]
cali_speed_value = [0, 0]
# 初始化PWM引脚
for pin in motor_speed_pins:
    pin.period(PERIOD)
    pin.prescaler(PRESCALER)


# @log_on_start(logging.DEBUG, "set motor speed ")
def set_motor_speed(motor, speed):
    global cali_speed_value, cali_dir_value
    motor -= 1
    if speed >= 0:
        direction = 1 * cali_dir_value[motor]
    elif speed < 0:
        direction = -1 * cali_dir_value[motor]
    speed = abs(speed)
    if speed != 0:
        speed = int(speed / 2) + 50
    speed = speed - cali_speed_value[motor]
    if direction < 0:
        motor_direction_pins[motor].high()
        motor_speed_pins[motor].pulse_width_percent(speed)
    else:
        motor_direction_pins[motor].low()
        motor_speed_pins[motor].pulse_width_percent(speed)


def motor_speed_calibration(value):
    global cali_speed_value, cali_dir_value
    cali_speed_value = value
    if value < 0:
        cali_speed_value[0] = 0
        cali_speed_value[1] = abs(cali_speed_value)
    else:
        cali_speed_value[0] = abs(cali_speed_value)
        cali_speed_value[1] = 0


def motor_direction_calibration(motor, value):
    # 0: positive direction
    # 1:negative direction
    global cali_dir_value
    motor -= 1
    if value == 1:
        cali_dir_value[motor] = -1 * cali_dir_value[motor]


def dir_servo_angle_calibration(value):
    global dir_cal_value
    dir_cal_value = value
    set_dir_servo_angle(dir_cal_value)
    # dir_servo_pin.angle(dir_cal_value)


def set_dir_servo_angle(value):
    global dir_cal_value
    dir_servo_pin.angle(value + dir_cal_value)


def camera_servo1_angle_calibration(value):
    global cam_cal_value_1
    cam_cal_value_1 = value
    set_camera_servo1_angle(cam_cal_value_1)
    # camera_servo_pin1.angle(cam_cal_value)


def camera_servo2_angle_calibration(value):
    global cam_cal_value_2
    cam_cal_value_2 = value
    set_camera_servo2_angle(cam_cal_value_2)
    # camera_servo_pin2.angle(cam_cal_value)


def set_camera_servo1_angle(value):
    global cam_cal_value_1
    camera_servo_pin1.angle(-1 * (value + cam_cal_value_1))


def set_camera_servo2_angle(value):
    global cam_cal_value_2
    camera_servo_pin2.angle(-1 * (value + cam_cal_value_2))


def get_adc_value():
    adc_value_list = []
    adc_value_list.append(S0.read())
    adc_value_list.append(S1.read())
    adc_value_list.append(S2.read())
    return adc_value_list


def set_power(speed):
    set_motor_speed(1, speed)
    set_motor_speed(2, speed)


def backward(speed, angle=0):
    if angle == 0:
        set_motor_speed(1, speed)
        set_motor_speed(2, speed)
    elif angle > 0:
        # car will turn right
        # right rear wheel will slower than left one
        set_motor_speed(1, speed)
        set_motor_speed(2, (angle / 90) * speed)
    elif angle < 0:
        # car will turn left
        # right rear wheel will faster than left one
        set_motor_speed(1, abs((angle / 90)) * speed)
        set_motor_speed(2, speed)


def forward(speed, angle=0):
    if angle == 0:
        set_motor_speed(1, -1 * speed)
        set_motor_speed(2, -1 * speed)
    elif angle > 0:
        # car will turn right
        # right rear wheel will slower than left one
        set_motor_speed(1, -1 * speed)
        set_motor_speed(2, -1 * (angle / 90) * speed)
    elif angle < 0:
        # car will turn left
        # right rear wheel will faster than left one
        set_motor_speed(1, -1 * abs((angle / 90)) * speed)
        set_motor_speed(2, -1 * speed)


def stop():
    set_motor_speed(1, 0)
    set_motor_speed(2, 0)


def Get_distance():
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


def test():
    # dir_servo_angle_calibration(-10)
    set_dir_servo_angle(-40)
    # time.sleep(1)
    # set_dir_servo_angle(0)
    # time.sleep(1)
    # set_motor_speed(1, 1)
    # set_motor_speed(2, 1)
    # camera_servo_pin.angle(0)


def maneuvering_a(cmd):
    # forward
    if cmd == 'forward':
        # set_dir_servo_angle(0)
        forward(50)
    # backward
    elif cmd == 'backward':
        backward(50)
    # forward turn right
    elif cmd == 'fright':
        set_dir_servo_angle(30)
        forward(50, 30)
    # forward turn left
    elif cmd == 'fleft':
        set_dir_servo_angle((-30))
        forward(50, (-30))
    # backward turn right
    elif cmd == 'bright':
        set_dir_servo_angle(30)
        backward(50, 30)
    # backward turn left
    elif cmd == 'bleft':
        set_dir_servo_angle((-30))
        backward(50, (-30))

    # stop
    elif cmd == 'stop':
        stop()


def maneuvering_b(cmd):
    # parallel-parking left
    if cmd == 'left':
        # initial servo angle
        set_dir_servo_angle(0)
        # go forward and turn left
        forward(50)
        set_dir_servo_angle((-30))
        time.sleep(1)
        # turn right
        set_dir_servo_angle(30)
        time.sleep(1)
        # back to initial state
        stop()
        set_dir_servo_angle(0)
    elif cmd == 'right':
        # initial servo angle
        set_dir_servo_angle(0)
        # go forward and turn left
        forward(50)
        set_dir_servo_angle(30)
        time.sleep(1)
        # turn right
        set_dir_servo_angle((-30))
        time.sleep(1)
        # back to initial state
        stop()
        set_dir_servo_angle(0)
    else:
        stop()
        set_dir_servo_angle(0)
        print('please re-type command(left/right)')


def maneuvering_c(cmd):
    # k turn
    if cmd == 'left':
        # initial servo angle
        set_dir_servo_angle(0)
        # go first step
        set_dir_servo_angle((-30))
        forward(50)
        time.sleep(1)
        # go second step
        stop()
        set_dir_servo_angle(30)
        backward(50)
        time.sleep(1)
        # third step
        stop()
        set_dir_servo_angle((-30))
        forward(50)
        time.sleep(1)
        set_dir_servo_angle(0)
        time.sleep(1)
        # back to initial state
        stop()
        set_dir_servo_angle(0)

    elif cmd == 'right':
        # initial servo angle
        set_dir_servo_angle(0)
        # go first step
        set_dir_servo_angle(30)
        forward(50)
        time.sleep(1)
        # go second step
        stop()
        set_dir_servo_angle((-30))
        backward(50)
        time.sleep(1)
        # third step
        stop()
        set_dir_servo_angle(30)
        forward(50)
        time.sleep(1)
        set_dir_servo_angle(0)
        time.sleep(1)
        # back to initial state
        stop()
        set_dir_servo_angle(0)


def maneuvering_d():
    # back to initial state
    stop()
    set_dir_servo_angle(0)


if __name__ == "__main__":
    run = True
    print('command info:\n')
    print('task a: Forward and backward in straight lines or with different steering angles\n')
    print('task b: Parallel-parking left and right\n')
    print('task c: Three-point turning (K-turning) with initial turn to the left or right\n')
    print('task d: Stop the car and back to initial state\n')
    print('task e: End the whole task')
    while run:
        cmd = input('Please choose a task(a/b/c/d/e): ')
        if cmd == 'a':
            d_cmd = input('Choose a command(forward/backward/fleft/fright/bleft/bright): ')
            maneuvering_a(d_cmd)
        elif cmd == 'b':
            d_cmd = input('Choose a command(left/right): ')
            maneuvering_b(d_cmd)
        elif cmd == 'c':
            d_cmd = input('Choose a command(left/right): ')
            maneuvering_c(d_cmd)
        elif cmd == 'd':
            maneuvering_d()
        elif cmd == 'e':
            stop()
            set_dir_servo_angle(0)
            print('Task will be end!')
            run = False
        else:
            print('Please choose a task(a/b/c/d): ')
