import atexit

try:
    from ezblock import *
except ImportError:
    print(
        "This computer does not appear to be a PiCar - X system(/ opt / ezblock is not present ) . Shadowing hardware "
        "calls with substitute functions ")
    from sim_ezblock import *

import time


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

    def test(self):
        # dir_servo_angle_calibration(-10)
        self.set_dir_servo_angle(-40)
        # time.sleep(1)
        # set_dir_servo_angle(0)
        # time.sleep(1)
        # set_motor_speed(1, 1)
        # set_motor_speed(2, 1)
        # camera_servo_pin.angle(0)

    def cleanup(self):
        self.set_motor_speed(1, 0)
        self.set_motor_speed(2, 0)


if __name__ == "__main__":
    pass
"""     try:
         # dir_servo_angle_calibration(-10)
         while 1:
             test()
     finally:
         stop()"""
