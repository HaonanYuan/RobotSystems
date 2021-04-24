import concurrent.futures
from threading import Lock
import time
from picar_class import picar

try:
    from ezblock import *
except ImportError:
    print(
        "This computer does not appear to be a PiCar - X system(/ opt / ezblock is not present ) . Shadowing hardware "
        "calls with substitute functions ")
    from sim_ezblock import *


class sensor_values_bus(object):

    def __init__(self):
        self.message = [0, 1, 0]

    def write(self, value):
        self.message = value

    def read(self):
        return self.message


def sensor_producer(sen, value=None):
    sen.write(value)


def sensor_consumer(sen):
    return sen.read()


def sensor_con_pro(sensor, delay):
    lock = Lock()
    with lock:
        sensor_poll_1 = ADC("A0")
        sensor_poll_2 = ADC("A1")
        sensor_poll_3 = ADC("A2")
        output = [sensor_poll_1.read(), sensor_poll_2.read(), sensor_poll_3.read()]
    while True:
        sensor_producer(sensor, output)
        time.sleep(delay)


class interpreter_bus:

    def __init__(self, sensitivity=1000, polarity=1):
        self.sensi = sensitivity
        self.plart = polarity
        self.message = ['center', 0]

    def write(self, value):
        self.message = value

    def read(self):
        return self.message


def inter_producer(inter, value=None):
    inter.write(value)


def inter_consumer(inter):
    value = inter.read()
    return value


def inter_con_pro(sensor, interpreter, delay):
    while True:
        left = sensor.read()[0]
        center = sensor.read()[1]
        right = sensor.read()[2]

        if interpreter.plart > 0:
            # darker than the surrounding floor
            if center < interpreter.sensi < left and interpreter.sensi < right:
                result = 'center'
                pos = 0
            elif right < interpreter.sensi < left and interpreter.sensi < center:
                result = 'right'
                pos = -(abs(right - interpreter.sensi) / interpreter.sensi)
            elif center > interpreter.sensi > left and interpreter.sensi < right:
                result = 'left'
                pos = abs(left - interpreter.sensi) / interpreter.sensi
            else:
                result = 'None'
                pos = 0
        else:
            # lighter than the surrounding floor
            if center > interpreter.sensi > right and interpreter.sensi > left:
                result = 'center'
                pos = 0
            elif right > interpreter.sensi > left and interpreter.sensi > center:
                result = 'right'
                pos = -(abs(interpreter.sensi) / interpreter.sensi)
            elif center < interpreter.sensi < left and interpreter.sensi > right:
                result = 'left'
                pos = abs(left - interpreter.sensi) / interpreter.sensi
            else:
                result = 'None'
                pos = 0
        inter_producer(interpreter, [result, pos])
        time.sleep(delay)


class control_bus:

    def __init__(self, message=None):
        self.picarx = picar(Servo, PWM, Pin)
        self.message = message

    def write(self, value):
        self.message = value

    def read(self):
        return self.message


def control_con_pro(interpreter, control, delay):
    while True:
        if interpreter.read()[0] == 'center':
            print('The car is running on the center line')
            control.picarx.set_dir_servo_angle(interpreter.read()[1])
            control.picarx.forward(50)
            time.sleep(delay)
            return interpreter.read()[1]
        elif interpreter.read()[0] == 'left':
            print('The car is running offset the center, need to turn left')
            control.picarx.set_dir_servo_angle(-interpreter.read()[1] * control.scale / 90)
            control.picarx.forward(50)
            time.sleep(delay)
            return -interpreter.read()[1] * control.scale / 90

        elif interpreter.read()[0] == 'right':
            print('The car is running offset the center, need to turn right')
            control.picarx.set_dir_servo_angle(-interpreter.read()[1] * control.scale / 90)
            control.picarx.forward(50)
            time.sleep(delay)
            return -interpreter.read()[1] * control.scale / 90
        else:
            print('The car is running in unknown environment')
            control.picarx.set_dir_servo_angle(0)
            control.picarx.forward(0)
            time.sleep(delay)
            return 0


if __name__ == "__main__":
    sensor_delay = 1
    interpreter_delay = 1
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        eSensor = executor.submit(sensor_con_pro, sensor_values_bus(), sensor_delay)
        eInterpreter = executor.submit(inter_con_pro, sensor_values_bus(), interpreter_bus(), interpreter_delay)
    eSensor.result()
