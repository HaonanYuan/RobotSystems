import concurrent.futures
from threading import Lock
import time
from picar_class import picar
import rossros as ros

try:
    from ezblock import *
except ImportError:
    print(
        "This computer does not appear to be a PiCar - X system(/ opt / ezblock is not present ) . Shadowing hardware "
        "calls with substitute functions ")
    from sim_ezblock import *


class Bus:

    def __init__(self, message=None, name='unnamed bus'):
        self.message = message
        self.name = name

    def write(self, value):
        self.message = value

    def read(self):
        return self.message


def sensor_producer(sen, value=None):
    sen.write(value)


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


def inter_producer(inter, value=None):
    inter.write(value)


def inter_consumer(inter):
    value = inter.read()
    return value


def inter_con_pro(sensor, interpreter, delay, sensi=1000, plart=1):
    while True:
        left = sensor.read()[0]
        center = sensor.read()[1]
        right = sensor.read()[2]

        if plart > 0:
            # darker than the surrounding floor
            if center < sensi < left and sensi < right:
                result = 'center'
                pos = 0
            elif right < sensi < left and sensi < center:
                result = 'right'
                pos = -(abs(right - sensi) / sensi)
            elif center > sensi > left and sensi < right:
                result = 'left'
                pos = abs(left - sensi) / sensi
            else:
                result = 'None'
                pos = 0
        else:
            # lighter than the surrounding floor
            if center > sensi > right and sensi > left:
                result = 'center'
                pos = 0
            elif right > sensi > left and sensi > center:
                result = 'right'
                pos = -(abs(sensi) / sensi)
            elif center < sensi < left and sensi > right:
                result = 'left'
                pos = abs(left - sensi) / sensi
            else:
                result = 'None'
                pos = 0
        inter_producer(interpreter, [result, pos])
        time.sleep(delay)


def control_con_pro(interpreter, control, delay):
    while True:
        if interpreter.read()[0] == 'center':
            print('The car is running on the center line')
            control.set_dir_servo_angle(interpreter.read()[1])
            control.forward(50)
            time.sleep(delay)
            return interpreter.read()[1]

        elif interpreter.read()[0] == 'left':
            print('The car is running offset the center, need to turn left')
            control.set_dir_servo_angle(-interpreter.read()[1] * control.scale / 90)
            control.forward(50)
            time.sleep(delay)
            return -interpreter.read()[1] * control.scale / 90

        elif interpreter.read()[0] == 'right':
            print('The car is running offset the center, need to turn right')
            control.set_dir_servo_angle(-interpreter.read()[1] * control.scale / 90)
            control.forward(50)
            time.sleep(delay)
            return -interpreter.read()[1] * control.scale / 90
        else:
            print('The car is running in unknown environment')
            control.set_dir_servo_angle(0)
            control.forward(0)
            time.sleep(delay)
            return 0


if __name__ == "__main__":
    sensor_delay = 1
    interpreter_delay = 1
    control_delay = 1
    sensor_bus = Bus(name='sensor')
    interpreter_bus = Bus(name='interpreter')
    control_bus = Bus(name='control')
    with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
        eSensor = executor.submit(sensor_con_pro, sensor_bus, sensor_delay)
        eInterpreter = executor.submit(inter_con_pro, sensor_bus, interpreter_bus, interpreter_delay)
        eControl = executor.submit(control_con_pro, interpreter_bus, picar(Servo, PWM, Pin), control_delay)
    eSensor.result()
