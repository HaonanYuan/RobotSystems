import rossros as ros
from picar_class import picar
import time

try:
    from ezblock import *
except ImportError:
    print(
        "This computer does not appear to be a PiCar - X system(/ opt / ezblock is not present ) . Shadowing hardware "
        "calls with substitute functions ")
    from sim_ezblock import *

sensor_bus = ros.Bus(name='sensor')
inter_bus = ros.Bus(name='interpreter')
control_bus = ros.Bus(name='control')
time_bus = ros.Bus(name='time')
ultra_bus = ros.Bus(name='ultrasonic')
ultra_inter_bus = ros.Bus(name='ultrasonic interpreter')


class UltrasonicProducer:

    def __init__(self):
        self.pin_D0 = Pin("D0")
        self.pin_D1 = Pin("D1")

    def ultra_value(self):
        pin_D0 = Pin("D0")
        pin_D1 = Pin("D1")
        distance = Ultrasonic(pin_D0, pin_D1).read()
        return distance

    def ultra_producer(self, output_bus, delay):
        while True:
            value = self.ultra_value()
            output_bus.set_message(value, 'ultrasonic')
            time.sleep(delay)


class UltrasonicInterpreter:

    def __init__(self, distance=10):
        self.distance = distance

    def go_stop(self, input_bus, output_bus, delay):
        while True:
            real_distance = input_bus.get_message('ultrasonic')
            if real_distance > self.distance:
                output_bus.set_message('go', 'ultrasonic interpreter')
            else:
                output_bus.set_message('stop', 'ultrasonic interpreter')
            time.sleep(delay)


def sensor_producer(value, output_bus):
    output_bus.set_message(value, 'sensor')


def sensor_value():
    sensor_poll_1 = ADC("A0")
    sensor_poll_2 = ADC("A1")
    sensor_poll_3 = ADC("A2")
    output = [sensor_poll_1.read(), sensor_poll_2.read(), sensor_poll_3.read()]
    return output


def inter_con_pro(sensor, interpreter, sensi=1000, plart=1):
    left = sensor.get_message('sensor')[0]
    center = sensor.get_message('sensor')[1]
    right = sensor.get_message('sensor')[2]

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
    interpreter.set_message([result, pos], 'interpreter')


def control_consumer(interpreter, ultra, control):
    if interpreter.get_message('interpreter')[0] == 'center':
        print('The car is running on the center line')
        control.set_dir_servo_angle(interpreter.get_message('interpreter')[1])
        if ultra.get_message('ultrasonic interpreter') == 'go':
            print('There is no obstacle')
            control.forward(50)
        elif ultra.get_message('ultrasonic interpreter') == 'stop':
            print('There is an obstacle')
            control.forward(0)

    elif interpreter.get_message('interpreter')[0] == 'left':
        print('The car is running offset the center, need to turn left')
        control.set_dir_servo_angle(-interpreter.get_message('interpreter')[1] * control.scale / 90)
        if ultra.get_message('ultrasonic interpreter') == 'go':
            print('There is no obstacle')
            control.forward(50)
        elif ultra.get_message('ultrasonic interpreter') == 'stop':
            print('There is an obstacle')
            control.forward(0)

    elif interpreter.get_message('interpreter')[0] == 'right':
        print('The car is running offset the center, need to turn right')
        control.set_dir_servo_angle(-interpreter.get_message('interpreter')[1] * control.scale / 90)
        if ultra.get_message('ultrasonic interpreter') == 'go':
            print('There is no obstacle')
            control.forward(50)
        elif ultra.get_message('ultrasonic interpreter') == 'stop':
            print('There is an obstacle')
            control.forward(0)

    else:
        print('The car is running in unknown environment')
        control.set_dir_servo_angle(0)
        if ultra.get_message('ultrasonic interpreter') == 'go':
            print('There is no obstacle')
            control.forward(50)
        elif ultra.get_message('ultrasonic interpreter') == 'stop':
            print('There is an obstacle')
            control.forward(0)


def sensor():
    res = time_bus.get_message('time')
    while not res:
        ros.Producer(producer_function=sensor_producer(sensor_value(), sensor_bus), output_busses=sensor_bus,
                     termination_busses=time_bus, delay=1, name='sensor_producer')
        time.sleep(1)


def sensor_inter():
    res = time_bus.get_message('time')
    while not res:
        ros.ConsumerProducer(consumer_producer_function=inter_con_pro(sensor_bus, inter_bus), input_busses=sensor_bus,
                             output_busses=inter_bus, termination_busses=time_bus, delay=1, name='interpreter')
        time.sleep(1)
    return inter_bus.get_message('interpreter')


def ultra():
    res = time_bus.get_message('time')
    while not res:
        ros.Producer(producer_function=UltrasonicProducer().ultra_producer(ultra_bus, 1), output_busses=ultra_bus,
                     termination_busses=time_bus, delay=1, name='ultrasonic_producer')
        time.sleep(1)


def ultra_inter():
    res = time_bus.get_message('time')
    while not res:
        ros.ConsumerProducer(consumer_producer_function=UltrasonicInterpreter().go_stop(ultra_bus, ultra_inter_bus, 1),
                             input_busses=ultra_bus, output_busses=ultra_inter_bus, termination_busses=time_bus,
                             delay=1, name='ultrasonic_interpreter')
        time.sleep(1)
    return ultra_inter_bus.get_message('ultrasonic_interpreter')


def control():
    res = time_bus.get_message('time')
    while not res:
        ros.Consumer(consumer_function=control_consumer(inter_bus, ultra_inter_bus, picar(Servo, PWM, Pin)),
                     input_busses=inter_bus,
                     termination_busses=time_bus, delay=1, name='control')
        time.sleep(1)


def time_control():
    res = ros.Timer(time_bus, delay=0, name='Test termination timer').timer()
    time_bus.set_message(res, 'Test termination timer')


if __name__ == '__main__':
    producer_consumer_list = [ultra, sensor, ultra_inter, sensor_inter, control, time_control]
    ros.runConcurrently(producer_consumer_list)
