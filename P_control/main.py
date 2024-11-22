#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import UARTDevice

ev3 = EV3Brick()
ser = UARTDevice(Port.S3, baudrate=115200)

left_motor = Motor (Port.C)
right_motor = Motor (Port.B)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=115)

def data_filter(data):
    decoded_data = data.decode().strip()
    if decoded_data.isdigit():
        return int(decoded_data)
    return None
def pd_control(cam_data, kp, kd, power):
    global previous_error
    error = threshold - cam_data
    derivative = error - previous_error
    output = (kp * error) + (kd * derivative)
    robot.drive(power, output)
    previous_error = error

threshold = 200
previous_error = 0

while True:
    data = ser.read_all()
    if data:
        filtered_data = data_filter(data)
        if filtered_data is not None:
            print(filtered_data)
            pd_control(filtered_data, kp = 0.5, kd = 0.5, power = 40)
    wait(10)