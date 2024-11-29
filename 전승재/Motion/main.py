#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Set wheel_diameter and axle_track
wheel_diameter = 5.6
axle_track = 115

# Initialize the drive base
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
gyro = GyroSensor(Port.S1)

# robot.drive(50,0)
# wait(1000)
# robot.stop()
# robot.drive(50,0)
# wait(1000)
# robot.brok()

gyro.reset_angle(0)

while True:
    robot.turn(1)
    if gyro.angle() <= 90:
        break
    wait(1000)
