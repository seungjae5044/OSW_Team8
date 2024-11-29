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

# robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)

def reset_motor():
    
    shooting_motor.run_until_stalled(-180,duty_limit=40)
    grab_motor.run_until_stalled(180,duty_limit=40)

    grab_motor.reset_angle(0)
    shooting_motor.reset_angle(0)

def grab_ball():
    grab_motor.run_until_stalled(180,duty_limit=40)
    grab_motor.reset_angle(0)

def find_ball():
    grab_motor.run_target(500,-90)

def open_arm():
    grab_motor.run_target(1000,-160)

def shooting():
    shooting_motor.run_target(2000,70)
    reset_motor()
    # shooting_motor.run_target(500,0)

def turn(degree):
    while True:
        angle = gyro.angle()
        if angle > degree:
            break
        robot.turn(1)


# Initialize the motors.
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Set wheel_diameter and axle_track
wheel_diameter = 5.6
axle_track = 115

# Initialize the drive base
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
gyro = GyroSensor(Port.S1)

grab_motor = Motor(Port.A)
shooting_motor = Motor(Port.D)
grab_motor.reset_angle(0)

gyro.reset_angle(0)
# reset_motor()
# find_ball()
# robot.straight(100)
# grab_ball()
# wait(1000)
# robot.straight(-50)
turn(90)
# open_arm()
# shooting()


# shooting_motor.reset_angle(0)
# shooting_motor.run_target(500,90)

# grab_motor.run_target(300,-90)
# Write your program here.
