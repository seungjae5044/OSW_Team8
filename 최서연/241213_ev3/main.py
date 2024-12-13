#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.iodevices import UARTDevice
from pybricks.ev3devices import Motor, TouchSensor, ColorSensor, InfraredSensor, UltrasonicSensor, GyroSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

#==========[Initialize]==========
#==========[sensors]==========
ev3 = EV3Brick()
gyro = GyroSensor(Port.S1)
ser = UARTDevice(Port.S2, baudrate=115200)

#==========[motors]==========
#grab_motor = Motor(Port.B)
shooting_motor = Motor(Port.C)

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
Color_sensor = ColorSensor(Port.S3)



robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=115)

#==========[target_angle turn(gyro)]==========
def turn(target_angle, power):
    
    left_motor.run(power)
    right_motor.run(-power)
    while True:
        angle=gyro.angle()
        
        if abs(angle)>target_angle-2:
            left_motor.stop()
            right_motor.stop()
            break

    robot.turn()

#---------------------------------------------    

def drive_with_turn(target_angle):
    while True:
        pd_control_gyro(gyro.angle(), kp=0.5, kd=0.1, power=100, target = target_angle)
        angle = gyro.angle()
        print(angle)
        if abs(angle-target_angle)-2:
            robot.stop()
            break

#==========[camera_chase]=====================
def process_uart_data(data):
    try:
        # 데이터를 문자열로 디코드 (키워드 인자 제거)
        data_str = data.decode().strip()
        if not data_str:
            pass

        # 문자열에서 리스트 파싱
        data_str = data_str.strip("[]")
        parsed_list = [int(value.strip()) for value in data_str.split(",")]
        if len(parsed_list) < 2:
            return [-1, -1]
        # 파싱된 결과 반환
        return parsed_list

    except:
        # 에러 처리
        return [-1,-1] # -1이 나오면 무시하는 코드 사용

def pd_control(cam_data, kp, kd, power):
    global previous_error
    error = cam_data - threshold
    derivative = error - previous_error
    output = (kp * error) + (kd * derivative)
    robot.drive(power, output*1.5)
    previous_error = error

#----------------------------------------------------

def pd_control_gyro(angle, kp, kd, power, target):
    global previous_error_gyro
    error = angle - target
    derivative = error - previous_error_gyro
    output = -((kp * error) + (kd * derivative))
    print(output)
    robot.drive(power, output*5)
    previous_error_gyro = error    

#==========[shooting positions]=======================

def shoot(command):
    if command == 'zero':
        #zero_position
        shooting_motor.run_until_stalled(-100,Stop.COAST,duty_limit=50)
    elif command == 'shoot':
        #shooting
        shooting_motor.run(1200)
        time.sleep(0.25)
        shooting_motor.stop()


#==========[setup]==========
ev3.speaker.beep()
threshold = 110
previous_error = 0
previous_error_gyro = 0
gyro.reset_angle(0)

#==========[zero set position setting]===============
shoot('zero') #shoot 모터가 안쪽이고,
time.sleep(1)

print("Zero set postion completed")

#-------------공 찾기-------------------

def search_for_ball(): # 도리도리 공찾기
    while True:
        robot.turn(30)
        time.sleep(0.5)
        robot.turn(-60)
        time.sleep(0.5)
        robot.turn(30)

        data = ser.read_all()
        filter_result = process_uart_data(data)

        if filter_result[0] != -1 and filter_result[1] != -1:  # 공 찾음
            return filter_result #공 위치 반환?


#==========[main loop]===========================

drive_power = 200
#t = 0
#c = 0

while True:
    data = ser.read_all()

    filter_result = process_uart_data(data)
    print(filter_result)

    if filter_result[0] != -1 and filter_result[1] != -1:
        if filter_result[1] > 90:  # 공이 가까워짐
            print("previous")
            drive_with_turn(0)  # 회전 ?
            pd_control(filter_result[0], kp=0.5, kd=0.1, power=100)
            robot.drive(drive_power,0)
        
            print("next")
            while True:

                #robot.drive(100, 0)  # 앞으로 직진
                color = Color_sensor.rgb()
                
                #if Color_sensor.reflection() >= 3 and Color_sensor.reflection() <= 6 : # 파란색 감지
                if abs(color[0]-3) < 2 and abs(color[1]-12) < 2 and abs(color[2]-38) < 2:
                    print(color)
                    robot.stop()
                    time.sleep(1)
                    shoot('shoot')  # 공 발사
                    time.sleep(1)
                    shoot('zero')   # 초기화
                    break

            robot.straight(-700)  # 후진
            time.sleep(1)

        else:  # 공이 멀리 있음
            pd_control(filter_result[0], kp=0.5, kd=0.1, power=130)
    
    #else:
    #    if c == 0:
    #        robot.t

    #else: # 센서가 공을 보지 못했을 경우의 움직임.
    #    robot.turn(30)
    #    time.sleep(0.5)
    #    robot.turn(-60)
    #    time.sleep(0.5)
    #    robot.turn(30)