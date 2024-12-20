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

#color = Color_sensor.color()

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

    print('robot turn')
    while True:
        
        pd_control_gyro(gyro.angle(), kp=0.5, kd=0.1, power=100, target = target_angle)
        angle = gyro.angle()


#---------------------------------------------
    print('robot turn')
    robot.drive(power, power)
    while True:
        angle = gyro.angle()
        print(angle)
        if abs(angle)>target_angle-2:
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
    robot.drive(power, output)
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

#==========[shooting positions]==========

def shoot(command):
    if command == 'zero':
        #zero_position
        shooting_motor.run_until_stalled(-100,Stop.COAST,duty_limit=50)
    elif command == 'shoot':
        #shooting
        shooting_motor.run(1000)
        time.sleep(0.25)
        shooting_motor.stop()


#==========[setup]==========
ev3.speaker.beep()
threshold = 90
previous_error = 0
gyro.reset_angle(0)


#==========[zero set position setting]===============
shoot('zero') #shoot 모터가 안쪽이고,
#grab('motion3') #grab 모터가 바깥쪽이므로 shoot먼저 세팅 후 grab을 세팅해야한다
time.sleep(1)
#grab('motion1') #공을 잡기 위한 높이로 열기

print("Zero set postion completed")

#==========[main loop]==========
while True:
    data = ser.read_all()
    # 데이터 처리 및 결과 필터링
    try:
        filter_result = process_uart_data(data)
        #filter_result[0] : x, filter_result[1] : y

        if filter_result[0]!= -1 and filter_result[1]!= -1:
        # if filter_result[0]!= -1 and filter_result[1]!= -1:

            if filter_result[1] > 90: #공이 카메라 화면 기준으로 아래에 위치 = 로봇에 가까워졌다
                drive_with_turn(90) #정면(상대방 진영)바라보기
                robot.straight(1000) #강제로 앞으로

                #robot.stop()
                #shoot('shoot') #공 날리기

                #파란색 감지
                if Color_sensor.color() == Color.BULE:
                    robot.stop()
                    time.sleep(1)
                    shoot('shoot') #공 날리기 
                    time.sleep(0.5)
                    shoot('zero')
                    robot.straight(-200)
                    continue  # 다시 공 찾기(?)

                # 파란색 감지
                #if color_sensor.color() == Color.BLUE:
                #    robot.stop()
                #    time.sleep(1)
                #    shoot('shoot') #공 날리기 
                #    time.sleep(0.5)
                #    shoot('zero')
                #    robot.straight(-200)
                #    continue  # 다시 공 찾기(?)

            else: #공이 카메라 화면 기준 멀리 위치해 있으면 chase한다
                pd_control(filter_result[0], kp=0.5, kd=0.1, power=100)

        #else: # 센서가 공을 보지 못했을 경우의 움직임.
        #    robot.straight(-50)
        #    robot.turn(10)
        #    continue

        #time.sleep_ms(50)

    except:
        pass

while True:
    try:
        data = ser.read_all()
        filter_result = process_uart_data(data)
        if filter_result[0]!= -1 and filter_result[1]!= -1:
            print(filter_result)
            pd_control(filter_result[0], kp=0.5, kd=0.1, power=100)
        wait(10)
    except:
        pass