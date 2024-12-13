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
ser = UARTDevice(Port.S3, baudrate=115200)
 
#==========[motors]==========
grab_motor = Motor(Port.A)
shooting_motor = Motor(Port.D)

left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
robot = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=115)

#==========[pd_control]==========
def pd_control(cam_data, kp, kd, power):
    global previous_error
    error = cam_data - threshold
    derivative = error - previous_error
    output = -(kp * error) + (kd * derivative)
    robot.drive(power, output*2)
    previous_error = error

def pd_control_gyro(angle, kp, kd, power, target):
    global previous_error_gyro
    error = angle - target
    derivative = error - previous_error_gyro
    output = ((kp * error) + (kd * derivative))
    print(output)
    robot.drive(power, output*5)
    previous_error_gyro = error

#==========[motion]==========
def turn(target_angle, power):
    angle = gyro.angle()
    t = target_angle - angle
    print('robot turn')
    robot.drive(0, -t/abs(t)*power)
    while True:
        angle = gyro.angle()
        print(angle, angle-target_angle)
        if abs(angle-target_angle) < 10:
            robot.stop()
            break

def drive_with_turn(target_angle, power):
    print('robot turn')
    while True:
        pd_control_gyro(gyro.angle(), kp=0.5, kd=0.1, power = power , target = target_angle)
        angle = gyro.angle()
        if abs(angle-target_angle) < 2:
            robot.stop()
            break

def drive_until_stalled(displacement, power):
    robot.drive(power,0)
    time.sleep(1)
    while True:
        previous_displacement = robot.distance()
        time.sleep(1)
        print(previous_displacement - robot.distance())
        if abs(previous_displacement - robot.distance()) < displacement:
            robot.stop()
            break
    
#==========[camera_chase]==========
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

#==========[shooting positions]==========
def grab(command):
    if command == 'zero':
        #close
        grab_motor.run_until_stalled(100,Stop.COAST,duty_limit=50)
        #set_zero point
        grab_motor.reset_angle(0)
    elif command == 'open':
        #open
        grab_motor.run_target(600,-200)
    elif command == 'grab':
        #only grab
        grab_motor.run_until_stalled(100,Stop.COAST,duty_limit=50)
    elif command == 'search':
        #search
        grab_motor.run_target(600,-150)

def shoot(command):
    if command == 'zero':
        #zero_position
        shooting_motor.run_until_stalled(-200,Stop.COAST,duty_limit=50)
    elif command == 'shoot':
        #shooting
        shooting_motor.run(2000)
        time.sleep(0.2)
        shooting_motor.stop()

def zero_set_position():
    shoot('zero') #shoot 모터가 안쪽이고,
    grab('zero') #grab 모터가 바깥쪽이므로 shoot먼저 세팅 후 grab을 세팅해야한다

def go_home():
    drive_until_stalled(150, -150)

def grab_true():
    print("===========debug===========")
    print(grab_motor.angle())
    if abs(grab_motor.angle()) > 3:
        return True
    else:
        return False



#==========[setup]==========
ev3.speaker.beep()
threshold = 80
previous_error = 0
previous_error_gyro = 0
gyro.reset_angle(0)
#==========[zero set position setting]==========

zero_set_position()
time.sleep(1)
grab('search') #공을 잡기 위한 높이로 열기

#==========[main loop]==========
t = 1
drive_power = 150


while True:
    data = ser.read_all()
    filter_result = process_uart_data(data)
    if filter_result[0]!= -1 and filter_result[1]!= -1:
        t = 0
        if filter_result[1] > 100: #공이 카메라 화면 기준으로 아래에 위치 = 로봇에 가까워졌다
            robot.straight(100) #강제로 앞으로 이동
            if grab_true():
                drive_with_turn(0, drive_power)
                #drive_until_stalled(100, 100)
                robot.drive(drive_power, 0)
                time.sleep(4)
                robot.stop()
                grab('grab')
                robot.straight(-300)
                turn(45,drive_power) #정면(상대방 진영)바라보기
                time.sleep(1) #동작간 딜레이
                grab('open') #슛을 위한 열기
                shoot('shoot') #공 날리기
                time.sleep(0.5) #동작간 딜레이
            zero_set_position()
            turn(0,drive_power)
            robot.drive(-drive_power*2, 0)
            time.sleep(4)
            robot.stop()
            grab('search')
            t = 1
        else: #공이 카메라 화면 기준 멀리 위치해 있으면 chase한다
            pd_control(filter_result[0], kp=0.5, kd=0.1, power=100)

    time.sleep_ms(50)