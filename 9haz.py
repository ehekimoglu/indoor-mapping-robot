from gpiozero import Robot
import time
import RPi.GPIO as GPIO
import turtle
import smbus
import math

#pins are in BCM
GPIO.setmode(GPIO.BCM)

#DISTANCE SENSOR PINS
f_trig = 5
f_echo = 21
r_trig = 23
r_echo = 24

check = 0

# ACCELEROMETER
address = 0x53
bus = smbus.SMBus(1)
bus.write_byte_data(address, 0x2C, 0x0A)
bus.write_byte_data(address, 0x2D, 0x08)
bus.write_byte_data(address, 0x31, 0x08)

t = turtle.Turtle()

# FIND A WALL
f_treshold = 30
r_treshold = 70  # in cm

front_wall_detected = False
right_wall_detected = False

#MOTOR DRIVER PINS
in1 = 16
in2 = 25
in3 = 22
in4 = 27

robot = Robot(left=(in4,in3),right=(in2,in1))

turn_angle = 90 #for turtle
scale = 20 #for turtle

counter = 0
temp1 = 0

orientation_list = []
turn_list =[]
wall_length = []
orientation = 0
coordinate = [0,0]

left = 1
right = -1
distance = 0
v_init = 78
end_time = time.time()
change = 0

previous_length = 0

away_time = 0
closer_time = 0

# GET DISTANCE DATA
def get_dist(myTrig, myEcho):  # (23,24) ve diğeri

    TRIG = myTrig
    ECHO = myEcho

    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    distance = pulse_duration * 17150

    distance = round(distance, 2)

    if TRIG == f_trig:
        print("Front distance:", distance, "cm")
    elif TRIG == r_trig:
        print("Right distance:", distance, "cm")

    return distance

#GET ACCELERATION
def get_accel(address):
    data0 = bus.read_byte_data(address, 0x34)
    data1 = bus.read_byte_data(address, 0x35)

    # Convert the data to 10-bits
    yAccl = ((data1 & 0x03) * 256) + data0
    if yAccl > 511:
        yAccl -= 1024
#    print("Acceleration in Y-Axis : ", -yAccl)
    return -yAccl

def completion(wall_length, turn_list, turn_angle):
    robot.stop()
    t = turtle.Turtle()
    for i in range(len(wall_length)):
        t.forward(wall_length[i]*scale)
        if turn_list[i] == left:
            t.left(turn_angle)
        elif turn_list[i] == right:
            t.right(turn_angle)
    sc = turtle.getscreen()
    sc.getcanvas().postscript(file="duck.eps")
    turtle.done()
    return 

def findWalls(r_trig, r_echo, f_trig, f_echo):
    right_wall_detected = True

    if not get_dist(r_trig, r_echo) <= r_treshold:
        right_wall_detected = False
    if get_dist(f_trig,f_echo) <= 30:
        robot.left(0.9)
        time.sleep(0.2)
    if get_dist(f_trig,f_echo)<=400 and 30 <= get_dist(f_trig,f_echo):
        robot.forward(0.7, curve_right = 0.31)
        time.sleep(0.2)
    while not right_wall_detected:
        print("looking for wall...")
        right_wall_detected = where_is_my_wall(r_trig, r_echo, f_trig, f_echo)
        
    return right_wall_detected

def where_is_my_wall(r_trig, r_echo,f_trig,f_echo):
    robot.left(0.4)
    print("Searching for wall...")
    time.sleep(0.1)
    robot.stop()
    right_wall_detected = False
    if get_dist(r_trig, r_echo) <= r_treshold:
        right_wall_detected = True
        robot.left(0.4)
        time.sleep(0.1)
        robot.stop()
    return right_wall_detected

def move_closer():
    robot.right(0.4)
    time.sleep(0.15)
    robot.right(0.3)
    time.sleep(0.2)
    robot.stop
    return

def move_away():
    robot.right(0.3)
    time.sleep(0.1)
    robot.right(0.3)
    time.sleep(0.07)
    robot.stop
    return

while counter == 0:
    right_wall_detected = findWalls(r_trig, r_echo, f_trig, f_echo)
    previousRight = get_dist(r_trig, r_echo)
    next_time = time.time()
    while right_wall_detected:
        time.sleep(0.1)
        RightDistance = get_dist(r_trig, r_echo)
        if RightDistance > 300:
            right_wall_detected = False
            print("Right wall lost...")
            temp = 0
        while not right_wall_detected:
            robot.right(0.7)
            print("searching for LOST wall")
            time.sleep(0.1)
            robot.stop()
            if temp ==3:
                robot.forward(0.7, curve_right = 0.25)
                time.sleep(0.5)
                robot.stop()
            temp +=1
            RightDistance = get_dist(r_trig, r_echo)
            if RightDistance <= r_treshold:
                right_wall_detected = True
        if RightDistance <= 10 and 3 <= end_time - away_time:
            move_away()
            away_time = time.time()
        if 40 <=RightDistance and 3 <= end_time - closer_time:
            move_away()
            away_time = time.time()
        robot.forward(0.7, curve_right = 0.35)
        print("Cruising...")
        time.sleep(0.15)
        start_time = end_time
        end_time = time.time()
        elapsed_time = end_time - (start_time + 0.5)
        accel = get_accel(address)/100
        distance += elapsed_time *(accel * elapsed_time / 2 + v_init)
        v_init += elapsed_time * accel
        
        FrontDistance = get_dist(f_trig, f_echo)
        if FrontDistance <= f_treshold:
            front_wall_detected = True
            print("!!FRONT WALL DETECTED!!")
            robot.stop()
            right_wall_detected = False
            robot.left(0.9)
            time.sleep(0.3)
            robot.stop()
            while not right_wall_detected:
                right_wall_detected = where_is_my_wall(r_trig, r_echo,f_trig,f_echo)
            wall_length.append(distance)
            orientation = orientation % 4
            orientation_list.append(orientation)
            turn_list.append(left)
            orientation += left
            distance = 0
            v_init = 78
            
        if 10 <= abs(RightDistance - previousRight):
            change = RightDistance - previousRight
        previousRight = RightDistance
        # continue cruising
            # If there is a small bump on the route
        if r_treshold > change > 10:
            new_wall = change
            orientation += right
            orientation = orientation % 4
            orientation_list.append(orientation)
            turn_list.append(right)
            wall_length.append(new_wall * scale)
            orientation += left
            turn_list.append(left)
            distance = 0

        elif -30 < change < -10:
            new_wall = -change
            orientation += left
            orientation = orientation % 4
            orientation_list.append(orientation)
            turn_list.append(left)
            wall_length.append(new_wall * scale)
            orientation += right
            turn_list.append(right)
            distance = 0
        if 240 < time.time() - next_time: 
            completion(wall_length, turn_list, turn_angle)
        continue  
        
            
        
