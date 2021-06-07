

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

# ACCELEROMETER
address = 0x53
bus = smbus.SMBus(1)
bus.write_byte_data(address, 0x2C, 0x0A)
bus.write_byte_data(address, 0x2D, 0x08)
bus.write_byte_data(address, 0x31, 0x08)

# FIND A WALL
f_treshold = 20
r_treshold = 100  # in cm

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

orientation_list = []
turn_list =[]
wall_length = []
orientation = 0

left = 1
right = -1
distance = 0
v_init = 0
end_time = time.time()

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
    print("Acceleration in Y-Axis : ", -yAccl)
    return -yAccl

def completion(orientation, coordinate, counter, wall_length, turn_list, orientation_list, distance, turn_angle):
    coordinate = [0, 0]
    for i in len(wall_length): #already appended walls
        if orientation_list[i] = 0: 
            coordinate[1] += wall_length[i]
        elif orientation_list[i] = 1:
            coordinate[0] -= wall_length[i]
        elif orientation_list[i] = 2:
            coordinate[1] -= wall_length[i]
        elif orientation_list[i] = 3:
            coordinate[0] += wall_length[i]
    
    #wall being currently traveled 
    if orientation = 0: 
        coordinate[1] += distance
    elif orientation = 1:
        coordinate[0] -= distance
    elif orientation = 2:
        coordinate[1] -= distance
    elif orientation = 3:
        coordinate[0] += distance
        
    if 5 > abs(coordinate[0]) and 5 > abs(coordinate[1]) and time.time() > 30: #is the room complete
        counter += 1
        robot.stop()
        t = turtle.Turtle()
        for i in len(wall_length):
            t.forward(wall_length[i]*scale)
            if turn_list[i] = left:
                t.left(turn_angle)
            elif turn_list[i] = right:
                t.right(turn_angle)
        sc = turtle.getscreen()
        sc.getcanvas().postscript(file="duck.eps")
        turtle.done()
    return counter

def findWalls(r_trig, r_echo, f_trig, f_echo):
    right_wall_detected = True

    if not get_dist(r_trig, r_echo) <= r_treshold:
        right_wall_detected = False
        
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
    robot.right(0.5)
    time.sleep(0.1)
    robot.stop()
    robot.left(0.3)
    time.sleep(0.1)
    robot.stop()
    return

def move_away():
    robot.left(0.4)
    time.sleep(0.1)
    robot.stop()
    robot.right(0.3)
    time.sleep(0.1)
    robot.stop()
    return

#main
while counter == 0:
    right_wall_detected = findWalls(r_trig, r_echo, f_trig, f_echo)
    previousRight = get_dist(r_trig, r_echo)
    while right_wall_detected:
        completion(orientation, coordinate, counter, wall_length, turn_list, orientation_list, distance, turn_angle) 
        time.sleep(0.1)
        RightDistance = get_dist(r_trig, r_echo)
        if RightDistance > r_treshold:
            right_wall_detected = False
            print("Right wall lost...")
        temp = 0
        while not right_wall_detected:
            robot.right(0.9)
            print("Searching for LOST wall")
            time.sleep(0.1)
            robot.stop()
            if temp == 3:
                robot.forward(0.7)
                time.sleep(0.5)
                robot.stop()
            temp =+ 1
            RightDistance = get_dist(r_trig, r_echo)
            if RightDistance <= r_treshold:
                right_wall_detected = True
        if RightDistance <= 10: #robot shouldnt get too close to wall
            move_away()
        elif 70 <= RightDistance: #robot shouldnt stray away from the wall
            move_closer()
        robot.forward(1)
        print("cruising...")
        time.sleep(0.15)
        start_time = end_time
        end_time = time.time()
        elapsed_time = end_time - start_time
        accel = get_accel(address)
        distance += elapsed_time * (accel * (elapsed_time) / 2 + v_init)
        v_init += elapsed_time * accel
        front_distance  = get_dist(f_trig, f_echo)

        if front_distance <= f_treshold:
            front_wall_detected = True
            print("!!Front wall detected!!")
            robot.stop()
            right_wall_detected = False
            while not right_wall_detected:
                robot.left(0.9)
                time.sleep(0.1)
                robot.stop()
                right_wall_detected = where_is_my_wall(r_trig, r_echo,f_trig,f_echo)
            wall_length.append(distance)
            orientation = orientation % 4
            orientation_list.append(orientation)
            turn_list.append(left) 
            orientation += left
            print("Current wall distance:", distance, "cm")
            distance = 0
        #
        change = RightDistance - previousRight
        previousRight = RightDistance
        # continue cruising
            # If there is a small bump on the route
        if r_treshold > change > 1:
            new_wall = change
            orientation += right
            orientation = orientation % 4
            orientation_list.append(orientation)
            turn_list.append(right)
            wall_length.append(new_wall * scale)
            orientation += left
            turn_list.append(left)
            print("Current wall distance:", distance, "cm")
            distance = 0

        elif -(r_treshold) < change < -1:
            new_wall = -change
            orientation += left
            orientation = orientation % 4
            orientation_list.append(orientation)
            turn_list.append(left)
            wall_length.append(new_wall * scale)
            orientation += right
            turn_list.append(right)
            print("Current wall distance:", distance, "cm")
            distance = 0
        continue  
