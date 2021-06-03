
#This code is meant to control the Raspberry pi robot for Indoor Mapping

#Get the LIBRARIES

from gpiozero import Robot, DistanceSensor, MCP3008
import time
import RPi.GPIO as GPIO
import turtle
import smbus
import math
#import pigpio
#from gpiozero.pins.pigpio import PiGPIOFactory


#factory = PiGPIOFactory(host='192.168.1.3') #change IP address if incorrect

GPIO.setmode(GPIO.BCM)

f_trig= 5
f_echo= 21
r_trig= 23
r_echo= 24

#ivme
address = 0x53
bus = smbus.SMBus(1)
# ADXL345 address, 0x53(83)
# Select bandwidth rate register, 0x2C(44)
#0x0A(10) Normal mode, Output data rate = 100 Hz
bus.write_byte_data(address, 0x2C, 0x0A)
# ADXL345 address, 0x53(83)
# Select power control register, 0x2D(45)
#0x08(08) Auto Sleep disable
bus.write_byte_data(address, 0x2D, 0x08)
# ADXL345 address, 0x53(83)
# Select data format register, 0x31(49)
#0x08(08) Self test disabled, 4-wire interface
#Full resolution, Range = +/-2g
bus.write_byte_data(address, 0x31, 0x08)

#GET DISTANCE
def get_dist(myTrig, myEcho): #(23,24) ve diğeri
    
    TRIG = myTrig 
    ECHO = myEcho

    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)
    
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO)==0:
        pulse_start = time.time()

    while GPIO.input(ECHO)==1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start

    distance = pulse_duration * 17150

    distance = round(distance, 2)
    
    if TRIG == f_trig:
        print ("Front distance:",distance,"cm")
    elif TRIG == r_trig:
        print ("Right distance:",distance,"cm")

    return distance
 
def get_accel(address): 
    
    data0 = bus.read_byte_data(address, 0x34)
    data1 = bus.read_byte_data(address, 0x35)

# Convert the data to 10-bits
    yAccl = ((data1 & 0x03) * 256) + data0
    if yAccl > 511 :
        yAccl -= 1024
    print ("Acceleration in Y-Axis : ",yAccl)
    return yAccl 

#Define the pins numbers of the Motor Driver
in1 = 16
in2 = 25
in3 = 22
in4 = 27

#Simpler code for controlling the robot by using the Robot class
robot = Robot(left=(in1, in2), right=(in3, in4))

turn_angle = 90
scale = 20
motor_turn = 1
#READING inputs
#input_value = GPIO.input(pin_number)

# Output to pin 12
#GPIO.output(12, GPIO.HIGH)

# voltage = [0,0,0,0,0,0,0,0]
# vref = 3.3
# #below code reads the accelerometer data and prints it
# while True:
#     for x in range(0, 3):
#         with MCP3008(channel=x) as reading:
#             voltage[x] = reading.value * vref
#             print(x,": ", voltage[x])
#         sleep(0.5)

#COMPLETION ALGORITHM

# (x,y) her bir wall'ı append ettiiğimizde bu orientation'a göre x veya y'yi değiştir
#0'sa distance +y
#1'se distance -x
#2'yse distance -y
#3'se distance +x
#(0.0)' a geldiğinde dur, completed.


counter = 0
coordinate = [0, 0]

def completion(orientation, coordinate, counter):
    if orientation == 0:
        coordinate[1] += distance
    elif orientation == 1:
        coordinate[0] -= distance
    elif orientation == 2:
        coordinate[1] -= distance
    elif orientation == 3:
        coordinate[0] += distance
    if  5 > abs(coordinate[0]) and 5 > abs(coordinate[1]) and time.time > 30:
        counter += 1
        sc = turtle.getscreen()
        sc.getcanvas().postscript(file="duck.eps")
        turtle.done()     
    return counter # check this false if it breaks the while loop above??? NOT SURE
        # map'i yolla bu sırada


#FIND A WALL
treshold = 50 #in cm

front_wall_detected = False
right_wall_detected = False

def findWalls(f_trig, f_echo, r_trig, r_echo):

    front_wall_detected = True
    right_wall_detected = True
    
    if not get_dist(r_trig, r_echo)<= treshold:
        right_wall_detected = False
    
    if not get_dist(f_trig, f_echo)<= treshold:
        front_wall_detected = False

    if not right_wall_detected and front_wall_detected:
        robot.left(motor_turn) #make a 90 degrees turn BURAYI DÜZELT,
        robot.stop()
        right_wall_detected = True
    
    if not right_wall_detected and not front_wall_detected:
        robot.forward()
        time.sleep(1)
    return right_wall_detected
# while not right_wall_detected and not front_wall_detected:
#     robot.forward()
#     time.sleep(1)
#     if get_dist(f_trig, f_echo) <= treshold:
#         front_wall_detected = True
#         while get_dist(f_trig, f_echo) >= treshold:
#             robot.front()
#         robot.stop() #not sure
#         robot.left(0.3) #make a 90 degrees turn BURAYI DÜZELT,
#         time.sleep(5)
#         robot.stop()
#         right_wall_detected = True
#     else:
#         if get_dist(r_trig, r_echo) <= treshold:
#             right_wall_detected = True




# START CRUISING
orientation_list = []
wall_length = []
orientation = 0
t = turtle.Turtle()

left = 1 #for orientation purposes
right = -1
distance = 0
v_init = 1
end_time = time.time()
       
while counter == 0:
    right_wall_detected = findWalls(f_trig, f_echo, r_trig, r_echo)
    previousRight = get_dist(r_trig,r_echo)
    while right_wall_detected:
        robot.forward()
        start_time = end_time #gets the starting time
        #read acceleromotion data
        RightDistance = get_dist(r_trig, r_echo) #write a function for getting the right wall distance in cm
        #get the accelerometer data for calculating the wall length
        #a_X = accelerometer_X.value #find a better way to calculate average speed
        time.sleep(0.5)
        end_time = time.time()
        elapsed_time = end_time - start_time
        distance += elapsed_time*(get_accel(address)*(elapsed_time)/2 + v_init)
        v_init += elapsed_time*get_accel(address)
        
        if get_dist(f_trig, f_echo) <= treshold:
                front_wall_detected = True

        #distance += v_init * elapsed_time + 0.5 * a_X * elapsed_time ^ 2 #define v_init
       #get the time so that x = v*t can be found

        if front_wall_detected:
            #move to a close proximity and
            if get_dist(f_trig, f_echo) <= treshold:
                robot.stop()
                robot.left(motor_turn)
                robot.stop()
                wall_length.append(distance)
                t.forward(distance*scale)
                orientation = orientation % 4
                orientation_list.append(orientation)
                t.left(turn_angle)
                completion(orientation, coordinate, counter) #checks if coordinates are 0,0 or not
                orientation += left
                distance = 0

        change = RightDistance - previousRight
        previousRight = RightDistance
                #continue cruising
        if abs(change) > treshold: #sudden increase in the right distance
            robot.stop()
            robot.right(motor_turn)
            robot.stop()
            wall_length.append(distance)
            t.forward(distance*scale)
            orientation = orientation % 4
            orientation_list.append(orientation)
            t.right(turn_angle)
            completion(orientation, coordinate, counter)
            orientation += right
            distance = 0
            if get_dist(r_trig, r_echo) > treshold:
                right_wall_detected = False

            #If there is a small bump on the route
        elif treshold > change > 1:
            new_wall = change
            orientation += right
            orientation = orientation % 4
            orientation_list.append(orientation)
            t.right(turn_angle)
            completion(orientation, coordinate, counter)
            wall_length.append(new_wall*scale)
            t.forward(distance*scale)
            orientation += left
            t.left(turn_angle)
            distance = 0
                
        elif -(treshold) < change < -1:
            new_wall = -change
            orientation += left
            orientation = orientation % 4
            orientation_list.append(orientation)
            t.left(turn_angle)
            completion(orientation, coordinate, counter)
            wall_length.append(new_wall*scale)
            t.forward(distance*scale)
            orientation += right
            t.right(turn_angle)
            distance = 0

