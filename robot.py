
#This code is meant to control the Raspberry pi robot for Indoor Mapping

#Get the LIBRARIES

from gpiozero import Robot, DistanceSensor, MCP3008
import RPi.GPIO as GPIO
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep

factory = PiGPIOFactory(host='192.168.1.3') #change IP address if incorrect

GPIO.setmode(GPIO.BOARD)

#input sensors. pin numbers are subject to change
front_distance_sensor = DistanceSensor(6)
rigth_distance_sensor = DistanceSensor(5)
accelerometer_X = MCP3008(23)
accelerometer_Y = MCP3008(24)
accelerometer_Z = MCP3008(25)

#Define the pins numbers of the Motor Driver
in1 = 16
in2 = 26
in3 = 22
in4 = 27

#Simpler code for controlling the robot by using the Robot class
robot = Robot(left=(in1, in2), rigth=(in3, in4))

robot.forward()
robot.rigth()
time.sleep(1)
robot.left()
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


#FIND A WALL
treshold = 10 #in cm

front_wall_detected = False
right_wall_detected = False

while not right_wall_detected and not front_wall_detected:
    robot.forward()
    if front_distance_sensor.distance*100 <= treshold:
        front_wall_detected = True
        while front_distance_sensor.distance*100 >= 10:
            robot.front()
        time.sleep(0.5) #check the timers
        robot.stop() #not sure
        robot.left() #make a 90 degrees turn BURAYI DÜZELT,
        time.sleep(1)
        robot.stop()
        right_wall_detected = True
    else:
        if rigth_distance_sensor.distance*100 <= treshold:
            right_wall_detected = True


# START CRUISING
orientation_list = []
wall_length = []
orientation = 0

left = 1 #for orientation purposes
right = -1
distance = 0
previousRight = 10

while right_wall_detected:
    start_time = time.time() #gets the starting time
    RightDistance = rigth_distance_sensor.distance *100 #write a function for getting the right wall distance in cm
    #get the accelerometer data for calculating the wall length
    a_X = accelerometer_X.value #find a better way to calculate average speed
    time.sleep(0.5)
    end_time = time.time()
    elapsed_time = start_time - end_time

    distance += v_init * elapsed_time + 0.5 * a_X * elapsed_time ^ 2 #define v_init
   #get the time so that x = v*t can be found
    robot.forward()

    if front_wall_detected:
        #move to a close proximity and
        if front_distance_sensor.distance *100 <= 10:
            robot.left()
            wall_length.append(distance)
            orientation_list.append(orientation)
            orientation += left

    change = RightDistance - previousRight
        #continue cruising
    if change > 5: #sudden increase in the right distance
        robot.right()
        wall_length.append(distance)
        orientation_list.append(orientation)
        orientation += right

    #If there is a small bump on the route
    elif 5 > change > 0:
        new_wall = change
        orientation += right
        orientation_list.append(orientation)
        wall_length.append(new_wall)
        orientation += left
    elif -5 < change < 0:
        new_wall = -change
        orientation += left
        orientation_list.append(orientation)
        wall_length.append(new_wall)
        orientation += right

orientation_list_final = orientation_list % 4
#In the later code display the wall_length and orientation_list_final on the app
