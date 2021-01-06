from dronekit import connect, VehicleMode, Command
import sched, time, math
from pymavlink import mavutil
import cv2
import cv2
import numpy as np
import serial
from time import sleep
import time
import argparse
import sys

# Get argument from terminal
ap = argparse.ArgumentParser()
ap.add_argument("-f", "--ftdi", help = "port to ftdi")
ap.add_argument("-a", "--arduino", help = "port to arduino")
args = vars(ap.parse_args())
connectionString = args["ftdi"]
COM = args["arduino"]

# Connect to the Vehicle
vehicle = connect(connectionString, wait_ready=True,baud=921600)
#vehicle = connect('127.0.0.1:14550', wait_ready=True)

# arduino connection
#COM = 'COM5'# /dev/ttyACM0 (Linux)
BAUD = 9600
ser = serial.Serial(COM, BAUD, timeout = .1)
print('Waiting for device')
sleep(3)
print("connected to arduino %s" % ser.name)
step = 0
print("ready to guided")

def parseArduino():
    val = str(ser.readline().decode().strip())#Capture serial output as a decoded string
    valA = val.split(",")
    return valA
    #print(valA, end="\r", flush=True)

def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


def set_attitude(roll_angle=0.0, pitch_angle=0.0, yaw_rate=0.0, thrust=0.5, duration=0):


    # Duration is seconds to do this for

    msg = vehicle.message_factory.set_attitude_target_encode(
        0,
        0,  # target system
        0,  # target component
        0b00000000,  # type mask: bit 1 is LSB
        to_quaternion(roll_angle, pitch_angle),  # q
        0,  # body roll rate in radian
        0,  # body pitch rate in radian
        math.radians(yaw_rate),  # body yaw rate in radian
        thrust)  # thrust

    vehicle.send_mavlink(msg)

    if duration != 0:
        # Divide the duration into the frational and integer parts
        modf = math.modf(duration)

        # Sleep for the fractional part
        time.sleep(modf[0])

        # Send command to vehicle on 1 Hz cycle
        for x in range(0, int(modf[1])):
            time.sleep(1)
            vehicle.send_mavlink(msg)

def arm_and_takeoffNoGPS(x, y ,x_tujuan,y_tujuan,heading,Target):
    global step
    if vehicle.armed == False:
        print("arming")
        vehicle.armed = True
        while not vehicle.armed:
            print("waiting for arming")
            time.sleep(5)  # Waiting for armed
    # Take Off
    thrust = 0.8
    pitch = 0
    roll = 0
    yaw = 0
    current_altitude = vehicle.rangefinder.distance
    print("altidude: %f"%current_altitude)
    if x > x_tujuan + 13:
        roll = -2.5
        print("kekiri")
    elif x < x_tujuan - 13:
        roll = 2.5
        print("kekanan")
    elif x_tujuan-13 <= x <= x_tujuan+13:
         roll =0
         print("diam x")
    if y < y_tujuan - 13:
        pitch = 2.5
        print("mundur")
    elif y > y_tujuan + 13:
        pitch = -2.5
        print("maju")
    elif y_tujuan - 13 <= y <= y_tujuan+13:
        pitch = 0
        print("diam y")
    if heading > :
        yaw = -1
    elif heading < :
        yaw = 1
    if current_altitude >= Target * 0.95:
        step = step + 1
    elif current_altitude >= Target * 0.6:
        thrust = 0.6
    set_attitude(pitch_angle=pitch, roll_angle=roll, yaw_rate=yaw, thrust=thrust)

def landing_disarm(x,y,x_tujuan,y_tujuan,heading):
    global step
    thrust = 0.35
    roll = 0
    pitch = 0
    current_altitude = vehicle.rangefinder.distance
    print("altidude: %f" % current_altitude)
    if current_altitude <= 0.2 * 0.95:
        vehicle.armed == False
        vehicle.mode = VehicleMode("LAND")
        print("disarm")
        step = step + 1
        time.sleep(1)
    else:
        if x > x_tujuan + 13:
            roll = -2.5
            print("kekiri")
        elif x < x_tujuan - 13:
            roll = 2.5
            print("kekanan")
        elif x_tujuan-13 <= x <= x_tujuan+13:
            roll =0
            print("diam x")
        if y > y_tujuan + 13:
            pitch = -2.5
            print("maju")
        elif y < y_tujuan - 13:
            pitch = 2.5
            print("mundur")
        elif y_tujuan - 13 <= y <= y_tujuan+13:
            pitch = 0
            print("diam y")
        if heading >:
            yaw = -1
        elif heading <:
            yaw = 1
        if current_altitude <= 0.5 * 0.95:
            thrust = 0.25
        elif current_altitude <= 0.3 * 0.95:
            thrust = 0.1
        set_attitude(pitch_angle=pitch,roll_angle=roll,yaw_rate=yaw, thrust=thrust)

def move_xy_axis(x,y,x_tujuan,y_tujuan):
    thrust = 0.5
    if x > x_tujuan + 10:
        roll = -2
        print("kekiri")
    elif x < x_tujuan - 10:
        roll = 2
        print("kekanan")
    if y > y_tujuan + 10:
        pitch = -2
        print("maju")
    elif y < y_tujuan - 10:
        pitch = 2
        print("mundur")
    set_attitude(pitch_angle=pitch, roll_angle=roll, thrust=thrust, duration=0.2)

while True:
    drone_position = parseArduino()
    x = int(drone_position[0])
    y = int(drone_position[1])
    heading = vehicle.heading
    print(x, y)
    if  vehicle.mode == VehicleMode("GUIDED_NOGPS"):
        if step == 0:
            print("takeoff function")
            arm_and_takeoffNoGPS(x,y,336,270,heading,1)
        elif step == 1:
            print("landing function")
            landing_disarm(x,y,336,270,heading)
        else:
            break
