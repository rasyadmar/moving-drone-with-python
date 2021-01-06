import sched, time, math
from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import numpy as np
import serial
import time
import sys
import argparse

# get argument from command line
ap = argparse.ArgumentParser()
ap.add_argument("-a", "--arduino", help = "port to arduino")
args = vars(ap.parse_args()) #port to arduino
COM = args["arduino"]

# arduino connection
# COM = 'COM5'# /dev/ttyACM0 (Linux)
BAUD = 9600
ser = serial.Serial(COM, BAUD, timeout=.1)
print('Waiting for device')
time.sleep(3)
print(ser.name)

# Connect to the Vehicle
vehicle = connect('127.0.0.1:14551', wait_ready=True)

#variable
step = 0

def parseArduino():
    val = str(ser.readline().decode().strip())  # Capture serial output as a decoded string
    valA = val.split(",")
    return valA


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

def move_x_axis(x,x_tujuan):
    if x > x_tujuan + 5:
        set_attitude(roll_angle=5, thrust=0.5, duration=1)
        print("kekiri")
    elif x < x_tujuan - 5:
        set_attitude(roll_angle=-5, thrust=0.5, duration=1)
        print("kekanan")

def arm_and_takeoffNoGPS(Target):
    global step
    # Arm the vehicle


    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)  # Waiting for armed
    # Take Off
    thrust = 0.7
    while True:
        current_altitude = vehicle.location.global_relative_frame.alt
        print(current_altitude)
        if current_altitude >= Target * 0.95:
            step = step + 1
            break
        elif current_altitude >= Target * 0.6:
            thrust = 0.6
        set_attitude(thrust=thrust)
        time.sleep(0.2)

while True:
    drone_position = parseArduino()
    x = int(drone_position[0])
    y = int(drone_position[1])
    print(x, y)
    if  vehicle.mode == VehicleMode("GUIDED"):
        if step == 0:
            arm_and_takeoffNoGPS(1)
        elif step == 1:
            move_x_axis(x,15)
