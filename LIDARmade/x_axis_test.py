from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import cv2
import cv2
import numpy as np
import serial
from time import sleep
import time
import sys
import argparse

# get argument from command line
ap = argparse.ArgumentParser()
ap.add_argument("-a", "--arduino", help = "port to arduino")
args = vars(ap.parse_args()) #port to arduino
COM = args["arduino"]

# Connect to the Vehicle
#connectionString = input("FTDI port: ")
#vehicle = connect(connectionString, wait_ready=True, baud=921600)
vehicle = connect('127.0.0.1:14551', wait_ready=True)

# arduino connection
# COM = 'COM5'# /dev/ttyACM0 (Linux)
BAUD = 9600
ser = serial.Serial(COM, BAUD, timeout=.1)
print('Waiting for device')
sleep(3)
print(ser.name)

# variable
step = int(input("input step: "))


# fungsi
def send_nav_velocity(velocity_x, velocity_y, velocity_z):
    # create the SET_POSITION_TARGET_LOCAL_NED command
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)  # yaw, yaw_rate (not used)
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()
    # gerakan x itu y di dunia asli dan begitu sebaliknya

def arm_and_takeoff_with_NED(x,y,x_tujuan,y_tujuan,z_tujuan):
    global step
    vel_x = 0
    vel_y = 0
    vel_z = 0
    print("Basic pre-arm checks")
    print("Arming")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Menunggu arming...")
        time.sleep(1)
    # if x > x_tujuan + 10:
    #     vel_x = -0.005
    #     print("kekiri")
    # elif x < x_tujuan - 10:
    #     vel_x = 0.005
    #     print("kekanan")
    # if y > y_tujuan + 10:
    #     vel_x = 0.005
    #     print("maju")
    # elif y < y_tujuan - 10:
    #     vel_x = -0.005
    #     print("mundur")
    if vehicle.location.global_relative_frame.alt < z_tujuan * 0.95:
        vel_z = -0.5
        print("ketinggian: "+str(vehicle.location.global_relative_frame.alt))
    elif vehicle.location.global_relative_frame.alt >= z_tujuan * 0.95:
        vel_z = 0
        print("sampai ketinggian")
    # if y_tujuan - 10 <= y <= y_tujuan+10 and x_tujuan-10 <= x <= x_tujuan+10:
    #     print("sampe")
    #     vel_y = 0
    #     vel_x=0
    #     step = step+1
    #     sleep(5)
    send_nav_velocity(vel_x, vel_y, vel_z)


def servo(channel, sv):
    # input the message
    msg = vehicle.message_factory.command_long_encode(0, 0,  # target system, target component
                                                      mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                                                      0,  # konfirmasi
                                                      channel,  # pin relay pada AUX OUT 3
                                                      sv,  # pwm value
                                                      0, 0, 0, 0, 0)  # param 1 ~ 5 ga dipake
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()


def parseArduino():
    val = str(ser.readline().decode().strip())  # Capture serial output as a decoded string
    valA = val.split(",")
    return valA


# print(valA, end="\r", flush=True)

def arm_and_takeoff(aTargetAltitude):
    global step
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Tunggu inisiasi...")
        time.sleep(1)

    print("Arming")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Menunggu arming...")
        time.sleep(1)

    print("Take off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("sampai target altitude")
            step = 1
            break
        time.sleep(1)


def moveLidar_rendah(x, x_tujuan):
    # tambahin bacaan kamera kodeQR
    global step
    vel_x = 0
    vel_y = 0
    vel_z = 0
    if x > x_tujuan + 5:
        vel_x = -5
        print("kekiri")
    elif x < x_tujuan - 5:
        vel_x = 5
        print("kekanan")

    send_nav_velocity(vel_y, vel_x, vel_z)

def land():
    global step
    vehicle.mode = VehicleMode("LAND")
    while not vehicle.mode.name == 'LAND':  # Wait until mode has changed
        print(" menunggu masuk mode land ...")
        time.sleep(1)
    step = 5


while True:
    drone_position = parseArduino()
    x = int(drone_position[0])
    y = int(drone_position[1])
    print(x, y)
    if vehicle.mode == VehicleMode("GUIDED"):
        if step == 0:
            arm_and_takeoff_with_NED(x,y,64,51,1) #x,y,alti tujuan takeoff
            #arm_and_takeoff(1)
        elif step == 1:
            print("pergi ke lokasi 1")
            moveLidar_rendah(x, 15)
        elif step == 2:
            print("landing")
            land()
        else:
            break
    # else:
    # break
