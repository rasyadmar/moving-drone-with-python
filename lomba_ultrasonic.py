from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import cv2
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
import serial
from time import sleep
import time
import sys

#variable
step = 0

# Connect to the Vehicle
connectionString = input("FTDI port: ")
vehicle = connect('/dev/ttyUSB0', wait_ready=True,baud=921600)
#vehicle = connect('127.0.0.1:14550', wait_ready=True)

#arduino connection
COM = input("Arduino Port: ") #'COM5' /dev/ttyACM0 (Linux)
#COM = 'COM5'# /dev/ttyACM0 (Linux)
BAUD = 115200
ser = serial.Serial(COM, BAUD, timeout = .1)
print('Waiting for device')
sleep(3)
print(ser.name)

#fungsi
def send_nav_velocity(velocity_x, velocity_y, velocity_z):
        # create the SET_POSITION_TARGET_LOCAL_NED command
        msg = vehicle.message_factory.set_position_target_local_ned_encode(
                                                     0,       # time_boot_ms (not used)
                                                     0, 0,    # target system, target component
                                                     mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
                                                     0b0000111111000111, # type_mask (only speeds enabled)
                                                     0, 0, 0, # x, y, z positions (not used)
                                                     velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
                                                     0, 0, 0, # x, y, z acceleration (not used)
                                                     0, 0)    # yaw, yaw_rate (not used)
        # send command to vehicle
        vehicle.send_mavlink(msg)
        vehicle.flush()


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
    val = str(ser.readline().decode().strip('\r\n'))#Capture serial output as a decoded string
    valA = val.split(",")
    return valA
	#print(valA, end="\r", flush=True)

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

def moveLidar_rendah(x,y,x_tujuan,y_tujuan,z_tujuan):
    #tambahin bacaan kamera kodeQR
    global step
    vel_z = 0
    if vehicle.mode.name != 'GUIDED':
        vehicle.mode = VehicleMode("GUIDED")
    if x > x_tujuan+100:
        vel_x = 0.7
    if x > x_tujuan+5 and x < x_tujuan+100:
        vel_x = 0.05
    if x < x_tujuan - 100:
        vel_x = -0.7
    if x < x_tujuan-5 and x > x_tujuan - 100:
        vel_x = -0.05
    if x_tujuan-5<x<x_tujuan+5:
        vel_x=0
        x_sampe = 1
    if y > y_tujuan + 100:
        vel_y = 0.7
    if y < y_tujuan +100 and y> y_tujuan +5:
        vel_y=0.05
    if y < y_tujuan - 100:
        vel_y = -0.7
    if y > y_tujuan - 100 and y < y_tujuan - 5:
        vel_y = -0.05
    if y_tujuan - 5 < y <y_tujuan+5 :
        vel_y = 0
        y_sampe = 1
    if x_sampe == 1 & y_sampe == 1:
        if vehicle.location.global_relative_frame.alt > z_tujuan+2:
            print("sampe di lokasi menurunkan ketinggian")
            vel_z = -0.05
        else:
            print("sampe di lokasi ")
            step = step+1
            sleep(5)

    send_nav_velocity(vel_x, vel_y, vel_z)

def moveLidar_tinggi(x,y,x_tujuan,y_tujuan,z_tujuan):
    # tambahin bacaan kamera kodeQR
    global step
    vel_z = 0
    if vehicle.mode.name != 'GUIDED':
        vehicle.mode = VehicleMode("GUIDED")
    if vehicle.location.global_relative_frame.alt < z_tujuan+2:
        print("menaikkan ketinggian")
        vel_z = 0.05
    else:
        if x > x_tujuan + 100:
            vel_x = 0.7
        if x > x_tujuan + 5 and x < x_tujuan + 100:
            vel_x = 0.05
        if x < x_tujuan - 100:
            vel_x = -0.7
        if x < x_tujuan - 5 and x > x_tujuan - 100:
            vel_x = -0.05
        if x_tujuan - 5 < x < x_tujuan + 5:
            vel_x = 0
            x_sampe = 1
        if y > y_tujuan + 100:
            vel_y = 0.7
        if y < y_tujuan + 100 and y > y_tujuan + 5:
            vel_y = 0.05
        if y < y_tujuan - 100:
            vel_y = -0.7
        if y > y_tujuan - 100 and y < y_tujuan - 5:
            vel_y = -0.05
        if y_tujuan - 5 < y < y_tujuan + 5:
            vel_y = 0
            y_sampe = 1
        if x_sampe == 1 & y_sampe == 1:
            print("sampe di lokasi ")
            step = step + 1
            sleep(5)

    send_nav_velocity(vel_x, vel_y, vel_z)

# def moveLidar_drop1(x,y,x_tujuan,y_tujuan):
#     global step
#     if vehicle.mode.name != 'GUIDED':
#         vehicle.mode = VehicleMode("GUIDED")
#     if x > x_tujuan:
#         vel_x = -0.1
#     if x <= x_tujuan:
#         vel_x = 0
#         x_sampe = 1
#     if y > y_tujuan:
#         vel_y = 0.1
#     if y <= y_tujuan:
#         vel_y = 0
#         y_sampe = 1
#     if x_sampe == 1 & y_sampe == 1:
#         print("sampe di lokasi 1")
#         step = 2
#
#     send_nav_velocity(vel_x, vel_y, 0)
#
# def moveLidar_drop2(x,y,x_tujuan,y_tujuan):
#     global step
#     if vehicle.mode.name != 'GUIDED':
#         vehicle.mode = VehicleMode("GUIDED")
#     if x < x_tujuan:
#         vel_x = 0.2
#     if x >= x_tujuan:
#         vel_x = 0
#         x_sampe = 1
#     if y > y_tujuan:
#         vel_y = 0.1
#     if y <= y_tujuan:
#         vel_y = 0
#         y_sampe = 1
#     if x_sampe == 1 & y_sampe == 1:
#         print("sampe di lokasi 2")
#         step = 3
#     send_nav_velocity(vel_x, vel_y, 0)
#
#
# def moveLidar_home(x,y,x_tujuan,y_tujuan):
#     global step
#     if vehicle.mode.name != 'GUIDED':
#         vehicle.mode = VehicleMode("GUIDED")
#     if x < x_tujuan:
#         vel_x = 0.2
#     if x >= x_tujuan:
#         vel_x = 0
#         x_sampe = 1
#     if y < y_tujuan:
#         vel_y = 0.2
#     if y >= y_tujuan:
#         vel_y = 0
#         y_sampe = 1
#     if x_sampe == 1 & y_sampe == 1:
#         print("sampe di home")
#         step = 4
#     send_nav_velocity(vel_x, vel_y, 0)

def land():
    global step
    vehicle.mode = VehicleMode("LAND")
    while not vehicle.mode.name == 'LAND':  # Wait until mode has changed
        print(" menunggu masuk mode land ...")
        time.sleep(1)
    step = 5


while True:
    drone_position = parseArduino()
    x = drone_position[0]
    y = drone_position[1]
    if step == 0:
        arm_and_takeoff(1)
    elif step == 1:
        print("pergi ke lokasi 1")
        moveLidar_rendah(x, y, x_tujuan, y_tujuan, z_tujuan)
    elif step == 2:
        print("pergi ke lokasi 2")
        moveLidar_rendah(x, y, x_tujuan, y_tujuan, z_tujuan)
    elif step == 3:
        print("pergi ke lokasi 3")
        moveLidar_tinggi(x, y, x_tujuan, y_tujuan, z_tujuan)
    elif step == 4:
        print("pulang ke home")
        moveLidar_rendah(x, y, x_tujuan, y_tujuan, z_tujuan)
    elif step == 5:
        print("landing")
        land()
    else:
        break