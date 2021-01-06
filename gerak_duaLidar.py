from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import cv2
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
import serial
from time import sleep
import sys

#variable
step = 1

#camera
cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_PLAIN

# Connect to the Vehicle
vehicle = connect('/dev/ttyUSB0', wait_ready=True,baud=921600)
#vehicle = connect('127.0.0.1:14550', wait_ready=True)

#arduino connection
COM = 'COM8'# /dev/ttyACM0 (Linux)
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

def servo(channel,sv):
         #input the message
         msg = vehicle.message_factory.command_long_encode(0, 0, # target system, target component
                                                     mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                                                     0, # konfirmasi
                                                     channel, # pin relay pada AUX OUT 3
                                                     sv, # pwm value
                                                     0, 0, 0, 0, 0) # param 1 ~ 5 ga dipake
         # send command to vehicle
         vehicle.send_mavlink(msg)
         vehicle.flush()

def parseArduino():
    val = str(ser.readline().decode().strip('\r\n'))#Capture serial output as a decoded string
    valA = val.split(",")
    return valA
	#print(valA, end="\r", flush=True) 

def moveLidar_drop1(x,y):
    # di home catat jarak lidar y dan lidar x
    # di tiap lokasi drop catat jarak lidar
    # berdasarakan jarak2 tadi jalankan perintah bergerak dengan velocity
    # drop pas ketinggian 2m
    step = 2

def moveLidar_drop2(x,y):
    # di home catat jarak lidar y dan lidar x
    # di tiap lokasi drop catat jarak lidar
    # berdasarakan jarak2 tadi jalankan perintah bergerak dengan velocity
    # drop pas ketinggian 1m
    step = 3    
def moveLidar_drop3(x,y):
    # di home catat jarak lidar y dan lidar x
    # di tiap lokasi drop catat jarak lidar
    # berdasarakan jarak2 tadi jalankan perintah bergerak dengan velocity
    # drop pas ketinggian 2.5m
    step = 4

def moveLidar_home(x,y):
    # di home catat jarak lidar y dan lidar x
    # di tiap lokasi drop catat jarak lidar
    # berdasarakan jarak2 tadi jalankan perintah bergerak dengan velocity
    # landing
    step = 5 

while True:
    _, frame = cap.read()
    decodedObjects = pyzbar.decode(frame)
    for obj in decodedObjects:
        print("Data", obj.data)
        cv2.putText(frame, str(obj.data), (50, 50), font, 2,(255, 0, 0), 3)

    cv2.imshow("Frame", frame)
    drone_position = parseArduino()
    x = drone_position[0]
    y = drone_position[1]
    if step == 1:
        moveLidar_drop1(x,y)
    elif step == 2:
        moveLidar_drop2(x,y)
    elif step == 3:
        moveLidar_drop3(x,y)
    elif step == 4:
        moveLidar_home(x,y)
    else:
        break

    key = cv2.waitKey(1)
    if key == 27:
        break