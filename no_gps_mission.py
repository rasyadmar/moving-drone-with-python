from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import serial
from time import sleep
import argparse
import cv2
import numpy as np

# Get argument from terminal
ap = argparse.ArgumentParser()
ap.add_argument("-f", "--ftdi", help = "port to ftdi")
ap.add_argument("-a", "--arduino", help = "port to arduino")
args = vars(ap.parse_args())
connectionString = args["ftdi"]
COM = args["arduino"]

# Connect to the Vehicle
vehicle = connect(connectionString, wait_ready=True, baud=921600)

# arduino connection
# COM = 'COM5'# /dev/ttyACM0 (Linux)
BAUD = 9600
ser = serial.Serial(COM, BAUD, timeout=.1)
print('Waiting for device')
sleep(3)
print(ser.name)

#camera connection
cap = cv2.VideoCapture(0)

# variable
current_pos = 0
next_pos = 1
step = 0
print("siap guided")


# fungsi

def goto_position_target_local_ned(north, east, down):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)

def arm_takeoff_local(altitude):
    # create the SET_POSITION_TARGET_LOCAL_NED command
    msg = vehicle.message_factory.mav_cmd_nav_takeoff_local_encode(
        0,  # pich
        0, #empty
        1.2,  # ascend rate m/s
        0,
        0, 0, altitude,  # position relative to frame y,x,z
    )
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

def send_nav_velocity(velocity_x, velocity_y, velocity_z):
    # create the SET_POSITION_TARGET_LOCAL_NED command
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_y, velocity_x, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)  # yaw, yaw_rate (not used)
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
        sleep(1)

    print("Arming")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Menunggu arming...")
        sleep(1)

    print("Take off!")
    vehicle.simple_takeoff(1)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        if vehicle.mode == VehicleMode("GUIDED"):
            print(" Altitude: ", vehicle.location.global_relative_frame.alt)
            if aTargetAltitude * 0.95 >= vehicle.location.global_relative_frame.alt >= 1 * 0.95:
                send_nav_velocity(0,0,-1)
            elif vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
                send_nav_velocity(0, 0, 0)
                print("sampai target altitude")
                step = 1
                break
            sleep(1)
        else:
            break

def land_function():
    while True:
        print("landing current Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.mode == VehicleMode("GUIDED"):
            if vehicle.location.global_relative_frame.alt >= 1 * 1.05:
                send_nav_velocity(0, 0, 0.05)
            else:
                vehicle.mode = VehicleMode("LAND")
        else:
            break
        sleep(1)

def move_to_waypoint(north,east,down):

def move_to_color(x,y):
    global step
    vx = 0
    vz = 0
    xSampe = 0
    if x > 4:
        vx = -0.005
    elif x < 2:
        vx = 0.005
    elif 2 <= x <= 4 :
        vx = 0
        xSampe = 1
    if y > 4 :
        vz = -0.005
    elif y < 2:
        vz = 0.005
    elif 2 <= y <= 4:
        vz = 0
        ySampe = 0
    if ySampe == 1 and xSampe == 1:
        #BUKA SERVO
        step = step + 1
    send_nav_velocity(vx,0,vz)




def detecting():
    #INGAT BUAT GAMBARNYA SESUAI SAMA ORIENTASI TANGAN KITA
    # INGAT BUAT GAMBARNYA SESUAI SAMA ORIENTASI TANGAN KITA
    # INGAT BUAT GAMBARNYA SESUAI SAMA ORIENTASI TANGAN KITA
    # INGAT BUAT GAMBARNYA SESUAI SAMA ORIENTASI TANGAN KITA
    _, img = cap.read()

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 0], dtype=np.uint8)
    upper_white = np.array([0, 0, 255], dtype=np.uint8)

    blur = cv2.GaussianBlur(hsv, (15, 15), cv2.BORDER_DEFAULT)

    white = cv2.inRange(blur,  lower_white, upper_white)

    # kernal = np.ones((5, 5), "uint8"
    # blue=cv2.dilate(yellow, kernal)

    #res = cv2.bitwise_and(img, img, mask=white)

    # Tracking Colour
    _, contours, hierarchy = cv2.findContours(white, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # contours_roi,hierarchy_roi=cv2.findContours(orange_roi,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 2000):
            c = max(contours, key=cv2.contourArea)
            cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
            move_to_color(cX,cY)

def mission():

while True:
    drone_position = parseArduino()
    x = int(drone_position[0])
    y = int(drone_position[1])
    print(x, y)
    if vehicle.mode == VehicleMode("GUIDED"):
        if step == 0:
            arm_and_takeoff(2.3)
        elif step == 1:
            print("pergi ke lokasi 1")
            moveLidar_tinggi_sama (x,y)
        elif step == 2:
            print("pergi ke lokasi 2")
            moveLidar_tinggi_sama(x,y)
        elif step == 0:
            print("landing")
            land()
        else:
            break
    # else:
    # break

# movement note

# takeoff to pos 1
send_nav_velocity(-0.5,1.6,0)
sleep(5)
send_nav_velocity(0,0,0)

#pos 1 to pos 2
send_nav_velocity(1.25,0,0.25)
sleep(2)
send_nav_velocity(0,0,0)

#pos 2 to pos 3
send_nav_velocity(1.25,0,-0.5)
sleep(2)
send_nav_velocity(0,0,0)

#pos 3 to pos 1
send_nav_velocity(-1.667,0,0.25)
sleep(3)
send_nav_velocity(0,0,0)

#pos 1 to land
send_nav_velocity(0.5,-1.6,0)
sleep(5)
send_nav_velocity(0,0,0)

#pos 2 to land
send_nav_velocity(0,-1.6,0)
sleep(5)
send_nav_velocity(0,0,0)

#pos 3 to land
send_nav_velocity(-0.5,-1.6,0)
sleep(5)
send_nav_velocity(0,0,0)


