# from dronekit import connect, VehicleMode, Command
# from pymavlink import mavutil
# import serial
# from time import sleep
import argparse
import cv2
# import numpy as np
# import pyzbar.pyzbar as pyzbar
from datetime import datetime

# Get argument from terminal
# ap = argparse.ArgumentParser()
# ap.add_argument("-f", "--ftdi", help="port to ftdi")
# ap.add_argument("-a", "--arduino", help="port to arduino")
# args = vars(ap.parse_args())
# connectionString = args["ftdi"]
# COM = args["arduino"]

# Connect to the Vehicle
# vehicle = connect(connectionString, wait_ready=True, baud=921600)

# arduino connection
# COM = 'COM5'# /dev/ttyACM0 (Linux)
# BAUD = 9600
# ser = serial.Serial(COM, BAUD, timeout=.1)
# print('Waiting for device')
# sleep(3)
# print(ser.name)

# camera connection
cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_PLAIN
vid_cod = cv2.VideoWriter_fourcc(*'mp4v')
filename = "videos/" + datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".mp4"
output = cv2.VideoWriter(filename, vid_cod, 20.0, (640, 480))
# variable
# current_pos = 0
# next_pos = 1
step = 0
do_mission = 0
waypoint_list = set([1, 2, 3])
vtol_1 = 0
vtol_2 = 0
vtol_3 = 0
clear = 0


# fungsi

def goto_position_target_local_ned(north, east, down):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111111000,  # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0,  # x, y, z velocity in m/s  (not used)
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)


def arm_takeoff_local(altitude):
    # create the SET_POSITION_TARGET_LOCAL_NED command
    msg = vehicle.message_factory.mav_cmd_nav_takeoff_local_encode(
        0,  # pich
        0,  # empty
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
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,  # frame
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
def scanning_qrcode():
    global frame
    value = ""
    decodedObjects = pyzbar.decode(frame)
    for obj in decodedObjects:
        cv2.putText(frame, str(obj.data), (50, 50), font, 2,
                    (255, 0, 0), 3)
        print("Data", obj.data)
        value = obj.data

    return value


def detecting():
    # INGAT BUAT GAMBARNYA SESUAI SAMA ORIENTASI TANGAN KITA
    # INGAT BUAT GAMBARNYA SESUAI SAMA ORIENTASI TANGAN KITA
    # INGAT BUAT GAMBARNYA SESUAI SAMA ORIENTASI TANGAN KITA
    # INGAT BUAT GAMBARNYA SESUAI SAMA ORIENTASI TANGAN KITA
    global frame

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 0], dtype=np.uint8)
    upper_white = np.array([0, 0, 255], dtype=np.uint8)

    blur = cv2.GaussianBlur(hsv, (15, 15), cv2.BORDER_DEFAULT)

    white = cv2.inRange(blur, lower_white, upper_white)

    # kernal = np.ones((5, 5), "uint8"
    # blue=cv2.dilate(yellow, kernal)

    # res = cv2.bitwise_and(frame, frame, mask=white)

    # Tracking Colour
    _, contours, hierarchy = cv2.findContours(white, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # contours_roi,hierarchy_roi=cv2.findContours(orange_roi,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if (area > 2000):
            c = max(contours, key=cv2.contourArea)
            cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
            move_to_color(cX, cY)


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
                send_nav_velocity(0, 0, -1)
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


def move_to_waypoint(north, east, down):
    print("blumdibuat")


def move_to_color(x, y):
    global step
    vx = 0
    vz = 0
    xSampe = 0
    if x > 4:
        vx = -0.005
    elif x < 2:
        vx = 0.005
    elif 2 <= x <= 4:
        vx = 0
        xSampe = 1
    if y > 4:
        vz = -0.005
    elif y < 2:
        vz = 0.005
    elif 2 <= y <= 4:
        vz = 0
        ySampe = 0
    if ySampe == 1 and xSampe == 1:
        # BUKA SERVO
        step = step + 1
    send_nav_velocity(vx, 0, vz)


def move_to_waypoint_lidar(x, y, alti, des_x, des_y, des_alt):
    global step
    global waypoint_list
    vx = 0
    vy = 0
    vz = 0
    xSampe = 0
    ySampe = 0
    zSampe = 0
    if x > des_x + 10:
        vx = -0.1
    elif x < des_x - 10:
        vx = 0.1
    elif des_x - 10 <= x <= des_x + 10:
        vx = 0
        xSampe = 1
    if y > des_y + 10:
        vy = 0.1
    elif y < des_y - 10:
        vy = -0.1
    elif 2 <= y <= 4:
        vy = 0
        ySampe = 1
    if alti > des_alt + 10:
        vy = 0.05
    elif alti < des_alt - 10:
        vy = -0.05
    elif 2 <= y <= 4:
        vz = 0
        zSampe = 1
    send_nav_velocity(vx, vy, vz)
    if ySampe == 1 and xSampe == 1 and zSampe == 1:
        scannedQR = scanning_qrcode()
        if scannedQR == "":
            print("scanning qr")
        else:
            if vtol_1 == 0:
                if scannedQR == "VTOL 1":
                    waypoint_list.remove(step)
                    servo(5, 2200)
                    sleep(0.5)
                    step = step + 1
                else:
                    step = step + 1
            elif vtol_2 == 0:
                if scannedQR == "VTOL 2":
                    waypoint_list.remove(step)
                    servo(5, 2200)
                    sleep(0.5)
                    step = step + 1
                else:
                    step = step + 1
            elif vtol_3 == 0:
                if scannedQR == "VTOL 3":
                    waypoint_list.remove(step)
                    servo(5, 2200)
                    sleep(0.5)
                    step = step + 1
                else:
                    step = step + 1


def move_to_land(x, y, alti, des_x, des_y, des_alt):
    vx = 0
    vy = 0
    vz = 0
    xSampe = 0
    ySampe = 0
    zSampe = 0
    if x > des_x + 10:
        vx = -0.1
    elif x < des_x - 10:
        vx = 0.1
    elif des_x - 10 <= x <= des_x + 10:
        vx = 0
        xSampe = 1
    if y > des_y + 10:
        vy = 0.1
    elif y < des_y - 10:
        vy = -0.1
    elif 2 <= y <= 4:
        vy = 0
        ySampe = 1
    if alti > des_alt + 10:
        vy = 0.05
    elif alti < des_alt - 10:
        vy = -0.05
    elif 2 <= y <= 4:
        vz = 0
        zSampe = 1
    send_nav_velocity(vx, vy, vz)
    if ySampe == 1 and xSampe == 1 and zSampe == 1:
        land_function()


while True:
    _, frame = cap.read()
    if step > 3:
        step = 1
    # drone_position = parseArduino()
    # x = int(drone_position[0])
    # y = int(drone_position[1])
    # alti = vehicle.rangefinde.distance
    # if vehicle.mode != VehicleMode("AUTO") and vehicle.mode != VehicleMode("GUIDED"):
    #     print("waiting")
    #     sleep(1)
    # elif vehicle.mode == VehicleMode("AUTO"):
    #     print("taking off")
    #     if alti >= 2 * 0.95:
    #         do_mission = 1
    #         vehicle.mode = VehicleMode("GUIDED")
    #     sleep(1)
    # elif vehicle.mode == VehicleMode("GUIDED") and do_mission == 0:
    #     print("waiting to takeoff")
    #     sleep(1)
    # elif vehicle.mode == VehicleMode("GUIDED") and do_mission == 1:
    #     if step == 0:
    #         print("maju ke titik deteksi way 2")
    #         send_nav_velocity(0,0.5,0)
    #         sleep(6)
    #         step = step + 1
    #     elif step == 1:
    #         print("kekiri way 1")
    #         send_nav_velocity(-0.2,0,-0.1)
    #         sleep(5)
    #     elif step == 2:
    #         print("kekanan way 3")
    #         send_nav_velocity(0.4, 0, -0.1)
    #         sleep(5)
    #     elif step == 3:
    #         print("kekiri way 2")
    #         send_nav_velocity(-0.2,0,0.2)
    #         sleep(5)

    output.write(frame)

# close the already opened camera
cap.release()
# close the already opened file
output.release()
# close the window and de-allocate any associated memory usage
cv2.destroyAllWindows()





