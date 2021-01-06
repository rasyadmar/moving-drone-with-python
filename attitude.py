#!/usr/bin/env python

"""
set_attitude_target.py: (Copter Only)
This example shows how to move/direct Copter and send commands
 in GUIDED_NOGPS mode using DroneKit Python.
Caution: A lot of unexpected behaviors may occur in GUIDED_NOGPS mode.
        Always watch the drone movement, and make sure that you are in dangerless environment.
        Land the drone as soon as possible when it shows any unexpected behavior.
Tested in Python 2.7.10
"""

from dronekit import connect, VehicleMode
from pymavlink import mavutil # Needed for command message definitions
import time
import math

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
font = cv2.FONT_HERSHEY_PLAIN
vid_cod = cv2.VideoWriter_fourcc(*'mp4v')
filename = "videos/" + datetime.now().strftime("%Y-%m-%d-%H-%M-%S") + ".mp4"
output = cv2.VideoWriter(filename, vid_cod, 20.0, (640,480))

do_mission = 0
step =1
sampe = 0
def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """

    ##### CONSTANTS #####
    DEFAULT_TAKEOFF_THRUST = 0.7
    SMOOTH_TAKEOFF_THRUST = 0.6

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check,
    # just comment it with your own responsibility.
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)


    print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)

    print("Taking off!")

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.rangefinder.distance
        print(" Altitude: %f  Desired: %f" %
              (current_altitude, aTargetAltitude))
        if current_altitude >= aTargetAltitude*0.95: # Trigger just below target alt.
            print("Reached target altitude")
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.2)

def move_up(aTargetAltitude):
    thrust = 0.75
    while True:
        if vehicle.mode == VehicleMode("GUIDED"):
            current_altitude = vehicle.rangefinder.distance
            print(" Moving up, Altitude: ",current_altitude)
            if current_altitude >= aTargetAltitude * 0.95:  # Trigger just below target alt.
                print("Reached target altitude")
                break
            set_attitude(roll_angle=-2,thrust=thrust)
            time.sleep(0.1)
        else:
            break   
def descend(aTargetAltitude):
    thrust = 0.45
    while True:
        if vehicle.mode == VehicleMode("GUIDED"):
            current_altitude = vehicle.rangefinder.distance
            print("Descending, Altitude: ",current_altitude)
            if current_altitude <= aTargetAltitude * 1.05:  # Trigger just below target alt.
                print("Reached target altitude")
                break
            set_attitude(roll_angle=-2,thrust=thrust)
            time.sleep(0.1)

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.6, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
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

def move_to_color(x,y):
    global step
    roll = -2
    pitch = 0
    xSampe = 0
    if x > pixelx:
        print("Move left")
        roll=-5
    elif x < pixelx :
        print("Move right")
        roll=0
    elif point_x - 30 <= x <= point_x+30:
        xSampe=1
    if y > 100:
        print("Move forward")
        pitch = -3
        roll = -2
    elif y < 100:
        print("Move backward")
        pitch = 3
        roll = -2
    elif 100 - 30 <= x <= 100 + 30:
        ySampe = 0
    if xSampe == 1 and ySampe == 1:
        #BUKA SERVO
        servo(5, 2200)
        print("buka servo")
        step = step + 1
    set_attitude(roll_angle=roll, pitch_angle=pitch, thrust=0.6)

def scanning_qrcode():
    global frame
    value = ""
    decodedObjects = pyzbar.decode(frame)
    for obj in decodedObjects:
        cv2.putText(frame, str(obj.data), (50, 50), font, 2,
                    (255, 0, 0), 3)
        print("Detected: ", str(obj.data))
        value = str(obj.data)

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

def parseArduino():
    val = str(ser.readline().decode().strip())  # Capture serial output as a decoded string
    valA = val.split(",")
    return valA

def move_to(x,y,point_x,point_y):
    global step
    roll = -2
    pitch = 0
    xSampe = 0
    ySampe = 0
    if x > point_x + 30:
        print("Move left")
        roll=-5
    elif x < point_x -30 :
        print("Move right")
        roll=0
    elif point_x - 30 <= x <= point_x+30:
        xSampe=1
    if y > point_y + 30:
        print("Move forward")
        pitch = -3
        roll = -2
    elif y < point_y - 30:
        print("Move backward")
        pitch = 3
        roll = -2
    elif point_y - 30 <= x <= point_y + 30:
        ySampe = 0
    if xSampe == 1 and ySampe ==1:
        step = step + 1
    set_attitude(roll_angle=roll,pitch_angle=pitch, thrust=0.6)

while True:
    _, frame = cap.read()
    alti = vehicle.rangefinder.distance
    drone_position = parseArduino()
    x = int(drone_position[0])
    y = int(drone_position[1])
    if vehicle.mode == VehicleMode("AUTO"):
        print("taking off")
        print("altitude : ",alti)
        if alti >= 1.5*0.95:
            vehicle.mode = VehicleMode("GUIDED")
            print("sampe")
            do_mission = 1
            print(vehicle.mode)
          
    elif vehicle.mode != VehicleMode("GUIDED") and vehicle.mode != VehicleMode("AUTO"):
        print("waiting to takeoff")
        time.sleep(1)
    elif vehicle.mode == VehicleMode("GUIDED") and do_mission == 0:
        print("waiting to go auto")
    elif vehicle.mode == VehicleMode("GUIDED") and do_mission == 1:
        print("altitude : ",alti)

        if step == 1:
            move_to(x,y,point_x,point_y)
        elif step == 2:
            move_up(2)
        elif step == 3:
            value=scanning_qrcode()
            if value == "b'VTOL 3'":
                detecting()
            else:
                print("scanning qr")
        elif step == 4:
            move_to(x,y,point_x,point_y)
        elif step == 5:
            descend(1)
            if alti <= 1 * 1.05:
                vehicle.mode = VehicleMode("LAND")
                print(vehicle.mode)







# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()


print("Completed")
