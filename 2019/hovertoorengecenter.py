from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import cv2
import numpy as np
import math

cap = cv2.VideoCapture(0)

#cap.set(3, 640)# width resolution
#cap.set(4, 480)# heigh resolution

# Connect to the Vehicle
vehicle = connect('/dev/ttyUSB0', wait_ready=True,baud=921600)
#vehicle = connect('127.0.0.1:14550', wait_ready=True)

cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
vel_x = 0
vel_y = 0
vel_z = 0

pointauth = 2
settime = 0
mode_changed = 0
count = 0
dropped = False
once = False
settime = 0

def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint.
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint=vehicle.commands.next
    if nextwaypoint ==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat=missionitem.x
    lon=missionitem.y
    alt=missionitem.z
    targetWaypointLocation=LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

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
         msg =vehicle.message_factory.command_long_encode(0, 0, # target system, target component
                                                     mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                                                     0, # konfirmasi
                                                     channel, # pin relay pada AUX OUT 3
                                                     sv, # pwm value
                                                     0, 0, 0, 0, 0) # param 1 ~ 5 ga dipake
         # send command to vehicle
         vehicle.send_mavlink(msg)
         vehicle.flush()
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
def counter():
    global settime
    settime=settime+1
    if settime >= 3000 :
        vehicle.mode=VehicleMode("AUTO")
        settime=0

def move_target_drop(cX, cY,channel,pwm,dropped):
    #global once
    if dropped == False  :
        vehicle.mode = VehicleMode("GUIDED")
    dropped=True
    vel_x =0
    vel_y =0
    vel_z =0
    if (cY>270):
        vel_y = -0.125
    elif (cY<90):
        vel_y = 0.125
    if (cX>430):
        vel_x = 0.125
    elif (cX<230):
        vel_x = -0.125
    if (cY<280 and cY>100 and cX<450 and cX>210):
        send_nav_velocity(0,0,0)
        vel_y = 0
        vel_x = 0
        servo(channel,pwm)
        vehicle.mode = VehicleMode("LOITER")
        
        
        #once = True
    else:
        counter()
    '''if (cX<= 420 and cX>=200 and cY<=310 and cY>=130):
        vel_x = 0
        vel_y = 0
        vel_z = 0
        send_nav_velocity(0,0,0)
        if(mode_changed == 1):
            vehicle.mode = VehicleMode("AUTO")
            mode_changed = 0
    else:
        if(mode_changed == 0):
                vehicle.mode = VehicleMode("GUIDED")
                mode_changed = 1
        count = count + 1
        if(count == 3000):
            vehicle.mode = VehicleMode("AUTO")
            count = 0
            mode_changed = 1'''
    send_nav_velocity(vel_y,vel_x,vel_z)
    return dropped

while(True):
    nextwaypoint = vehicle.commands.next
    _, img = cap.read()

    #converting frame(img) from BGR (Blue-Green-Red) to HSV (hue-saturation-value)

    #roi = frame[100:280 , 230:430]
    cv2.rectangle(img,(230,100),(430,280),(0,255,0),5)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    #defining the range of orange color
    #orange_lower = np.array([5,40,100],np.uint8)
    orange_lower = np.array([5,40,60],np.uint8)
    orange_upper = np.array([19,255,255],np.uint8)

    #bluring image
    blur = cv2.GaussianBlur(hsv,(15,15),cv2.BORDER_DEFAULT)

    #finding the range orange colour in the image
    orange = cv2.inRange(blur, orange_lower, orange_upper)
    #orange_roi=cv2.inRange(hsv_roi, orange_lower, orange_upper)
    #Morphological transformation, Dilation
    kernal = np.ones((5 ,5), "uint8")

    #blue=cv2.dilate(yellow, kernal)

    res=cv2.bitwise_and(img, img, mask = orange)

    #Tracking Colour (orange)
    _,contours,hierarchy=cv2.findContours(orange,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #contours_roi,hierarchy_roi=cv2.findContours(orange_roi,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            if(area>2000):
                c = max(contours, key = cv2.contourArea)
                cv2.drawContours(img, [c], -1, (0, 255, 0), 2)
                M = cv2.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                print (cX,cY)
                cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)
                if (nextwaypoint == 3):
                    
                    if (pointauth == 2):
                        dropped = False
                    pointauth = 3
                    dropped = move_target_drop(cX,cY,5,1700,dropped)
                if (nextwaypoint == 4):
                    
                    if (pointauth == 3):
                        dropped = False
                    pointauth = 4
                    dropped = move_target_drop(cX,cY,5,2400,dropped)
                if (nextwaypoint == 5):
                    
                    if (pointauth == 4):
                        dropped = False
                    pointauth = 5
                    dropped = move_target_drop(cX,cY,6,1700,dropped)
                if (nextwaypoint == 6):
                    
                    if (pointauth == 5):
                        dropped = False
                    pointauth = 6
                    dropped = move_target_drop(cX,cY,6,2400,dropped)
                if (nextwaypoint == 7):
                    
                    if (pointauth == 6):
                        dropped = False
                    pointauth = 7
                    dropped = move_target_drop(cX,cY,7,1700,dropped)
                if (nextwaypoint == 8):
                    
                    if (pointauth == 7):
                        dropped = False
                    pointauth = 8
                    dropped = move_target_drop(cX,cY,7,2400,dropped)
                '''if (nextwaypoint == 9):
                    
                    if (pointauth == 8):
                        dropped = False
                    pointauth = 9
                    dropped = move_target_drop(cX,cY,8,1700,dropped)
		        if (nextwaypoint == 10):
                    
                    if (pointauth == 9):
                        dropped = False
                    pointauth = 10
                    dropped = move_target_drop(cX,cY,7,2400,dropped)'''
    cv2.imshow("Color Tracking",img)
    #cv2.imshow("Orange",res)

    if cv2.waitKey(10) & 0xFF == 27:
            cap.release()
            cv2.destroyAllWindows()
            break
