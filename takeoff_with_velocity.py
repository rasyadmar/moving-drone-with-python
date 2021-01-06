from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import time

vehicle = connect('COM9', wait_ready=True, baud=57600)
print("connected")

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
    # gerakan x itu y di dunia asli dan begitu sebaliknya

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
            time.sleep(1)
        else:
            break

def land_function():
    while True:
        print("landing current Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.mode == VehicleMode("GUIDED"):
            if vehicle.location.global_relative_frame.alt >= 1 * 1.05:
                send_nav_velocity(0, 0, 0.5)
            else:
                vehicle.mode = VehicleMode("LAND")
        else:
            break
        time.sleep(1)

arm_and_takeoff(2)
land_function()

