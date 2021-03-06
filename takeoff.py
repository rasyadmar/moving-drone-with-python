from dronekit import connect, VehicleMode, Command
import time

vehicle = connect('127.0.0.1:14550', wait_ready=True,baud=57600)

def arm_and_takeoff(aTargetAltitude):
    global step
    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    # while not vehicle.is_armable:
    #     print(" Tunggu inisiasi...")
    #     time.sleep(1)

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
        print(" Altitude: ", vehicle.rangefinder.distance)
        if vehicle.rangefinder.distance >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("sampai target altitude")
            step = 1
            break
        time.sleep(1)

if vehicle.mode == VehicleMode("GUIDED"):
    arm_and_takeoff(3)
    time.sleep(3)
    vehicle.mode = VehicleMode("LAND")