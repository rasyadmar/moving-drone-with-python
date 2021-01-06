from dronekit import connect, VehicleMode, Command
from time import sleep
import time
print("hey brother")
vehicle = connect('COM9', wait_ready=True,baud=921600)
print("connected")
print("Basic pre-arm checks")
# Don't let the user try to arm until autopilot is ready
while not vehicle.is_armable:
    print(" Tunggu inisiasi...")
    time.sleep(1)

print("Arming")
# Copter should arm in GUIDED mode
vehicle.mode = VehicleMode("GUIDED")
print("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print(" Menunggu arming...")
    time.sleep(1)

print("Take off!")
vehicle.simple_takeoff(2)  # Take off to target altitude

# Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
#  after Vehicle.simple_takeoff will execute immediately).
while True:
    print(" Altitude: ", vehicle.rangefinder.distance)
    if vehicle.rangefinder.distance >= 1 * 0.95:  # Trigger just below target alt.
        print("sampai target altitude")
        step = 1
        break
    time.sleep(1)

sleep(10)

vehicle.mode = VehicleMode("LAND")
print("landing")