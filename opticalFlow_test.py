from dronekit import connect, VehicleMode, Command
from pymavlink import mavutil
import sched, time, math

# Connect to the Vehicle
#connectionString = input("FTDI port: ")
#vehicle = connect(connectionString, wait_ready=True, baud=921600)
vehicle = connect('/dev/ttyUSB0', wait_ready=True, baud=921600)

# variable
step = 0

# fungsi
def nav_velocity_local(velocity_x, velocity_y, velocity_z):
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

def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
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

def set_attitude(roll_angle=0.0, pitch_angle=0.0, yaw_rate=0.0, thrust=0.5, duration=0):

    # Duration is seconds to do this for

    msg = vehicle.message_factory.set_attitude_target_encode(
        0,
        0,  # target system
        0,  # target component
        0b00000000,  # type mask: bit 1 is LSB
        to_quaternion(roll_angle, pitch_angle),  # q
        0,  # body roll rate in radian
        0,  # body pitch rate in radian
        math.radians(yaw_rate),  # body yaw rate in radian
        thrust)  # thrust

    vehicle.send_mavlink(msg)

    if duration != 0:
        # Divide the duration into the frational and integer parts
        modf = math.modf(duration)

        # Sleep for the fractional part
        time.sleep(modf[0])

        # Send command to vehicle on 1 Hz cycle
        for x in range(0, int(modf[1])):
            time.sleep(1)
            vehicle.send_mavlink(msg)

def arm_and_takeoffNoGPS(Target):

    # Arm the vehicle

    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)  # Waiting for armed
    # Take Off
    thrust = 0.7
    while True:
        current_altitude = vehicle.rangefinder.distance
        print("altitude: %f" % current_altitude)
        if current_altitude >= Target * 0.95:
            break
        elif current_altitude >= Target * 0.6:
            thrust = 0.6
        set_attitude(thrust=thrust)
        time.sleep(0.2)

def send_nav_velocity(velocity_x, velocity_y, velocity_z):
    # create the SET_POSITION_TARGET_LOCAL_NED command
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_y, velocity_x, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)  # yaw, yaw_rate (not used)
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()
    # gerakan x itu y di dunia asli dan begitu sebaliknya

def send_ned_distance(distance_x,distance_y, distance_z):
    # create the SET_POSITION_TARGET_LOCAL_NED command
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        distance_y, distance_x, distance_z,  # north, east, down positions
        0, 0, 0,  # x, y, z velocity in m/s
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

    print("Arming")

    vehicle.armed = True

    while not vehicle.armed:
        print(" Menunggu arming...")
        time.sleep(1)

    print("Take off!")
    #vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.rangefinder.distance)
        dis_z = - 2
        #vel_z = -0.5
        if vehicle.rangefinder.distance >= aTargetAltitude * 0.95:  # Trigger just below target alt.
            print("sampai target altitude")
            send_ned_distance(0, 0, 0)
            #send_nav_velocity(0, 0, 0)
            step = 1
            break
        send_ned_distance(0, 0, dis_z)
        #send_nav_velocity(0, 0, vel_z)
        time.sleep(1)

def test_distance():
    print("distance test")
    global step
    global move
    dis_x = 0
    dis_y = 0
    dis_z = 0
    if move == 0:
        print("maju")
        dis_y = 1
    elif move == 1:
        print("mundur")
        dis_y = -1
    elif move == 2:
        print("kanan")
        dis_x = 1
    elif move == 3:
        print("kiri")
        dis_x = -1
    elif move == 4:
        print("turun")
        dis_z = 1
    elif move == 5:
        print("naik")
        dis_z = -1
    send_ned_distance(dis_x,dis_y,dis_z)
    time.sleep(4)
    move = + 1
    if move == 5:
        step = +1

def test_velocity():
    print("velocity test")
    global step
    global move
    vel_x = 0
    vel_y = 0
    vel_z = 0
    if move == 0:
        print("maju")
        vel_y = 0.5
    elif move == 1:
        print("mundur")
        vel_y =-0.5
    elif move == 2:
        print("kanan")
        vel_x = 0.5
    elif move == 3:
        print("kiri")
        vel_x = -0.5
    elif move == 4:
        print("turun")
        vel_z = 0.1
    elif move == 5:
        print("naik")
        vel_z =-0.1
    send_nav_velocity(vel_y, vel_x, vel_z)
    time.sleep(4)
    move =+ 1
    if move == 5:
        step=+1

def land():
    global step
    vehicle.mode = VehicleMode("LAND")
    while not vehicle.mode.name == 'LAND':  # Wait until mode has changed
        print(" menunggu masuk mode land ...")
        time.sleep(1)
    step = 5


while True:
    if vehicle.mode == VehicleMode("GUIDED"):
    #if vehicle.mode == VehicleMode("GUIDED_NOGPS"):
        if step == 0:
            arm_takeoff_local(2)
            #arm_and_takeoffNoGPS(2)
            #arm_and_takeoff(2)
        elif step == 1:
            nav_velocity_local(0,0.1,0)
            #test_distance()
        elif step == 2:
            print("landing")
            land()
        else:
            break
    # else:
    # break
