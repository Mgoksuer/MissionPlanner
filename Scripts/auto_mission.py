import time
from pymavlink import mavutil
import pymavlink.dialects.v20.all as dialect
import geopy.distance

TAKEOFF_ALT = 30 # meters to takeoff

TARGET_LOCATIONS = [
    {"latitude": 0, "longitude": 0, "altitude": 50, "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, "current": 1},  # home
    {"latitude": 0, "longitude": 0, "altitude": TAKEOFF_ALT, "command": mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, "current": 0},  # takeoff
    {"latitude": -35.36024505, "longitude": 149.16777915, "altitude": 50, "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, "current": 0}, # sağ üst
    {"latitude": -35.36065266, "longitude": 149.15990337, "altitude": 40, "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, "current": 0}, # sol üst
    {"latitude": -35.36575376, "longitude": 149.15979735, "altitude": 50, "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, "current": 0}, # sol alt
    {"latitude": -35.36535853, "longitude": 149.16955120, "altitude": 50, "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, "current": 0}, # sağ alt
    {"latitude": -35.36718312, "longitude": 149.16554466, "altitude": 50, "command": mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, "current": 0}, # pistin arkasından o doğrultuda uzak bir nokta, landing e hazırlık için
    {"latitude": -35.36272610, "longitude": 149.16514406, "altitude": 0, "command": mavutil.mavlink.MAV_CMD_NAV_LAND, "current": 0} # land
]

# Linux Terminal command to run mavproxy with this script:
# mavproxy.py --master= --baudrate=57600 --console --map --load-module=horizon --out=udp:127.0.0.1:14560

# Establish a MAVLink connection to the vehicle using UDP
vehicle = mavutil.mavlink_connection(device="udpin:127.0.0.1:14560")

# Wait for the first heartbeat message to confirm communication
vehicle.wait_heartbeat()
print(f"Connected to system {vehicle.target_system}, component {vehicle.target_component}")

# Retrieve a parameter value from the vehicle by name
def get_parameter_value(param_name):
    # Request the parameter value
    vehicle.mav.param_request_read_send(
        vehicle.target_system,
        vehicle.target_component,
        param_name.encode(),  # Convert string to bytes
        -1 # Parameter index (-1 means use the name)
    )

    # Listen for the response
    while True:
        msg = vehicle.recv_match(type='PARAM_VALUE', blocking=True)
        if msg is not None and msg.param_id.strip('\x00') == param_name:
            print(f"Received parameter {param_name}: {msg.param_value}")
            return msg.param_value
        else:
            print("Parameter request unsuccessful.")

# Set the vehicle to a specific flight mode
def set_mode(mode_name):
    if mode_name not in vehicle.mode_mapping():
        print(f"Mode {mode_name} not supported.")
        exit(1)

    mode_id = vehicle.mode_mapping()[mode_name]

    # Send mode change request
    vehicle.mav.set_mode_send(
        vehicle.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode set to {mode_name} requested.")

    # Wait for confirmation of mode change
    while True:
        current_mode = vehicle.recv_match(type='HEARTBEAT', blocking=True).to_dict()
        if current_mode['custom_mode'] == mode_id:
            print(f"Mode change to {mode_name} confirmed.")
            break

# Perform prearm checks and arm the vehicle if ready
def arm_vehicle():
    while True:
        message = vehicle.recv_match(type=dialect.MAVLink_sys_status_message.msgname, blocking=True)
        if message:
            message = message.to_dict()

            # Check the prearm status bit from onboard sensor health flags
            prearm_status_bit = message["onboard_control_sensors_health"] & dialect.MAV_SYS_STATUS_PREARM_CHECK
            if prearm_status_bit == dialect.MAV_SYS_STATUS_PREARM_CHECK:
                print("Vehicle is armable")
                break
            else:
                print("Prearm not OK! Waiting...")
                time.sleep(2)

    # Send the arm command
    print("Arming vehicle...")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    vehicle.motors_armed_wait()
    print("Vehicle armed.")

# Start the takeoff sequence and record the exact takeoff location
def initiate_takeoff():
    print("Initiating takeoff...")
    # Ensure vehicle is armed

    # Monitor altitude to ensure takeoff is in progress
    while True:
        msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True).to_dict()
        relative_alt = msg.get("relative_alt", 0) * 1e-3
        if relative_alt >= TAKEOFF_ALT - 5:
            print("Takeoff complete.")
            break

# Upload the mission waypoints to the vehicle, including a final LAND command
def upload_mission(waypoints):
    vehicle.waypoint_clear_all_send()  # Clear any existing mission

    # Inform vehicle how many mission items to expect
    vehicle.mav.mission_count_send(vehicle.target_system, vehicle.target_component, len(waypoints))

    # Send each waypoint or mission item
    for i, wp in enumerate(waypoints):
        command = wp.get("command")

        msg = mavutil.mavlink.MAVLink_mission_item_int_message(
            target_system=vehicle.target_system,
            target_component=vehicle.target_component,
            seq=i,
            frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            command=command,
            current=wp["current"],
            autocontinue=1,
            param1=0, param2=0, param3=0, param4=0,
            x=int(wp["latitude"] * 1e7),
            y=int(wp["longitude"] * 1e7),
            z=wp["altitude"]
        )
        vehicle.mav.send(msg)
        print(f"Sent waypoint {i} (command: {command})")

    # Wait for vehicle to acknowledge mission upload
    while True:
        ack = vehicle.recv_match(type='MISSION_ACK', blocking=True).to_dict()
        if ack['type'] == 0:
            print("Mission upload acknowledged.")
            break
        else:
            print("Mission upload failed.")
            exit(1)

# Switch to AUTO mode and start the mission
def start_mission():
    set_mode("AUTO")
    print("Mission started in AUTO mode.")

# Monitor the mission execution by tracking current waypoint index
def track_mission(waypoints_count):
    print("Tracking mission progress...")
    last_seq = -1
    while True:
        msg = vehicle.recv_match(type='MISSION_CURRENT', blocking=True)
        if msg:
            current_seq = msg.seq
            if current_seq != last_seq:
                print(f"Flying to waypoint {current_seq}")
                last_seq = current_seq
            if current_seq >= waypoints_count - 1:
                print("Mission complete. LANDING should be in progress.")
                break

# Main mission flow
upload_mission(TARGET_LOCATIONS)
start_mission()
initiate_takeoff()
track_mission(len(TARGET_LOCATIONS) + 1)  # Include LAND as final waypoint
