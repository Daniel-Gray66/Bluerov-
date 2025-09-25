"""
Example script to continuously move a BlueROV (ArduSub) in a loop
until user stops it with Ctrl+C.
"""

import sys
import time
from pymavlink import mavutil

# -----------------------------------------------------------------------------
# Helper functions
# -----------------------------------------------------------------------------

# This function establishes a MAVLink connection and returns the connection object if successful, or False if not
def create_connectionlink():
    try:
        # The connection to the raspberry pi in the bluerov
        print("Attempting to connect to BlueROV at 192.168.2.2:14550 (client mode)...")
        # Use the client mode (udpout) that worked in the simple test
        master = mavutil.mavlink_connection('udpout:192.168.2.2:14550')
        
        print("Connection created, sending initial heartbeats...")
        # Send our own heartbeat to trigger response
        for i in range(3):
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,  # GCS type
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 
                0, 0, 0
            )
            print(f"Sent heartbeat {i+1}/3")
            time.sleep(1)

        print("Waiting for heartbeat...")
        if master.wait_heartbeat(timeout=10):  
            print("Heartbeat received. Connection established.")

            # This is just printing out data requests.
            master.mav.request_data_stream_send(
                master.target_system, 
                master.target_component, 
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
            )

            return master  # Returning the connection link
        else:
            #if no heartbeat was detected then we return false
            print("Heartbeat not received. Connection failed.")
            return None
    except Exception as e:
        print(f"An error occurred: {e}")
        return None



def force_reboot_autopilot(master):
    """
    Sends a MAVLink command to reboot the autopilot.
    """
    # MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
    # param1=1 => Reboot autopilot
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        1, 0, 0, 0, 0, 0, 0
    )

    # Optionally wait for ACK
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if msg:
        print(f"COMMAND_ACK received: command={msg.command}, result={msg.result}")
    else:
        print("No ACK received. The autopilot might already be rebooting.")


def arm_vehicle(master, timeout=10):
    """
    Attempts to arm the vehicle within `timeout` seconds.
    Returns True if armed, False otherwise.
    """
    print("Sending arm command...")
    master.arducopter_arm()

    start_time = time.time()
    while time.time() - start_time < timeout:
        # Check if motors are armed
        if master.motors_armed():
            print("Vehicle is armed.")
            return True

        # Optionally, check the COMMAND_ACK message to see if it was accepted
        msg = master.recv_match(type='COMMAND_ACK', blocking=False)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Arming accepted by autopilot.")
                return True
            else:
                print(f"Arming rejected (result code {msg.result}).")
                return False

        time.sleep(0.5)

    print("Arming timed out. Vehicle did not arm.")
    return False

def set_mode(master, mode='MANUAL', timeout=5):
    """
    (Optional) Switch flight mode (e.g., MANUAL for ArduSub).
    You can skip this if you're already in a mode that accepts RC override.
    """
    # Attempt to get the custom mode mapping from the autopilot
    mode_id = master.mode_mapping().get(mode)
    if mode_id is None:
        print(f"Mode {mode} not found in mode mapping. Check available modes.")
        return


def disarm_vehicle(master):
    """
    Disarms the vehicle.
    """
    print("Sending disarm command...")
    master.arducopter_disarm()
    time.sleep(1)  # Let the disarm command process
    if not master.motors_armed():
        print("Vehicle is now disarmed.")
    else:
        print("Warning: Vehicle still armed.")


def control_motors(master, roll=1500, pitch=1500, throttle=1500, yaw=1500, ch5=1500, ch6=1500, ch7 =1900):
    """
    Send RC override commands to control the motors.

    By default, 1500 is neutral. 1000-2000 is the typical PWM range.
    """
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        roll,     # Channel 1: Roll
        pitch,    # Channel 2: Pitch
        throttle, # Channel 3: Throttle
        yaw,      # Channel 4: Yaw
        ch5,      # Channel 5
        ch6,      # Channel 6
        ch7,        # Channel 7 (unused) might be lights
        0         # Channel 8 (unused)
    )



def is_too_close(box, frame_width, frame_height, threshold=0.3):
    """
    Determines if the ROV is too close to the object based on the size of the bounding box.
    Returns True if too close, False otherwise.
    """
    x_min, y_min, x_max, y_max = box['bbox']
    box_width = x_max - x_min
    box_height = y_max - y_min

    # Calculate the area of the bounding box as a fraction of the frame area
    box_area = box_width * box_height
    frame_area = frame_width * frame_height
    area_ratio = box_area / frame_area

    # If the area ratio exceeds the threshold, the ROV is too close
    return area_ratio > threshold


    

def control_direction_and_motors(box, master, frame_width, frame_height):
    """
    Adjusts motor control based on the position of the detected object.
    """
    x_min, y_min, x_max, y_max = box['bbox']
    center_x = (x_min + x_max) / 2
    center_y = (y_min + y_max) / 2

    # Calculate the center of the frame
    frame_center_x = frame_width / 2
    frame_center_y = frame_height / 2

    # Determine the direction to move based on the object's position
    if center_x < frame_center_x - 50:  # Object is to the left
        roll = 1400  # Adjust roll to move left
    elif center_x > frame_center_x + 50:  # Object is to the right
        roll = 1600  # Adjust roll to move right
    else:
        roll = 1500  # Neutral roll

    if center_y < frame_center_y - 50:  # Object is above
        throttle = 1600  # Adjust throttle to move up
    elif center_y > frame_center_y + 50:  # Object is below
        throttle = 1400  # Adjust throttle to move down
    else:
        throttle = 1500  # Neutral throttle

    # Send the motor control command
    control_motors(master, roll, 1500, throttle, 1500, 1500, 1500)
    
def set_thruster_values(master, thruster_values):
    """
    Send individual thruster commands to the BlueROV.
    
    thruster_values: A list of 8 values (-1.0 to 1.0), corresponding to the 8 thrusters.
    """
    master.mav.set_actuator_control_target_send(
        0,  # Time in microseconds (0 = use system time)
        master.target_system,  # Target system (ROV)
        master.target_component,  # Target component
        0,  # Control group (0 = main outputs)
        thruster_values  # List of thruster values [-1.0 to 1.0]
    )