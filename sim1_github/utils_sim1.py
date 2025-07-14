#!/usr/bin/env python
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pymavlink import mavutil
from geopy.distance import distance


def move_gps(lat, lon, meters,dir):
    # Define the starting point
    start_point = (lat, lon)
    # Calculate the new point after moving forward by 'meters'
    new_point = distance(meters=meters).destination(start_point, dir)
    return [new_point.latitude, new_point.longitude]

def make_connections():
    connections = []
    try: 
        for i in range(NUM_DRONES): #i=0,1,2,3...

            port = 14550 + (i*10)   #originally was 14551
            conn = mavutil.mavlink_connection(f"udpin:localhost:{port}")
            conn.wait_heartbeat()
            print("Heartbeat from system (system %u component %u)" % (conn.target_system, conn.target_component))
            connections.append(conn)
        
        return connections
    
    except Exception as e:
        print("Error occurred")
        print(e)
        return []

def wait_for_ack(conn,cmd):         
    """
        Wait until it gets an Acknowledgment,
        If >5 seconds pass while waiting for ACK, 
        then it stops waiting, moving on to next function
        Script terminates if encounters an error
    """
    start_time = time.time()
    while True:
        elapsed_time = time.time() - start_time
        if elapsed_time > 5:
            return
        msg = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if msg is not None:
            # print(msg)
            msg = msg.to_dict()
            if msg['result'] == 0:
                # print(f"{cmd} ACK:  {msg}")
                time.sleep(3)
                return
            else:
                print(f"some ERROR occured - {cmd} ACK:  {msg}")
                sys.exit()


def setmode(conn,mode):

    # Check if mode is available
    if mode not in conn.mode_mapping():
        print('Unknown mode : {}'.format(mode))
        print('Try:', list(conn.mode_mapping().keys()))
        return False

    mode_id = conn.mode_mapping()[mode]  

    # conn.mav.set_mode_send(conn.target_system,mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_id)
    conn.set_mode(mode_id) 
    # conn.mav.command_long_send(conn.target_system, conn.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 0, mode_id, 0, 0, 0, 0, 0)

    while True:
    #     keep checking mode until it is set to GUIDED
        msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            print(mode)
            # print(msg)
            if mode == 'GUIDED':
                break


    # wait_for_ack(conn,"Set Mode")
        
    return True


def arm(conn):
    # Arm the UAVS
    conn.mav.command_long_send(conn.target_system, conn.target_component ,
                                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)  
    #Third argument represents force ..... In Second argument 1 means arm , 0 means disarm ^

    wait_for_ack(conn,"Arm")
    conn.motors_armed_wait()


def disarm(conn):
    # Disarm the UAVS
    conn.mav.command_long_send(conn.target_system, conn.target_component ,
                                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)  
    #Third argument represents force ..... In Second argument 1 means arm , 0 means disarm ^


    wait_for_ack(conn,"Disarm")


def takeoff(conn):

    # Command Takeoff
    altitude = 3
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                                            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)

    wait_for_ack(conn,"Takeoff")


def land(conn):

    # Command Land
    latitude = 0
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                                            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, latitude, 0, 0)

    wait_for_ack(conn,"Landing")


def return_to_start(conn):

    # Command Return to starting point
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                                            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)

    wait_for_ack(conn,"Returning to Start")

def moveglobal(conn,lat,long,h):
    mask = GLOBAL_SET_POSITION_DICTIONARY['pos']
    conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, conn.target_system,conn.target_component, 
                  mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mask, 
                  int(lat * 10 **7 ),int(long * 10 **7), 1+h, 0, 0, 0, 0, 0, 0, 0, 0))
                #   int(-35.3629849 * 10 ** 7), int(149.1649185 * 10 ** 7), 10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))
    time.sleep(10)
    
def moveglobalfast(conn,lat,long,h):
    mask = GLOBAL_SET_POSITION_DICTIONARY['pos']
    conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, conn.target_system,conn.target_component, 
                  mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mask, 
                  int(lat * 10 **7 ),int(long * 10 **7), 1+h, 0, 0, 0, 0, 0, 0, 0, 0))
                #   int(-35.3629849 * 10 ** 7), int(149.1649185 * 10 ** 7), 10, 0, 0, 0, 0, 0, 0, 1.57, 0.5))
    time.sleep(3)

def dance(conn,lat,long):
    i = 0
    while i < 3:
        k = 2
        mask = GLOBAL_SET_POSITION_DICTIONARY['pos']
        conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, conn.target_system,conn.target_component, 
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mask, 
                    int(lat * 10 **7 ),int(long * 10 **7), 2, 0, 0, 0, 0, 0, 0, 0, 0))

        time.sleep(k)
        conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, conn.target_system,conn.target_component, 
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mask, 
                    int(lat * 10 **7 ),int(long * 10 **7), 1, 0, 0, 0, 0, 0, 0, 0, 0))
        time.sleep(k)
        i+=1

def move(conn,cmd):

    dist_x = 0
    dist_y = 0
    height = -4

    if cmd == "right":  #if you are NOT rotating the drone while turning, u r just going forward, reverse, left
        cmd = "left"

    if cmd == "":
        height = -10        #  -10 means North in z-axis, and +10 means South in z-axis
    elif cmd == "forward":
        dist_y = 2
    elif cmd == "backward":
        dist_y = -2
    elif cmd == "left":
        dist_x = -2
    elif cmd == "right":
       dist_x = 2
    else :
        print("wrong cmd to MOVE")
        sys.exit()


    mask = LOCAL_SET_POSITION_DICTIONARY['pos']
    conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, conn.target_system,conn.target_component, 
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, dist_x, dist_y, height, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(10)
    
    # while 1:          # for position feedback w.r.t goal point
    #     msg = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    #                                  or
    #     msg = conn.recv_match(type='NAV_CONTROLLER_OUTPUT', blocking=True)
    #     print(msg)


def movewrtlocal(conn,cmd,prev):
    dist_x = prev[0]
    dist_y = prev[1]
    height = -4

    if cmd == "right":  #if you are NOT rotating the drone while turning, u r just going forward, reverse, left
        cmd = "left"

    if cmd == "":
        print("Stopping The Drone")
        height = -10        #  -10 means North in z-axis, and +10 means South in z-axis
    elif cmd == "forward":
        dist_x -= 2
    elif cmd == "backward":
        dist_x += 2
    elif cmd == "left":
        dist_y += 2
    elif cmd == "right":
       dist_y -= 2
    else :
        print("wrong cmd to MOVE")
        sys.exit()


    mask = LOCAL_SET_POSITION_DICTIONARY['pos']
    conn.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, conn.target_system,conn.target_component, 
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, dist_x, dist_y, height, 0, 0, 0, 0, 0, 0, 0, 0))
    time.sleep(10)


def simultaneous_takeoff(conn):
    """
    Function to set the mode, arm, and takeoff a single drone.
    """
    ready = setmode(conn, 'GUIDED')
    if ready:
        arm(conn)
        takeoff(conn)
        conn.close()
        return conn.target_system, True
    else:
        return conn.target_system, False

def handle_simultaneous_takeoff(connections):
    # Create a ThreadPoolExecutor to handle drones concurrently
    with ThreadPoolExecutor(max_workers=len(connections)) as executor:
        # Submit all tasks to the executor
        futures = [executor.submit(simultaneous_takeoff, conn) for conn in connections]

        # Process the results as they complete
        for future in as_completed(futures):
            target_system, success = future.result()
            if success:
                print(f"Drone {target_system} successfully took off.")
            else:
                print(f"Drone {target_system} will not take off --- due to some error")

def simultaneous_land(conn,coord,i):
    """
    Function to set the mode, arm, and takeoff, and land a single drone to its initial location.
    """
    ready = setmode(conn, 'GUIDED')
    if ready:
        arm(conn)
        takeoff(conn)
        moveglobal(conn,coord[0],coord[1],i)
        moveglobal(conn,coord[0],coord[1],1)
        return conn.target_system, True
    else:
        return conn.target_system, False

def handle_simultaneous_landing(connections,coordinates):
        # Create a ThreadPoolExecutor to handle drones concurrently
    with ThreadPoolExecutor(max_workers=len(connections)) as executor:
        # Submit all tasks to the executor
        futures = [executor.submit(simultaneous_land, conn, coordinates[i],i) for i,conn in enumerate(connections)]

        # Process the results as they complete
        for future in as_completed(futures):
            target_system, success = future.result()
            if success:
                print(f"Drone {target_system} successfully landed to its initial location.")
            else:
                print(f"Drone {target_system} will not take off --- due to some error")


GLOBAL_SET_POSITION_DICTIONARY = {
    'pos' : 3576,
    'vel' : 3527,
    'acc' : 3128,
    'pos+vel' : 3520,
    'pos+vel+acc' : 3072,
    'yaw' : 2559,
    'yaw_rate' : 1535,
}

LOCAL_SET_POSITION_DICTIONARY = GLOBAL_SET_POSITION_DICTIONARY.copy()
LOCAL_SET_POSITION_DICTIONARY['acc'] = 3135
NUM_DRONES = 4
