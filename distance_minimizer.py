#!/usr/bin/env python
# -*- coding: utf-8 -*-
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time, math
from pymavlink import mavutil
import random

# Connect to SITL vehicle
from dronekit_sitl import SITL
sitl = SITL()
sitl.download('copter', '3.3', verbose=True)
sitl_args = ['-I0', '--model', 'quad', '--home=49.26778354815404,-123.17927956581117,0,90']
sitl.launch(sitl_args, await_ready=True, restart=True)
connect_string = 'tcp:127.0.0.1:5760'  # Local TCP endpoint for SITL runs. Can connect MP to 5763
vehicle = connect(connect_string, wait_ready=True)

# # Connect to Physical vehicle
# connect_string = '/dev/ttyAMA0'  # Serial device endpoint for connecting to physical drone
# vehicle = connect(connect_string, wait_ready=True, baud=57600)


# We want a probability distribution between choices (1m in every direction)
# We want this distribution to change according to whether the last move got us closer or farther away

# Define movement choices and initialize choice weights evenly
north = {'n':1, 'e':0}
south = {'n':-1, 'e':0}
east = {'n':0, 'e':1}
west = {'n':0, 'e':-1}
choices = (north, south, east, west)
weights = [25, 25, 25, 25]
weight_range = (10, 40)


def send_velocity_cmd(north_vel, east_vel, down_vel, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions
        north_vel, east_vel, down_vel,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


def get_location_meters(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation = LocationGlobal(newlat, newlon, original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation = LocationGlobalRelative(newlat, newlon, original_location.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation


def get_distance_meters(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def arm_and_takeoff(target):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)
    
    while not vehicle.home_location:
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        if not vehicle.home_location:
            print("Waiting for home location ...")
            time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    # (otherwise the command after Vehicle.simple_takeoff will execute immediately).
    while vehicle.location.global_relative_frame.alt <= target * 0.95:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)
        time.sleep(1)
    print("Reached target altitude")


def move(n_target, e_target, tolerance=0.5):
    """
    Moves quad n_target meters north, e_target meters east
    tolerance defines how close the quad must get before moving on
    """
    print("Moving %g meters north and %g meters east" % (n_target, e_target))
    target = get_location_meters(vehicle.location.global_relative_frame, n_target, e_target)  # NED velocities relative to copter frame (north forward, east right)
    vehicle.simple_goto(target)
    distance = get_distance_meters(vehicle.location.global_relative_frame, target)

    while distance >= tolerance:
        #print("Distance to Target: %s" % distance)
        distance = get_distance_meters(vehicle.location.global_relative_frame, target)
        time.sleep(1)


def minimize_distance(location, tolerance=0.5):
    dist = get_distance_meters(vehicle.location.global_relative_frame, location)
    while dist >= tolerance:
        # Choose action and execute
        action = random.choices(choices, weights=weights, k=1)[0]
        move(action['n'], action['e'])
        
        # Update distance reading and calculate action choice weight shift
        last_dist = dist
        dist = get_distance_meters(vehicle.location.global_relative_frame, location)
        choice_ind = choices.index(action)
        impact_factor = abs(dist - last_dist) # Magnitude of difference indicates how directly towards/away from the target the movement was
        weight_shift = 10 #* impact_factor # Changes weight most significantly when actions result in large changes in distance from target

        if weights[choice_ind] in range(weight_range[0], weight_range[1]):
            print(weights)
            # That move got us closer
            # Increase likelihood that next move is the same
            if last_dist > dist:
                # choice_ind odd means choice was south or west
                # Opposing direction is choice_ind - 1
                if choice_ind % 2 == 1:
                    weights[choice_ind] = weights[choice_ind] + weight_shift
                    weights[choice_ind-1] = weights[choice_ind-1] - weight_shift
                # choice_ind even means choice was north or east
                # Opposing direction is choice_ind + 1
                else:
                    weights[choice_ind] = weights[choice_ind] + weight_shift
                    weights[choice_ind+1] = weights[choice_ind+1] - weight_shift
            # That move put us father away from the target
            # Decrease likelihood that next move is the same
            else:
                # choice_ind odd means choice was south or west
                # Opposing direction is choice_ind - 1
                if choice_ind % 2 == 1:
                    weights[choice_ind] = weights[choice_ind] - weight_shift
                    weights[choice_ind-1] = weights[choice_ind-1] + weight_shift
                # choice_ind even means choice was north or east
                # Opposing direction is choice_ind + 1
                else:
                    weights[choice_ind] = weights[choice_ind] - weight_shift
                    weights[choice_ind+1] = weights[choice_ind+1] + weight_shift
    print("CLOSE ENOUGH")
    return 

arm_and_takeoff(2)
move(20, 25)

minimize_distance(vehicle.home_location, tolerance=3)

vehicle.mode = VehicleMode("LAND")
vehicle.parameters["WPNAV_SPEED_DN"] = 10  # Set landing speed to 10 cm/s

while vehicle.location.global_relative_frame.alt > 0:
    print("Altitude: %s" % vehicle.location.global_relative_frame.alt)
    time.sleep(1)

print("Returned to launch")
