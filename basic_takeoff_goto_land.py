#!/usr/bin/env python
# -*- coding: utf-8 -*-
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time, math
from pymavlink import mavutil

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

# Getting state information stored in Vehicle class atributes
print("Autopilot Firmware version: %s" % vehicle.version)
print("Autopilot capabilities (supports ftp): %s" % vehicle.capabilities.ftp)
print("Global Location: %s" % vehicle.location.global_frame)
print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print("Local Location: %s" % vehicle.location.local_frame)    # NED
# Need to download vehicle commands before getting home locaiton
# Get Vehicle Home location - will be `None` until first set by autopilot
while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    if not vehicle.home_location:
        print("Waiting for home location ...")
        time.sleep(1)
print("Home location: %s" % vehicle.home_location)
print("Attitude: %s" % vehicle.attitude)
print("Velocity: %s" % vehicle.velocity)
print("GPS: %s" % vehicle.gps_0)
print("Last Heartbeat: %s" % vehicle.last_heartbeat)
print("Heading: %s" % vehicle.heading)
print("Is Armable?: %s" % vehicle.is_armable)
print("System status: %s" % vehicle.system_status.state)
print("Mode: %s" % vehicle.mode.name)  # Settable
print("Armed: %s" % vehicle.armed)  # Settable

# Print the value of the THR_MIN parameter.
print("Minimum Throttle: %s" % vehicle.parameters['THR_MIN'])


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


def get_location_metres(original_location, dNorth, dEast):
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


def get_distance_metres(aLocation1, aLocation2):
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


arm_and_takeoff(2)

print("Moving 2m north, 5m east")
target = get_location_metres(vehicle.location.global_relative_frame, 2, 5)  # NED velocities relative to copter frame (north forward, east right)
vehicle.simple_goto(target)
distance = get_distance_metres(vehicle.location.global_relative_frame, target)

# vehicle.mode = VehicleMode("LOITER")
# while not vehicle.mode.name == "LOITER":
#     print("Waiting for hovering...")
#     time.sleep(1)

print("We Hovering!")
time.sleep(5)

print("We Landing!")
vehicle.parameters["WPNAV_SPEED_DN"] = 10  # Set landing speed to 10 cm/s
vehicle.mode = VehicleMode("LAND")

while vehicle.location.global_relative_frame.alt >= 0:
    print("Altitude: %s" % vehicle.location.global_relative_frame.alt)
    time.sleep(1)

print("LANDED!")

# Close vehicle
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()
