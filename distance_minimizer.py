#!/usr/bin/env python
# -*- coding: utf-8 -*-
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
import time, math
from pymavlink import mavutil
import random
from Beacon import Beacon

# Connect to SITL vehicle
from dronekit_sitl import SITL
sitl = SITL()
sitl.download('copter', '3.3', verbose=True)
sitl_args = ['-I0', '--model', 'quad', '--home=49.26778354815404,-123.17927956581117,0,0']
sitl.launch(sitl_args, await_ready=True, restart=True)
connectString = 'tcp:127.0.0.1:5760'  # Local TCP endpoint for SITL runs. Can connect MP to 5763
vehicle = connect(connectString, wait_ready=True)

# # Connect to Physical vehicle
# connectString = '/dev/ttyAMA0'  # Serial device endpoint for connecting to physical drone
# vehicle = connect(connectString, wait_ready=True, baud=57600)

# Define flight parameters
redirectDelay = 2
airSpeed = 2



def sendVelocityCmd(north_vel, east_vel, down_vel, duration=None):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions
        north_vel, east_vel, down_vel,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle if provided a duration
    if duration:
        for x in range(0, duration):
            vehicle.send_mavlink(msg)
            time.sleep(1)
    else:
        vehicle.send_mavlink(msg)


def getLocationMeters(originalLocation, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `originalLocation`. The returned LocationGlobal has the same `alt` value
    as `originalLocation`.

    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.

    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.

    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius = 6378137.0  # Radius of "spherical" earth
    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * originalLocation.lat / 180))

    # New position in decimal degrees
    newlat = originalLocation.lat + (dLat * 180 / math.pi)
    newlon = originalLocation.lon + (dLon * 180 / math.pi)
    if type(originalLocation) is LocationGlobal:
        targetlocation = LocationGlobal(newlat, newlon, originalLocation.alt)
    elif type(originalLocation) is LocationGlobalRelative:
        targetlocation = LocationGlobalRelative(newlat, newlon, originalLocation.alt)
    else:
        raise Exception("Invalid Location object passed")

    return targetlocation


def getDistanceMeters(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code:
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def armAndTakeoff(target):
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
    target = getLocationMeters(vehicle.location.global_relative_frame, n_target, e_target)
    vehicle.simple_goto(target)
    distance = getDistanceMeters(vehicle.location.global_relative_frame, target)

    while distance >= tolerance:
        distance = getDistanceMeters(vehicle.location.global_relative_frame, target)
        time.sleep(1)


def minimizeNS(beacon, tolerance):
    """
    North-South Minimization Phase
    Move North, if you are getting closer, keep going, otherwise go South instead
    Keep going in a direction, tracking latitude along the way, until you start getting further from the beacon
    Return to latitude at which distance was minimum
    """
    increasingCount = 0
    lastDist = beacon.getDistance(vehicle.location.global_relative_frame)
    bestLat = vehicle.location.global_relative_frame.lat
    print("Going North")
    sendVelocityCmd(airSpeed, 0, 0, redirectDelay)
    dist = beacon.getDistance(vehicle.location.global_relative_frame)
    while increasingCount < 4:
        sendVelocityCmd(airSpeed, 0, 0)
        bestLat = vehicle.location.global_relative_frame.lat
        lastDist = dist
        dist = beacon.getDistance(vehicle.location.global_relative_frame)
        if lastDist < dist:
            increasingCount = increasingCount + 1
        elif lastDist > dist:
            increasingCount = 0
            bestLat = vehicle.location.global_relative_frame.lat
        print("dist: %.4f" % dist)
        time.sleep(0.1)

    increasingCount = 0
    lastDist = beacon.getDistance(vehicle.location.global_relative_frame)
    print("Going South")
    sendVelocityCmd(0 - airSpeed, 0, 0, redirectDelay)
    dist = beacon.getDistance(vehicle.location.global_relative_frame)
    while increasingCount < 4:
        sendVelocityCmd(0 - airSpeed, 0, 0)
        lastDist = dist
        dist = beacon.getDistance(vehicle.location.global_relative_frame)
        if lastDist < dist:
            increasingCount = increasingCount + 1
        elif lastDist > dist:
            increasingCount = 0
            bestLat = vehicle.location.global_relative_frame.lat
        print("dist: %.4f" % dist)
        time.sleep(0.1)

    return bestLat


def minimizeEW(beacon, tolerance):
    """
    East-West Minimization Phase
    Move East, if you are getting closer, keep going, otherwise go West instead
    Keep going in a direction, tracking longitude along the way, until you start getting further from the beacon
    Return to latitude at which distance was minimum
    """
    increasingCount = 0
    lastDist = beacon.getDistance(vehicle.location.global_relative_frame)
    bestLon = vehicle.location.global_relative_frame.lon
    print("Going East")
    sendVelocityCmd(0, airSpeed, 0, redirectDelay)
    dist = beacon.getDistance(vehicle.location.global_relative_frame)
    while increasingCount < 4:
        sendVelocityCmd(0, airSpeed, 0)
        bestLon = vehicle.location.global_relative_frame.lon
        lastDist = dist
        dist = beacon.getDistance(vehicle.location.global_relative_frame)
        if lastDist < dist:
            increasingCount = increasingCount + 1
        elif lastDist > dist:
            increasingCount = 0
            bestLon = vehicle.location.global_relative_frame.lon
        print("dist: %.4f" % dist)
        time.sleep(0.1)

    increasingCount = 0
    lastDist = beacon.getDistance(vehicle.location.global_relative_frame)
    print("Going West")
    sendVelocityCmd(0, 0 - airSpeed, 0, redirectDelay)
    dist = beacon.getDistance(vehicle.location.global_relative_frame)
    while increasingCount < 4:
        sendVelocityCmd(0, 0 - airSpeed, 0)
        lastDist = dist
        dist = beacon.getDistance(vehicle.location.global_relative_frame)
        if lastDist < dist:
            increasingCount = increasingCount + 1
        elif lastDist > dist:
            increasingCount = 0
            bestLon = vehicle.location.global_relative_frame.lon
        print("dist: %.4f" % dist)
        time.sleep(0.1)

    return bestLon


def minimizeDistance(beacon, tolerance=0.5):
    bestLon = minimizeEW(beacon, tolerance)
    sendVelocityCmd(0, 0, 0)
    bestLat = minimizeNS(beacon, tolerance)
    sendVelocityCmd(0, 0, 0, 2)
    target = vehicle.location.global_relative_frame
    target.lon = bestLon
    target.lat = bestLat
    vehicle.simple_goto(target)
    while getDistanceMeters(vehicle.location.global_relative_frame, target) > tolerance:
        print("going to beacon")
        vehicle.simple_goto(target)
        time.sleep(1)
    return target


armAndTakeoff(2)
beacon = Beacon(getLocationMeters(vehicle.home_location, 20, 25))

startTime = time.time()
target = minimizeDistance(beacon, tolerance=0.5)
print("--- %s seconds ---" % (time.time() - startTime))
print("--- beacon %s meters off target ---" % beacon.getDistance(target))
vehicle.mode = VehicleMode("LAND")
vehicle.parameters["WPNAV_SPEED_DN"] = 10  # Set landing speed to 10 cm/s

while vehicle.location.global_relative_frame.alt > 0:
    print("Altitude: %s" % vehicle.location.global_relative_frame.alt)
    time.sleep(1)

print("--- landed %s meters off beacon ---" % beacon.getDistance(vehicle.location.global_relative_frame))

time.sleep(10)

vehicle.close()
# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()
