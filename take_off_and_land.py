#!/usr/bin/env python
# -*- coding: utf-8 -*-
from dronekit import connect, VehicleMode
import time

# Connect to SITL vehicle
# from dronekit_sitl import SITL
# sitl = SITL()
# sitl.download('copter', '3.3', verbose=True)
# sitl_args = ['-I0', '--model', 'quad', '--home=49.26778354815404,-123.17927956581117,0,90']
# sitl.launch(sitl_args, await_ready=True, restart=True)
# connect_string = 'tcp:127.0.0.1:5760'  # Local TCP endpoint for SITL runs. Can connect MP to 5763
# vehicle = connect(connect_string, wait_ready=True)

# # Connect to Physical vehicle
connect_string = '/dev/ttyS0'  # Serial device endpoint for connecting to physical drone
vehicle = connect(connect_string, wait_ready=True, baud=57600)
print("Connected to vehicle on %s" % connect_string)

# Getting state information stored in Vehicle class atributes
print("Global Location: %s" % vehicle.location.global_frame)
print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print("Local Location: %s" % vehicle.location.local_frame)    # NED

# Need to download vehicle commands before getting home locaiton
# Get Vehicle Home location - will be `None` until first set by autopilot
while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
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

vehicle.parameters["WPNAV_SPEED_DN"] = 10  # Set landing speed to 10 cm/s
vehicle.parameters["WPNAV_SPEED_UP"] = 10  # Set takeoff speed to 10 cm/s


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

    while not (vehicle.armed and vehicle.mode == VehicleMode("GUIDED")):
        vehicle.mode = VehicleMode("GUIDED")
        vehicle.armed = True
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    # (otherwise the command after Vehicle.simple_takeoff will execute immediately).
    alt = vehicle.location.global_relative_frame.alt
    while alt <= target * 0.95:
        print("Altitude: %s" % alt)
        alt = vehicle.location.global_relative_frame.alt
        time.sleep(0.25)
    print("Reached target altitude")


arm_and_takeoff(1)

vehicle.mode = VehicleMode("LOITER")
while not vehicle.mode.name == "LOITER":
    print("Waiting for hovering...")
    vehicle.mode = VehicleMode("LOITER")
    time.sleep(0.25)

# Hover for 2 seconds
print("Hovering!")
for i in range(8):
    print("Altitude: ", vehicle.location.global_relative_frame.alt)
    time.sleep(0.25)

# Land
vehicle.mode = VehicleMode("LAND")
while not vehicle.mode.name == "LAND":
    print("Switching to Landing Mode...")
    vehicle.mode = VehicleMode("LAND")
    time.sleep(0.25)
print("Landing!")

zeroCount = 0
alt = vehicle.location.global_relative_frame.alt
# Barometric readings can vary, to ensure a landing has actually occured wait for the altitude reading to be ground-level for a second
while alt >= 0 or zeroCount < 4:
    print("Altitude: %s" % alt)
    alt = vehicle.location.global_relative_frame.alt
    time.sleep(0.25)
    if alt <= 0:
        zeroCount += 1
    else:
        zeroCount = 0

print("LANDED!")

while vehicle.armed:
    vehicle.armed = False
    print("Disarming...")
    time.sleep(0.25)

# Close vehicle
vehicle.close()
