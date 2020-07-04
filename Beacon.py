from math import sqrt
from dronekit import LocationGlobalRelative


class Beacon:
    def __init__(self, location):
        self.location = location

    def getDistance(self, fromLoc):
        """
        Returns the ground distance in metres from beacon to fromLoc

        This method is an approximation, and will not be accurate over large distances and close to the 
        earth's poles. It comes from the ArduPilot test code:
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = self.location.lat - fromLoc.lat
        dlong = self.location.lon - fromLoc.lon
        return sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5
