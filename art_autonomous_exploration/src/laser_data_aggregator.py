#!/usr/bin/env python
# coding=utf-8

import rospy
from sensor_msgs.msg import LaserScan
from math import radians, degrees, floor
from threading import Thread, Lock

PROCESS_IN_PARALLEL = True


# Class for reading the data from the laser sensor
class LaserDataAggregator:
    STATE_OAH = -1
    STATE_OA = 0
    STATE_CLOSE = 1
    STATE_MEDIUM = 2
    STATE_FAR = 3

    DISTANCE_OA = 1
    DISTANCE_OAH = 0.5 * DISTANCE_OA
    DISTANCE_CLOSE = DISTANCE_OA + 1
    DISTANCE_MEDIUM = DISTANCE_OA + 2
    # DISTANCE_FAR = inf

    LASER_SCAN_MIN = 1e6
    LASER_SCAN_REGION_WITH_MIN = None

    LASER_SCAN_LEFT_SIDE_MIN = LASER_SCAN_MIN
    LASER_SCAN_RIGHT_SIDE_MIN = LASER_SCAN_MIN
    LASER_SCAN_LEFT_SIDE_SUM = 0
    LASER_SCAN_RIGHT_SIDE_SUM = 0

    LASER_SCAN_PRECISE_DEGREES = 15
    LASER_SCAN_REGIONS = {
        # Front-Precise Regions
        'FPL': {'bounds': (-LASER_SCAN_PRECISE_DEGREES, 0), 'state': STATE_FAR, 'min': LASER_SCAN_MIN, 'sum': 0},
        'FPR': {'bounds': (0, LASER_SCAN_PRECISE_DEGREES), 'state': STATE_FAR, 'min': LASER_SCAN_MIN, 'sum': 0},
        # Side-Precise Regions
        'SPL': {'bounds': (-90 - LASER_SCAN_PRECISE_DEGREES, -90 + LASER_SCAN_PRECISE_DEGREES), 'state': STATE_FAR,
                'min': LASER_SCAN_MIN, 'sum': 0},
        'SPR': {'bounds': (90 - LASER_SCAN_PRECISE_DEGREES, 90 + LASER_SCAN_PRECISE_DEGREES), 'state': STATE_FAR,
                'min': LASER_SCAN_MIN, 'sum': 0},
        # Front-Coarse Regions
        'FCL': {'bounds': (LASER_SCAN_PRECISE_DEGREES, 90 - LASER_SCAN_PRECISE_DEGREES), 'state': STATE_FAR,
                'min': LASER_SCAN_MIN, 'sum': 0},
        'FCR': {'bounds': (-90 + LASER_SCAN_PRECISE_DEGREES, -LASER_SCAN_PRECISE_DEGREES), 'state': STATE_FAR,
                'min': LASER_SCAN_MIN, 'sum': 0},
        # Back Regions
        'BL': {'bounds': (-120, -90 - LASER_SCAN_PRECISE_DEGREES), 'state': STATE_FAR, 'min': LASER_SCAN_MIN, 'sum': 0},
        'BR': {'bounds': (90 + LASER_SCAN_PRECISE_DEGREES, 120), 'state': STATE_FAR, 'min': LASER_SCAN_MIN, 'sum': 0},
    }

    LAST_TURN_DIRECTION = None
    THREAD_MUTEX = Lock()

    # Constructor
    def __init__(self):

        # Initialization of laser scan 
        self.laser_scan = []

        # Initialization of laser params
        self.angle_range = [0, 0]  # in rads
        self.angle_step = 0  # in rads
        self.laser_scan_len = 0  # number of measurements in each laser scan

        # ROS Subscribers to the robot's laser
        laser_topic = rospy.get_param("laser_topic")
        rospy.Subscriber(laser_topic, LaserScan, self.getDataLaser)

        # Getting data from the laser

    def getDataLaser(self, data):
        self.angle_range = [data.angle_min, data.angle_max]
        self.angle_step = data.angle_increment

        # Get the measurements
        self.laser_scan = list(data.ranges)
        self.laser_scan_len = len(self.laser_scan)

        # Pay attention for special values
        for i in range(0, self.laser_scan_len):
            if self.laser_scan[i] > data.range_max:
                self.laser_scan[i] = data.range_max
            elif self.laser_scan[i] < data.range_min:
                self.laser_scan[i] = data.range_min

        # Reset Local Data
        self.LASER_SCAN_LEFT_SIDE_MIN = self.LASER_SCAN_MIN
        self.LASER_SCAN_RIGHT_SIDE_MIN = self.LASER_SCAN_MIN
        self.LASER_SCAN_LEFT_SIDE_SUM = 0
        self.LASER_SCAN_RIGHT_SIDE_SUM = 0

        # Process laser data
        #  - assign/update region states
        if PROCESS_IN_PARALLEL:
            # Create 1 thread/region to find min value from laser scan
            set_laser_scan_region_min_threads = [
                Thread(target=self.setLaserScanRegionData, args=(region_label, region_data['bounds']))
                for region_label, region_data in self.LASER_SCAN_REGIONS.iteritems()]
            # Execute threads
            for t in set_laser_scan_region_min_threads:
                t.start()
            for t in set_laser_scan_region_min_threads:
                t.join()
        else:
            for region_label, region_data in self.LASER_SCAN_REGIONS.iteritems():
                self.setLaserScanRegionData(region_label, region_data['bounds'])

    def getScanForAngleRadRange(self, from_angle_rad, to_angle_rad):
        """
        Returns subarray of self.laser_scan containing only measurements in given angle range
        :param from_angle_rad: starting angle of angle range in rads
        :param to_angle_rad: ending angle of angle range in rads
        :return: sublist of self.laser_scan
        """
        laser_scan_start = (from_angle_rad - self.angle_range[0]) / self.angle_step
        laser_scan_start = int(laser_scan_start)

        laser_scan_end = (self.angle_range[1] - to_angle_rad) / self.angle_step + 1
        laser_scan_end = int(laser_scan_end)
        laser_scan_end = self.laser_scan_len - laser_scan_end

        return self.laser_scan[laser_scan_start:laser_scan_end + 1]

    def getScanForAngleDegRange(self, from_angle_deg, to_angle_deg):
        """
        Returns subarray of self.laser_scan containing only measurements in given angle range
        :param from_angle_deg: starting angle of angle range in degrees
        :param to_angle_deg: ending angle of angle range in degrees
        :return: sublist of self.laser_scan
        """
        return self.getScanForAngleRadRange(radians(from_angle_deg), radians(to_angle_deg))

    def setLaserScanRegionData(self, region_label, region_bounds_deg):
        """
        Set the minimum of laser scans in $region_label laser scan region.
        :param str region_label: region's label (key)
        :param tuple region_bounds_deg: region's bounds in degree (key)
        :return: void
        """
        if region_bounds_deg is None:
            return

        laser_scan_region = self.getScanForAngleDegRange(region_bounds_deg[0], region_bounds_deg[1])
        laser_scan_region_min = min(laser_scan_region)
        laser_scan_region_sum = min(laser_scan_region)
        self.LASER_SCAN_REGIONS[region_label]['min'] = laser_scan_region_min
        self.LASER_SCAN_REGIONS[region_label]['sum'] = laser_scan_region_sum

        self.THREAD_MUTEX.acquire()
        # -----------------------------------------------
        # Update Global Min
        if laser_scan_region_min < self.LASER_SCAN_MIN:
            self.LASER_SCAN_MIN = laser_scan_region_min
            self.LASER_SCAN_REGION_WITH_MIN = region_label
        # Update Side's Min & Sum
        if region_label[-1:] == 'L':
            if laser_scan_region_min < self.LASER_SCAN_LEFT_SIDE_MIN:
                self.LASER_SCAN_LEFT_SIDE_MIN = laser_scan_region_min
            self.LASER_SCAN_LEFT_SIDE_SUM += laser_scan_region_sum
        else:
            if laser_scan_region_min < self.LASER_SCAN_RIGHT_SIDE_MIN:
                self.LASER_SCAN_RIGHT_SIDE_MIN = laser_scan_region_min
            self.LASER_SCAN_RIGHT_SIDE_SUM += laser_scan_region_sum
        # -----------------------------------------------
        self.THREAD_MUTEX.release()

    def getLaserScanRegionMin(self, region_label):
        """
        Get the minimum of laser scans in $region_label laser scan region.
        :param region_label: region's label (key)
        :return: int >= 0 | -1 if min not initialized
        """
        return self.LASER_SCAN_REGIONS[region_label]['min']

    def getTurnDirection(self):
        """
        Determine the direction that the robot should steer to avoid possible obstacles
        :return: -1 for left turn | 1 for right
        """
        self.LAST_TURN_DIRECTION = -1 if self.LASER_SCAN_LEFT_SIDE_SUM > self.LASER_SCAN_RIGHT_SIDE_SUM else 1
        return self.LAST_TURN_DIRECTION

    def getTurnForce(self):
        """
        Determine the force with which the robot should steer to avoid possible obstacles
        :return: number in range [0: no turn, 1: hand-break turn]
        """
        # if self.LASER_SCAN_MIN < self.DISTANCE_OA and self.LASER_SCAN_REGION_WITH_MIN in ['FPL', 'FPR']:
        #     return 1
        #
        # if self.LASER_SCAN_MIN < self.DISTANCE_OA and self.LASER_SCAN_REGION_WITH_MIN in ['FCL', 'FCR']:
        #     return 0.5
        #
        # if self.LASER_SCAN_MIN < self.DISTANCE_OAH and self.LASER_SCAN_REGION_WITH_MIN in ['SPL', 'SPR']:
        #     return 0.2

        if (self.LASER_SCAN_REGIONS['FPL']['min'] < self.DISTANCE_OA) or \
                (self.LASER_SCAN_REGIONS['FPR']['min'] < self.DISTANCE_OA):
            return 1

        # if (self.LASER_SCAN_REGIONS['FCL']['min'] < self.DISTANCE_OA) or \
        #         (self.LASER_SCAN_REGIONS['FCR']['min'] < self.DISTANCE_OA):
        #     return 0.2
        #
        # if (self.LASER_SCAN_REGIONS['SPL']['min'] < self.DISTANCE_OA) or \
        #         (self.LASER_SCAN_REGIONS['SPR']['min'] < self.DISTANCE_OA):
        #     return 0.1

        self.LAST_TURN_DIRECTION = None
        return 0

    def getNextAngularVelocity(self):
        """

        :return: -1 if True for Left region ('**L'), 1 if True for Right ('**R'), 0 if should not turn at all
        """
        if self.LASER_SCAN_REGION_WITH_MIN is None:
            return 0

        turn_dir = self.getTurnDirection() if self.LAST_TURN_DIRECTION is None else self.LAST_TURN_DIRECTION
        turn_force = self.getTurnForce()
        return turn_dir * turn_force
