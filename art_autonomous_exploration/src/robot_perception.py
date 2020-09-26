#!/usr/bin/env python

import rospy
import tf
import numpy
import time
import math
import operator
import scipy.misc
from utilities import Print

from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


# Class implementing the robot perception: Reading the map, the coverage map
# and the robot pose
class RobotPerception:

    # Constructor
    def __init__(self):

        # Flags for debugging and synchronization
        self.print_robot_pose = False
        self.have_map = False
        self.map_token = False
        self.map_compute = False

        # Holds the occupancy grid map
        self.ogm = 0
        self.ros_ogm = 0
        self.ogm_copy = 0

        # Holds the ogm info for copying reasons -- do not change
        self.ogm_info = 0
        self.prev_ogm_info = 0

        # Holds the robot's total path
        self.robot_trajectory = []
        self.previous_trajectory_length = 0

        # Holds the coverage information. This has the same size as the ogm
        # If a cell has the value of 0 it is uncovered
        # In the opposite case the cell's value will be 100
        self.coverage = []

        # Holds the resolution of the occupancy grid map
        self.resolution = 0.05  # m/pixel

        # Origin is the translation between the (0,0) of the robot pose and the
        # (0,0) of the map
        self.origin = {'x': 0, 'y': 0}  # in meter units
        self.origin_px = [0, 0]  # in pixel units

        # Initialization of robot pose
        # x,y are in meters
        # x_px, y_px are in pixels
        self.robot_pose = {'x': 0, 'y': 0, 'th': 0, 'x_px': 0, 'y_px': 0}

        self.coverage_ogm = OccupancyGrid()
        self.coverage_ogm.header.frame_id = "map"

        ogm_topic = rospy.get_param('ogm_topic')
        robot_trajectory_topic = rospy.get_param('robot_trajectory_topic')
        coverage_pub_topic = rospy.get_param('coverage_pub_topic')
        self.map_frame = rospy.get_param('map_frame')
        self.base_footprint_frame = rospy.get_param('base_footprint_frame')

        # Use tf to read the robot pose
        self.listener = tf.TransformListener()

        # Read robot pose with a timer
        rospy.Timer(rospy.Duration(0.11), self.readRobotPose)

        # ROS Subscriber to the occupancy grid map
        ogm_topic = rospy.get_param('ogm_topic')
        rospy.Subscriber(ogm_topic, OccupancyGrid, self.readMap)

        # Publisher of the robot trajectory
        robot_trajectory_topic = rospy.get_param('robot_trajectory_topic')
        self.robot_trajectory_publisher = rospy.Publisher(robot_trajectory_topic,
                                                          Path, queue_size=10)

        # Publisher of the coverage field
        coverage_pub_topic = rospy.get_param('coverage_pub_topic')
        self.coverage_publisher = rospy.Publisher(coverage_pub_topic,
                                                  OccupancyGrid, queue_size=10)

        # Read Cell size
        self.cell_size = rospy.get_param('cell_size')
        self.cell_matrix = numpy.zeros((1, 1))
        self.current_cell = []

        Print.art_print("Robot perception initialized", Print.GREEN)

    # Getter for OGM. Must use flags since its update is asynchronous
    def getMap(self):
        print "Robot perception: Map requested"
        # The map is being processed ... waiting
        while self.map_compute:
            pass

        # Locking the map
        self.map_token = True
        # Copying it
        cp = numpy.copy(self.ogm)
        # Unlocking it
        self.map_token = False

        # Return the copy
        return cp

    def getRosMap(self):
        return self.ros_ogm

    # Getter for Coverage
    def getCoverage(self):
        return numpy.copy(self.coverage)

    # Reading the robot pose
    def readRobotPose(self, event):
        try:
            # Reads the robot pose from tf
            (translation, rotation) = self.listener.lookupTransform(
                self.map_frame, self.base_footprint_frame, rospy.Time(0)
            )
        # Catch the exception if something is wrong
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            # Just print the error to try again
            print "Error in tf..."
            return

        # Updating the robot pose
        self.robot_pose['x'] = translation[0]
        self.robot_pose['y'] = translation[1]
        rp_px = self.toPixelUnits(self.robot_pose)
        self.robot_pose['x_px'] = rp_px[0]
        self.robot_pose['y_px'] = rp_px[1]

        # Getting the Euler angles
        angles = tf.transformations.euler_from_quaternion(rotation)
        self.robot_pose['th'] = angles[2]  # yaw (around z axis): starting from horizontal and moving to the left until
        # (global) x-axis meets robot's direction

        # For debugging purposes
        if self.print_robot_pose == True:
            print self.robot_pose

        if [self.robot_pose['x'], self.robot_pose['y']] not in self.robot_trajectory:
            self.robot_trajectory.append([self.robot_pose['x'], self.robot_pose['y']])

        t_path = Path()
        t_path.header.frame_id = "map"
        for p in self.robot_trajectory:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.pose.position.x = p[0]
            ps.pose.position.y = p[1]
            t_path.poses.append(ps)
        self.robot_trajectory_publisher.publish(t_path)

    # Getting the occupancy grid map
    def readMap(self, data):

        # OGM is a 2D array of size width x height
        # The values are from 0 to 100
        # 0 is an unoccupied pixel
        # 100 is an occupied pixel
        # 50 or -1 is the unknown

        # Locking the map
        self.map_compute = True

        self.ros_ogm = data
        # Reading the map pixels
        self.ogm_info = data.info

        if self.have_map == False or \
                self.ogm_info.width != self.prev_ogm_info.width or \
                self.ogm_info.height != self.prev_ogm_info.height:

            self.ogm = numpy.zeros((data.info.width, data.info.height),
                                   dtype=numpy.int)
            Print.art_print("Map & coverage expansion!", Print.GREEN)
            self.prev_ogm_info = self.ogm_info

            # Update coverage container as well
            coverage_copy = numpy.zeros([self.ogm_info.width, self.ogm_info.height])
            print "Coverage copy new size: " + str(coverage_copy.shape)
            self.coverage_ogm.info = self.ogm_info
            self.coverage_ogm.data = \
                numpy.zeros(self.ogm_info.width * self.ogm_info.height)

            if self.have_map == True:
                print "Copying coverage field" + str(self.coverage.shape)
                for i in range(0, self.coverage.shape[0]):
                    for j in range(0, self.coverage.shape[1]):
                        coverage_copy[i][j] = self.coverage[i][j]

            # Coverage now gets the new size
            self.coverage = numpy.zeros([self.ogm_info.width, self.ogm_info.height])
            for i in range(0, self.coverage.shape[0]):
                for j in range(0, self.coverage.shape[1]):
                    self.coverage[i][j] = coverage_copy[i][j]
            print "New coverage info: " + str(self.coverage.shape)

            for i in range(0, self.coverage.shape[0]):
                for j in range(0, self.coverage.shape[1]):
                    index = int(i + self.ogm_info.width * j)
                    self.coverage_ogm.data[index] = self.coverage[i][j]
                    # TODO: That's not quite right - the origins must be checked

        for x in range(0, data.info.width):
            for y in range(0, data.info.height):
                self.ogm[x][y] = data.data[x + data.info.width * y]

        # Get the map's resolution - each pixel's side in meters (m/pixel)
        self.resolution = data.info.resolution

        # Get the map's origin
        self.origin['x'] = data.info.origin.position.x
        self.origin['y'] = data.info.origin.position.y
        self.origin_px = self.toPixelUnits(self.origin)

        # Keep a copy
        self.ogm_copy = numpy.copy(self.ogm)

        # Update coverage
        x = self.robot_pose['x_px']
        y = self.robot_pose['y_px']
        xx = self.robot_pose['x_px'] + abs(self.origin['x'] / self.resolution)
        yy = self.robot_pose['y_px'] + abs(self.origin['y'] / self.resolution)
        for i in range(-20, 20):
            for j in range(-20, 20):
                try:
                    if self.ogm[xx + i, yy + j] > 49 or self.ogm[xx + i, yy + j] == -1:
                        continue
                except:
                    continue
                self.coverage[xx + i, yy + j] = 100
                index = int((xx + i) + self.ogm_info.width * (yy + j))
                self.coverage_ogm.data[index] = 100
        self.coverage_publisher.publish(self.coverage_ogm)

        # NOTE: uncomment this to save coverage in image
        # scipy.misc.imsave('~/Desktop/test.png', self.coverage)

        # Unlock the map
        self.map_compute = False

        # If it is copied wait ...
        while self.map_token == True:
            pass

        # This is for the navigation
        if self.have_map == False:
            self.have_map = True
            Print.art_print("Robot perception: Map initialized", Print.GREEN)

    def getGlobalCoordinates(self, p, with_resolution=True):
        return self.toGlobalCoordinates(p, with_resolution)

    # ------------------------------------------------------------------------------ #

    def toPixelUnits(self, v_m, xy_labels=True):
        """
        Convert vector from meter-units to pixel-units based on resolution (m/pixel)
        :param v_m: vector in meters
        :param xy_labels: if True will use v['x'], v['y'] instead of v[0], v[1]
        :return: v_p
        """
        return [
            int((v_m['x'] if xy_labels else v_m[0]) / self.resolution),
            int((v_m['y'] if xy_labels else v_m[1]) / self.resolution)
        ]

    def toMeterUnits(self, v_p, xy_labels=True):
        """
        Convert vector from pixel-units to meter-units based on resolution (m/pixel)
        :param v_p: vector in pixel
        :param xy_labels: if True will use v['x'], v['y'] instead of v[0], v[1]
        :return: v_m
        """
        return [
            int((v_p['x'] if xy_labels else v_p[0]) * self.resolution),
            int((v_p['y'] if xy_labels else v_p[1]) * self.resolution)
        ]

    def toGlobalCoordinates(self, p, with_resolution=True):
        """
        Transform relative coordinates (where origin is image origin) to global coordinates (with map's origin)
        :param p: point in relative coordinate system
        :param with_resolution: if True converts meters to pixes, else assumes resolution = 1
        :return:
        """
        return map(operator.sub, p, self.origin_px) if with_resolution else \
            map(operator.sub, p, [self.origin['x'], self.origin['y']])

    def toRelativeCoordinates(self, p, with_resolution=True):
        """
        Transform global coordinates (with map's origin) to relative coordinates (based on images origin at (0,0))
        :param p: point in global (map's) coordinate system
        :param with_resolution: if True converts meters to pixes, else assumes resolution = 1
        :return:
        """
        return map(operator.add, p, self.origin_px) if with_resolution else \
            map(operator.add, p, [self.origin['x'], self.origin['y']])
