#!/usr/bin/env python
# coding=utf-8

import rospy
import math
import time

from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from sonar_data_aggregator import SonarDataAggregator
from laser_data_aggregator import LaserDataAggregator
from navigation import Navigation


# Class for assigning the robot speeds
class RobotController:
    LINEAR_VELOCITY_MAX = 0.3  # in m/s
    ANGULAR_VELOCITY_MAX = 0.3  # in rad/s

    OBSTACLE_SEARCH_ANGLE_MIN = -60  # in degrees
    OBSTACLE_SEARCH_ANGLE_MAX = +60  # in degrees
    OBSTACLE_DISTANCE_IN_SIGHT = 1.0  # in m (when angular motors should start to activate)
    OBSTACLE_DISTANCE_MIN = 0.5  # in m

    # Constructor
    def __init__(self):

        # Debugging purposes
        self.print_velocities = rospy.get_param('print_velocities')

        # Where and when should you use this?
        self.stop_robot = False

        # Create the needed objects
        self.sonar_aggregation = SonarDataAggregator()
        self.laser_aggregation = LaserDataAggregator()
        self.navigation = Navigation()

        self.linear_velocity = 0
        self.angular_velocity = 0

        self.previous_turn_dir = None

        # Check if the robot moves with target or just wanders
        self.move_with_target = rospy.get_param("calculate_target")

        # The timer produces events for sending the speeds every 110 ms
        rospy.Timer(rospy.Duration(0.11), self.publishSpeeds)
        self.velocity_publisher = rospy.Publisher(
            rospy.get_param('speeds_pub_topic'), Twist,
            queue_size=10)

    # This function publishes the speeds and moves the robot
    def publishSpeeds(self, event):

        # Produce speeds
        self.produceSpeeds()

        # Create the commands message
        twist = Twist()
        twist.linear.x = self.linear_velocity
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.angular_velocity

        # Send the command
        self.velocity_publisher.publish(twist)

        # Print the speeds for debugging purposes
        if self.print_velocities == True:
            print "[L,R] = [" + str(twist.linear.x) + " , " + \
                  str(twist.angular.z) + "]"

    # def turnDirection(self, from_angle_deg, to_angle_deg):
    #     """
    #     Check laser scan measurements on the left vs on the right side of the robot and determine
    #     the type of the rotation
    #     :param from_angle_deg: left minimum angle in degrees
    #     :param to_angle_deg: right maximum angle in degrees
    #     :return: -1: right rotation | 1 : left rotation
    #     """
    #     laser_scan_left = self.laser_aggregation.getScanForAngleDegRange(from_angle_deg, 0)
    #     laser_scan_right = self.laser_aggregation.getScanForAngleDegRange(0, to_angle_deg)
    #     return -1 \
    #         if (sum(laser_scan_left) - sum(laser_scan_right)) > 2 \
    #         and max(laser_scan_left) > max(laser_scan_right) else \
    #         1

    # Produces speeds from the laser
    def produceSpeedsLaser(self):
        """
        :return: linear 0 to 1 and angular -1 to 1
        """

        # laser_scan = self.laser_aggregation.getScanForAngleDegRange(self.OBSTACLE_SEARCH_ANGLE_MIN,
        #                                                             self.OBSTACLE_SEARCH_ANGLE_MAX)
        # """laser_scan contains ~335 measurements from ~[-90, 90]° angles"""
        # laser_scan_min = min(laser_scan)

        ############################### NOTE QUESTION ############################
        # Check what laser_scan contains and create linear and angular speeds
        # for obstacle avoidance

        angular = self.laser_aggregation.getNextAngularVelocity()
        linear = 1 - abs(angular)

        ##########################################################################
        return [linear, angular]

    # Combines the speeds into one output using a motor schema approach
    def produceSpeeds(self):

        # Produce target if not existent
        if self.move_with_target == True and \
                self.navigation.target_exists == False:
            # Create the commands message
            twist = Twist()
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = 0

            # Send the command
            self.velocity_publisher.publish(twist)
            self.navigation.selectTarget()

        # Get the submodule's speeds
        [l_laser, a_laser] = self.produceSpeedsLaser()

        # You must fill these
        self.linear_velocity = 1
        self.angular_velocity = 0

        if self.move_with_target:
            [l_goal, a_goal] = self.navigation.velocitiesToNextSubtarget()
            ############################### NOTE QUESTION ############################
            # You must combine the two sets of speeds. You can use motor schema,
            # subsumption of whatever suits your better.

            if l_laser == 0:
                # Hand-break must be done now
                # ---> Subsumes Path Following <---
                self.linear_velocity = self.LINEAR_VELOCITY_MAX * l_laser
                self.angular_velocity = self.ANGULAR_VELOCITY_MAX * a_laser
            else:
                self.linear_velocity = self.LINEAR_VELOCITY_MAX * l_goal
                self.angular_velocity = self.ANGULAR_VELOCITY_MAX * a_goal

            ##########################################################################
        else:
            ############################### NOTE QUESTION ############################
            # Implement obstacle avoidance here using the laser speeds.
            # Hint: Subtract them from something constant

            self.linear_velocity = self.LINEAR_VELOCITY_MAX * l_laser
            self.angular_velocity = self.ANGULAR_VELOCITY_MAX * a_laser

            ##########################################################################

    # Assistive functions
    def stopRobot(self):
        self.stop_robot = True

    def resumeRobot(self):
        self.stop_robot = False
