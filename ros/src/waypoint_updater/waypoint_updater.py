#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32

import math
import numpy as np


'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECELERATION = 0.8 # m/s^2


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.__waypoints_2d = None
        self.__pose = None
        self.__base_waypoints = None
        self.__waypoint_tree = None
        # -1 -> next traffic light is green otherwise this will be the index of the stop line 
        self.__next_traffic_light_idx = -1 
        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.__pose and self.__base_waypoints and self.__waypoint_tree:
            # Closest waypoint
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.__pose.pose.position.x
        y = self.__pose.pose.position.y
        closest_idx = self.__waypoint_tree.query([x,y],1)[1]

        closest_coord = self.__waypoints_2d[closest_idx]
        prev_coord = self.__waypoints_2d[closest_idx - 1]

        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x,y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.__waypoints_2d)
        return closest_idx

    def publish_waypoints(self, closest_idx):
        lane = Lane()
        end_idx = closest_idx + LOOKAHEAD_WPS

        lane.header = self.__base_waypoints.header
        if self.__next_traffic_light_idx == -1 or (self.__next_traffic_light_idx >= end_idx):
            # Green light
            lane.waypoints = self.__base_waypoints.waypoints[closest_idx:end_idx]
            self.final_waypoints_pub.publish(lane)
        elif closest_idx < self.__next_traffic_light_idx:
            # Read light
            tmp_waypoints = self.__base_waypoints.waypoints[closest_idx:self.__next_traffic_light_idx]

            tmp_waypoint = tmp_waypoints[0]
            velocity_setpoint = tmp_waypoint.twist.twist.linear.x
        
            dist_to_traffic_light = self.distance(tmp_waypoints, 0, len(tmp_waypoints) - 1)

            for i, waypoint in enumerate(tmp_waypoints):

                p = Waypoint()
                p.pose = waypoint.pose

                if dist_to_traffic_light < 5:
                    vel = 0        
                else:
                    dist_tmp = self.distance(tmp_waypoints, i, len(tmp_waypoints) - 5)
                    vel = math.sqrt(2 * MAX_DECELERATION * dist_tmp)
                    if vel < 3:
                        vel = 0

                p.twist.twist.linear.x = min(vel, waypoint.twist.twist.linear.x)
                lane.waypoints.append(p)

            self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        self.__pose = msg

    def waypoints_cb(self, waypoints):
        self.__base_waypoints = waypoints
        if not self.__waypoints_2d:
            self.__waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.__waypoint_tree = KDTree(self.__waypoints_2d)

    def traffic_cb(self, msg):
        self.__next_traffic_light_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
