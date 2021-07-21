#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.__pose = None
        self.__waypoints = None
        self.__waypoints_2d = None
        self.__waypoint_tree = None
        self.__camera_image = None
        self.__lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.__config = yaml.load(config_string)

        self.__upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.__bridge = CvBridge()
        self.__light_classifier = TLClassifier()
        self.__listener = tf.TransformListener()

        self.__state = TrafficLight.UNKNOWN
        self.__last_state = TrafficLight.UNKNOWN
        self.__last_wp = -1
        self.__state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.__pose = msg

    def waypoints_cb(self, waypoints):
        self.__waypoints = waypoints
        if not self.__waypoints_2d:
            self.__waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.__waypoint_tree = KDTree(self.__waypoints_2d)

    def traffic_cb(self, msg):
        # 0 = red, 1 = yellow, 2 = red
        self.__lights = msg.lights


    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.__has_image = True
        self.__camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.__state != state:
            self.__state_count = 0
            self.__state = state
        elif self.__state_count >= STATE_COUNT_THRESHOLD:
            self.__last_state = self.__state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.__last_wp = light_wp
            self.__upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.__upcoming_red_light_pub.publish(Int32(self.__last_wp))
        self.__state_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """

        idx = self.__waypoint_tree.query([x, y], 1)[1]
        return idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.__has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.__bridge.imgmsg_to_cv2(self.__camera_image, "bgr8")

        # Return ground trouth light state
        return light.state

        # ToDo: Get classification
        # return self.__light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
       

        # List of positions that correspond to the line to stop in front of for a given intersection
        cls_light = None
        stop_line_positions = self.__config['stop_line_positions']
        if(self.__pose and self.__waypoint_tree and self.__waypoints.waypoints):
            car_position_idx = self.get_closest_waypoint(self.__pose.pose.position.x, self.__pose.pose.position.y)
            # Find the closest visible traffic light (if one exists)
            diff = len(self.__waypoints.waypoints)
            for i, light in enumerate(self.__lights):
                line = stop_line_positions[i]
                wp_tmp_idx = self.get_closest_waypoint(line[0], line[1])
                diff_idx = wp_tmp_idx - car_position_idx
                if diff_idx >= 0 and diff_idx < diff:
                    diff = diff_idx
                    cls_light = light
                    line_wp_index = wp_tmp_idx


        if cls_light:
            state = self.get_light_state(cls_light)
            return line_wp_index, state
        
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
