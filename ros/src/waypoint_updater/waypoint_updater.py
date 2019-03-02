#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import Lane, Waypoint
from styx_msgs.msg import TrafficLightArray, TrafficLight
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

import math
from copy import deepcopy

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
# Profile for slowing at traffic light: v = K_SLOW * sqrt(dist - DIST_MIN)
# This is equivalent to considering a constant deceleration
K_SLOW = 5     # in m^1/2 . s^-1
DIST_MIN = 6   # distance we need to be from stop line
VEHICLE_LENGTH = 4


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_lights_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.speed_limit = rospy.get_param('/waypoint_loader/velocity') * 1000 / 3600.

        # TODO: Add other member variables you need below
        self.light_wp = None
        self.lights = None
        self.current_pose = None
        self.base_waypoints = None

        self.planner = PathPlanner(LOOKAHEAD_WPS)
        self.planner.set_speed_limit(self.speed_limit)

        # rospy.spin()

    def publish(self, waypoints):
        if waypoints == []:
            return

        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.get_rostime()
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)

    def run(self):
        rospy.loginfo('### Run')
        rate = rospy.Rate(5) # 10hz
        while not rospy.is_shutdown():
            # rospy.loginfo('### looping ... ')
            waypoints = self.planner.generate_waypoints()
            self.publish(waypoints)
            rate.sleep()

    def pose_cb(self, msg):
        # msg - geometry_msgs/PoseStamped
        # rospy.loginfo('### pose Received')
        self.current_pose = msg
        self.planner.update_vehicle_location(self.current_pose)

    def waypoints_cb(self, lane):
        # waypoints - styx_msgs/Lane, only received once
        rospy.loginfo('### BASE Waypoint Received. speed_limit: %f', self.speed_limit)
        self.base_waypoints = lane.waypoints
        self.planner.set_base_waypoints(lane.waypoints)

    def traffic_cb(self, msg):
        # msg - std_msgs/Int32
        self.light_wp = msg.data
        self.planner.update_tl_wp(self.light_wp)
        rospy.loginfo('### TrafficLightIndex Received: %i', msg.data)

    def traffic_lights_cb(self, msg):
        # msg - styx_msgs/TrafficLightArray
        self.lights = msg.lights
        # self.planner.update_traffic_lights(msg.lights)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

def get_plane_distance(target1, target2):
    delta_x = target1.pose.position.x - target2.pose.position.x
    delta_y = target1.pose.position.y - target2.pose.position.y
    return math.sqrt(delta_x * delta_x + delta_y * delta_y)

def distance_sq_between_waypoints(waypoints, wp_idx1, wp_idx2):
    dl = lambda a, b: (a.x - b.x) ** 2 + (a.y-b.y)**2  + (a.z-b.z)**2
    wp1 = waypoints[wp_idx1]
    wp2 = waypoints[wp_idx2]
    return dl(wp1.pose.pose.position, wp2.pose.pose.position)

class PathPlanner(object):
    def __init__(self, lookahead_wps):
        self.current_pose = None
        self.base_waypoints = None
        self.lookahead_wps = lookahead_wps
        # vehicle initial and next index into base_waypoints
        self.init_index = None
        self.next_index = None
        self.red_light_wp_idx = -1 # -1 is representing traffic light is not red
        self.speed_limit = 1
        self.lights = None
        self.lights_wps = []
        self.waypoints = []
        self.idx_at_stop = None

    def set_base_waypoints(self, waypoints):
        self.base_waypoints = waypoints

    def set_speed_limit(self, speed):
        self.speed_limit = speed

    def update_traffic_lights(self, lights):
        self.lights = lights
        if len(self.lights_wps) == 0:
            for i in range(len(lights)):
                idx = self.find_closest_waypoint_index(0, lights[i].pose)
                if idx:
                    self.lights_wps.append(idx)
            rospy.loginfo('### Found TrafficLightWPS: %d', len(self.lights_wps))

        self.red_light_wp_idx = -1
        if len(self.lights_wps) > 0:
            for i in reversed(range(len(self.lights_wps))):
                light = lights[i]
                if self.lights_wps[i] > self.next_index and light.state == 0:
                    self.red_light_wp_idx = self.lights_wps[i]

    def update_vehicle_location(self, curr_pose):
        self.current_pose = curr_pose
        if self.init_index is None:
            self.init_index = self.find_closest_waypoint_index(0, curr_pose)

    def update_tl_wp(self, wp_idx):
        self.red_light_wp_idx = wp_idx

    def find_closest_waypoint_index(self, start_index, curr_pose):
        if self.base_waypoints is None:
            return None

        search_dist = 5.0
        max_distance = float('inf') if start_index == 0 else search_dist
        min_dist = float('inf')
        min_index = -1
        for i in range(start_index, len(self.base_waypoints)):
            d = get_plane_distance(self.base_waypoints[i].pose, curr_pose)
            if d > max_distance:
                break;
            if d < min_dist:
                min_dist = d
                min_index = i

        return min_index

    def find_idx_at_stop(self, start_index, end_index, distance):
        search_delta = 5.0
        min_index = start_index
        wp = self.base_waypoints[start_index]
        min_delta = float('inf')
        for i in range(start_index, end_index):
            d = get_plane_distance(self.base_waypoints[i].pose, wp.pose)
            if math.fabs(distance - d) < min_delta:
                min_delta = math.fabs(distance - d)
                min_index = i

        return min_index

    def generate_waypoints(self):
        waypoints = []
        if self.current_pose is None or self.base_waypoints is None:
            return waypoints

        self.next_index = self.init_index
        if self.next_index is None:
            self.next_index = 0

        self.next_index = self.find_closest_waypoint_index(self.next_index, self.current_pose)

        cur_x = self.current_pose.pose.position.x
        cur_y = self.current_pose.pose.position.y
        next_x = self.base_waypoints[self.next_index].pose.pose.position.x
        #rospy.loginfo('#__ cur_x: %f, index: %d, next_x: %f', cur_x, self.next_index, next_x)

        end_wp_idx = self.next_index + self.lookahead_wps
        end_wp_idx = end_wp_idx if end_wp_idx<len(self.base_waypoints) else len(self.base_waypoints)-self.next_index

        #TODO: how to wrap around instead of stop at the last waypoint
        if self.next_index < len(self.base_waypoints):
            waypoints = deepcopy(self.base_waypoints[self.next_index: end_wp_idx])
        else:
            # Last waypoint
            last_waypoint = deepcopy(self.base_waypoints[-1])
            waypoints.append(last_waypoint)
            self.set_waypoint_velocity(waypoints, 0, 0.0)

        self.handle_vehicle_stop(waypoints)

        self.init_index = self.next_index
        # rospy.loginfo('### generated # of waypoints: %d', len(waypoints))
        return waypoints

    def handle_vehicle_stop(self, waypoints):
        if self.red_light_wp_idx > 0:
            distance_to_light = math.sqrt(distance_sq_between_waypoints(self.base_waypoints, self.next_index, self.red_light_wp_idx))
            waypoints_span = self.distance(waypoints, 0, len(waypoints)-1)

            if self.idx_at_stop is None:
                latency = 0.1 * self.get_waypoint_velocity(waypoints[0])
                distance_to_stop = max(distance_to_light - DIST_MIN - VEHICLE_LENGTH - latency, 0)
                self.idx_at_stop = self.find_idx_at_stop(self.next_index, self.red_light_wp_idx, distance_to_stop)
            idx_delta = self.idx_at_stop - self.next_index

            #rospy.loginfo('Cur Pos: %d, RedLight at: %d, Stop at: %d', self.next_index, self.red_light_wp_idx, self.idx_at_stop)
            #rospy.loginfo('Distance to RedLight: %f', distance_to_light)
            if idx_delta > 0 and idx_delta < len(waypoints):
                for i in reversed(range(len(waypoints))):
                    if i < idx_delta:
                        dist = get_plane_distance(waypoints[i].pose, waypoints[idx_delta-1].pose)
                        vel = math.sqrt(2.0 * K_SLOW * dist)
                        if vel < 1.0:
                            vel = 0.0
                        v = self.get_waypoint_velocity(waypoints[i])
                        v = min(vel, v)
                        self.set_waypoint_velocity(waypoints, i, v)
                    else:
                        self.set_waypoint_velocity(waypoints, i, 0.0)
                    # rospy.loginfo('RedLight: speed %f', self.get_waypoint_velocity(waypoints[i]))
            elif idx_delta > len(waypoints):
                pass
            elif idx_delta <= 0:
                for i in range(len(waypoints)):
                    self.set_waypoint_velocity(waypoints, i, 0.0)
        else:
            self.idx_at_stop = None

    def update_speed(self, waypoints):
        # adopted from team member Arunana's updater (not working yet)
        if self.red_light_wp_idx > 0 and self.red_light_wp_idx > self.next_index+300:
            # find index of waypoint corresponding to traffic light in final_wps
            red_idx_final_wps = (self.red_light_wp_idx - self.next_index) % len(self.base_waypoints)

            # get position of traffic light
            traffic_light_waypoint = self.base_waypoints[self.red_light_wp_idx]

            # if it is far, we start reducing the speed slowly until our horizon
            if red_idx_final_wps >= self.lookahead_wps:
                red_idx_final_wps = self.lookahead_wps - 1

            # Reduce velocity based on distance to light
            for idx in range(red_idx_final_wps + 1):
                distance_to_light = math.sqrt(distance_sq_between_waypoints(waypoints,idx,self.red_light_wp_idx))
                # use a profile based on constant deceleration
                distance_to_stop = max(distance_to_light - DIST_MIN, 0)
                waypoints[idx].twist.twist.linear.x = min(K_SLOW * math.sqrt(distance_to_stop) * 1000 / 3600,
                                                                            waypoints[idx].twist.twist.linear.x)

            # Set all future points to zero speed
            for idx in range(red_idx_final_wps + 1, len(waypoints) - 1):
                waypoints[idx].twist.twist.linear.x = 0

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
        WaypointUpdater().run()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
