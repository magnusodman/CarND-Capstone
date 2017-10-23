#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

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


class WaypointUpdater(object):
    def __init__(self):
        rospy.loginfo("Waypoint updater starting..")
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)


        # TODO: Add other member variables you need below
        self.pose = None
        self.waypoints = []

        rospy.loginfo("Waypoint updater started")
        self.publish_waypoints()
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg.pose

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        rospy.loginfo("Waypoints message received")
        
    def publish_waypoints(self):
        while self.pose == None or self.waypoints == None:
            pass
        rospy.loginfo("Pose and waypoints loaded")

        rate = rospy.Rate(1)
 
        while True:
            next_waypoints = self.get_next_waypoints()
            
            pub_lane_msg = Lane()
            pub_lane_msg.header.frame_id = '/world'
            pub_lane_msg.header.stamp = rospy.Time(0)
            pub_lane_msg.waypoints = next_waypoints
            self.final_waypoints_pub.publish(pub_lane_msg)
            rate.sleep()

    def get_next_waypoints(self):
        print("Current pose:")
        print("x: ", self.pose.position.x)
        print("y: ", self.pose.position.y)
        
        if len(self.waypoints) > 0:
            distances = [(i, self.dist(self.pose.position, w.pose.pose.position), w) for i,w in enumerate(self.waypoints)]
            distances.sort(key=lambda t: t[1])
            #TODO Handle wraparound
            start_index = distances[0][0]
            print("Closest waypoint:")
            print("index: ", start_index)
            print("x: ", self.waypoints[start_index].pose.pose.position.x)
            print("y: ", self.waypoints[start_index].pose.pose.position.y)
            return self.waypoints[start_index:start_index+LOOKAHEAD_WPS]
        return None
    
    def dist(self, p1, p2):
        return math.sqrt((p1.x-p2.x)**2 + (p1.y -p2.y)**2)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
