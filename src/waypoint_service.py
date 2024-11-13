#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from waypoint_navigation.srv import GetWaypoints


class WayPoints(Node):

    def __init__(self):
        super().__init__('waypoints_service')
        self.srv = self.create_service(GetWaypoints, 'waypoints', self.waypoint_callback)
        self.waypoints = [[2.0, 2.0, 27.0], [2.0, -2.0, 27.0], [-2.0, -2.0, 27.0], [-2.0, 2.0, 27.0], [1.0, 1.0, 27.0]]

    def waypoint_callback(self, request, response):
        # Check if request asks to get waypoints
        if request.get_waypoints:
            # Initialize PoseArray for response.waypoints
            pose_array = PoseArray()
            
            # Populate each Pose in PoseArray
            for wp in self.waypoints:
                pose = Pose()
                pose.position.x = wp[0]
                pose.position.y = wp[1]
                pose.position.z = wp[2]
                pose_array.poses.append(pose)
            
            # Assign pose_array to the response's waypoints field
            response.waypoints = pose_array
            self.get_logger().info("Incoming request for Waypoints")
        else:
            self.get_logger().info("Request rejected")

        # Return the populated response
        return response


def main():
    rclpy.init()
    waypoints = WayPoints()

    try:
        rclpy.spin(waypoints)
    except KeyboardInterrupt:
        waypoints.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoints.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
