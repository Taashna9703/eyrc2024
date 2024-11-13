#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Import the action and service
from waypoint_navigation.action import NavToWaypoint
from waypoint_navigation.srv import GetWaypoints


class WayPointClient(Node):

    def __init__(self):
        super().__init__('waypoint_client')
        self.goals = []
        self.goal_index = 0

        # Create an action client for 'NavToWaypoint'
        self.action_client = ActionClient(self, NavToWaypoint, 'waypoint_navigation')

        # Create a client for the 'GetWaypoints' service
        self.cli = self.create_client(GetWaypoints, 'waypoints')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create a request object for GetWaypoints service
        self.req = GetWaypoints.Request()
    
    ### Action client functions

    def send_goal(self, waypoint):
        # Create a NavToWaypoint goal object.
        goal_msg = NavToWaypoint.Goal()

        # Set the waypoint position
        goal_msg.waypoint.position.x = waypoint[0]
        goal_msg.waypoint.position.y = waypoint[1]
        goal_msg.waypoint.position.z = waypoint[2]
    
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server to be available...')
        self.action_client.wait_for_server()
        self.get_logger().info('Action server is available')
    
        self.get_logger().info(f'Sending goal: {waypoint}')
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)    
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self.get_logger().info('Waiting for result...')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.hov_time}')

        self.goal_index += 1
        if self.goal_index < len(self.goals):
            self.send_goal(self.goals[self.goal_index])
        else:
            self.get_logger().info('All waypoints have been reached successfully')      

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        x = feedback.current_waypoint.pose.position.x
        y = feedback.current_waypoint.pose.position.y
        z = feedback.current_waypoint.pose.position.z
        t = feedback.current_waypoint.header.stamp.sec
        self.get_logger().info(f'Received feedback! The current waypoint position is: ({x}, {y}, {z})')
        self.get_logger().info(f'Max time inside sphere: {t}')


    ### Service client functions

    def send_request(self):
        # Set get_waypoints to True to request the waypoints
        self.req.get_waypoints = True
        self.get_logger().info('Sending request for waypoints...')
        return self.cli.call_async(self.req)

    def receive_goals(self):
        future = self.send_request()
        # Wait for the service to complete the future
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response is None:
            self.get_logger().error("Failed to get a response from the service.")
            return
        elif not response.waypoints.poses:
            self.get_logger().error("Received empty waypoint list from the service.")
            return

        self.get_logger().info('Waypoints received by the action client')
        
        for pose in response.waypoints.poses:
            waypoint = [pose.position.x, pose.position.y, pose.position.z]
            self.goals.append(waypoint)
            self.get_logger().info(f'Waypoint: {waypoint}')

        if self.goals:
            self.send_goal(self.goals[0])
        else:
            self.get_logger().error("No goals to send to the action server.")
    

def main(args=None):
    rclpy.init(args=args)

    waypoint_client = WayPointClient()
    waypoint_client.receive_goals()

    try:
        rclpy.spin(waypoint_client)
    except KeyboardInterrupt:
        waypoint_client.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoint_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
