#!/usr/bin/env python3

import time
import math
from tf_transformations import euler_from_quaternion

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

#import the action
from waypoint_navigation.action import NavToWaypoint

#pico control specific libraries
from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
from nav_msgs.msg import Odometry

class WayPointServer(Node):

    def __init__(self):
        super().__init__('waypoint_server')

        self.pid_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()
        self.last_time = self.get_clock().now()

        self.time_inside_sphere = 0
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.duration = 0


        self.drone_position = [0.0, 0.0, 0.0, 0.0]
        self.setpoint = [0, 0, 27, 0] 
        self.dtime = 0

        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

        # Max and Min values for roll, pitch, and throttle control signals
        self.max_values = [2000, 2000, 2000]
        self.min_values = [1000, 1000, 1000]

        #old VALUES
        # kp= [300, 450, 985]
        # ki= [0, 0, 2]
        # kd= [100, 150, 48]
        


         #NEW VALUES
        # self.Kp = [6,12, 38, 0]
        # self.Ki = [0.001, 0.001, 0.015, 0]
        # self.Kd = [2, 3, 27 ,0]

        #Kp, Ki and Kd values here
        self.Kp = [6,12, 61, 0]
        self.Ki = [0.001, 0.001, 0.01, 0]
        self.Kd = [2, 3, 37 ,0]

        self.initial_yaw_value = 1500  # Set the initial yaw value here
        

        #variables for storing different kinds of errors
        self.error = [0.0, 0.0, 0.0, 0.0]
        self.prev_error = [0.0, 0.0, 0.0, 0.0]
        self.integral = [0.0, 0.0, 0.0, 0.0]
        self.derivative = [0.0, 0.0, 0.0, 0.0]
        

        self.pid_error = PIDError()

        self.sample_time = 0.060

        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        
        #Add other sunscribers here
        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)
        self.create_subscription(PIDTune, "/yaw_pid", self.yaw_set_pid, 1)



        self.create_subscription(Odometry, '/rotors/odometry', self.odometry_callback, 10)

        #create an action server for the action 'NavToWaypoint'. Refer to Writing an action server and client (Python) in ROS 2 tutorials
        #action name should 'waypoint_navigation'.
        #include the action_callback_group in the action server. Refer to executors in ROS 2 concepts
        

        
        self.arm()
        
        self._action_server = ActionServer(
            self,
            NavToWaypoint,
            'waypoint_navigation',  # Action name
            self.execute_callback,
            callback_group=self.action_callback_group
        )

        self.timer = self.create_timer(self.sample_time, self.pid, callback_group=self.pid_callback_group)

    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)


    def arm(self):
        self.disarm()
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)


    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        #Set the remaining co-ordinates of the drone from msg
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z




        self.dtime = msg.header.stamp.sec

    # def altitude_set_pid(self, alt):
    #     self.Kp[1] = alt.kp * 1.0 
    #     self.Ki[1] = alt.ki * 0.001
    #     self.Kd[1] = alt.kd * 1.0

    # #Define callback function like altitide_set_pid to tune pitch, roll
    # #and yaw PID values
    # def pitch_set_pid(self, pitch):
    #     self.Kp[0] = pitch.kp * 1.0
    #     self.Ki[0] = pitch.ki * 0.001
    #     self.Kd[0] = pitch.kd * 1.0

    # def roll_set_pid(self, roll):
    #     self.Kp[2] = roll.kp * 1.0
    #     self.Ki[2] = roll.ki * 0.001
    #     self.Kd[2] = roll.kd * 1.0

    # def yaw_set_pid(self, yaw):
    #     self.Kp[3] = yaw.kp * 1.0
    #     self.Ki[3] = yaw.ki * 0.001
    #     self.Kd[3] = yaw.kd * 1.0

    def altitude_set_pid(self, alt):
        self.Kp[1] = self.Kp[1] * 1.0 
        self.Ki[1] = self.Ki[1] * 0.001
        self.Kd[1] =self.Kd[1] * 1.0

    #Define callback function like altitide_set_pid to tune pitch, roll
    #and yaw PID values
    def pitch_set_pid(self, pitch):
        self.Kp[0] = self.Kp[0] * 1.0
        self.Ki[0] = self.Ki[0] * 0.001
        self.Kd[0] = self.Kd[0] * 1.0

    def roll_set_pid(self, roll):
        self.Kp[2] = self.Kp[2] * 1.0
        self.Ki[2] = self.Ki[2] * 0.001
        self.Kd[2] = self.Kd[2]* 1.0

    def yaw_set_pid(self, yaw):
        self.Kp[3] = self.Kp[3] * 1.0
        self.Ki[3] = self.Ki[3] * 0.001
        self.Kd[3] = self.Kd[3] * 1.0

    def odometry_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        self.roll_deg = math.degrees(roll)
        self.pitch_deg = math.degrees(pitch)
        self.yaw_deg = math.degrees(yaw)
        self.drone_position[3] = self.yaw_deg	

    def pid(self):

        #write your PID algorithm here. This time write equations for throttle, pitch, roll and yaw. 
        #Follow the steps from task 1b.
        
        # Error in [roll, pitch, throttle]
        error = [
            -(self.drone_position[0] - self.setpoint[0]),  # Roll error
            self.drone_position[1] - self.setpoint[1],  # Pitch error
            self.drone_position[2] - self.setpoint[2],   # Throttle error
            self.drone_position[3] - self.setpoint[3]   # Yaw error
        ]

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Time difference in seconds

        if dt >= self.sample_time:
            pid_output = [0.0, 0.0, 0.0, 0.0]  # PID output for [roll, pitch, throttle, yaw]

            for i in range(4):
                if i != 3:  # Skip yaw (index 3)
                    # Proportional term
                    P = self.Kp[i] * error[i]

                    # Integral term
                    self.integral[i] += error[i] * dt
                    I = self.Ki[i] * self.integral[i]

                    # Derivative term
                    derivative = (error[i] - self.prev_error[i]) / dt if dt > 0 else 0.0
                    D = self.Kd[i] * derivative

                    # PID output
                    pid_output[i] = P + I + D

                    # Command values based on PID output for roll, pitch, throttle
                    if i == 0:  # Roll
                        self.cmd.rc_roll = int(1500 + pid_output[i])
                    elif i == 1:  # Pitch
                        self.cmd.rc_pitch = int(1500 + pid_output[i])
                    elif i == 2:  # Throttle
                        self.cmd.rc_throttle = int(1500 + pid_output[i])

            # Limit the output values to the max and min
            self.cmd.rc_roll = max(self.min_values[0], min(self.max_values[0], self.cmd.rc_roll))
            self.cmd.rc_pitch = max(self.min_values[1], min(self.max_values[1], self.cmd.rc_pitch))
            self.cmd.rc_throttle = max(self.min_values[2], min(self.max_values[2], self.cmd.rc_throttle))
            #self.cmd.rc_yaw = max(self.min_values[3], min(self.max_values[3], self.cmd.rc_yaw))

            # Update previous errors and time
            self.prev_error = error.copy()
            self.last_time = self.get_clock().now()


            # Publish command and PID error
            self.command_pub.publish(self.cmd)

            # Create and publish PID error message
            pid_error = PIDError()
            pid_error.roll_error = error[0]
            pid_error.pitch_error = error[1]
            pid_error.throttle_error = error[2]
            pid_error.yaw_error = error[3]
            #publish the PID error
            self.command_pub.publish(self.cmd)
            self.pid_error_pub.publish(self.pid_error)

    def execute_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')
        self.setpoint[0] = goal_handle.request.waypoint.position.x
        self.setpoint[1] = goal_handle.request.waypoint.position.y
        self.setpoint[2] = goal_handle.request.waypoint.position.z
        self.get_logger().info(f'New Waypoint Set: {self.setpoint}')
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.time_inside_sphere = 0
        self.duration = self.dtime

        #create a NavToWaypoint feedback object. Refer to Writing an action server and client (Python) in ROS 2 tutorials.
        feedback_msg = NavToWaypoint.Feedback()
        
        #--------The script given below checks whether you are hovering at each of the waypoints(goals) for max of 3s---------#
        # This will help you to analyse the drone behaviour and help you to tune the PID better.

        while True:
            feedback_msg.current_waypoint.pose.position.x = self.drone_position[0]
            feedback_msg.current_waypoint.pose.position.y = self.drone_position[1]
            feedback_msg.current_waypoint.pose.position.z = self.drone_position[2]
            feedback_msg.current_waypoint.header.stamp.sec = self.max_time_inside_sphere

            goal_handle.publish_feedback(feedback_msg)

            drone_is_in_sphere = self.is_drone_in_sphere(self.drone_position, goal_handle, 0.4) #the value '0.4' is the error range in the whycon coordinates that will be used for grading. 
            #You can use greater values initially and then move towards the value '0.4'. This will help you to check whether your waypoint navigation is working properly. 

            if not drone_is_in_sphere and self.point_in_sphere_start_time is None:
                        pass
            
            elif drone_is_in_sphere and self.point_in_sphere_start_time is None:
                        self.point_in_sphere_start_time = self.dtime
                        self.get_logger().info('Drone in sphere for 1st time')                        #you can choose to comment this out to get a better look at other logs

            elif drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                        self.time_inside_sphere = self.dtime - self.point_in_sphere_start_time
                        self.get_logger().info('Drone in sphere')                                     #you can choose to comment this out to get a better look at other logs
                             
            elif not drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                        self.get_logger().info('Drone out of sphere')                                 #you can choose to comment this out to get a better look at other logs
                        self.point_in_sphere_start_time = None

            if self.time_inside_sphere > self.max_time_inside_sphere:
                 self.max_time_inside_sphere = self.time_inside_sphere

            if self.max_time_inside_sphere >= 3:
                 break
                        

        goal_handle.succeed()

        #create a NavToWaypoint result object. Refer to Writing an action server and client (Python) in ROS 2 tutorials.
        result = NavToWaypoint.Result()

        
        #  Refer to Writing an action server and client (Python) in ROS 2 tutorials



        result.hov_time = self.dtime - self.duration #this is the total time taken by the drone in trying to stabilize at a point
        return result

    def is_drone_in_sphere(self, drone_pos, sphere_center, radius):
        return (
            (drone_pos[0] - sphere_center.request.waypoint.position.x) ** 2
            + (drone_pos[1] - sphere_center.request.waypoint.position.y) ** 2
            + (drone_pos[2] - sphere_center.request.waypoint.position.z) ** 2
        ) <= radius**2


def main(args=None):
    rclpy.init(args=args)

    waypoint_server = WayPointServer()
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_server)
    
    try:
         executor.spin()
    except KeyboardInterrupt:
        waypoint_server.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
         waypoint_server.destroy_node()
         rclpy.shutdown()


if __name__ == '__main__':
    main()

