#!/usr/bin/env python3


'''
This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
This node publishes and subscribes to the following topics:

        PUBLICATIONS			SUBSCRIPTIONS
        /drone_command			/whycon/poses
        /pid_error			/throttle_pid
                        /pitch_pid
                        /roll_pid
                    
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAIN MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries
from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node


class SwiftPico(Node):
    def __init__(self):
        super().__init__('pico_controller')  # Initializing ROS node with name pico_controller

        # Current position of the drone [x, y, z]
        self.drone_position = [0.0, 0.0, 0.0]

        # Desired setpoint [x_setpoint, y_setpoint, z_setpoint]
        self.setpoint = [2, 2, 19]  # Whycon marker position on dummy in the scene

        # Command message initialization
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

        #  Initial PID gains [roll, pitch, throttle]
        # self.Kp = [3333.4, 5166.7, 36666.7]  # Kp for roll, pitch, throttle
        # self.Ki = [125, 0 , 125]           # Ki for roll, pitch, throttle
        # self.Kd = [60 , 83.4, 91.7]              # Kd for roll, pitch, throttle
        # self.Kp = [120, 450, 1120]
        # self.Ki = [0, 0, 2]
        # self.Kd = [100, 150,65]



        #         kp= [300, 450, 0]
        # ki= [0, 0, 0]
        # kd= [100, 150, 0]
        # self.Kp = [9, 13.5, 33.6]
        # self.Ki = [0, 0, 0.016]
        # self.Kd = [60,90,39]

        #OLD VALUES

        # self.Kp = [300, 450, 985, 0]
        # self.Ki = [0, 0, 2, 0]
        # self.Kd = [100, 150, 48 ,0]
        
         #NEW VALUES
        # self.Kp = [6,6, 60, 0]
        # self.Ki = [1, 0, 25, 0]
        # self.Kd = [2, 3, 35 ,0]



        self.Kp = [0,0, 0]
        self.Ki = [0, 0, 0]
        self.Kd = [0,0,0]
        
        # PID variables
        self.prev_error = [0.0, 0.0, 0.0]  # Previous error for [roll, pitch, throttle]
        self.integral = [0.0, 0.0, 0.0]    # Integral error for [roll, pitch, throttle]

        # Max and Min values for roll, pitch, and throttle control signals
        self.max_values = [2000, 2000, 2000]
        self.min_values = [1000, 1000, 1000]

        # Time variables for PID
        self.last_time = self.get_clock().now()

        # Sample time for PID loop
        self.sample_time = 0.060  # in seconds

        # Publishers
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

        # Subscribers
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, '/throttle_pid', self.altitude_set_pid, 1)
        self.create_subscription(PIDTune, '/pitch_pid', self.pitch_set_pid, 1)
        self.create_subscription(PIDTune, '/roll_pid', self.roll_set_pid, 1)

        # Arming the drone
        self.arm()

        # Timer to run the PID function periodically
        self.create_timer(self.sample_time, self.pid)

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

    # Whycon callback function for drone position
    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x  # x-coordinate
        self.drone_position[1] = msg.poses[0].position.y  # y-coordinate
        self.drone_position[2] = msg.poses[0].position.z  # z-coordinate

    # # Callback function for /throttle_pid
    # def altitude_set_pid(self, alt):
    #     self.Kp[2] = self.Kp[2] * 0.03
    #     self.Ki[2] = self.Ki[2] * 0.008
    #     self.Kd[2] = self.Kd[2] * 0.6

    # # Callback function for /pitch_pid
    # def pitch_set_pid(self, pitch):
    #     self.Kp[1] = self.Kp[1] * 0.03
    #     self.Ki[1] = self.Ki[1] * 0.008
    #     self.Kd[1] = self.Kd[1] * 0.6

    # # Callback function for /roll_pid
    # def roll_set_pid(self, roll):
    #     self.Kp[0] = self.Kp[0] * 0.03
    #     self.Ki[0] = self.Ki[0] * 0.008
    #     self.Kd[0] = self.Kd[0] * 0.6

# Callback function for /throttle_pid
    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.kp * 1  # Adjust scaling factor accordingly
        self.Ki[2] = alt.ki * 0.001
        self.Kd[2] = alt.kd * 1

    # Callback function for /pitch_pid
    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.kp 
        self.Ki[1] = pitch.ki * 0.001
        self.Kd[1] = pitch.kd 

    # Callback function for /roll_pid
    def roll_set_pid(self, roll):
        self.Kp[0] = roll.kp 
        self.Ki[0] = roll.ki * 0.001
        self.Kd[0] = roll.kd 
        
    # PID control loop
    def pid(self):
        # Error in [roll, pitch, throttle]
        error = [
            -(self.drone_position[0] - self.setpoint[0]),  # Roll error
            self.drone_position[1] - self.setpoint[1],  # Pitch error
            self.drone_position[2] - self.setpoint[2]   # Throttle error
        ]

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Time difference in seconds

        if dt >= self.sample_time:
            pid_output = [0.0, 0.0, 0.0]  # PID output for [roll, pitch, throttle]

            for i in range(3):
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

				# Here 0 for roll, 1 for pitch and 2 for throttle
                
                # Command values based on PID output
                if i == 0:  # Roll
                    self.cmd.rc_roll = int(1500 + pid_output[i])
                elif i == 1:  # Pitch
                    self.cmd.rc_pitch = int(1500 + pid_output[i])
                else:  # Throttle
                    self.cmd.rc_throttle = int(1500 + pid_output[i])

            # Limit the output values to the max and min
            self.cmd.rc_roll = max(self.min_values[0], min(self.max_values[0], self.cmd.rc_roll))
            self.cmd.rc_pitch = max(self.min_values[1], min(self.max_values[1], self.cmd.rc_pitch))
            self.cmd.rc_throttle = max(self.min_values[2], min(self.max_values[2], self.cmd.rc_throttle))

            # Update previous errors and time
            self.prev_error = error.copy()
            self.last_time = current_time

            # Publish command and PID error
            self.command_pub.publish(self.cmd)

            # Create and publish PID error message
            pid_error_msg = PIDError()
            pid_error_msg.roll_error = error[0]
            pid_error_msg.pitch_error = error[1]
            pid_error_msg.throttle_error = error[2]
            pid_error_msg.yaw_error = 0.0
            self.pid_error_pub.publish(pid_error_msg)


def main(args=None):
    rclpy.init(args=args)
    swift_pico = SwiftPico()
    rclpy.spin(swift_pico)
    swift_pico.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()    
