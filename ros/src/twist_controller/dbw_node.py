#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

DBW_NODE_RATE = 50 #50Hz

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node', log_level=rospy.INFO)

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                        SteeringCmd, queue_size=2)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                        ThrottleCmd, queue_size=2)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                        BrakeCmd, queue_size=2)
        # Velocity PID parameters
        velocity_kp = 0.2
        velocity_ki = 0.0
        velocity_kd = 0.1

        # Steering PID parameters
        steer_kp = 1.0
        steer_ki = 0.0
        steer_kd = 0.2
        
        # Create `Controller` object
        self.controller = Controller(vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, 
                                            wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle,
                                            velocity_kp, velocity_ki, velocity_kd,
                                            steer_kp, steer_ki, steer_kd)

        # Subscribe to all the topics you need to
        rospy.Subscriber('/dbw_enabled', Bool, self.callback_dbw, queue_size=2)
        rospy.Subscriber('/current_velocity', TwistStamped, self.callback_vel, queue_size=2)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.callback_twist, queue_size=2)

        # Init state variables
        self.dbw_enabled        = True
        self.current_velocity   = None
        self.twist_cmd          = None
        self.prev_time          = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(DBW_NODE_RATE) # 50Hz
        while not rospy.is_shutdown():
            if self.twist_cmd is None or self.current_velocity is None:
                # Not fully initialized
                continue
            if self.prev_time is None:
                self.prev_time = rospy.get_rostime()
                continue
                
            curr_time = rospy.get_rostime()
            delta_time = curr_time - self.prev_time
            dt = delta_time.secs + (1e-9 * delta_time.nsecs)
            
            throttle, brake, steering = self.controller.control(    self.twist_cmd.twist.linear,
                                                                    self.twist_cmd.twist.angular,
                                                                    self.current_velocity.twist.linear,
                                                                    self.dbw_enabled,
                                                                    dt)
            self.prev_time = curr_time
            
            if self.dbw_enabled:
                self.publish(throttle, brake, steering)    

            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def callback_dbw(self, msg):
        self.dbw_enabled = msg

    def callback_vel(self, msg):
        self.current_velocity = msg

    def callback_twist(self, msg):
        self.twist_cmd = msg

if __name__ == '__main__':
    DBWNode()
