#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

from math import sqrt, atan2, asin
import time
import numpy as np

import os

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import OffboardControlMode, VehicleControlMode, VehicleStatus, ActionRequest
from px4_msgs.msg import VehicleAttitude, VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleAngularVelocity, VehicleRatesSetpoint
from px4_msgs.msg import SensorCombined, VehicleLocalPosition, Airspeed, AirspeedWind, VehicleAcceleration
from px4_msgs.msg import ActuatorMotors, ActuatorServos

from time import sleep, time

class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Set up subscribers and link to callbacks
        # Vehicle Attitude
        self.att_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.vehicle_attitude_callback,
            qos_profile)
        
        # Attitude Setpoint
        self.att_setpoint_sub = self.create_subscription(
            VehicleAttitudeSetpoint,
            '/fmu/out/vehicle_attitude_setpoint',
            self.vehicle_attitude_setpoint_callback,
            qos_profile)
        
        # Vehicle Angular Velocity
        self.rate_sub = self.create_subscription(
            VehicleAngularVelocity,
            '/fmu/out/vehicle_angular_velocity',
            self.vehicle_angular_velocity_callback,
            qos_profile)

        # Vehicle Rates Setpoint
        self.rate_setpoint_sub = self.create_subscription(
            VehicleRatesSetpoint,
            '/fmu/out/vehicle_rates_setpoint',
            self.vehicle_rates_setpoint_callback,
            qos_profile)
        
        # Sensor Combined
        self.sensor_sub = self.create_subscription(
            SensorCombined,
            '/fmu/out/sensor_combined',
            self.sensor_combined_callback,
            qos_profile)

        # Vehicle Acceleration
        self.accel_sub = self.create_subscription(
            VehicleAcceleration,
            '/fmu/out/vehicle_acceleration',
            self.vehicle_acceleration_callback,
            qos_profile)
        
        # Actuator Motors
        self.motors_sub = self.create_subscription(
            ActuatorMotors,
            '/fmu/out/actuator_motors',
            self.actuator_motors_callback,
            qos_profile)
        
        # Actuator Servos
        self.servos_sub = self.create_subscription(
            ActuatorServos,
            '/fmu/out/actuator_servos',
            self.actuator_servos_callback,
            qos_profile)

        # Torque Setpoints (?)
        # Thrust Setpoints (?)

        # Vehicle Local Position
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile)
        
        # Airspeed
        self.airspeed_sub = self.create_subscription(
            Airspeed,
            '/fmu/out/airspeed',
            self.airspeed_callback,
            qos_profile)
        
        # Airspeed Wind
        self.airspeed_wind_sub = self.create_subscription(
            AirspeedWind,
            '/fmu/out/airspeed_wind',
            self.airspeed_wind_callback,
            qos_profile)

        # Control mode
        self.mode_sub = self.create_subscription(
            VehicleControlMode,
            '/fmu/out/vehicle_control_mode',
            self.vehicle_mode_callback,
            qos_profile)
        
        # Vehicle status
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile)
        
        # Set up publishers
        self.publisher_offboard_mode    = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_motors           = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)
        self.publisher_servos           = self.create_publisher(ActuatorServos, '/fmu/in/actuator_servos', qos_profile)
        self.publisher_action           = self.create_publisher(ActionRequest, '/fmu/in/action_request', qos_profile)
        self.publisher_attitude         = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)

        # Internal Timer
        timer_period = 0.01  # 100 Hz
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.dt = timer_period

        # Member Variables
        self.offboard_active = False
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED        
        self.last_nav_state = 0
        self.has_triggered = False
        self.trigger_count = 0
        self.trigger_time = 0


    # Define Callback Functions
    # Callback to handle vehicle attitude message
    def vehicle_attitude_callback(self, msg):
       self.get_logger().info(f'Attitude Rx', once=True)
       [roll, pitch, yaw] = quaternion_to_euler(msg.q[0], msg.q[1], msg.q[2], msg.q[3])
       
    # Callback to handle vehicle attitude setpoint message
    def vehicle_attitude_setpoint_callback(self, msg):
        self.get_logger().info(f'Attitude Setpoint Rx', once=True)

    # Callback to handle vehicle angular velocity message
    def vehicle_angular_velocity_callback(self, msg):
        self.get_logger().info(f'Rate Rx', once=True)

    # Callback to handle vehicle rates setpoint message
    def vehicle_rates_setpoint_callback(self, msg):
        self.get_logger().info(f'Rate Setpoint Rx', once=True)

    # Callback to handle sensor combined message
    def sensor_combined_callback(self, msg):
        self.get_logger().info(f'Sensor Rx', once=True)
    
    # Callback to handle vehicle acceleration message
    def vehicle_acceleration_callback(self, msg):
        self.get_logger().info(f'Acceleration Rx', once=True)

    # Callback to handle actuator motors message
    def actuator_motors_callback(self, msg):
        self.get_logger().info(f'Motors Rx', once=True)

    # Callback to handle actuator servos message
    def actuator_servos_callback(self, msg):
        self.get_logger().info(f'Servos Rx', once=True)
    
    # Callback to handle vehicle local position message
    def vehicle_local_position_callback(self, msg):
        self.get_logger().info(f'Local Pos Rx', once=True)

    # Callback to handle airspeed message
    def airspeed_callback(self, msg):
        self.get_logger().info(f'Airspeed Rx', once=True)

    # Callback to handle airspeed wind message
    def airspeed_wind_callback(self, msg):
        self.get_logger().info(f'Airspeed Wind Rx', once=True)
        
    # Callback to handle vehicle mode message
    def vehicle_mode_callback(self, msg):
        self.get_logger().info(f'Mode Rx', once=True)
        self.offboard_active = msg.flag_control_offboard_enabled
        
    # Callback to handle vehicle status message
    def vehicle_status_callback(self, msg):
        self.get_logger().info(f'Status Rx', once=True)
        self.arming_state = msg.arming_state
        
        if(not msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
            self.last_nav_state = msg.nav_state # Get last mode to revert to

        if(self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
            self.get_logger().warn(f'Armed!', once=True) # Check for arming

    # Timed command loop
    def cmdloop_callback(self):
        
        # IF armed
        if(self.arming_state == VehicleStatus.ARMING_STATE_ARMED):          

            if(self.offboard_active): # AND in offboard mode
                self.get_logger().info(f'Start Offboard', once=True)          
           
            # Else armed but not in offboard
            else:           
                self.get_logger().info(f'Standby',throttle_duration_sec=30)
                    
                
    def send_attitude_cmds_quaternion(self, roll, pitch, thr):
        
        att_msg = VehicleAttitudeSetpoint()
        att_msg.q_d = euler_to_quaternion(roll,pitch,0)
        att_msg.thrust_body[0] = thr
        
        self.get_logger().info(f'Roll, Pitch, thr: {180/np.pi * roll}, {180/np.pi * pitch}, {att_msg.thrust_body[0]}')
        self.publisher_attitude.publish(att_msg)

    def send_attitude_cmds(self, roll, pitch, thr):
        
        att_msg = VehicleAttitudeSetpoint()
        att_msg.pitch_body  = pitch
        att_msg.roll_body   = roll 
        att_msg.thrust_body[0] = thr
        
        self.get_logger().info(f'Roll, Pitch, Thr: {180/np.pi * att_msg.roll_body, 180/np.pi * att_msg.pitch_body, att_msg.thrust_body[0]}')
        self.publisher_attitude.publish(att_msg)

    def send_servo_cmds(self, servos):
        
        servo_msg = ActuatorServos()
        servo_msg.control = servos
        servo_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher_servos.publish(servo_msg)
                
    def send_motor_cmd(self, motor):
       
        motor_msg = ActuatorMotors()
        motor_msg.control[0] = np.clip(motor, 0, 1)
        motor_msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.publisher_motors.publish(motor_msg)
    
    def send_mode_cmd(self, mode):
        
        action_msg = ActionRequest()
        action_msg.action = ActionRequest.ACTION_SWITCH_MODE
        action_msg.mode = mode
        self.publisher_action.publish(action_msg)

    def send_offboard_mode(self, actuator, attitude):
        # Publish offboard control modes
        offboard_msg = OffboardControlMode()
        offboard_msg.timestamp          = int(Clock().now().nanoseconds / 1000) # PX4 expects microseconds
        offboard_msg.position           = False
        offboard_msg.velocity           = False
        offboard_msg.acceleration       = False
        offboard_msg.thrust_and_torque  = False
        offboard_msg.attitude           = attitude
        offboard_msg.direct_actuator    = actuator

        self.publisher_offboard_mode.publish(offboard_msg)
    
def quaternion_to_euler(w, x, y, z):
        
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = atan2(t3, t4)

        # Euler angles returned in units of Radians
        return roll, pitch, yaw

def euler_to_quaternion(roll, pitch, yaw):
        
        # Euler angles expected in units of Radians
        x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [w, x, y, z]


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()
    
    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
