#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from collections import deque
from copy import deepcopy

class IMUBiasCorrector(Node):
    def __init__(self):
        super().__init__('imu_bias_corrector')
        
        # Declare parameters
        self.declare_parameter('calibration_samples', 200)
        self.declare_parameter('calibration_time', 10.0)
        self.declare_parameter('input_topic', '/imu/data')
        self.declare_parameter('output_topic', '/imu/data_corrected')
        
        # Get parameters
        self.calibration_samples = self.get_parameter('calibration_samples').value
        self.calibration_time = self.get_parameter('calibration_time').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        # Bias values (to be calibrated)
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0
        
        # For initial calibration
        self.calibration_buffer = deque(maxlen=self.calibration_samples)
        self.is_calibrated = False
        self.calibration_start_time = None
        
        # Create publisher and subscriber
        self.publisher = self.create_publisher(Imu, output_topic, 10)
        self.subscription = self.create_subscription(
            Imu,
            input_topic,
            self.imu_callback,
            10
        )
        
        self.get_logger().info(f'IMU Bias Corrector started. Keep robot stationary for {self.calibration_time} seconds for calibration...')
        
    def imu_callback(self, msg):
        if not self.is_calibrated:
            self.calibrate(msg)
        else:
            self.publish_corrected(msg)
    
    def calibrate(self, msg):
        """Collect samples while robot is stationary to determine bias"""
        current_time = self.get_clock().now()
        
        if self.calibration_start_time is None:
            self.calibration_start_time = current_time
            
        # Add sample to buffer
        self.calibration_buffer.append({
            'gyro_x': msg.angular_velocity.x,
            'gyro_y': msg.angular_velocity.y,
            'gyro_z': msg.angular_velocity.z
        })
        
        # Check if calibration period is over
        elapsed_time = (current_time - self.calibration_start_time).nanoseconds / 1e9
        
        if elapsed_time >= self.calibration_time and len(self.calibration_buffer) >= 50:
            # Calculate average bias
            samples = list(self.calibration_buffer)
            self.gyro_bias_x = np.mean([s['gyro_x'] for s in samples])
            self.gyro_bias_y = np.mean([s['gyro_y'] for s in samples])
            self.gyro_bias_z = np.mean([s['gyro_z'] for s in samples])
            
            # Calculate standard deviation for logging
            std_x = np.std([s['gyro_x'] for s in samples])
            std_y = np.std([s['gyro_y'] for s in samples])
            std_z = np.std([s['gyro_z'] for s in samples])
            
            self.is_calibrated = True
            
            self.get_logger().info(
                f'Calibration complete! Bias values:\n'
                f'  Gyro X: {self.gyro_bias_x:.6f} ± {std_x:.6f} rad/s\n'
                f'  Gyro Y: {self.gyro_bias_y:.6f} ± {std_y:.6f} rad/s\n'
                f'  Gyro Z: {self.gyro_bias_z:.6f} ± {std_z:.6f} rad/s'
            )
            
            # Also log if bias seems unusually high
            if abs(self.gyro_bias_z) > 0.01:
                self.get_logger().warn(
                    f'High gyro Z bias detected ({self.gyro_bias_z:.6f} rad/s). '
                    'Consider checking IMU mounting and calibration.'
                )
        else:
            remaining = self.calibration_time - elapsed_time
            if int(remaining) % 2 == 0 and int(remaining) > 0:
                self.get_logger().info(f'Calibrating... {remaining:.1f} seconds remaining')
    
    def publish_corrected(self, msg):
        """Publish IMU message with bias correction applied"""
        corrected_msg = deepcopy(msg)
        
        # Apply bias correction to angular velocities
        corrected_msg.angular_velocity.x = msg.angular_velocity.x - self.gyro_bias_x
        corrected_msg.angular_velocity.y = msg.angular_velocity.y - self.gyro_bias_y
        corrected_msg.angular_velocity.z = msg.angular_velocity.z - self.gyro_bias_z
        
        # Optionally apply deadband to remove noise near zero
        deadband = 0.001  # rad/s
        if abs(corrected_msg.angular_velocity.x) < deadband:
            corrected_msg.angular_velocity.x = 0.0
        if abs(corrected_msg.angular_velocity.y) < deadband:
            corrected_msg.angular_velocity.y = 0.0
        if abs(corrected_msg.angular_velocity.z) < deadband:
            corrected_msg.angular_velocity.z = 0.0
        
        # Publish corrected message
        self.publisher.publish(corrected_msg)
        
        # Periodic status logging (every 100 messages)
        if hasattr(self, 'msg_count'):
            self.msg_count += 1
        else:
            self.msg_count = 0
            
        if self.msg_count % 100 == 0:
            self.get_logger().debug(
                f'Publishing corrected IMU data. Raw Z: {msg.angular_velocity.z:.6f}, '
                f'Corrected Z: {corrected_msg.angular_velocity.z:.6f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = IMUBiasCorrector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

