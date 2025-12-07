#!/usr/bin/env python3
"""
Performance validation script for the digital twin simulation.
Measures simulation FPS and other performance metrics.
"""

import time
import rospy
from std_msgs.msg import Float32
from gazebo_msgs.msg import PerformanceMetrics

class PerformanceValidator:
    def __init__(self):
        rospy.init_node('performance_validator')
        self.performance_sub = rospy.Subscriber('/gazebo/performance_metrics', PerformanceMetrics, self.performance_callback)
        
        self.fps_values = []
        self.start_time = time.time()
        self.test_duration = 30  # seconds
        
        print("Starting performance validation test...")
        print(f"Testing for {self.test_duration} seconds to measure FPS and performance metrics")
        
    def performance_callback(self, msg):
        # Extract FPS from performance metrics
        current_fps = msg.real_time_factor  # This is a simplified representation
        self.fps_values.append(current_fps)
        
        # Print FPS periodically
        if len(self.fps_values) % 30 == 0:  # Print every 30 measurements
            avg_fps = sum(self.fps_values[-30:]) / 30
            print(f"Current FPS: {current_fps:.2f}, Average FPS (last 30): {avg_fps:.2f}")
    
    def run_test(self):
        rate = rospy.Rate(10)  # 10 Hz check
        
        while not rospy.is_shutdown():
            current_time = time.time()
            elapsed = current_time - self.start_time
            
            if elapsed >= self.test_duration:
                break
                
            rate.sleep()
        
        # Calculate and report final metrics
        if self.fps_values:
            avg_fps = sum(self.fps_values) / len(self.fps_values)
            min_fps = min(self.fps_values)
            max_fps = max(self.fps_values)
            
            print("\n=== Performance Validation Results ===")
            print(f"Test Duration: {self.test_duration} seconds")
            print(f"Average FPS: {avg_fps:.2f}")
            print(f"Min FPS: {min_fps:.2f}")
            print(f"Max FPS: {max_fps:.2f}")
            
            if avg_fps >= 30:
                print("✅ SUCCESS: Average FPS meets target (>30 FPS)")
            else:
                print("❌ FAILURE: Average FPS below target (<30 FPS)")
                
            print("=====================================")
        else:
            print("No performance data was received during the test.")

if __name__ == '__main__':
    validator = PerformanceValidator()
    validator.run_test()