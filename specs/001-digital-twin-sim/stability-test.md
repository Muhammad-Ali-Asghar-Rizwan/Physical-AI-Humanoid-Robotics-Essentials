# Robot Stability Test Script

## Purpose
This script runs a 60+ second simulation of the humanoid robot to test its stability. It measures key metrics related to balance and joint stability.

## Metrics to Track
- Joint position drift over time
- Center of mass stability
- Ground contact consistency
- Joint velocity and acceleration bounds

## Testing Process
1. Initialize the humanoid in a standing position
2. Run the simulation for 60 seconds with default standing controller
3. Monitor joint positions for unexpected drift
4. Track center of mass position relative to feet
5. Log any instability events (falling, joint limits exceeded)

## Expected Results
- The humanoid should maintain standing position for full 60 seconds
- Joint positions should remain within reasonable bounds
- Center of mass should stay within support polygon of feet
- No joint limit violations should occur

## Stability Validation Code (Pseudocode)
```python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
import time

class StabilityTester:
    def __init__(self):
        rospy.init_node('stability_tester')
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.tf_callback)
        
        self.joint_positions = {}
        self.foot_positions = {}
        self.com_positions = []
        self.start_time = rospy.Time.now().to_sec()
        self.test_duration = 60  # seconds
        
        self.stability_metrics = {
            'max_com_drift': 0.0,
            'max_joint_drift': 0.0,
            'fallen': False,
            'joint_limit_violations': 0
        }
    
    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.joint_positions[name] = msg.position[i]
    
    def tf_callback(self, msg):
        # Extract foot and torso positions for CoM calculation
        pass
    
    def calculate_center_of_mass(self):
        # Calculate estimated center of mass based on link positions
        pass
    
    def check_stability(self):
        # Check if robot has fallen (torso too low or tilted excessively)
        pass
    
    def run_test(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            current_time = rospy.Time.now().to_sec()
            elapsed = current_time - self.start_time
            
            if elapsed >= self.test_duration:
                break
                
            # Calculate stability metrics
            com = self.calculate_center_of_mass()
            self.com_positions.append(com)
            
            # Check for stability
            if self.check_stability():
                print("Robot has fallen!")
                self.stability_metrics['fallen'] = True
                break
            
            rate.sleep()
        
        # Analyze results
        self.analyze_results()
    
    def analyze_results(self):
        # Calculate drift, joint limits, etc.
        print("Stability Test Results:")
        print(f"- Test Duration: {self.test_duration} seconds")
        print(f"- Robot Fallen: {self.stability_metrics['fallen']}")
        print(f"- Max CoM Drift: {self.stability_metrics['max_com_drift']}")
        print(f"- Joint Limit Violations: {self.stability_metrics['joint_limit_violations']}")
        print(f"- Result: {'PASS' if not self.stability_metrics['fallen'] else 'FAIL'}")

if __name__ == '__main__':
    tester = StabilityTester()
    tester.run_test()
```

## Running the Test
1. Launch the Gazebo world with the humanoid robot
2. Start the balance controller
3. Run the stability test script
4. Monitor the results and adjust physics parameters if needed