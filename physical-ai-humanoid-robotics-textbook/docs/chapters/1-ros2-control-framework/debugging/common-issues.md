# Common Failures and Solutions

When working with ROS 2, you'll encounter common issues. Here are solutions to the most frequent problems:

## Node Communication Problems

**Issue**: Nodes can't communicate with each other
**Solution**:
- Check that both nodes are on the same network (same ROS_DOMAIN_ID)
- Verify topic names match exactly between publisher and subscriber
- Ensure the messaging types are identical between publisher and subscriber
- Use `ros2 topic list` to verify the topic exists
- Use `ros2 topic info /topic_name` to check publishers/subscribers

## URDF Loading Errors

**Issue**: URDF file fails to load or displays incorrectly
**Solution**:
- Verify the XML is well-formed (no unmatched tags)
- Ensure all joint parent/child links exist
- Check that all referenced materials exist
- Make sure the root link is properly defined
- Use `check_urdf` to validate: `check_urdf /path/to/your.urdf`

## Gazebo Integration Issues

**Issue**: Gazebo doesn't work correctly with ROS 2 nodes
**Solution**:
- Make sure you're using Gazebo Garden or Fortress with ROS 2 Humble
- Verify plugin compatibility
- Check that ROS 2 bridges are properly configured
- Ensure Gazebo environment variables are set

## Performance Troubleshooting

**Issue**: Nodes run slowly or with high CPU usage
**Solution**:
- Reduce the frequency of your timers if they're too high
- Check for memory leaks in your nodes
- Optimize the amount of data being published
- Consider using different QoS profiles for your publishers/subscribers