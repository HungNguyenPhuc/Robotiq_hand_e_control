#!/usr/bin/env python3

import rospy

from robotiq_hand_e_control.msg import GripperCommand, GripperStatus


def gripper_status_callback(status):
    rospy.loginfo("Received gripper status")
    # Add status processing here if needed


def test_gripper_commands():
    # Initialize the node
    rospy.init_node("gripper_test_node", anonymous=True)

    # Create publisher for commands
    cmd_pub = rospy.Publisher("/robotiq_hand_e/command", GripperCommand, queue_size=10)

    # Subscribe to status
    rospy.Subscriber("/robotiq_hand_e/status", GripperStatus, gripper_status_callback)

    # Wait for connections
    rospy.sleep(1)

    # Create command message
    cmd = GripperCommand()

    # Test sequence
    try:
        # 1. Activate gripper
        rospy.loginfo("Testing gripper activation...")
        cmd.command_type = "activate"
        cmd.position = 0
        cmd.speed = 255
        cmd.force = 255
        cmd_pub.publish(cmd)
        rospy.sleep(2)

        # 2. Open gripper
        rospy.loginfo("Testing gripper opening...")
        cmd.command_type = "open"
        cmd_pub.publish(cmd)
        rospy.sleep(2)

        # 3. Close gripper
        rospy.loginfo("Testing gripper closing...")
        cmd.command_type = "close"
        cmd_pub.publish(cmd)
        rospy.sleep(2)

        # 4. Custom position (50%)
        rospy.loginfo("Testing custom position (50%)...")
        cmd.command_type = "custom"
        cmd.position = 128  # Mid position (0-255)
        cmd.speed = 128  # Medium speed
        cmd.force = 128  # Medium force
        cmd_pub.publish(cmd)
        rospy.sleep(2)

    except rospy.ROSInterruptException:
        rospy.logwarn("Test interrupted!")


if __name__ == "__main__":
    try:
        test_gripper_commands()
        rospy.loginfo("Test completed successfully!")
    except rospy.ROSInterruptException:
        pass
