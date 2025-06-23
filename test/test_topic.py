#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


def test_gripper_commands():
    # Initialize the node
    rospy.init_node("gripper_test_node", anonymous=True)

    # Create publisher for commands
    cmd_pub = rospy.Publisher("/gripper_command_string", String, queue_size=10)

    # Wait for connections
    rospy.sleep(1)

    # Command sequence
    commands = [
        ("activate", "Kích hoạt gripper"),
        ("open", "Mở gripper"),
        ("close", "Đóng gripper"),
        ("custom:128,128,128", "Vị trí custom 50%"),
        ("stop", "Dừng gripper"),
    ]

    # Test sequence
    try:
        for cmd, desc in commands:
            rospy.loginfo(f"Gửi lệnh: {desc} ({cmd})")
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
