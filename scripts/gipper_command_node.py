#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

class GripperCommander:
    def __init__(self):
        self.pub = rospy.Publisher("/gripper_driver_command", String, queue_size=10)
        rospy.Subscriber("/gripper_command_string", String, self.cmd_cb)
        rospy.loginfo("Gripper Commander node sẵn sàng")

    def cmd_cb(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ["activate", "open", "close"]:
            self.pub.publish(cmd)
        else:
            rospy.logwarn(f"Lệnh không hợp lệ: {cmd}")

if __name__ == "__main__":
    rospy.init_node("gripper_command_node")
    GripperCommander()
    rospy.spin()
