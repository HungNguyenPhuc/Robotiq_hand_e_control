#!/usr/bin/env python3
import rospy
from robotiq_hand_e_control.msg import GripperCommand 
from pyRobotiqGripper.driver import RobotiqGripper
import traceback

class GripperTopicNode:
    def __init__(self):
        rospy.init_node("gripper_topic_node")
        self.gripper = RobotiqGripper()
        rospy.Subscriber("/gripper_command_topic", GripperCommand, self.command_callback)
        rospy.loginfo("Gripper topic node ready.")
        rospy.spin()

    def command_callback(self, msg):
        cmd = msg.command.lower()
        a1, a2, a3 = msg.arg1, msg.arg2, msg.arg3
        try:
            if cmd == "open":
                self.gripper.open(speed=int(a1), force=int(a2))
            elif cmd == "close":
                self.gripper.close(speed=int(a1), force=int(a2))
            elif cmd == "goto":
                self.gripper.goTo(position=int(a1), speed=int(a2), force=int(a3))
            elif cmd == "gopos":
                self.gripper.goTo(position=int(a1))
            elif cmd == "goposmm":
                self.gripper.goTomm(positionmm=float(a1), speed=int(a2), force=int(a3))
            elif cmd == "getpos":
                pos = self.gripper.getPosition()
                rospy.loginfo(f"Position (bit): {pos}")
            elif cmd == "getposmm":
                posmm = self.gripper.getPositionmm()
                rospy.loginfo(f"Position (mm): {posmm}")
            elif cmd == "reset":
                self.gripper.reset()
            elif cmd == "activate":
                self.gripper.activate()
            elif cmd == "resetactivate":
                self.gripper.resetActivate()
            elif cmd == "calibrate":
                self.gripper.calibrate(closemm=float(a1), openmm=float(a2))
            elif cmd == "reset_calib":
                self.gripper.resetCalibration()
            elif cmd == "info":
                self.gripper.printInfo()
            else:
                rospy.logwarn(f"[GripperTopicNode] Unknown command: {cmd}")
        except Exception as e:
            rospy.logerr(f"[GripperTopicNode] Failed command '{cmd}': {e}")
            rospy.logdebug(traceback.format_exc())

if __name__ == "__main__":
    GripperTopicNode()
