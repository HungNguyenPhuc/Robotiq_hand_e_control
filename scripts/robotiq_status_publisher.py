#!/usr/bin/env python3
import rospy
from robotiq_hand_e_control.msg import GripperStatus
from pyRobotiqGripper.driver import RobotiqGripper

def publish_status():
    rospy.init_node('gripper_status_publisher')
    pub = rospy.Publisher('/robotiq_gripper/status', GripperStatus, queue_size=10)
    rate = rospy.Rate(5)

    gripper = RobotiqGripper()
    gripper.activate()

    while not rospy.is_shutdown():
        gripper.readAll()
        status = GripperStatus()

        for key in gripper.paramDic:
            value = gripper.paramDic[key]
            desc = str(gripper.registerDic.get(key, {}).get(value, "N/A"))

            setattr(status, key, value)
            setattr(status, f"{key}_desc", desc)

        # gCU (motor current) scale to mA
        status.gCU = gripper.paramDic["gCU"] * 10
        status.gCU_desc = str(gripper.registerDic["gCU"].get(gripper.paramDic["gCU"], "N/A"))

        pub.publish(status)
        rate.sleep()
if __name__ == '__main__':
    try:
        publish_status()
    except rospy.ROSInterruptException:
        pass