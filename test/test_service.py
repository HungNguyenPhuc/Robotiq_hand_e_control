#!/usr/bin/env python3

import rospy

from robotiq_hand_e_control.srv import GripperControl


def test_gripper_service():
    # Initialize the node
    rospy.init_node("gripper_service_test_node", anonymous=True)

    # Wait for service
    service_name = "gripper_control"
    rospy.loginfo(f"Waiting for service {service_name}")
    rospy.wait_for_service(service_name)

    try:
        # Create service proxy
        gripper_service = rospy.ServiceProxy(service_name, GripperControl)

        # Test sequence
        commands = [
            ("activate", "Activating gripper"),
            ("open", "Opening gripper"),
            ("close", "Closing gripper"),
            ("custom:128,128,128", "Moving to 50% position"),
            ("stop", "Stopping gripper"),
        ]

        for command, description in commands:
            rospy.loginfo(description)

            try:
                # Call service
                response = gripper_service(command)
                rospy.loginfo(
                    f"Response: success={response.success}, "
                    f"message={response.message}"
                )
                rospy.sleep(2)  # Wait for action to complete

            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
                return

        rospy.loginfo("Service test completed!")

    except rospy.ROSInterruptException:
        rospy.logwarn("Test interrupted!")


if __name__ == "__main__":
    try:
        test_gripper_service()
    except rospy.ROSInterruptException:
        pass
