#!/usr/bin/env python3

import rospy
from robotiq_hand_e_control.srv import GripperControl, GripperControlResponse
from pyRobotiqGripper.driver import RobotiqGripper

gripper = None

def handle_gripper_control(req):
    global gripper
    cmd = req.command.lower()
    a1, a2, a3 = req.arg1, req.arg2, req.arg3

    try:
        if cmd == "activate":
            gripper.activate()
            return GripperControlResponse(True, "Gripper activated.")
        elif cmd == "reset":
            gripper.reset()
            return GripperControlResponse(True, "Gripper reset.")
        elif cmd == "resetactivate":
            gripper.resetActivate()
            return GripperControlResponse(True, "Reset + Activate complete.")
        elif cmd == "open":
            gripper.open(int(a1), int(a2))
            return GripperControlResponse(True, "Gripper opened.")
        elif cmd == "close":
            gripper.close(int(a1), int(a2))
            return GripperControlResponse(True, "Gripper closed.")
        elif cmd == "goto":
            pos, detected = gripper.goTo(int(a1), int(a2), int(a3))
            msg = f"Gripper moved to {pos}. Object detected: {detected}"
            return GripperControlResponse(True, msg)
        elif cmd == "gotomm":
            gripper.goTomm(float(a1), int(a2), int(a3))
            return GripperControlResponse(True, f"Gripper moved to {a1}mm.")
        elif cmd == "getpos":
            pos = gripper.getPosition()
            return GripperControlResponse(True, f"Position (bits): {pos}")
        elif cmd == "getposmm":
            posmm = gripper.getPositionmm()
            return GripperControlResponse(True, f"Position (mm): {posmm:.2f}")
        elif cmd == "isactivated":
            return GripperControlResponse(gripper.isActivated(), "Checked activation.")
        elif cmd == "iscalibrated":
            return GripperControlResponse(gripper.isCalibrated(), "Checked calibration.")
        elif cmd == "calibrate":
            gripper.calibrate(a1, a2)
            return GripperControlResponse(True, f"Calibrated: closemm={a1}, openmm={a2}")
        else:
            return GripperControlResponse(False, f"Unknown command: {cmd}")
    except Exception as e:
        return GripperControlResponse(False, str(e))

def gripper_service_server():
    global gripper
    rospy.init_node('gripper_service_server')
    gripper = RobotiqGripper()
    rospy.Service('gripper_command', GripperControl, handle_gripper_control)
    rospy.loginfo("Gripper service ready.")
    rospy.spin()

if __name__ == "__main__":
    gripper_service_server()
