#!/usr/bin/env python3
import rospy
from robotiq_hand_e_control.msg import GripperCommand, GripperStatus
from pyRobotiqGripper.driver import RobotiqGripper
import traceback
import threading

class GripperDriverNode:
    def __init__(self):
        rospy.init_node("gripper_driver_node")
        
        # Initialize gripper
        self.gripper = RobotiqGripper()
        
        # Initialize subscriber for commands
        rospy.Subscriber("/gripper_command_topic", GripperCommand, self.command_callback)
        
        # Initialize publisher for status
        self.status_pub = rospy.Publisher('/robotiq_gripper/status', GripperStatus, queue_size=10)
        
        # Status publishing rate
        self.status_rate = rospy.Rate(5)  # 5 Hz
        
        # Activate gripper
        try:
            self.gripper.activate()
            rospy.loginfo("Gripper activated successfully")
        except Exception as e:
            rospy.logerr(f"Failed to activate gripper: {e}")
        
        rospy.loginfo("Gripper driver node ready.")
        
        # Start status publishing thread
        self.status_thread = threading.Thread(target=self.publish_status_loop)
        self.status_thread.daemon = True
        self.status_thread.start()
        
        # Keep node running
        rospy.spin()
    
    def command_callback(self, msg):
        """Handle gripper commands"""
        cmd = msg.command.lower()
        a1, a2, a3 = msg.arg1, msg.arg2, msg.arg3
        
        try:
            if cmd == "open":
                self.gripper.open(speed=int(a1), force=int(a2))
                rospy.loginfo(f"Gripper opened with speed={a1}, force={a2}")
            elif cmd == "close":
                self.gripper.close(speed=int(a1), force=int(a2))
                rospy.loginfo(f"Gripper closed with speed={a1}, force={a2}")
            elif cmd == "goto":
                self.gripper.goTo(position=int(a1), speed=int(a2), force=int(a3))
                rospy.loginfo(f"Gripper goto position={a1}, speed={a2}, force={a3}")
            elif cmd == "gopos":
                self.gripper.goTo(position=int(a1))
                rospy.loginfo(f"Gripper goto position={a1}")
            elif cmd == "goposmm":
                self.gripper.goTomm(positionmm=float(a1), speed=int(a2), force=int(a3))
                rospy.loginfo(f"Gripper goto {a1}mm with speed={a2}, force={a3}")
            elif cmd == "getpos":
                pos = self.gripper.getPosition()
                rospy.loginfo(f"Position (bit): {pos}")
            elif cmd == "getposmm":
                posmm = self.gripper.getPositionmm()
                rospy.loginfo(f"Position (mm): {posmm}")
            elif cmd == "reset":
                self.gripper.reset()
                rospy.loginfo("Gripper reset")
            elif cmd == "activate":
                self.gripper.activate()
                rospy.loginfo("Gripper activated")
            elif cmd == "resetactivate":
                self.gripper.resetActivate()
                rospy.loginfo("Gripper reset and activated")
            elif cmd == "calibrate":
                self.gripper.calibrate(closemm=float(a1), openmm=float(a2))
                rospy.loginfo(f"Gripper calibrated: close={a1}mm, open={a2}mm")
            elif cmd == "reset_calib":
                self.gripper.resetCalibration()
                rospy.loginfo("Gripper calibration reset")
            elif cmd == "info":
                self.gripper.printInfo()
            else:
                rospy.logwarn(f"[GripperDriverNode] Unknown command: {cmd}")
                
        except Exception as e:
            rospy.logerr(f"[GripperDriverNode] Failed command '{cmd}': {e}")
            rospy.logdebug(traceback.format_exc())
    
    def publish_status_loop(self):
        """Continuously publish gripper status"""
        while not rospy.is_shutdown():
            try:
                # Read all gripper parameters
                self.gripper.readAll()
                
                # Create status message
                status = GripperStatus()
                
                # Fill status with all parameters
                for key in self.gripper.paramDic:
                    value = self.gripper.paramDic[key]
                    desc = str(self.gripper.registerDic.get(key, {}).get(value, "N/A"))
                    setattr(status, key, value)
                    setattr(status, f"{key}_desc", desc)
                
                # Special handling for gCU (motor current) - scale to mA
                if "gCU" in self.gripper.paramDic:
                    status.gCU = self.gripper.paramDic["gCU"] * 10
                    status.gCU_desc = str(self.gripper.registerDic["gCU"].get(self.gripper.paramDic["gCU"], "N/A"))
                
                # Publish status
                self.status_pub.publish(status)
                
            except Exception as e:
                rospy.logerr(f"[GripperDriverNode] Failed to publish status: {e}")
                rospy.logdebug(traceback.format_exc())
            
            # Sleep according to rate
            self.status_rate.sleep()

if __name__ == "__main__":
    try:
        GripperDriverNode()
    except rospy.ROSInterruptException:
        rospy.loginfo("Gripper driver node interrupted")
    except Exception as e:
        rospy.logerr(f"Gripper driver node failed: {e}")
        rospy.logdebug(traceback.format_exc())