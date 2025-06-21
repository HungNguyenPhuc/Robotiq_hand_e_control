#!/usr/bin/env python3
import rospy
from pymodbus.client import ModbusSerialClient
from robotiq_hand_e_control.srv import GripperControl, GripperControlResponse
from robotiq_hand_e_control.msg import GripperCommand


class RobotiqHandEService:
    def __init__(self):
        self.client = ModbusSerialClient(
            method='rtu',
            port='/dev/ttyUSB0',
            baudrate=115200,
            timeout=1
        )
        self.unit_id = 0x09

        if not self.client.connect():
            rospy.logerr("Không thể kết nối với gripper")
            exit(1)

        rospy.Service("gripper_control", GripperControl, self.handle_command)
        rospy.loginfo("Đã sẵn sàng nhận lệnh điều khiển gripper")

    def handle_command(self, req):
        cmd = req.command.command_type.lower()
        pos = req.command.position
        spd = req.command.speed
        frc = req.command.force

        try:
            if cmd == "activate":
                self.client.write_register(0x03E8, 0x0001, unit=self.unit_id)
            elif cmd == "open":
                self.client.write_registers(0x03EC, [0x0001, 0x00, spd, frc], unit=self.unit_id)
            elif cmd == "close":
                self.client.write_registers(0x03EC, [0x0001, 0xFF, spd, frc], unit=self.unit_id)
            else:
                return GripperControlResponse(False, f"Lệnh không hợp lệ: {cmd}")
        except Exception as e:
            return GripperControlResponse(False, str(e))

        return GripperControlResponse(True, f"Đã gửi lệnh '{cmd}' thành công")

if __name__ == "__main__":
    rospy.init_node("robotiq_hand_e_service_node")
    RobotiqHandEService()
    rospy.spin()

