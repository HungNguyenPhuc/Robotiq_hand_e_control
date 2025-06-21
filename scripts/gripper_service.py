#!/usr/bin/env python3
import rospy
from pymodbus.client import ModbusSerialClient
from robotiq_hand_e_control.srv import GripperControl, GripperControlResponse


class RobotiqHandEService:
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~baudrate", 115200)
        self.unit_id = rospy.get_param("~unit_id", 0x09)
        self.client = ModbusSerialClient(
            method='rtu',
            port=self.port,
            baudrate=self.baudrate,
            timeout=1,
            stopbits=1,
            bytesize=8,
            parity='N'
        )

        if not self.client.connect():
            rospy.logerr("Không thể kết nối với gripper")
            exit(1)

        rospy.Service("gripper_control", GripperControl, self.handle_command)
        rospy.loginfo("Đã sẵn sàng nhận lệnh điều khiển gripper")

    def handle_command(self, req):
        cmd = req.command.strip().lower()
        try:
            if cmd == "activate":
                self.client.write_register(0x03E8, 0x0001, unit=self.unit_id)
            elif cmd == "reset":
                self.client.write_register(0x03E8, 0x0000, unit=self.unit_id)
            elif cmd == "open":
                self.client.write_registers(0x03EC, [0x0001, 0, 128, 50], unit=self.unit_id)
            elif cmd == "close":
                self.client.write_registers(0x03EC, [0x0001, 255, 128, 100], unit=self.unit_id)
            elif cmd == "stop":
                self.client.write_register(0x03E9, 0x0000, unit=self.unit_id)
            elif cmd.startswith("custom:"):
                parts = cmd.replace("custom:", "").split(",")
                pos, spd, frc = int(parts[0]), int(parts[1]), int(parts[2])
                self.client.write_registers(0x03EC, [0x0001, pos, spd, frc], unit=self.unit_id)
            else:
                return GripperCommandResponse(False, "Invalid command")

            return GripperCommandResponse(True, "Command sent successfully")

        except Exception as e:
            return GripperCommandResponse(False, f"Error: {str(e)}")


if __name__ == "__main__":
    rospy.init_node("robotiq_hand_e_service_node")
    RobotiqHandEService()
    rospy.spin()

