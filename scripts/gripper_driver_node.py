#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from robotiq_hand_e_control.msg import GripperStatus
from pymodbus.client import ModbusSerialClient


class RobotiqGripperModbus:
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
            rospy.logerr("❌ Không kết nối được với gripper qua Modbus RTU")
            #exit(1)
        rospy.loginfo("✅ Đã kết nối với gripper tại %s", self.port)

        # ROS interface
        rospy.Subscriber("/gripper_command_string", String, self.command_callback)
        self.status_pub = rospy.Publisher("/gripper_status", String, queue_size=10)
        rospy.Timer(rospy.Duration(1.0), self.publish_status)

    def command_callback(self, msg):
        cmd = msg.data.strip().lower()
        try:
            if cmd == "activate":
                self.client.write_register(0x03E8, 0x0001, unit=self.unit_id)
                rospy.loginfo("▶️ Lệnh: Activate")

            elif cmd == "reset":
                self.client.write_register(0x03E8, 0x0000, unit=self.unit_id)
                rospy.loginfo("🔄 Lệnh: Reset")

            elif cmd == "open":
                self.client.write_registers(0x03EC, [0x0001, 0, 128, 50], unit=self.unit_id)
                rospy.loginfo("🟢 Lệnh: Open")

            elif cmd == "close":
                self.client.write_registers(0x03EC, [0x0001, 255, 128, 100], unit=self.unit_id)
                rospy.loginfo("🔴 Lệnh: Close")

            elif cmd == "stop":
                self.client.write_register(0x03E9, 0x0000, unit=self.unit_id)
                rospy.loginfo("⛔ Lệnh: Stop")

            elif cmd.startswith("custom:"):
                parts = cmd.replace("custom:", "").split(",")
                pos, spd, frc = int(parts[0]), int(parts[1]), int(parts[2])
                self.client.write_registers(0x03EC, [0x0001, pos, spd, frc], unit=self.unit_id)
                rospy.loginfo(f"⚙️ Lệnh: Custom Move: pos={pos}, spd={spd}, frc={frc}")

            elif cmd == "read":
                self.publish_status(None)
                rospy.loginfo("📥 Lệnh: Đọc trạng thái")

            else:
                rospy.logwarn("⚠️ Lệnh không hợp lệ: %s", cmd)

        except Exception as e:
            rospy.logerr(f"❌ Lỗi gửi lệnh: {str(e)}")

    def publish_status(self, _):
        try:
            rr = self.client.read_holding_registers(0x07D0, 6, unit=self.unit_id)
            if rr.isError():
                rospy.logwarn("⚠️ Không đọc được trạng thái gripper")
                return

            reg = rr.registers
            gSTA = (reg[0] >> 6) & 0x03
            gOBJ = reg[0] & 0x03
            gFLT = reg[2]
            gPOS = reg[3]
            gCUR = reg[4]

            sta_dict = {0: "Reset", 1: "Activated", 2: "Ready", 3: "Fault"}
            obj_dict = {0: "Moving", 1: "Stopped(outer)", 2: "Stopped(inner)", 3: "At rest"}
            flt_dict = {
                0: "No Fault", 5: "Overcurrent", 7: "Internal Fault",
                8: "Under-voltage", 9: "Automatic Release", 10: "Reset Fault"
            }

            msg = GripperStatus()
            msg.gSTA = gSTA
            msg.gOBJ = gOBJ
            msg.gFLT = gFLT
            msg.gPOS = gPOS
            msg.gCUR = gCUR
            msg.sta_text = sta_dict.get(gSTA, f"Unknown({gSTA})")
            msg.obj_text = obj_dict.get(gOBJ, f"Unknown({gOBJ})")
            msg.flt_text = flt_dict.get(gFLT, f"Code {gFLT}")
            self.status_pub.publish(msg)

        except Exception as e:
            rospy.logerr(f"❌ Lỗi đọc trạng thái: {str(e)}")

if __name__ == "__main__":
    rospy.init_node("gripper_modbus_driver_node")
    RobotiqGripperModbus()
    rospy.spin()
