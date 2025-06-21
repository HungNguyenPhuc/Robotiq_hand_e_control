#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from pymodbus.client import ModbusSerialClient


class GripperDriverNode:
    def __init__(self):
        # Kết nối Modbus
        self.client = ModbusSerialClient(
            method='rtu',
            port=rospy.get_param("~port", "/dev/ttyUSB0"),
            baudrate=rospy.get_param("~baudrate", 115200),
            timeout=1
        )
        self.unit_id = 0x09

        if not self.client.connect():
            rospy.logerr("❌ Không thể kết nối gripper")
            exit(1)
        rospy.loginfo("✅ Kết nối gripper thành công")

        # ROS interface
        rospy.Subscriber("/gripper_command_string", String, self.command_callback)
        self.status_pub = rospy.Publisher("/gripper_status", String, queue_size=10)

        # Đọc trạng thái định kỳ
        rospy.Timer(rospy.Duration(1.0), self.publish_status)

    def command_callback(self, msg):
        cmd = msg.data.strip().lower()
        try:
            if cmd == "activate":
                self.client.write_register(0x03E8, 0x0001, unit=self.unit_id)
                rospy.loginfo("▶️ Gửi lệnh: activate")

            elif cmd == "reset":
                self.client.write_register(0x03E8, 0x0000, unit=self.unit_id)
                rospy.loginfo("🔄 Gửi lệnh: reset")

            elif cmd == "open":
                self.client.write_registers(0x03EC, [0x0001, 0x00, 150, 50], unit=self.unit_id)
                rospy.loginfo("🟢 Gửi lệnh: open")

            elif cmd == "close":
                self.client.write_registers(0x03EC, [0x0001, 0xFF, 150, 100], unit=self.unit_id)
                rospy.loginfo("🔴 Gửi lệnh: close")

            elif cmd == "stop":
                self.client.write_register(0x03E9, 0x0000, unit=self.unit_id)
                rospy.loginfo("⛔ Gửi lệnh: stop")

            elif cmd.startswith("custom:"):
                parts = cmd.replace("custom:", "").split(",")
                pos, spd, frc = int(parts[0]), int(parts[1]), int(parts[2])
                self.client.write_registers(0x03EC, [0x0001, pos, spd, frc], unit=self.unit_id)
                rospy.loginfo(f"⚙️ Gửi custom lệnh: pos={pos}, spd={spd}, frc={frc}")

            elif cmd == "read":
                self.publish_status(None)
                rospy.loginfo("📥 Đọc trạng thái ngay")

            else:
                rospy.logwarn(f"⚠️ Lệnh không hợp lệ: {cmd}")

        except Exception as e:
            rospy.logerr(f"❌ Lỗi khi xử lý lệnh '{cmd}': {str(e)}")

    def publish_status(self, _):
        try:
            rr = self.client.read_holding_registers(0x07D0, 6, unit=self.unit_id)
            if rr.isError():
                self.status_pub.publish("⚠️ Lỗi đọc trạng thái")
                return

            reg = rr.registers
            gSTA = (reg[0] >> 6) & 0x03
            gOBJ = reg[0] & 0x03
            gFLT = reg[2]
            gPOS = reg[3]
            gCUR = reg[4]

            # Diễn giải trạng thái
            sta_dict = {0: "Reset", 1: "Activated", 2: "Ready", 3: "Fault"}
            obj_dict = {0: "No object", 1: "Moving", 2: "Soft object", 3: "Hard object"}
            flt_dict = {
                0: "No Fault", 5: "Overcurrent", 10: "Internal Fault", 11: "Under-voltage"
            }

            sta_text = sta_dict.get(gSTA, f"Unknown({gSTA})")
            obj_text = obj_dict.get(gOBJ, f"Unknown({gOBJ})")
            flt_text = flt_dict.get(gFLT, f"Code {gFLT}")

            msg = f"[STATUS] {sta_text} | OBJ: {obj_text} | POS: {gPOS} | CUR: {gCUR} | FAULT: {flt_text}"
            self.status_pub.publish(msg)

        except Exception as e:
            self.status_pub.publish("❌ Lỗi Modbus: " + str(e))



if __name__ == "__main__":
    rospy.init_node("gripper_driver_node")
    GripperDriverNode()
    rospy.spin()
