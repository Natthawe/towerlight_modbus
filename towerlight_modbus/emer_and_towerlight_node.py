import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException, ConnectionException
import logging
import time
import atexit
from rclpy.timer import Timer

logging.getLogger('pymodbus').setLevel(logging.CRITICAL)


class ModbusNode(Node):
    def __init__(self):
        super().__init__('modbus_node')

        self.client = ModbusSerialClient(
            port='/dev/towerlight', baudrate=9600, parity='N', stopbits=1, bytesize=8, timeout=2
        )
        self.connected = self.client.connect()

        self.subscription = self.create_subscription(
            Int32, 'monitor_topic', self.listener_callback, 10)

        self.emergency_sub = self.create_subscription(
            Bool, 'emergency_stop', self.emergency_callback, 10)

        if self.connected:
            self.enable_modbus_rtu()
            self.timer = self.create_timer(5.0, self.turn_green)
            self.set_light(0x004, 0x002, 1)  # เปิดไฟแดง
        else:
            self.get_logger().error("❌ Failed to connect to Modbus device")

        self.reconnect_timer = self.create_timer(3.0, self.reconnect)
        self.read_data_timer = self.create_timer(3.0, self.read_modbus_data)

        atexit.register(self.clear_all)

        self.emergency_active = True

        self.get_logger().info(f"Emergency State: {self.emergency_active}")

    def enable_modbus_rtu(self):
        """Enable MODBUS RTU communication"""
        enable_address, enable_value, slave_id = 0x006, 0x001, 1

        while True:
            try:
                response = self.client.write_register(enable_address, enable_value, slave=slave_id)
                if response.isError():
                    self.get_logger().error("❌ Failed to enable MODBUS RTU")
                    time.sleep(1)
                else:
                    self.get_logger().info("✅ MODBUS RTU enabled")
                    break
            except (ModbusIOException, ConnectionException) as e:
                self.get_logger().error(f"❌ Connection Error: {e}")
                time.sleep(1)
        time.sleep(2)

    def reconnect(self):
        """Attempt to reconnect if disconnected"""
        if self.connected:
            return

        self.get_logger().info("🔄 Attempting to reconnect...")
        try:
            self.client.close()
            self.connected = self.client.connect()
            if self.connected:
                self.get_logger().info("✅ Reconnected to Modbus device")
                self.enable_modbus_rtu()
                self.set_light(0x004, 0x002, 1)  # เปิดไฟแดง
                time.sleep(5)
                self.set_light(0x004, 0x000, 1)  # ปิดไฟแดง
                self.set_light(0x002, 0x002, 1)  # เปิดไฟเขียว
                self.read_data_timer.cancel()  # ยกเลิก timer ก่อนหน้า
                self.read_data_timer = self.create_timer(
                    3.0, self.read_modbus_data)  # สร้าง timer ใหม่
            else:
                self.get_logger().error("❌ Reconnection failed")
        except Exception as e:
            self.get_logger().error(f"❌ Reconnection attempt failed: {e}")
            self.connected = False

    def read_modbus_data(self):
        """Read and log Modbus data if connected"""
        if not self.connected:
            self.get_logger().warn("⚠️ Tower Light is disconnected!")
            return

        try:
            response = self.client.read_holding_registers(address=0x000, count=10, slave=1)
            if response.isError():
                raise ModbusIOException("Modbus response error")
        except (ModbusIOException, ConnectionException) as e:
            self.get_logger().error(f"❌ Modbus Error: {e}")
            self.connected = False
            self.reconnect()
        except Exception as e:
            self.get_logger().error(f"❌ Unexpected Error: {e}")
            self.connected = False
            self.reconnect()

    def turn_green(self):
        """เปลี่ยนไฟเป็นสีเขียวหลังจาก 5 วินาที"""
        self.set_light(0x004, 0x000, 1)  # ปิดไฟแดง
        self.set_light(0x002, 0x002, 1)  # เปิดไฟเขียว
        self.timer.cancel()  # ยกเลิกไทม์เมอร์หลังจากทำงานครั้งแรก

    def set_light(self, address, value, slave_id=1):
        """Write a value to a Modbus register"""
        if not self.connected:  # เพิ่มการตรวจสอบการเชื่อมต่อ
            self.get_logger().error("❌ Device not connected")
            return
        try:
            response = self.client.write_register(address, value, slave=slave_id)
            if response.isError():
                self.get_logger().error(f"❌ Failed to set register {address:#05x}")
        except (ModbusIOException, ConnectionException) as e:
            self.get_logger().error(f"❌ Modbus Error: {e}")

    def clear_all(self):
        """Turn off all LEDs and Buzzer"""
        slave_id = 1
        for address in [0x002, 0x003, 0x004, 0x005]:
            self.set_light(address, 0x000, slave_id)
        self.get_logger().info("✅ All lights cleared.")

    def listener_callback(self, msg):
        if self.emergency_active:  # เพิ่มเงื่อนไขตรวจสอบ
            return
        self.get_logger().info(f"Received value: {msg.data}")
        self.control_modbus(msg.data)

    def emergency_callback(self, msg):
        new_emergency_state = msg.data
        if new_emergency_state != self.emergency_active:
            self.emergency_active = new_emergency_state

            if self.emergency_active:
                self.get_logger().error("🛑 EMERGENCY ACTIVATED!")
                self.get_logger().info(f"Emergency State: {self.emergency_active}")
                self.timer_emergency = self.create_timer(1.0, self.emergency_activated)
            else:
                self.get_logger().info("✅ EMERGENCY DEACTIVATED")
                self.get_logger().info(f"Emergency State: {self.emergency_active}")
                self.timer_emergency = self.create_timer(1.0, self.emergency_deactivated)

    def emergency_activated(self):
        self.clear_all()
        self.set_light(0x004, 0x002, 1)  # ไฟแดง
        # self.set_light(0x005, 0x002, 1)  # ออด
        self.timer_emergency.cancel()

    def emergency_deactivated(self):
        self.clear_all()
        self.set_light(0x002, 0x002, 1)  # ไฟเขียว
        # self.set_light(0x005, 0x002, 1)  # ออด
        self.timer_emergency.cancel()

    def control_modbus(self, value):
        slave_id = 1
        green_addr = 0x002
        yellow_addr = 0x003
        red_addr = 0x004
        buzzer_addr = 0x005
        color_values = 0x002
        buzzer_values = 0x002

        def write_color_and_buzzer(color_address, color_value, buzzer_value):
            color_response = self.client.write_register(color_address, color_value, slave=slave_id)
            buzzer_response = self.client.write_register(buzzer_addr, buzzer_value, slave=slave_id)

            if color_response.isError():
                self.get_logger().error(
                    f"❌ Error writing to color register at {
                        color_address:#05x}")
            else:
                self.get_logger().info(
                    f"✅ Successfully wrote {color_value} to color register {
                        color_address:#05x}")

            if buzzer_response.isError():
                self.get_logger().error("❌ Error writing to Buzzer register at 0x005")
            else:
                self.get_logger().info(
                    f"✅ Successfully wrote {buzzer_value} to Buzzer register 0x005")

        # Turn off the lights and buzzer first.
        self.clear_all()

        time.sleep(1.5)

        # Control LED and Buzzer according to received values
        if value == 1:
            write_color_and_buzzer(green_addr, color_values, buzzer_values)
        elif value == 2:
            write_color_and_buzzer(yellow_addr, color_values, buzzer_values)
        elif value == 3:
            write_color_and_buzzer(red_addr, color_values, buzzer_values)
        elif value == 0:
            self.clear_all()


def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
