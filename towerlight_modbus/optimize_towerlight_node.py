import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException, ConnectionException
import logging
import time
import atexit

logging.getLogger('pymodbus').setLevel(logging.CRITICAL)


class ModbusNode(Node):
    def __init__(self):
        super().__init__('modbus_node')

        self.client = ModbusSerialClient(
            port='/dev/towerlight', baudrate=9600, parity='N', stopbits=1, bytesize=8, timeout=2
        )
        self.connected = self.client.connect()

        self.timer = self.create_timer(3.0, self.read_modbus_data)
        self.reconnect_timer = self.create_timer(3.0, self.reconnect)

        self.subscription = self.create_subscription(
            Int32, 'monitor_topic', self.listener_callback, 10)

        atexit.register(self.clear_all)

        if self.connected:
            self.enable_modbus_rtu()
            self.initial_light_setup()
        else:
            self.get_logger().error("❌ Failed to connect to Modbus device")

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

    def initial_light_setup(self):
        """Turn on initial lights and buzzer sequence"""
        slave_id = 1
        self.set_light(0x004, 0x002, slave_id)  # Red LED
        # self.set_light(0x005, 0x002, slave_id)  # Buzzer
        self.get_logger().info("✅ Red LED and Buzzer turned on.")
        time.sleep(5)
        self.set_light(0x004, 0x000, slave_id)  # Turn off Red LED
        self.set_light(0x002, 0x002, slave_id)  # Turn on Green LED
        # self.set_light(0x005, 0x002, slave_id)  # Buzzer on again
        self.get_logger().info("✅ Green LED and Buzzer turned on.")

    def reconnect(self):
        """Attempt to reconnect if disconnected"""
        if self.connected:
            return
        self.get_logger().info("🔄 Attempting to reconnect...")
        self.connected = self.client.connect()
        if self.connected:
            self.get_logger().info("✅ Reconnected to Modbus device")
            self.initial_light_setup()
        else:
            self.get_logger().error("❌ Reconnection failed")

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
        except Exception as e:
            self.get_logger().error(f"❌ Unexpected Error: {e}")
            self.connected = False

    def set_light(self, address, value, slave_id=1):
        """Write a value to a Modbus register"""
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
        # self.set_light(0x005, 0x000, slave_id)  # Turn off Buzzer
        self.get_logger().info("✅ All lights and buzzer cleared.")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received value: {msg.data}")
        self.control_modbus(msg.data)

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
            # time.sleep(0.5)
        elif value == 2:
            write_color_and_buzzer(yellow_addr, color_values, buzzer_values)
            # time.sleep(0.5)
        elif value == 3:
            write_color_and_buzzer(red_addr, color_values, buzzer_values)
            # time.sleep(0.5)
        elif value == 0:
            self.clear_all()
            # time.sleep(0.5)


def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
