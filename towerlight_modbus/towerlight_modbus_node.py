import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from pymodbus.client import ModbusSerialClient
import atexit
import time
from pymodbus.exceptions import ModbusIOException, ConnectionException
import logging

logging.getLogger('pymodbus').setLevel(logging.CRITICAL)


class ModbusNode(Node):
    def __init__(self):
        super().__init__('modbus_node')

        # Create Client for connection Modbus
        self.client = ModbusSerialClient(
            port='/dev/ttyUSB0',
            baudrate=9600,
            parity='N',
            stopbits=1,
            bytesize=8,
            timeout=2
        )

        self.connected = False  # status connection
        self.check_connection()  # check connection
        self.timer = self.create_timer(3.0, self.read_modbus_data)  # read data every 3 seconds
        self.reconnect_timer = self.create_timer(
            3.0, self.reconnect)  # read data every 3 seconds

        # Subscribe to topic repeat_topic
        self.subscription = self.create_subscription(
            Int32,
            'repeat_topic',
            self.listener_callback,
            10
        )

        # define clear_all function to be called when the node is shutdown
        atexit.register(self.clear_all)

    def check_connection(self):
        """Check Connection Tower Light"""
        # Connecting to Modbus
        if self.client.connect():
            self.get_logger().info("✅ Connecting to Modbus device . . .")

            # Enable MODBUS RTU
            enable_address = 0x006
            enable_value = 0x001
            slave_id = 1

            success = False
            while not success:
                try:
                    response = self.client.write_register(
                        enable_address, enable_value, slave=slave_id)
                    if response.isError():
                        self.get_logger().error("❌ Failed to enable MODBUS RTU")
                        time.sleep(1)
                    else:
                        self.get_logger().info("✅ Connected to Modbus device")
                        self.get_logger().info(
                            f"✅ Enabled MODBUS RTU: wrote {enable_value} to register {
                                enable_address:#05x}")
                        success = True
                except ModbusIOException as e:
                    self.get_logger().error(f"❌ Modbus IO Error: {e}")
                    time.sleep(1)
                except ConnectionException as e:
                    self.get_logger().error(f"❌ Connection Error: {e}")
                    time.sleep(1)

            time.sleep(2)

            # read value from Address 0x000 - 0x009 on start node
            # self.read_modbus_registers(0x000, 10, slave_id)

            # open red led color + Buzzer on start node
            red_addr = 0x004
            buzzer_addr = 0x005
            init_color_value = 0x002
            init_buzzer_value = 0x002

            red_response = self.client.write_register(red_addr, init_color_value, slave=slave_id)
            buzzer_response = self.client.write_register(
                buzzer_addr, init_buzzer_value, slave=slave_id)

            if red_response.isError():
                self.get_logger().error(f"❌ Failed to set Red LED at {red_addr:#05x}")
            else:
                self.get_logger().info(f"✅ Red LED turned on at {red_addr:#05x}")

            if buzzer_response.isError():
                self.get_logger().error("❌ Failed to turn on Buzzer at 0x005")
            else:
                self.get_logger().info("✅ Buzzer turned on at 0x005")

        else:
            self.get_logger().error("❌ Failed to connect")
            return

        if not self.client.connect():
            self.get_logger().error("❌ Failed to connect to Tower Light!")
            self.connected = False
        else:
            self.get_logger().info("✅ Connected to Modbus device")
            self.connected = True

    def reconnect(self):
        """Attempt to reconnect when device disconnects"""
        if self.connected:
            return  # If already connected, no need to reconnect.
        else:
            self.check_connection()  # Check connection

        self.get_logger().info("🔄 Attempting to reconnect to Tower Light . . .")

        if self.client.connect():
            self.get_logger().info("✅ Reconnected to Tower Light!")
            self.connected = True

            # Turn on the red light + turn on the Buzzer immediately after successful connection.
            red_addr = 0x004
            buzzer_addr = 0x005
            color_value = 0x002
            buzzer_value = 0x002

            time.sleep(2)

            red_response = self.client.write_register(red_addr, color_value, slave=1)
            buzzer_response = self.client.write_register(buzzer_addr, buzzer_value, slave=1)

            if red_response.isError():
                self.get_logger().error(f"❌ Failed to set Red LED at {red_addr:#05x}")
            else:
                self.get_logger().info(f"✅ Red LED turned on at {red_addr:#05x}")

            if buzzer_response.isError():
                self.get_logger().error("❌ Failed to turn on Buzzer at 0x005")
            else:
                self.get_logger().info("✅ Buzzer turned on at 0x005")
        else:
            self.get_logger().warn("⚠️ Reconnection failed, retrying in 5 seconds . . .")

    def read_modbus_data(self):
        """ Read values from Tower Light and check status """
        if not self.connected:
            self.get_logger().warn("⚠️ Tower Light is disconnected!")
            return

        try:
            response = self.client.read_holding_registers(address=0x000, count=10, slave=1)
            if response.isError():
                raise ModbusIOException("Modbus response error")

            # # Read values from registers and display them.
            # for i, value in enumerate(response.registers):
            #     self.get_logger().info(f"📖 Read address 0x{hex(i)}: {value}")

        except ModbusIOException as e:
            self.get_logger().error(f"❌ Modbus IO Error: {e}")
            self.connected = False  # Set the status of the device that is disconnected.

        except ConnectionException as e:
            self.get_logger().error(f"❌ Modbus Error: {e}")
            self.connected = False
        except Exception as e:
            self.get_logger().error(f"❌ Unexpected Error: {e}")
            self.connected = False

    def read_modbus_registers(self, start_address=0x000, count=10, slave_id=1):
        """อ่านค่าจาก Modbus Holding Registers"""
        try:
            # response = self.client.read_holding_registers(start_address, count, slave=slave_id)
            response = self.client.read_holding_registers(
                start_address, count=count, slave=slave_id)

            if response.isError():
                self.get_logger().error(
                    f"❌ Failed to read registers {start_address:#05x} - {start_address + count - 1:#05x}")
            else:
                values = response.registers
                for i, value in enumerate(values):
                    self.get_logger().info(f"📖 Read address {start_address + i:#05x}: {value}")
        except ModbusIOException as e:
            self.get_logger().error(f"❌ Modbus IO Error: {e}")
        except ConnectionException as e:
            self.get_logger().error(f"❌ Connection Error: {e}")

    def clear_all(self):
        """Turn off all LEDs and Buzzers"""
        color_addresses = [0x002, 0x003, 0x004]  # Green, Yellow, Red
        slave_id = 1

        for address in color_addresses:
            response = self.client.write_register(address, 0x000, slave=slave_id)
            if response.isError():
                self.get_logger().error(f"❌ Error clearing color register at {address:#05x}")
            else:
                self.get_logger().info(f"✅ Cleared color register at {address:#05x}")

        # Mute Buzzer
        buzzer_response = self.client.write_register(0x005, 0x000, slave=slave_id)
        if buzzer_response.isError():
            self.get_logger().error("❌ Error clearing Buzzer register at 0x005")
        else:
            self.get_logger().info("✅ Cleared Buzzer register 0x005")

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
