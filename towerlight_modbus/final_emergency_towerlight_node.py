#!/usr/bin/env python3
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

        # ==========================
        # Declare Parameters
        # ==========================
        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial_port', '/dev/towerlight'),
                ('baudrate', 9600),
                ('parity', 'N'),
                ('stopbits', 1),
                ('bytesize', 8),
                ('timeout', 2.0),
                ('slave_id', 1),

                ('reconnect_period', 3.0),
                ('read_period', 1.0),

                ('emergency_topic', 'emergency_stop'),
                ('monitor_topic', 'monitor_topic'),

                ('default_emergency_active', True),   # เริ่มต้น emergency = True
                ('auto_green_on_clear', True),        # ปลด emergency แล้วขึ้นเขียว
                ('clear_all_on_shutdown', True),      # ตอนปิด node ให้ clear ไฟ
                ('log_modbus_status', False),         # log ค่า register ทุกครั้ง
            ]
        )

        # อ่านค่าพารามิเตอร์
        port      = self.get_parameter('serial_port').value
        baudrate  = self.get_parameter('baudrate').value
        parity    = self.get_parameter('parity').value
        stopbits  = self.get_parameter('stopbits').value
        bytesize  = self.get_parameter('bytesize').value
        timeout   = self.get_parameter('timeout').value
        self.slave_id = self.get_parameter('slave_id').value

        self.reconnect_period = float(self.get_parameter('reconnect_period').value)
        self.read_period      = float(self.get_parameter('read_period').value)

        emergency_topic_name = self.get_parameter('emergency_topic').value
        monitor_topic_name   = self.get_parameter('monitor_topic').value

        self.emergency_active: bool = bool(
            self.get_parameter('default_emergency_active').value
        )
        self.auto_green_on_clear: bool = bool(
            self.get_parameter('auto_green_on_clear').value
        )
        self.clear_all_on_shutdown: bool = bool(
            self.get_parameter('clear_all_on_shutdown').value
        )
        self.log_modbus_status: bool = bool(
            self.get_parameter('log_modbus_status').value
        )

        self.timer_emergency_check: Timer | None = None
        self.timer_triggered: bool = False

        # ----- Modbus Client -----
        self.client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize,
            timeout=timeout,
        )
        self.connected = self.client.connect()

        # ----- ROS Interfaces -----
        self.subscription = self.create_subscription(
            Int32,
            monitor_topic_name,
            self.listener_callback,
            10
        )
        self.emergency_sub = self.create_subscription(
            Bool,
            emergency_topic_name,
            self.emergency_callback,
            10
        )

        self.get_logger().info("===== ModbusNode Parameters =====")
        self.get_logger().info(f"  serial_port: {port}")
        self.get_logger().info(f"  baudrate   : {baudrate}")
        self.get_logger().info(f"  parity     : {parity}")
        self.get_logger().info(f"  stopbits   : {stopbits}")
        self.get_logger().info(f"  bytesize   : {bytesize}")
        self.get_logger().info(f"  timeout    : {timeout}")
        self.get_logger().info(f"  slave_id   : {self.slave_id}")
        self.get_logger().info(f"  reconnect_period   : {self.reconnect_period}")
        self.get_logger().info(f"  read_period        : {self.read_period}")
        self.get_logger().info(f"  emergency_topic    : {emergency_topic_name}")
        self.get_logger().info(f"  monitor_topic      : {monitor_topic_name}")
        self.get_logger().info(f"  default_emergency_active: {self.emergency_active}")
        self.get_logger().info(f"  auto_green_on_clear    : {self.auto_green_on_clear}")
        self.get_logger().info(f"  clear_all_on_shutdown  : {self.clear_all_on_shutdown}")
        self.get_logger().info(f"  log_modbus_status      : {self.log_modbus_status}")
        self.get_logger().info("=================================")

        self.get_logger().info(f"Emergency State (initial): {self.emergency_active}")

        # ----- Enable Modbus RTU -----
        if self.connected:
            self.enable_modbus_rtu()

            # อ่านสถานะครั้งแรกทันที ไม่ต้องรอ timer
            self.read_modbus_data()
        else:
            self.get_logger().error("❌ Failed to connect to Modbus device")

        # ----- Timers อื่น ๆ -----
        # เช็คการ reconnect ทุก reconnect_period วินาที
        self.reconnect_timer = self.create_timer(
            self.reconnect_period,
            self.reconnect
        )
        # อ่าน Modbus status ทุก ๆ read_period วินาที
        self.read_data_timer = self.create_timer(
            self.read_period,
            self.read_modbus_data
        )

        # ----- Clear all ตอนปิดโปรแกรม (ถ้าพารามิเตอร์อนุญาต) -----
        if self.clear_all_on_shutdown:
            atexit.register(self.clear_all)

    # ==========================
    # Modbus Low-level
    # ==========================

    def enable_modbus_rtu(self):
        """Enable MODBUS RTU communication"""
        enable_address, enable_value = 0x006, 0x001

        while True:
            try:
                response = self.client.write_register(
                    enable_address,
                    enable_value,
                    slave=self.slave_id
                )
                if response.isError():
                    self.get_logger().error("❌ Failed to enable MODBUS RTU")
                    time.sleep(1)
                else:
                    self.get_logger().info("✅ Tower Light is connected!")
                    self.get_logger().info("✅ MODBUS RTU enabled")
                    break
            except (ModbusIOException, ConnectionException) as e:
                self.get_logger().error(f"❌ Connection Error: {e}")
                time.sleep(1)

        # หน่วงให้ device มีเวลาเริ่มต้น
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
                self.get_logger().info("✅ Reconnected to Modbus device. Enabling RTU...")
                self.enable_modbus_rtu()

                # อ่านสถานะทันทีหลัง reconnect
                self.read_modbus_data()

                # รีสร้าง read_data_timer เผื่อก่อนหน้านี้มี error
                try:
                    self.read_data_timer.cancel()
                except Exception:
                    pass
                self.read_data_timer = self.create_timer(
                    self.read_period,
                    self.read_modbus_data
                )
            else:
                self.get_logger().error("❌ Reconnection failed")
        except Exception as e:
            self.get_logger().error(f"❌ Reconnection attempt failed: {e}")
            self.connected = False

    def read_modbus_data(self):
        """Read and log Modbus data if connected"""
        if not self.connected:
            self.get_logger().warn("⚠️ Tower Light is disconnected! (read_modbus_data)")
            return

        try:
            # อ่านจาก address 0x000 จำนวน 10 registers (0x000 - 0x009)
            response = self.client.read_holding_registers(
                address=0x000,
                count=10,
                slave=self.slave_id
            )
            if response.isError():
                raise ModbusIOException("Modbus response error")

            regs = response.registers
            base_addr = 0x000

            # name mapping สำหรับ register
            addr_names = {
                0x000: "WHITE",
                0x001: "BLUE",
                0x002: "GREEN",
                0x003: "YELLOW",
                0x004: "RED",
                0x005: "BUZZER",
                0x006: "MODBUS_RTU_ENABLE",
                0x007: "PARITY",
                0x008: "BAUDRATE",
                0x009: "ADDRESS",
            }

            lines = []
            for i, val in enumerate(regs):
                addr = base_addr + i
                name = addr_names.get(addr, "")
                label = f" ({name})" if name else ""
                # รูปแบบ:  0x002 (GREEN):   2 (0x0002)
                lines.append(
                    f"  0x{addr:03X}{label}: {val:5d} (0x{val:04X})"
                )

            log_msg = "📥 Modbus status:\n" + "\n".join(lines)

            if self.log_modbus_status:
                self.get_logger().info(log_msg)

        except (ModbusIOException, ConnectionException) as e:
            self.get_logger().error(f"❌ Modbus Error: {e}")
            self.connected = False
            self.reconnect()
        except Exception as e:
            self.get_logger().error(f"❌ Unexpected Error while reading Modbus: {e}")
            self.connected = False
            self.reconnect()

    def set_light(self, address, value):
        """Write a value to a Modbus register"""
        if not self.connected:
            self.get_logger().error("❌ Device not connected (set_light)")
            return
        try:
            response = self.client.write_register(address, value, slave=self.slave_id)
            if response.isError():
                self.get_logger().error(f"❌ Failed to set register {address:#05x}")
        except (ModbusIOException, ConnectionException) as e:
            self.get_logger().error(f"❌ Modbus Error in set_light: {e}")

    def clear_all(self):
        """Turn off all LEDs and Buzzer"""
        for address in [0x002, 0x003, 0x004, 0x005]:
            try:
                self.set_light(address, 0x000)
            except Exception:
                # ป้องกันไม่ให้ atexit ทำให้โปรแกรมล้ม
                pass
        self.get_logger().info("✅ All lights cleared.")

    # ==========================
    # ROS Callbacks
    # ==========================

    def listener_callback(self, msg: Int32):
        """รับค่าจาก monitor_topic แล้วสั่งไฟ/ออด ถ้าไม่ได้อยู่ในสถานะ emergency"""
        if self.emergency_active:
            # ถ้า emergency อยู่ ไม่ให้เปลี่ยนไฟจาก monitor_topic
            return

        self.get_logger().info(f"📨 Received value from monitor_topic: {msg.data}")
        self.control_modbus(msg.data)

    def emergency_callback(self, msg: Bool):
        """รับ emergency_stop (True = emergency active)"""
        new_emergency_state = msg.data

        # ตรวจสอบว่าเป็นการเรียกจาก timer หรือมีการเปลี่ยนแปลงสถานะจริง ๆ
        if self.timer_triggered or new_emergency_state != self.emergency_active:
            self.emergency_active = new_emergency_state
            self.timer_triggered = False  # รีเซ็ต flag

            if self.emergency_active:
                self.get_logger().error("🛑 EMERGENCY ACTIVATED!")
                self.get_logger().info(f"Emergency State: {self.emergency_active}")
                self.emergency_activated()
                # ถ้ามี timer_emergency_check อยู่ ให้ยกเลิก
                if self.timer_emergency_check is not None:
                    self.timer_emergency_check.cancel()
                    self.timer_emergency_check = None
            else:
                self.get_logger().info("✅ EMERGENCY DEACTIVATED")
                self.get_logger().info(f"Emergency State: {self.emergency_active}")
                self.emergency_deactivated()
                if self.timer_emergency_check is not None:
                    self.timer_emergency_check.cancel()
                    self.timer_emergency_check = None

    # ==========================
    # Emergency Handlers
    # ==========================

    def emergency_activated(self):
        """เมื่อ emergency = True"""
        self.clear_all()
        # ไฟแดง + ออด
        self.set_light(0x004, 0x002)  # ไฟแดง
        self.set_light(0x005, 0x002)  # ออด

    def emergency_deactivated(self):
        """เมื่อ emergency = False"""
        self.clear_all()
        # ถ้าต้องการให้กลับไปเขียวปกติ ขึ้นกับพารามิเตอร์
        if self.auto_green_on_clear:
            self.set_light(0x002, 0x001)  # ไฟเขียว

    # ==========================
    # High-level Modbus Control
    # ==========================

    def control_modbus(self, value: int):
        """ควบคุมไฟ + ออด ตามค่าที่ได้จาก monitor_topic"""
        if not self.connected:
            self.get_logger().error("❌ Device not connected (control_modbus)")
            return

        green_addr = 0x002
        yellow_addr = 0x003
        red_addr = 0x004
        buzzer_addr = 0x005

        color_values = 0x002
        buzzer_values = 0x002

        def write_color_and_buzzer(color_address, color_value, buzzer_value):
            try:
                color_response = self.client.write_register(
                    color_address,
                    color_value,
                    slave=self.slave_id
                )
                buzzer_response = self.client.write_register(
                    buzzer_addr,
                    buzzer_value,
                    slave=self.slave_id
                )
            except (ModbusIOException, ConnectionException) as e:
                self.get_logger().error(f"❌ Modbus Error in write_color_and_buzzer: {e}")
                return

            if color_response.isError():
                self.get_logger().error(
                    f"❌ Error writing to color register at {color_address:#05x}")
            else:
                self.get_logger().info(
                    f"✅ Successfully wrote {color_value} to color register {color_address:#05x}")

            if buzzer_response.isError():
                self.get_logger().error("❌ Error writing to Buzzer register at 0x005")
            else:
                self.get_logger().info(
                    f"✅ Successfully wrote {buzzer_value} to Buzzer register 0x005")

        # ปิดไฟและออดทั้งหมดก่อน
        self.clear_all()

        time.sleep(0.2)  # หน่วงนิดหน่อยกัน device ไม่ทัน

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
