#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Towerlight Modbus Node (ROS 2) — Serial RTU only

- รองรับ pymodbus 3.8.x (ไม่มี method='rtu')
- ตรวจอัตโนมัติว่าใช้ kw 'slave' หรือ 'unit'
- Non-blocking (ไม่มี time.sleep ใน callback)
- Reconnect อัตโนมัติ + restore ค่าล่าสุดเมื่อกลับมา
- Verbose log เห็นเหตุการณ์ทั้งหมด
"""

import threading
import logging
import inspect
from typing import Optional, Any

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32, Bool

from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException

logging.getLogger('pymodbus').setLevel(logging.CRITICAL)

# ===== Registers / Values =====
REG_GREEN  = 0x002
REG_YELLOW = 0x003
REG_RED    = 0x004
REG_BUZZ   = 0x005
REG_ENABLE = 0x006
VAL_ON     = 0x002
VAL_OFF    = 0x000


class ModbusNode(Node):
    def __init__(self):
        super().__init__('towerlight_serial_node')
        self.cb = ReentrantCallbackGroup()

        # ===== parameters =====
        self.declare_parameter('port', '/dev/towerlight')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('parity', 'N')   # 'N','E','O'
        self.declare_parameter('stopbits', 1)   # 1 หรือ 2
        self.declare_parameter('bytesize', 8)   # 7 หรือ 8
        self.declare_parameter('timeout',  0.3) # วินาที
        self.declare_parameter('slave_id', 1)

        self.declare_parameter('poll_period', 3.0)
        self.declare_parameter('reconnect_period', 2.0)
        self.declare_parameter('turn_green_delay', 0.0)
        self.declare_parameter('emergency_auto_trigger_delay', 0.0)

        self.declare_parameter('enable_buzzer', True)
        self.declare_parameter('buzzer_on_red', True)
        self.declare_parameter('buzzer_on_yellow', False)
        self.declare_parameter('buzzer_on_green', False)
        self.declare_parameter('buzzer_on_emergency', True)

        p = self.get_parameter
        self.port       = p('port').value
        self.baudrate   = int(p('baudrate').value)
        self.parity     = p('parity').value
        self.stopbits   = int(p('stopbits').value)
        self.bytesize   = int(p('bytesize').value)
        self.timeout    = float(p('timeout').value)
        self.slave      = int(p('slave_id').value)

        self.poll_period      = float(p('poll_period').value)
        self.reconnect_period = float(p('reconnect_period').value)
        self.turn_green_delay = float(p('turn_green_delay').value)
        self.auto_emerg_delay = float(p('emergency_auto_trigger_delay').value)

        self.enable_buzzer     = bool(p('enable_buzzer').value)
        self.buzz_red          = bool(p('buzzer_on_red').value)
        self.buzz_yellow       = bool(p('buzzer_on_yellow').value)
        self.buzz_green        = bool(p('buzzer_on_green').value)
        self.buzz_on_emergency = bool(p('buzzer_on_emergency').value)

        # ===== Modbus Serial (pymodbus 3.8.x ไม่มี method='rtu') =====
        try:
            self.client = ModbusSerialClient(
                method='rtu',  # บางรุ่นเก่าใช้ได้ แต่ใน 3.8.x จะ TypeError
                port=self.port, baudrate=self.baudrate, parity=self.parity,
                stopbits=self.stopbits, bytesize=self.bytesize, timeout=self.timeout
            )
        except TypeError:
            # เส้นทางหลักสำหรับ 3.8.x
            self.client = ModbusSerialClient(
                port=self.port, baudrate=self.baudrate, parity=self.parity,
                stopbits=self.stopbits, bytesize=self.bytesize, timeout=self.timeout
            )

        # ตรวจจาก signature ว่าต้องใช้ kw 'slave' หรือ 'unit'
        self._addr_kw = self._detect_addr_kw()

        self.modbus_lock = threading.Lock()
        self.connected = False

        # ===== state =====
        self.emergency_active = False
        self.last_cmd_value = 0      # 0=all off, 1=green, 2=yellow, 3=red
        self.pending_timers = []     # one-shot timers (ยกเลิกตอน emergency)

        # ===== topics =====
        self.create_subscription(Int32, 'monitor_topic', self.listener_callback, 10, callback_group=self.cb)
        self.create_subscription(Bool,  'emergency_stop', self.emergency_callback, 10, callback_group=self.cb)

        # ===== timers =====
        self.create_timer(self.reconnect_period, self._ensure_connection, callback_group=self.cb)
        self.create_timer(self.poll_period, self._poll_registers, callback_group=self.cb)

        # Boot sequence
        self._ensure_connection(first_time=True)
        if self.turn_green_delay > 0.0:
            self._set_red(buzzer=self.buzz_red)
            self._arm_timer(self.turn_green_delay, self._set_green)
        else:
            self._set_green()

        if self.auto_emerg_delay > 0.0:
            self._arm_timer(self.auto_emerg_delay, lambda: self.emergency_callback(Bool(data=True)))

        self.get_logger().info("✅ Towerlight serial node started")

    # ---------- compat helpers ----------
    def _detect_addr_kw(self) -> str:
        sig = inspect.signature(self.client.write_register)
        if 'slave' in sig.parameters:
            return 'slave'
        if 'unit' in sig.parameters:
            return 'unit'
        # ค่าเริ่มต้นปลอดภัยในรุ่นใหม่ ๆ
        return 'unit'

    def _call_write(self, addr: int, val: int):
        kw = {self._addr_kw: self.slave}
        return self.client.write_register(addr, val, **kw)

    def _call_read_hr(self, address: int, count: int):
        kw = {self._addr_kw: self.slave}
        return self.client.read_holding_registers(address=address, count=count, **kw)

    # ---------- low level (non-blocking) ----------
    def _write_reg(self, addr: int, val: int) -> bool:
        if not self.connected:
            self.get_logger().warn(f"skip write 0x{addr:03X} (not connected)")
            return False
        with self.modbus_lock:
            try:
                resp = self._call_write(addr, val)
                ok = (resp is not None) and (not resp.isError())
                self.get_logger().info(f"WRITE 0x{addr:03X} = {val} -> {'OK' if ok else 'ERR'}")
                return ok
            except Exception as e:
                self.get_logger().error(f"Modbus write error @0x{addr:03X}: {e}")
                self.connected = False
                return False

    def _enable_rtu(self) -> bool:
        # บางอุปกรณ์ต้อง enable ก่อน
        return self._write_reg(REG_ENABLE, 0x001)

    def _all_off(self) -> None:
        self.get_logger().info("ALL OFF")
        self._write_reg(REG_GREEN,  VAL_OFF)
        self._write_reg(REG_YELLOW, VAL_OFF)
        self._write_reg(REG_RED,    VAL_OFF)
        self._write_reg(REG_BUZZ,   VAL_OFF)

    def _apply_buzzer(self, want: bool) -> None:
        if self.enable_buzzer:
            self._write_reg(REG_BUZZ, VAL_ON if want else VAL_OFF)

    def _set_green(self) -> None:
        self.last_cmd_value = 1
        self.get_logger().info("SET GREEN")
        self._all_off()
        self._write_reg(REG_GREEN, VAL_ON)
        self._apply_buzzer(self.buzz_green and not self.emergency_active)

    def _set_yellow(self) -> None:
        self.last_cmd_value = 2
        self.get_logger().info("SET YELLOW")
        self._all_off()
        self._write_reg(REG_YELLOW, VAL_ON)
        self._apply_buzzer(self.buzz_yellow and not self.emergency_active)

    def _set_red(self, buzzer: Optional[bool] = None) -> None:
        self.last_cmd_value = 3
        self.get_logger().info("SET RED")
        self._all_off()
        self._write_reg(REG_RED, VAL_ON)
        if buzzer is None:
            buzzer = self.buzz_red and not self.emergency_active
        self._apply_buzzer(buzzer)

    def _restore_last(self) -> None:
        self.get_logger().info(f"RESTORE last={self.last_cmd_value}")
        {0: self._all_off, 1: self._set_green, 2: self._set_yellow, 3: self._set_red}.get(self.last_cmd_value, self._all_off)()

    def _arm_timer(self, delay_sec: float, fn) -> None:
        t = self.create_timer(delay_sec, lambda: self._fire_one_shot(t, fn), callback_group=self.cb)
        self.pending_timers.append(t)

    def _fire_one_shot(self, timer_obj, fn) -> None:
        try:
            fn()
        finally:
            try:
                timer_obj.cancel()
            except Exception:
                pass
            if timer_obj in self.pending_timers:
                self.pending_timers.remove(timer_obj)

    def _cancel_all_one_shots(self) -> None:
        for t in list(self.pending_timers):
            try:
                t.cancel()
            except Exception:
                pass
        self.pending_timers.clear()

    # ---------- connection / polling ----------
    def _ensure_connection(self, first_time: bool = False) -> None:
        if self.connected:
            return
        try:
            self.client.close()
        except Exception:
            pass
        self.connected = self.client.connect()
        self.get_logger().info(f"CONNECT -> {'OK' if self.connected else 'FAIL'} (port={self.port}, baud={self.baudrate})")
        if self.connected:
            if not self._enable_rtu():
                self.get_logger().warn("⚠️ enable RTU failed (will keep trying on writes)")
            self._restore_last()
        elif first_time:
            self.get_logger().error("❌ Failed to connect to Modbus device (will retry)")

    def _poll_registers(self) -> None:
        if not self.connected:
            return
        try:
            with self.modbus_lock:
                resp = self._call_read_hr(address=0x000, count=4)
            if not resp or resp.isError():
                raise ModbusIOException("read error")
        except Exception as e:
            self.get_logger().warn(f"Modbus poll failed: {e}")
            self.connected = False

    # ---------- topics ----------
    def listener_callback(self, msg: Int32) -> None:
        self.get_logger().info(f"RX /monitor_topic: {msg.data} (emergency={self.emergency_active})")
        if self.emergency_active:
            self.get_logger().info("ignore because emergency active")
            return
        v = int(msg.data)
        if   v == 1: self._set_green()
        elif v == 2: self._set_yellow()
        elif v == 3: self._set_red()
        else:
            self.last_cmd_value = 0
            self._all_off()

    def emergency_callback(self, msg: Bool) -> None:
        new_state = bool(msg.data)
        self.get_logger().info(f"RX /emergency_stop: {new_state}")
        if new_state == self.emergency_active:
            self.get_logger().info("no change")
            return

        self.emergency_active = new_state
        self._cancel_all_one_shots()

        if self.emergency_active:
            self.get_logger().error("🛑 EMERGENCY ACTIVATED!")
            self._set_red(buzzer=self.buzz_on_emergency)
        else:
            self.get_logger().info("✅ EMERGENCY DEACTIVATED")
            self._restore_last()

    # ---------- shutdown ----------
    def destroy_node(self):
        try:
            self._cancel_all_one_shots()
            self._all_off()
        finally:
            super().destroy_node()
            try:
                self.client.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()
    executor = MultiThreadedExecutor(num_threads=2)
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
