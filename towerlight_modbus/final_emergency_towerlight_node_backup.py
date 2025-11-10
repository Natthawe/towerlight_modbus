#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import logging
import inspect
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
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

VAL_OFF   = 0x000
VAL_ON    = 0x001   # Continuous ON (no flash)

class ModbusNode(Node):
    def __init__(self):
        super().__init__('final_emergency_towerlight_node')
        self.cb = ReentrantCallbackGroup()

        # ===== Parameters =====
        self.declare_parameter('port', '/dev/towerlight')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('parity', 'N')
        self.declare_parameter('stopbits', 1)
        self.declare_parameter('bytesize', 8)
        self.declare_parameter('timeout',  0.5)
        self.declare_parameter('slave_id', 1)

        self.declare_parameter('poll_period', 5.0)
        self.declare_parameter('reconnect_period', 2.0)
        self.declare_parameter('turn_green_delay', 0.0)

        self.declare_parameter('enable_buzzer', False)
        self.declare_parameter('buzzer_on_red', False)
        self.declare_parameter('buzzer_on_yellow', False)
        self.declare_parameter('buzzer_on_green', False)
        self.declare_parameter('buzzer_on_emergency', False)

        # anti-latch / speed options
        self.declare_parameter('emergency_buzzer_pulse_ms', 0)     # 0 = never beep
        self.declare_parameter('force_clear_before_green', True)    # clear->green
        self.declare_parameter('force_clear_before_red', True)      # clear->red (NEW)
        self.declare_parameter('force_block_write', True)           # write 0x002..0x005 in one packet

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

        self.enable_buzzer     = bool(p('enable_buzzer').value)
        self.buzz_red          = bool(p('buzzer_on_red').value)
        self.buzz_yellow       = bool(p('buzzer_on_yellow').value)
        self.buzz_green        = bool(p('buzzer_on_green').value)
        self.buzz_on_emergency = bool(p('buzzer_on_emergency').value)

        self.emg_buzz_pulse_ms        = int(p('emergency_buzzer_pulse_ms').value)
        self.force_clear_before_green  = bool(p('force_clear_before_green').value)
        self.force_clear_before_red    = bool(p('force_clear_before_red').value)   # NEW
        self.force_block_write         = bool(p('force_block_write').value)

        # ===== Modbus Serial =====
        try:
            self.client = ModbusSerialClient(
                method='rtu',
                port=self.port, baudrate=self.baudrate, parity=self.parity,
                stopbits=self.stopbits, bytesize=self.bytesize, timeout=self.timeout
            )
        except TypeError:
            self.client = ModbusSerialClient(
                port=self.port, baudrate=self.baudrate, parity=self.parity,
                stopbits=self.stopbits, bytesize=self.bytesize, timeout=self.timeout
            )

        self._addr_kw = self._detect_addr_kw()
        self._has_multi = hasattr(self.client, 'write_registers')

        self.modbus_lock = threading.Lock()
        self.connected = False

        # ===== state =====
        self.emergency_active = False
        self.pre_emergency_value: Optional[int] = None
        self.last_cmd_value = 0  # 0=off, 1=G, 2=Y, 3=R
        self._poll_paused = False

        # cache
        self._reg_cache = {
            REG_GREEN: VAL_OFF,
            REG_YELLOW: VAL_OFF,
            REG_RED: VAL_OFF,
            REG_BUZZ: VAL_OFF,
        }

        # QoS latest-only (ลด backlog เวลา pub 20Hz)
        qos_latest = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.create_subscription(Int32, 'monitor_topic', self.listener_callback, qos_latest, callback_group=self.cb)
        self.create_subscription(Bool,  'emergency_stop', self.emergency_callback, qos_latest, callback_group=self.cb)

        # timers
        self.create_timer(self.reconnect_period, self._ensure_connection, callback_group=self.cb)
        self.create_timer(self.poll_period, self._poll_registers, callback_group=self.cb)

        # Boot flow
        self._ensure_connection(first_time=True)
        if self.turn_green_delay > 0:
            self._set_red(record=True, buzzer=self.buzz_red)
            self._arm_once(self.turn_green_delay, self._set_green)
        else:
            self._set_green()

        self.get_logger().info("✅ Towerlight serial node started (clear→red enabled)")

    # ---------- utils ----------
    def _detect_addr_kw(self) -> str:
        sig = inspect.signature(self.client.write_register)
        if 'slave' in sig.parameters: return 'slave'
        if 'unit'  in sig.parameters: return 'unit'
        return 'unit'

    def _kw(self): return {self._addr_kw: self.slave}

    def _arm_once(self, delay, fn):
        t = self.create_timer(delay, lambda: self._fire_once(t, fn), callback_group=self.cb)
    def _fire_once(self, t, fn):
        try: fn()
        finally:
            try: t.cancel()
            except: pass

    # ---------- low-level ----------
    def _write_reg(self, addr, val) -> bool:
        if not self.connected:
            self.get_logger().warn(f"skip write 0x{addr:03X} (not connected)")
            return False
        with self.modbus_lock:
            try:
                resp = self.client.write_register(addr, val, **self._kw())
                ok = (resp is not None) and (not resp.isError())
                self.get_logger().info(f"WRITE 0x{addr:03X} = {val} -> {'OK' if ok else 'ERR'}")
                if ok: self._reg_cache[addr] = val
                return ok
            except Exception as e:
                self.get_logger().error(f"write@0x{addr:03X} err: {e}")
                self.connected = False
                return False

    def _write_regs(self, start_addr: int, values: List[int]) -> bool:
        if not self.connected:
            self.get_logger().warn("skip write_registers (not connected)")
            return False
        if not self._has_multi:
            ok = True
            for i, v in enumerate(values):
                ok &= self._write_reg(start_addr + i, v)
            return ok
        with self.modbus_lock:
            try:
                resp = self.client.write_registers(start_addr, values, **self._kw())
                ok = (resp is not None) and (not resp.isError())
                self.get_logger().info(
                    f"WRITE_MULTI 0x{start_addr:03X}..0x{start_addr+len(values)-1:03X} <- {values} -> {'OK' if ok else 'ERR'}")
                if ok:
                    for i, v in enumerate(values):
                        self._reg_cache[start_addr + i] = v
                return ok
            except Exception as e:
                self.get_logger().error(f"write_registers err @{start_addr:#05x}: {e}")
                self.connected = False
                return False

    def _apply_scene(self, g, y, r, b, *, force: bool = False):
        """ตั้งค่า (G,Y,R,B) ให้ตรงเป้า
           force=True  : บังคับเขียน (ไม่ดู cache)
           force_block_write=True: เขียน 0x002..0x005 ทีเดียว
        """
        self._poll_paused = True
        try:
            if self.force_block_write:
                # ถ้า force ⇒ เขียนเลย; ถ้าไม่ force ⇒ เขียนเฉพาะต่างจาก cache
                if not force:
                    target = [g, y, r, b]
                    current = [self._reg_cache[REG_GREEN],
                               self._reg_cache[REG_YELLOW],
                               self._reg_cache[REG_RED],
                               self._reg_cache[REG_BUZZ]]
                    if target == current:
                        return True
                return self._write_regs(REG_GREEN, [g, y, r, b])

            # โหมดเดิม (diff + contiguous)
            target = {REG_GREEN:g, REG_YELLOW:y, REG_RED:r, REG_BUZZ:b}
            dirty = [a for a in (REG_GREEN, REG_YELLOW, REG_RED, REG_BUZZ)
                     if force or self._reg_cache.get(a) != target[a]]
            if not dirty: return True
            dirty.sort()
            first, last = dirty[0], dirty[-1]
            if last - first + 1 == len(dirty) and len(dirty) > 1:
                vals = [target[a] for a in range(first, last+1)]
                return self._write_regs(first, vals)
            ok = True
            for a in dirty:
                ok &= self._write_reg(a, target[a])
            return ok
        finally:
            self._poll_paused = False

    # ---------- high level ----------
    def _enable_rtu(self) -> bool:
        return self._write_reg(REG_ENABLE, 0x001)

    def _all_off(self, *, force: bool = False):
        self.get_logger().info("ALL OFF")
        self._apply_scene(VAL_OFF, VAL_OFF, VAL_OFF, VAL_OFF, force=force)

    def _set_green(self, record: bool = True):
        if record: self.last_cmd_value = 1
        self.get_logger().info("SET GREEN")
        buzz = VAL_ON if (self.enable_buzzer and self.buzz_green and not self.emergency_active) else VAL_OFF
        self._apply_scene(VAL_ON, VAL_OFF, VAL_OFF, buzz)

    def _set_yellow(self, record: bool = True):
        if record: self.last_cmd_value = 2
        self.get_logger().info("SET YELLOW")
        buzz = VAL_ON if (self.enable_buzzer and self.buzz_yellow and not self.emergency_active) else VAL_OFF
        self._apply_scene(VAL_OFF, VAL_ON, VAL_OFF, buzz)

    def _set_red(self, *, buzzer: Optional[bool] = None, record: bool = True, preclear: bool = False):
        if record: self.last_cmd_value = 3
        self.get_logger().info("SET RED")
        # buzz logic
        if self.emg_buzz_pulse_ms == 0:
            effective_buzz = False
        else:
            if buzzer is None:
                buzzer = self.buzz_red and not self.emergency_active
            effective_buzz = bool(self.enable_buzzer and buzzer)
        buzz_val = VAL_ON if effective_buzz else VAL_OFF

        # ถ้าต้องเคลียร์ก่อน (เพื่อกัน latch/flash) ให้บังคับเขียน clear→red
        if preclear:
            self._apply_scene(VAL_OFF, VAL_OFF, VAL_OFF, VAL_OFF, force=True)
            self._apply_scene(VAL_OFF, VAL_OFF, VAL_ON,  buzz_val, force=True)
        else:
            self._apply_scene(VAL_OFF, VAL_OFF, VAL_ON,  buzz_val)

    def _restore_last(self):
        self.get_logger().info(f"RESTORE last={self.last_cmd_value}")
        {0:self._all_off,1:self._set_green,2:self._set_yellow,3:self._set_red}.get(self.last_cmd_value,self._all_off)()

    # ---------- connection / polling ----------
    def _ensure_connection(self, first_time=False):
        if self.connected: return
        try: self.client.close()
        except: pass
        self.connected = self.client.connect()
        self.get_logger().info(f"CONNECT -> {'OK' if self.connected else 'FAIL'} (port={self.port}, baud={self.baudrate})")
        if self.connected:
            if not self._enable_rtu():
                self.get_logger().warn("⚠️ enable RTU failed (will keep trying)")
            self._restore_last()
        elif first_time:
            self.get_logger().error("❌ Failed to connect (retrying)")

    def _poll_registers(self):
        if self._poll_paused or not self.connected: return
        try:
            with self.modbus_lock:
                resp = self.client.read_holding_registers(address=0x000, count=4, **self._kw())
            if not resp or resp.isError(): raise ModbusIOException("read error")
        except Exception as e:
            self.get_logger().warn(f"poll failed: {e}")
            self.connected = False

    # ---------- topics ----------
    def listener_callback(self, msg: Int32):
        if self.emergency_active:
            return
        v = int(msg.data)
        if   v == 1: self._set_green(record=True)
        elif v == 2: self._set_yellow(record=True)
        elif v == 3: self._set_red(record=True)
        else:
            self.last_cmd_value = 0
            self._all_off()

    def emergency_callback(self, msg: Bool):
        new_state = bool(msg.data)
        if new_state == self.emergency_active:
            return

        self.emergency_active = new_state

        if self.emergency_active:
            # จำสีเดิมก่อนฉุกเฉิน
            self.pre_emergency_value = self.last_cmd_value
            self.get_logger().error("🛑 EMERGENCY ACTIVATED!")
            want_buzz = self.buzz_on_emergency and (self.emg_buzz_pulse_ms != 0)
            # เคลียร์ก่อนเข้าแดง ถ้าเปิดไว้
            self._set_red(buzzer=want_buzz, record=False, preclear=self.force_clear_before_red)
            # ถ้ามี pulse ให้ตัดบัซเซอร์ตามเวลา (ไม่บล็อก)
            if want_buzz and self.emg_buzz_pulse_ms > 0:
                self._arm_once(self.emg_buzz_pulse_ms/1000.0,
                               lambda: self._apply_scene(self._reg_cache[REG_GREEN],
                                                         self._reg_cache[REG_YELLOW],
                                                         self._reg_cache[REG_RED],
                                                         VAL_OFF, force=True))
        else:
            self.get_logger().info("✅ EMERGENCY DEACTIVATED")
            if self.force_clear_before_green:
                self._apply_scene(VAL_OFF, VAL_OFF, VAL_OFF, VAL_OFF, force=True)
            # คืนสีเดิม หรือไม่ทราบ -> เขียว
            v = self.pre_emergency_value
            self.pre_emergency_value = None
            if   v == 1: self._set_green(record=True)
            elif v == 2: self._set_yellow(record=True)
            elif v == 3: self._set_red(record=True)
            else:        self._set_green(record=True)

    # ---------- shutdown ----------
    def destroy_node(self):
        try:
            self._apply_scene(VAL_OFF, VAL_OFF, VAL_OFF, VAL_OFF, force=True)
        finally:
            super().destroy_node()
            try: self.client.close()
            except: pass


def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()
    executor = MultiThreadedExecutor(num_threads=2)
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
