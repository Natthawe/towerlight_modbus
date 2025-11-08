#!/usr/bin/env python3

import threading
import logging
import inspect
import time
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
VAL_ON     = 0x002
VAL_OFF    = 0x000

# ===== Coalesce window (วินาที) =====
COALESCE_SEC = 0.03   # 30 ms รวมคำสั่งที่วิ่งมาถี่ๆ


class ModbusNode(Node):
    def __init__(self):
        super().__init__('final_emergency_towerlight_node')
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

        # ตรวจจาก signature ว่าต้องใช้ kw 'slave' หรือ 'unit'
        self._addr_kw = self._detect_addr_kw()
        # ตรวจว่า client รองรับ write_registers หรือไม่
        self._has_multi = hasattr(self.client, 'write_registers')

        self.modbus_lock = threading.Lock()
        self.connected = False

        # ===== state (register cache) =====
        self._reg_cache = {
            REG_GREEN: VAL_OFF,
            REG_YELLOW: VAL_OFF,
            REG_RED: VAL_OFF,
            REG_BUZZ: VAL_OFF,
        }

        self.emergency_active = False
        self.pre_emergency_value: Optional[int] = None
        self.last_cmd_value = 0      # 0=all off, 1=green, 2=yellow, 3=red

        # coalesce
        self._pending_scene = None   # (g,y,r,b) ที่รอรวมคำสั่ง
        self._coalesce_timer = None

        # ===== topics (QoS: latest-only) =====
        qos_latest = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Int32, 'monitor_topic', self.listener_callback, qos_latest, callback_group=self.cb)
        self.create_subscription(Bool,  'emergency_stop', self.emergency_callback, qos_latest, callback_group=self.cb)

        # ===== timers =====
        self._poll_paused = False
        self.create_timer(self.reconnect_period, self._ensure_connection, callback_group=self.cb)
        self.create_timer(self.poll_period, self._poll_registers, callback_group=self.cb)

        # Boot sequence
        self._ensure_connection(first_time=True)
        if self.turn_green_delay > 0.0:
            self._set_red(buzzer=self.buzz_red, record=True)
            self._arm_timer(self.turn_green_delay, self._set_green)
        else:
            self._set_green()

        if self.auto_emerg_delay > 0.0:
            self._arm_timer(self.auto_emerg_delay, lambda: self.emergency_callback(Bool(data=True)))

        self.get_logger().info("✅ Towerlight serial node (low-latency) started")

    # ---------- compat helpers ----------
    def _detect_addr_kw(self) -> str:
        sig = inspect.signature(self.client.write_register)
        if 'slave' in sig.parameters:
            return 'slave'
        if 'unit' in sig.parameters:
            return 'unit'
        return 'unit'

    def _kwaddr(self):
        return {self._addr_kw: self.slave}

    # ---------- write primitives ----------
    def _write_reg(self, addr: int, val: int) -> bool:
        if not self.connected:
            self.get_logger().warn(f"skip write 0x{addr:03X} (not connected)")
            return False
        with self.modbus_lock:
            try:
                resp = self.client.write_register(addr, val, **self._kwaddr())
                ok = (resp is not None) and (not resp.isError())
                self.get_logger().info(f"WRITE 0x{addr:03X} = {val} -> {'OK' if ok else 'ERR'}")
                if ok:
                    self._reg_cache[addr] = val
                return ok
            except Exception as e:
                self.get_logger().error(f"Modbus write error @0x{addr:03X}: {e}")
                self.connected = False
                return False

    def _write_regs(self, start_addr: int, values: List[int]) -> bool:
        if not self.connected:
            self.get_logger().warn("skip multi-write (not connected)")
            return False
        if not self._has_multi:
            ok_all = True
            for i, v in enumerate(values):
                ok_all &= self._write_reg(start_addr + i, v)
            return ok_all

        with self.modbus_lock:
            try:
                resp = self.client.write_registers(start_addr, values, **self._kwaddr())
                ok = (resp is not None) and (not resp.isError())
                self.get_logger().info(
                    f"WRITE_MULTI 0x{start_addr:03X}..0x{start_addr+len(values)-1:03X} <- {values} -> {'OK' if ok else 'ERR'}"
                )
                if ok:
                    for i, v in enumerate(values):
                        self._reg_cache[start_addr + i] = v
                return ok
            except Exception as e:
                self.get_logger().error(f"Modbus write_registers error @{start_addr:#05x}: {e}")
                self.connected = False
                return False

    # ---------- scenes & coalesce ----------
    def _apply_scene(self, g: int, y: int, r: int, b: int):
        """ตั้งค่าฉากไฟแบบ diff-only + multi-write ในช่วง 0x002..0x005"""
        # ระงับ poll ชั่วคราวเพื่อลดชนบัส
        self._poll_paused = True
        try:
            target = {
                REG_GREEN: g,
                REG_YELLOW: y,
                REG_RED: r,
                REG_BUZZ: b,
            }
            # diff-only
            addrs = []
            vals = []
            for a in (REG_GREEN, REG_YELLOW, REG_RED, REG_BUZZ):
                if self._reg_cache.get(a, None) != target[a]:
                    addrs.append(a)
                    vals.append(target[a])

            if not addrs:
                return True

            # ถ้าส่วนที่เปลี่ยนต่อเนื่องกันทั้งหมด -> เขียนทีเดียว
            if min(addrs) + len(addrs) - 1 == max(addrs) and len(addrs) > 1:
                return self._write_regs(min(addrs), vals)
            else:
                # กระจัดกระจาย -> เขียนทีละอัน
                ok_all = True
                for a, v in zip(addrs, vals):
                    ok_all &= self._write_reg(a, v)
                return ok_all
        finally:
            # ปล่อย poll หลังจบ (ดีเลย์นิดเพื่อให้อุปกรณ์สลับเสร็จ)
            self._poll_paused = False

    def _set_scene_coalesced(self, g: int, y: int, r: int, b: int):
        """รวมคำสั่งที่วิ่งถี่ ภายใน COALESCE_SEC จะยิงแค่รอบสุดท้าย"""
        self._pending_scene = (g, y, r, b)
        if self._coalesce_timer is None:
            self._coalesce_timer = self.create_timer(COALESCE_SEC, self._flush_coalesce, callback_group=self.cb)

    def _flush_coalesce(self):
        scene = self._pending_scene
        self._pending_scene = None
        try:
            if scene is not None:
                g, y, r, b = scene
                self._apply_scene(g, y, r, b)
        finally:
            try:
                self._coalesce_timer.cancel()
            except Exception:
                pass
            self._coalesce_timer = None

    # ---------- high level ----------
    def _enable_rtu(self) -> bool:
        return self._write_reg(REG_ENABLE, 0x001)

    def _all_off(self):
        self.get_logger().info("ALL OFF")
        self._set_scene_coalesced(VAL_OFF, VAL_OFF, VAL_OFF, VAL_OFF)

    def _set_green(self, record: bool = True):
        if record:
            self.last_cmd_value = 1
        self.get_logger().info("SET GREEN")
        buzz = VAL_ON if (self.enable_buzzer and self.buzz_green and not self.emergency_active) else VAL_OFF
        self._set_scene_coalesced(VAL_ON, VAL_OFF, VAL_OFF, buzz)

    def _set_yellow(self, record: bool = True):
        if record:
            self.last_cmd_value = 2
        self.get_logger().info("SET YELLOW")
        buzz = VAL_ON if (self.enable_buzzer and self.buzz_yellow and not self.emergency_active) else VAL_OFF
        self._set_scene_coalesced(VAL_OFF, VAL_ON, VAL_OFF, buzz)

    def _set_red(self, buzzer: Optional[bool] = None, record: bool = True):
        if record:
            self.last_cmd_value = 3
        self.get_logger().info("SET RED")
        if buzzer is None:
            buzzer = self.buzz_red and not self.emergency_active
        buzz = VAL_ON if (self.enable_buzzer and buzzer) else VAL_OFF
        self._set_scene_coalesced(VAL_OFF, VAL_OFF, VAL_ON, buzz)

    def _restore_last(self):
        self.get_logger().info(f"RESTORE last={self.last_cmd_value}")
        {0: self._all_off, 1: self._set_green, 2: self._set_yellow, 3: self._set_red}.get(self.last_cmd_value, self._all_off)()

    # ---------- timers ----------
    def _arm_timer(self, delay_sec: float, fn):
        t = self.create_timer(delay_sec, lambda: self._fire_one_shot(t, fn), callback_group=self.cb)
        # ไม่ต้องเก็บลิสต์ one-shots เพราะฉากเรา coalesce อยู่แล้ว

    def _fire_one_shot(self, timer_obj, fn):
        try:
            fn()
        finally:
            try:
                timer_obj.cancel()
            except Exception:
                pass

    # ---------- connection / polling ----------
    def _ensure_connection(self, first_time: bool = False):
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

    def _poll_registers(self):
        if self._poll_paused or not self.connected:
            return
        try:
            with self.modbus_lock:
                resp = self.client.read_holding_registers(address=0x000, count=4, **self._kwaddr())
            if not resp or resp.isError():
                raise ModbusIOException("read error")
        except Exception as e:
            self.get_logger().warn(f"Modbus poll failed: {e}")
            self.connected = False

    # ---------- topics ----------
    def listener_callback(self, msg: Int32):
        self.get_logger().info(f"RX /monitor_topic: {msg.data} (emergency={self.emergency_active})")
        if self.emergency_active:
            self.get_logger().info("ignore because emergency active")
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
        self.get_logger().info(f"RX /emergency_stop: {new_state}")
        if new_state == self.emergency_active:
            self.get_logger().info("no change")
            return

        self.emergency_active = new_state

        if self.emergency_active:
            # เก็บสีเดิมก่อนเข้า EMERGENCY แล้วสั่งแดงโดยไม่ทับ last_cmd_value
            self.pre_emergency_value = self.last_cmd_value
            self.get_logger().error("🛑 EMERGENCY ACTIVATED!")
            self._set_red(buzzer=self.buzz_on_emergency, record=False)
        else:
            self.get_logger().info("✅ EMERGENCY DEACTIVATED")
            # คืนค่าสีก่อนฉุกเฉิน (ถ้าไม่ทราบ ให้กลับไปเขียว)
            v = self.pre_emergency_value
            self.pre_emergency_value = None
            if   v == 1: self._set_green(record=True)
            elif v == 2: self._set_yellow(record=True)
            elif v == 3: self._set_red(record=True)
            else:        self._set_green(record=True)

    # ---------- shutdown ----------
    def destroy_node(self):
        try:
            # ยิงฉากล่าสุดให้ปิดทั้งหมดอย่างรวดเร็ว
            try:
                self._apply_scene(VAL_OFF, VAL_OFF, VAL_OFF, VAL_OFF)
            except Exception:
                pass
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
