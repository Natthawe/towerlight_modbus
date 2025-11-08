#!/usr/bin/env python3

import threading
import logging
import inspect
from typing import Optional, Any, Dict

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32, Bool

# ===== pymodbus (optional import) =====
HAS_PYMODBUS = True
try:
    from pymodbus.client import ModbusSerialClient, ModbusTcpClient
    from pymodbus.exceptions import ModbusIOException
    logging.getLogger('pymodbus').setLevel(logging.CRITICAL)
except Exception:
    HAS_PYMODBUS = False
    class ModbusIOException(Exception):
        pass

# ===== Registers / Values =====
REG_GREEN  = 0x002
REG_YELLOW = 0x003
REG_RED    = 0x004
REG_BUZZ   = 0x005
REG_ENABLE = 0x006
VAL_ON     = 0x002
VAL_OFF    = 0x000


# ===== Client interfaces =====
class BaseClient:
    def connect(self) -> bool: raise NotImplementedError
    def close(self) -> None:   raise NotImplementedError
    def write_register(self, addr: int, val: int, **kw) -> Any: raise NotImplementedError
    def read_holding_registers(self, address: int, count: int, **kw) -> Any: raise NotImplementedError


class FakeClient(BaseClient):
    """โหมดจำลองในหน่วยความจำ — ไม่แตะฮาร์ดแวร์/เน็ตเวิร์ก"""
    def __init__(self): self.reg: Dict[int, int] = {}
    def connect(self) -> bool: return True
    def close(self) -> None:   pass
    def write_register(self, addr: int, val: int, **kw) -> Any:
        self.reg[addr] = val
        print(f"[FAKE] write 0x{addr:03X} <- {val}")
        class Resp:  # จำลอง response object ของ pymodbus
            def isError(self) -> bool: return False
        return Resp()
    def read_holding_registers(self, address: int, count: int, **kw) -> Any:
        class Resp:
            def __init__(self, ok: bool): self._ok = ok
            def isError(self) -> bool:     return not self._ok
        return Resp(True)


class SerialClient(BaseClient):
    """Wrapper สำหรับ ModbusSerialClient (รองรับรุ่นที่ไม่มี method='rtu')"""
    def __init__(self, port: str, baudrate: int, parity: str, stopbits: int, bytesize: int, timeout: float):
        if not HAS_PYMODBUS:
            raise RuntimeError("pymodbus not installed")
        # บางรุ่นไม่มี method='rtu' -> สร้างแบบไม่ใส่ method
        try:
            self._cli = ModbusSerialClient(
                method='rtu', port=port, baudrate=baudrate, parity=parity,
                stopbits=stopbits, bytesize=bytesize, timeout=timeout
            )
        except TypeError:
            self._cli = ModbusSerialClient(
                port=port, baudrate=baudrate, parity=parity,
                stopbits=stopbits, bytesize=bytesize, timeout=timeout
            )
    def connect(self) -> bool: return self._cli.connect()
    def close(self) -> None:   self._cli.close()
    def write_register(self, addr: int, val: int, **kw) -> Any:
        return self._cli.write_register(addr, val, **kw)
    def read_holding_registers(self, address: int, count: int, **kw) -> Any:
        return self._cli.read_holding_registers(address=address, count=count, **kw)


class TcpClient(BaseClient):
    """Wrapper สำหรับ ModbusTcpClient"""
    def __init__(self, host: str, port: int, timeout: float):
        if not HAS_PYMODBUS:
            raise RuntimeError("pymodbus not installed")
        self._cli = ModbusTcpClient(host, port=port, timeout=timeout)
    def connect(self) -> bool: return self._cli.connect()
    def close(self) -> None:   self._cli.close()
    def write_register(self, addr: int, val: int, **kw) -> Any:
        return self._cli.write_register(addr, val, **kw)
    def read_holding_registers(self, address: int, count: int, **kw) -> Any:
        return self._cli.read_holding_registers(address=address, count=count, **kw)


class ModbusNode(Node):
    def __init__(self):
        super().__init__('towerlight_node')
        self.cb = ReentrantCallbackGroup()

        # ===== parameters =====
        self.declare_parameter('transport', 'fake')  # 'serial'|'tcp'|'fake'

        # serial
        self.declare_parameter('port', '/dev/towerlight')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('parity', 'N')
        self.declare_parameter('stopbits', 1)
        self.declare_parameter('bytesize', 8)

        # tcp
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port_tcp', 5020)

        # common
        self.declare_parameter('timeout',  0.3)
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

        g = self.get_parameter
        self.transport: str = g('transport').value

        # serial
        self.port: str      = g('port').value
        self.baudrate: int  = int(g('baudrate').value)
        self.parity: str    = g('parity').value
        self.stopbits: int  = int(g('stopbits').value)
        self.bytesize: int  = int(g('bytesize').value)

        # tcp
        self.host: str      = g('host').value
        self.port_tcp: int  = int(g('port_tcp').value)

        # common
        self.timeout: float = float(g('timeout').value)
        self.slave: int     = int(g('slave_id').value)

        self.poll_period: float      = float(g('poll_period').value)
        self.reconnect_period: float = float(g('reconnect_period').value)
        self.turn_green_delay: float = float(g('turn_green_delay').value)
        self.auto_emerg_delay: float = float(g('emergency_auto_trigger_delay').value)

        self.enable_buzzer: bool     = bool(g('enable_buzzer').value)
        self.buzz_red: bool          = bool(g('buzzer_on_red').value)
        self.buzz_yellow: bool       = bool(g('buzzer_on_yellow').value)
        self.buzz_green: bool        = bool(g('buzzer_on_green').value)
        self.buzz_on_emergency: bool = bool(g('buzzer_on_emergency').value)

        # ===== client factory =====
        if self.transport == 'serial':
            self.client: BaseClient = SerialClient(
                port=self.port, baudrate=self.baudrate, parity=self.parity,
                stopbits=self.stopbits, bytesize=self.bytesize, timeout=self.timeout
            )
        elif self.transport == 'tcp':
            self.client = TcpClient(self.host, self.port_tcp, self.timeout)
        else:
            self.client = FakeClient()

        # ตรวจจาก signature ของ client ว่าใช้ kw 'slave' หรือ 'unit'
        self._addr_kw = self._detect_addr_kw(self.client)

        # ===== state =====
        self.modbus_lock = threading.Lock()
        self.connected = False
        self.emergency_active = False
        self.last_cmd_value = 0
        self.pending_timers = []

        # ===== topics =====
        self.create_subscription(Int32, 'monitor_topic', self.listener_callback, 10, callback_group=self.cb)
        self.create_subscription(Bool,  'emergency_stop', self.emergency_callback, 10, callback_group=self.cb)

        # ===== timers =====
        self.create_timer(self.reconnect_period, self._ensure_connection, callback_group=self.cb)
        self.create_timer(self.poll_period, self._poll_registers, callback_group=self.cb)

        # boot
        self._ensure_connection(first=True)
        if self.turn_green_delay > 0.0:
            self._set_red(buzzer=self.buzz_red)
            self._arm_timer(self.turn_green_delay, self._set_green)
        else:
            self._set_green()

        if self.auto_emerg_delay > 0.0:
            self._arm_timer(self.auto_emerg_delay, lambda: self.emergency_callback(Bool(data=True)))

        self.get_logger().info(f"✅ towerlight_node started (transport={self.transport})")

    # ---------- compat ----------
    @staticmethod
    def _detect_addr_kw(client: BaseClient) -> str:
        """ดู signature ของ write_register เพื่อตัดสินว่าจะส่ง kw 'slave' หรือ 'unit' """
        try:
            sig = inspect.signature(client.write_register)
            if 'slave' in sig.parameters:
                return 'slave'
            if 'unit' in sig.parameters:
                return 'unit'
        except Exception:
            pass
        # ค่าเริ่มต้นที่ปลอดภัยใน pymodbus รุ่นใหม่ ๆ
        return 'unit'

    def _call_write(self, addr: int, val: int):
        kw = {self._addr_kw: self.slave}
        return self.client.write_register(addr, val, **kw)

    def _call_read_hr(self, address: int, count: int):
        kw = {self._addr_kw: self.slave}
        return self.client.read_holding_registers(address=address, count=count, **kw)

    # ---------- low level ----------
    def _write(self, addr: int, val: int) -> bool:
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
                self.get_logger().error(f"write err @0x{addr:03X}: {e}")
                self.connected = False
                return False

    def _enable_rtu(self) -> bool:
        if self.transport == 'fake':
            return True
        return self._write(REG_ENABLE, 0x001)

    def _all_off(self) -> None:
        self.get_logger().info("ALL OFF")
        self._write(REG_GREEN,  VAL_OFF)
        self._write(REG_YELLOW, VAL_OFF)
        self._write(REG_RED,    VAL_OFF)
        self._write(REG_BUZZ,   VAL_OFF)

    def _apply_buzzer(self, want: bool) -> None:
        if self.enable_buzzer:
            self._write(REG_BUZZ, VAL_ON if want else VAL_OFF)

    def _set_green(self) -> None:
        self.last_cmd_value = 1
        self.get_logger().info("SET GREEN")
        self._all_off()
        self._write(REG_GREEN, VAL_ON)
        self._apply_buzzer(self.buzz_green and not self.emergency_active)

    def _set_yellow(self) -> None:
        self.last_cmd_value = 2
        self.get_logger().info("SET YELLOW")
        self._all_off()
        self._write(REG_YELLOW, VAL_ON)
        self._apply_buzzer(self.buzz_yellow and not self.emergency_active)

    def _set_red(self, buzzer: Optional[bool] = None) -> None:
        self.last_cmd_value = 3
        self.get_logger().info("SET RED")
        self._all_off()
        self._write(REG_RED, VAL_ON)
        if buzzer is None:
            buzzer = self.buzz_red and not self.emergency_active
        self._apply_buzzer(buzzer)

    def _restore_last(self) -> None:
        self.get_logger().info(f"RESTORE last={self.last_cmd_value}")
        {0: self._all_off, 1: self._set_green, 2: self._set_yellow, 3: self._set_red}.get(self.last_cmd_value, self._all_off)()

    def _arm_timer(self, delay: float, fn) -> None:
        t = self.create_timer(delay, lambda: self._fire_one_shot(t, fn), callback_group=self.cb)
        self.pending_timers.append(t)

    def _fire_one_shot(self, t, fn) -> None:
        try: fn()
        finally:
            try: t.cancel()
            except Exception: pass
            if t in self.pending_timers:
                self.pending_timers.remove(t)

    def _cancel_oneshots(self) -> None:
        for t in list(self.pending_timers):
            try: t.cancel()
            except Exception: pass
        self.pending_timers.clear()

    # ---------- connection / polling ----------
    def _ensure_connection(self, first: bool = False) -> None:
        if self.connected:
            return
        try:
            self.client.close()
        except Exception:
            pass
        self.connected = self.client.connect()
        self.get_logger().info(f"CONNECT -> {'OK' if self.connected else 'FAIL'} (transport={self.transport})")
        if self.connected:
            if not self._enable_rtu():
                self.get_logger().warn("⚠️ enable RTU failed (will keep trying on writes)")
            self._restore_last()
        elif first:
            self.get_logger().error("❌ cannot connect (will retry)")

    def _poll_registers(self) -> None:
        if not self.connected:
            return
        try:
            with self.modbus_lock:
                resp = self._call_read_hr(address=0x000, count=4)
            if not resp or resp.isError():
                raise ModbusIOException("read error")
        except Exception as e:
            self.get_logger().warn(f"poll fail: {e}")
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
        self._cancel_oneshots()
        if self.emergency_active:
            self.get_logger().error("🛑 EMERGENCY ACTIVATED!")
            self._set_red(buzzer=self.buzz_on_emergency)
        else:
            self.get_logger().info("✅ EMERGENCY DEACTIVATED")
            self._restore_last()


def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()
    executor = MultiThreadedExecutor(num_threads=2)
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
