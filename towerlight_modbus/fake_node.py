# final_emergency_towerlight_node.py  (verbose logging)
import threading
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32, Bool
from pymodbus.client import ModbusSerialClient, ModbusTcpClient
from pymodbus.exceptions import ModbusIOException
import logging
logging.getLogger('pymodbus').setLevel(logging.CRITICAL)

REG_GREEN, REG_YELLOW, REG_RED, REG_BUZZ, REG_ENABLE = 0x002, 0x003, 0x004, 0x005, 0x006
VAL_ON, VAL_OFF = 0x002, 0x000

class FakeClient:
    def __init__(self): self.reg = {}
    def connect(self): return True
    def close(self): pass
    def write_register(self, addr, val, slave=1):
        # เหลือ print ไว้ด้วย เผื่อดู stdout
        print(f"[FAKE] write 0x{addr:03X} <- {val}")
        class R: 
            def isError(self): return False
        self.reg[addr]=val
        return R()
    def read_holding_registers(self, address=0, count=1, slave=1):
        class R:
            def __init__(self, ok): self._ok=ok
            def isError(self): return not self._ok
        return R(True)

class ModbusNode(Node):
    def __init__(self):
        super().__init__('fake_node_params')  # ชื่อชัด ๆ ให้จำง่าย
        self.cb = ReentrantCallbackGroup()

        # ------- parameters -------
        self.declare_parameter('transport', 'fake')  # 'serial'|'tcp'|'fake'
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port_tcp', 5020)
        self.declare_parameter('port', '/dev/towerlight')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('parity', 'N')
        self.declare_parameter('stopbits', 1)
        self.declare_parameter('bytesize', 8)
        self.declare_parameter('timeout', 0.3)
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
        self.transport = g('transport').value
        self.host = g('host').value
        self.port_tcp = int(g('port_tcp').value)
        self.port = g('port').value
        self.baudrate = int(g('baudrate').value)
        self.parity = g('parity').value
        self.stopbits = int(g('stopbits').value)
        self.bytesize = int(g('bytesize').value)
        self.timeout = float(g('timeout').value)
        self.slave = int(g('slave_id').value)
        self.poll_period = float(g('poll_period').value)
        self.reconnect_period = float(g('reconnect_period').value)
        self.turn_green_delay = float(g('turn_green_delay').value)
        self.auto_emerg_delay = float(g('emergency_auto_trigger_delay').value)
        self.enable_buzzer = bool(g('enable_buzzer').value)
        self.buzz_red = bool(g('buzzer_on_red').value)
        self.buzz_yellow = bool(g('buzzer_on_yellow').value)
        self.buzz_green = bool(g('buzzer_on_green').value)
        self.buzz_on_emergency = bool(g('buzzer_on_emergency').value)

        # client
        if self.transport == 'serial':
            self.client = ModbusSerialClient(port=self.port, baudrate=self.baudrate,
                                             parity=self.parity, stopbits=self.stopbits,
                                             bytesize=self.bytesize, timeout=self.timeout)
        elif self.transport == 'tcp':
            self.client = ModbusTcpClient(self.host, port=self.port_tcp, timeout=self.timeout)
        else:
            self.client = FakeClient()

        self.modbus_lock = threading.Lock()
        self.connected = False
        self.emergency_active = False
        self.last_cmd_value = 0
        self.pending_timers = []

        self.create_subscription(Int32, 'monitor_topic', self.listener_callback, 10, callback_group=self.cb)
        self.create_subscription(Bool,  'emergency_stop', self.emergency_callback, 10, callback_group=self.cb)

        self.create_timer(self.reconnect_period, self._ensure_connection, callback_group=self.cb)
        self.create_timer(self.poll_period, self._poll_registers, callback_group=self.cb)

        self._ensure_connection(first=True)
        if self.turn_green_delay > 0: self._arm_timer(self.turn_green_delay, self._set_green)
        else: self._set_green()
        if self.auto_emerg_delay > 0: self._arm_timer(self.auto_emerg_delay, lambda: self.emergency_callback(Bool(data=True)))
        self.get_logger().info(f"✅ node started (transport={self.transport})")

    # ---- helpers ----
    def _write(self, addr, val):
        if not self.connected:
            self.get_logger().warn(f"skip write 0x{addr:03X} (not connected)")
            return False
        with self.modbus_lock:
            try:
                r = self.client.write_register(addr, val, slave=self.slave)
                ok = (r is not None and not r.isError())
                self.get_logger().info(f"WRITE 0x{addr:03X} = {val} -> {'OK' if ok else 'ERR'}")
                return ok
            except Exception as e:
                self.get_logger().error(f"write err @0x{addr:03X}: {e}")
                self.connected=False
                return False

    def _enable_rtu(self):
        if self.transport == 'fake':
            return True
        return self._write(REG_ENABLE, 0x001)

    def _all_off(self):
        self.get_logger().info("ALL OFF")
        self._write(REG_GREEN, VAL_OFF); self._write(REG_YELLOW, VAL_OFF)
        self._write(REG_RED, VAL_OFF);   self._write(REG_BUZZ, VAL_OFF)

    def _apply_buzz(self, want):
        if self.enable_buzzer:
            self._write(REG_BUZZ, VAL_ON if want else VAL_OFF)

    def _set_green(self):
        self.last_cmd_value=1
        self.get_logger().info("SET GREEN")
        self._all_off(); self._write(REG_GREEN, VAL_ON)
        self._apply_buzz(self.buzz_green and not self.emergency_active)

    def _set_yellow(self):
        self.last_cmd_value=2
        self.get_logger().info("SET YELLOW")
        self._all_off(); self._write(REG_YELLOW, VAL_ON)
        self._apply_buzz(self.buzz_yellow and not self.emergency_active)

    def _set_red(self, buzzer=None):
        self.last_cmd_value=3
        self.get_logger().info("SET RED")
        self._all_off(); self._write(REG_RED, VAL_ON)
        if buzzer is None: buzzer = self.buzz_red and not self.emergency_active
        self._apply_buzz(buzzer)

    def _restore_last(self):
        self.get_logger().info(f"RESTORE last={self.last_cmd_value}")
        {0:self._all_off,1:self._set_green,2:self._set_yellow,3:self._set_red}.get(self.last_cmd_value,self._all_off)()

    def _arm_timer(self, delay, fn):
        t = self.create_timer(delay, lambda: self._fire_one_shot(t, fn), callback_group=self.cb)
        self.pending_timers.append(t)

    def _fire_one_shot(self, t, fn):
        try: fn()
        finally:
            try: t.cancel()
            except: pass
            if t in self.pending_timers: self.pending_timers.remove(t)

    def _cancel_oneshots(self):
        for t in list(self.pending_timers):
            try: t.cancel()
            except: pass
        self.pending_timers.clear()

    def _ensure_connection(self, first=False):
        if self.connected: return
        try:
            self.client.close()
        except: pass
        self.connected = self.client.connect()
        self.get_logger().info(f"CONNECT -> {'OK' if self.connected else 'FAIL'} (transport={self.transport})")
        if self.connected:
            self._enable_rtu()
            self._restore_last()
        elif first:
            self.get_logger().error("❌ cannot connect (waiting for simulator?)")

    def _poll_registers(self):
        if not self.connected: return
        try:
            with self.modbus_lock:
                r = self.client.read_holding_registers(address=0x000, count=4, slave=self.slave)
            if not r or r.isError(): raise ModbusIOException("read error")
        except Exception as e:
            self.get_logger().warn(f"poll fail: {e}")
            self.connected=False

    # ---- topics ----
    def listener_callback(self, msg: Int32):
        self.get_logger().info(f"RX /monitor_topic: {msg.data} (emergency={self.emergency_active})")
        if self.emergency_active: 
            self.get_logger().info("ignore because emergency active")
            return
        v=int(msg.data)
        if   v==1: self._set_green()
        elif v==2: self._set_yellow()
        elif v==3: self._set_red()
        else: self.last_cmd_value=0; self._all_off()

    def emergency_callback(self, msg: Bool):
        new = bool(msg.data)
        self.get_logger().info(f"RX /emergency_stop: {new}")
        if new == self.emergency_active: 
            self.get_logger().info("no change")
            return
        self.emergency_active = new
        self._cancel_oneshots()
        if new:
            self.get_logger().error("🛑 EMERGENCY ON")
            self._set_red(buzzer=self.buzz_on_emergency)
        else:
            self.get_logger().info("✅ EMERGENCY OFF")
            self._restore_last()

def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()
    ex = MultiThreadedExecutor(num_threads=2)
    rclpy.spin(node, executor=ex)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
