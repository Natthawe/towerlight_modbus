#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import logging
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from std_msgs.msg import Int32, Bool

from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException, ConnectionException

logging.getLogger('pymodbus').setLevel(logging.CRITICAL)


class ModbusNode(Node):
    def __init__(self):
        super().__init__('modbus_node')

        # ---------- Parameters ----------
        self.declare_parameter('port', '/dev/towerlight')
        self.declare_parameter('baudrate', 9600)
        self.declare_parameter('parity', 'N')     # 'N','E','O'
        self.declare_parameter('stopbits', 1)
        self.declare_parameter('bytesize', 8)
        self.declare_parameter('timeout', 2.0)
        self.declare_parameter('slave_id', 1)

        self.declare_parameter('poll_period', 3.0)               # sec
        self.declare_parameter('reconnect_period', 3.0)          # sec
        self.declare_parameter('turn_green_delay', 5.0)          # sec after startup red
        self.declare_parameter('emergency_auto_trigger_delay', 5.0)  # sec after green

        self.declare_parameter('enable_buzzer', True)
        # NEW: per-color buzzer policy
        self.declare_parameter('buzzer_on_red', True)
        self.declare_parameter('buzzer_on_yellow', False)
        self.declare_parameter('buzzer_on_green', False)
        self.declare_parameter('buzzer_on_emergency', True)  # emergency red only

        self.cfg = {
            'port': self.get_parameter('port').get_parameter_value().string_value,
            'baudrate': int(self.get_parameter('baudrate').value),
            'parity': self.get_parameter('parity').get_parameter_value().string_value.upper(),
            'stopbits': int(self.get_parameter('stopbits').value),
            'bytesize': int(self.get_parameter('bytesize').value),
            'timeout': float(self.get_parameter('timeout').value),
            'slave_id': int(self.get_parameter('slave_id').value),

            'poll_period': float(self.get_parameter('poll_period').value),
            'reconnect_period': float(self.get_parameter('reconnect_period').value),
            'turn_green_delay': float(self.get_parameter('turn_green_delay').value),
            'emergency_auto_trigger_delay': float(self.get_parameter('emergency_auto_trigger_delay').value),

            'enable_buzzer': bool(self.get_parameter('enable_buzzer').value),
            'buzzer_on_red': bool(self.get_parameter('buzzer_on_red').value),
            'buzzer_on_yellow': bool(self.get_parameter('buzzer_on_yellow').value),
            'buzzer_on_green': bool(self.get_parameter('buzzer_on_green').value),
            'buzzer_on_emergency': bool(self.get_parameter('buzzer_on_emergency').value),
        }

        # ---------- Modbus client & lock ----------
        self.client = ModbusSerialClient(
            port=self.cfg['port'],
            baudrate=self.cfg['baudrate'],
            parity=self.cfg['parity'],
            stopbits=self.cfg['stopbits'],
            bytesize=self.cfg['bytesize'],
            timeout=self.cfg['timeout'],
        )
        self._mb_lock = threading.Lock()
        self.connected: bool = False
        self.initialized: bool = False  # MODBUS RTU enabled?

        # ---------- State ----------
        # เริ่มด้วย emergency = True (แดง) เพื่อ safety
        self.emergency_active: bool = True
        self.timer_emergency_auto: Optional[Timer] = None
        self.timer_startup_green: Optional[Timer] = None

        # ---------- Subscriptions ----------
        self.subscription = self.create_subscription(Int32, 'stopRobotX', self._monitor_cb, 10)
        self.emergency_sub = self.create_subscription(Bool, 'emergency_stop', self._emergency_cb, 10)

        # ---------- Periodic timers ----------
        self.reconnect_timer = self.create_timer(self.cfg['reconnect_period'], self._reconnect_tick)
        self.poll_timer = self.create_timer(self.cfg['poll_period'], self._poll_tick)

        # ---------- Boot: kick off connection sequence ----------
        self._reconnect_tick()  # call immediately

        self.get_logger().info(f'🔧 Config: {self.cfg}')
        self.get_logger().info(f'🚨 Emergency State (start): {self.emergency_active}')

    # =================== Helpers (Modbus safe ops) ===================
    def _safe_write_register(self, address: int, value: int, slave: Optional[int] = None) -> bool:
        if not self.connected:
            self.get_logger().warn('⚠️ write_register skipped: not connected')
            return False
        slave = self.cfg['slave_id'] if slave is None else slave
        try:
            with self._mb_lock:
                resp = self.client.write_register(address, value, slave=slave)
            if resp.isError():
                self.get_logger().error(f'❌ write_register({address:#05x},{value}) error')
                return False
            return True
        except (ModbusIOException, ConnectionException) as e:
            self.get_logger().error(f'❌ Modbus write exception: {e}')
            self._mark_disconnected()
            return False
        except Exception as e:
            self.get_logger().error(f'❌ Unexpected write exception: {e}')
            self._mark_disconnected()
            return False

    def _safe_read_holding(self, address: int, count: int, slave: Optional[int] = None):
        if not self.connected:
            self.get_logger().warn('⚠️ read_holding skipped: not connected')
            return None
        slave = self.cfg['slave_id'] if slave is None else slave
        try:
            with self._mb_lock:
                resp = self.client.read_holding_registers(address=address, count=count, slave=slave)
            if resp.isError():
                self.get_logger().error('❌ read_holding error')
                return None
            return resp.registers
        except (ModbusIOException, ConnectionException) as e:
            self.get_logger().error(f'❌ Modbus read exception: {e}')
            self._mark_disconnected()
            return None
        except Exception as e:
            self.get_logger().error(f'❌ Unexpected read exception: {e}')
            self._mark_disconnected()
            return None

    def _mark_disconnected(self):
        if self.connected:
            self.get_logger().warn('⚠️ Marking client disconnected')
        self.connected = False
        self.initialized = False

    # =================== Connection / Init ===================
    def _reconnect_tick(self):
        if self.connected:
            # ensure initialized (MODBUS RTU enable) once after connect
            if not self.initialized:
                self._enable_modbus_rtu_once()
            return

        try:
            with self._mb_lock:
                try:
                    self.client.close()
                except Exception:
                    pass
                ok = self.client.connect()
            self.connected = bool(ok)
        except Exception as e:
            self.get_logger().error(f'❌ connect() exception: {e}')
            self.connected = False

        if self.connected:
            self.get_logger().info('✅ Connected to Modbus device')
            self._set_only_red()
            self.initialized = False
            self._schedule_startup_green()
        else:
            self.get_logger().error('❌ Reconnection failed')

    def _enable_modbus_rtu_once(self):
        ok = self._safe_write_register(0x006, 0x001, self.cfg['slave_id'])
        if ok:
            self.initialized = True
            self.get_logger().info('✅ MODBUS RTU enabled')
        else:
            self.get_logger().warn('⏳ Will retry enabling MODBUS RTU on next reconnect tick')

    # =================== Timed sequences (no sleep) ===================
    def _schedule_startup_green(self):
        if self.timer_startup_green is not None:
            self.timer_startup_green.cancel()
            self.timer_startup_green = None
        self.timer_startup_green = self.create_timer(self.cfg['turn_green_delay'], self._turn_green_once)

    def _turn_green_once(self):
        if self.timer_startup_green is not None:
            self.timer_startup_green.cancel()
            self.timer_startup_green = None

        if not self.initialized:
            self.get_logger().warn('⛔ skip turn_green: RTU not enabled yet')
            return

        self._set_only_green()

        if self.cfg['emergency_auto_trigger_delay'] > 0:
            if self.timer_emergency_auto is not None:
                self.timer_emergency_auto.cancel()
            self.timer_emergency_auto = self.create_timer(
                self.cfg['emergency_auto_trigger_delay'],
                self._auto_trigger_emergency_once
            )

    def _auto_trigger_emergency_once(self):
        if self.timer_emergency_auto is not None:
            self.timer_emergency_auto.cancel()
            self.timer_emergency_auto = None
        self.get_logger().info('🔄 Auto-triggering emergency (test)')
        self._set_emergency(True)

    # =================== Poller ===================
    def _poll_tick(self):
        if not self.connected or not self.initialized:
            return
        regs = self._safe_read_holding(0x000, 10, self.cfg['slave_id'])
        if regs is not None:
            self.get_logger().debug(f'📖 Holding[0..9]={regs}')

    # =================== Subscriptions ===================
    def _monitor_cb(self, msg: Int32):
        if self.emergency_active:
            return
        value = int(msg.data)
        self.get_logger().info(f'Received monitor value: {value}')
        self._control_light(value)

    def _emergency_cb(self, msg: Bool):
        self._set_emergency(bool(msg.data))

    # =================== State handlers ===================
    def _set_emergency(self, active: bool):
        if active == self.emergency_active:
            return
        self.emergency_active = active
        if self.emergency_active:
            self.get_logger().error('🛑 EMERGENCY ACTIVATED!')
            # emergency red can have its own buzzer policy
            self._all_off()
            self._safe_write_register(self.RED, self.VAL_ON)
            if self.cfg['enable_buzzer'] and self.cfg['buzzer_on_emergency']:
                self._safe_write_register(self.BUZZER, self.VAL_ON)
            if self.timer_emergency_auto is not None:
                self.timer_emergency_auto.cancel()
                self.timer_emergency_auto = None
        else:
            self.get_logger().info('✅ EMERGENCY DEACTIVATED')
            self._set_only_green()

    # =================== Light/Buzzer primitives ===================
    GREEN = 0x002
    YELLOW = 0x003
    RED = 0x004
    BUZZER = 0x005
    VAL_ON = 0x002
    VAL_OFF = 0x000

    def _all_off(self):
        if not self.connected:
            return
        self._safe_write_register(self.GREEN, self.VAL_OFF)
        self._safe_write_register(self.YELLOW, self.VAL_OFF)
        self._safe_write_register(self.RED, self.VAL_OFF)
        self._safe_write_register(self.BUZZER, self.VAL_OFF)

    def _set_only_red(self):
        self._all_off()
        self._safe_write_register(self.RED, self.VAL_ON)
        if self.cfg['enable_buzzer'] and self.cfg['buzzer_on_red']:
            self._safe_write_register(self.BUZZER, self.VAL_ON)

    def _set_only_green(self):
        self._all_off()
        self._safe_write_register(self.GREEN, self.VAL_ON)
        if self.cfg['enable_buzzer'] and self.cfg['buzzer_on_green']:
            self._safe_write_register(self.BUZZER, self.VAL_ON)

    def _set_only_yellow(self):
        self._all_off()
        self._safe_write_register(self.YELLOW, self.VAL_ON)
        if self.cfg['enable_buzzer'] and self.cfg['buzzer_on_yellow']:
            self._safe_write_register(self.BUZZER, self.VAL_ON)

    def _control_light(self, value: int):
        # 0=off, 1=green, 2=yellow, 3=red
        # if value == 0:
        #     self._all_off()
        # elif value == 1:
        #     self._set_only_green()
        # elif value == 2:
        #     self._set_only_yellow()
        # elif value == 3:
        #     self._set_only_red()
        # else:
        #     self.get_logger().warn(f'Unknown value: {value}')

        if value == 5:
            self._all_off()
        elif value == 0:
            self._set_only_green()
        elif value == 6:
            self._set_only_yellow()
        elif value == 1:
            self._set_only_red()
        else:
            self.get_logger().warn(f'Unknown value: {value}')

    # =================== Cleanup ===================
    def destroy_node(self):
        try:
            self.get_logger().info('🔚 Cleaning up: turning all lights off')
            self._all_off()
        except Exception as e:
            self.get_logger().warn(f'cleanup error: {e}')
        try:
            with self._mb_lock:
                try:
                    self.client.close()
                except Exception:
                    pass
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ModbusNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
