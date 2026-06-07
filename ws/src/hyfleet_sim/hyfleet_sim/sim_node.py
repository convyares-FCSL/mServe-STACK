"""
hyfleet_sim.sim_node
====================
Simulated ADS bridge + PLC + physical plant for HyFleet compression tests.

Boundary
--------
    [ booster / coordinator nodes ]    (unchanged)
          |  BoosterCmd / CompressorCmd service calls   ^  CompressorTelemetry (~10 Hz)
          v                                             |
    [ CompressorSimNode ]  ==  ADS bridge + PLC + plant (simulated)

Physics (trivial, command-driven)
----------------------------------
  pcsv_enabled AND vfd_running  =>  outlet_p += bar_per_cpm * cpm * dt  (per tick)
  outlet_p >= target_pressure   =>  hold at target_pressure
  pcsv_enabled=False            =>  hold current pressure (no decay)
  inlet_p                       =>  constant inlet_pressure_bar (standalone)
                                    or low_booster outlet (SYNC mode)
  SYNC interstage draw          =>  when high compressing, low outlet falls at
                                    interstage_draw_bar_per_s; allows compress_hold
                                    maintain loop to be exercised
  VFD                           =>  instant on/off; vfd_speed_rpm reflects setpoint
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from mserve_interfaces.msg import CompressorTelemetry
from mserve_interfaces.srv import BoosterCmd, CompressorCmd
from std_srvs.srv import Trigger


# ---------------------------------------------------------------------------
# Per-booster mutable state
# ---------------------------------------------------------------------------

class BoosterState:
    """All runtime state for one simulated booster."""

    def __init__(self) -> None:
        self.vfd_running: bool = False
        self.vfd_speed_rpm: float = 0.0
        self.pcsv_enabled: bool = False
        self.cpm: float = 0.0

        self.outlet_p: float = 265.0
        self.inlet_p: float = 265.0

        self.sv: list[bool] = [False] * 5

        self.target_pressure: float = 900.0


# ---------------------------------------------------------------------------
# Simulator node
# ---------------------------------------------------------------------------

class CompressorSimNode(Node):
    """
    Single ROS 2 node that simulates the ADS bridge, PLC, and plant.

    Parameters
    ----------
    physics.bar_per_cpm              Rise rate (bar per cycle). Default 1.0.
    physics.physics_rate_hz          Physics tick rate. Default 100.0.
    physics.telemetry_rate_hz        Telemetry publish rate. Default 10.0.
    physics.sync_mode                High-booster inlet = low-booster outlet. Default false.
    physics.interstage_draw_bar_per_s  Rate at which high booster draws from interstage
                                       when compressing (bar/s). Only active in sync_mode.
                                       Default 8.0.

    low_booster / high_booster.*     Per-booster index maps and initial conditions.
    compressor.interstage_sv_index   Index of the interstage SV in sv[5]. Default 2.
    """

    BOOSTER_NAMES = ('low_booster', 'high_booster')

    def __init__(self) -> None:
        super().__init__('compressor_sim')

        # ------------------------------------------------------------------
        # Declare parameters
        # ------------------------------------------------------------------
        self.declare_parameter('physics.bar_per_cpm',                1.0)
        self.declare_parameter('physics.physics_rate_hz',            100.0)
        self.declare_parameter('physics.telemetry_rate_hz',          10.0)
        self.declare_parameter('physics.sync_mode',                  False)
        self.declare_parameter('physics.interstage_draw_bar_per_s',  8.0)

        defaults = {
            'low_booster': dict(
                inlet_pressure_bar=265.0,
                initial_outlet_bar=265.0,
                target_pressure_bar=500.0,
                inlet_pt_index=0,
                outlet_pt_index=7,
                inlet_tt_index=0,
                outlet_tt_index=2,
                vfd_index=0,
            ),
            'high_booster': dict(
                inlet_pressure_bar=265.0,
                initial_outlet_bar=265.0,
                target_pressure_bar=900.0,
                inlet_pt_index=1,
                outlet_pt_index=2,
                inlet_tt_index=4,
                outlet_tt_index=6,
                vfd_index=1,
            ),
        }
        for name, d in defaults.items():
            self.declare_parameter(f'{name}.inlet_pressure_bar',  d['inlet_pressure_bar'])
            self.declare_parameter(f'{name}.initial_outlet_bar',  d['initial_outlet_bar'])
            self.declare_parameter(f'{name}.target_pressure_bar', d['target_pressure_bar'])
            self.declare_parameter(f'{name}.inlet_pt_index',      d['inlet_pt_index'])
            self.declare_parameter(f'{name}.outlet_pt_index',     d['outlet_pt_index'])
            self.declare_parameter(f'{name}.inlet_tt_index',      d['inlet_tt_index'])
            self.declare_parameter(f'{name}.outlet_tt_index',     d['outlet_tt_index'])
            self.declare_parameter(f'{name}.vfd_index',           d['vfd_index'])

        self.declare_parameter('compressor.interstage_sv_index', 2)

        # ------------------------------------------------------------------
        # Read parameters
        # ------------------------------------------------------------------
        self._bar_per_cpm             = self.get_parameter('physics.bar_per_cpm').value
        self._physics_rate_hz         = self.get_parameter('physics.physics_rate_hz').value
        self._telemetry_rate_hz       = self.get_parameter('physics.telemetry_rate_hz').value
        self._sync_mode               = self.get_parameter('physics.sync_mode').value
        self._interstage_draw         = self.get_parameter('physics.interstage_draw_bar_per_s').value
        self._interstage_sv_index     = self.get_parameter('compressor.interstage_sv_index').value

        self._idx: dict[str, dict] = {}
        for name in self.BOOSTER_NAMES:
            self._idx[name] = {
                'inlet_pt':  self.get_parameter(f'{name}.inlet_pt_index').value,
                'outlet_pt': self.get_parameter(f'{name}.outlet_pt_index').value,
                'inlet_tt':  self.get_parameter(f'{name}.inlet_tt_index').value,
                'outlet_tt': self.get_parameter(f'{name}.outlet_tt_index').value,
                'vfd':       self.get_parameter(f'{name}.vfd_index').value,
            }

        self._initial: dict[str, dict] = {}
        self._state: dict[str, BoosterState] = {}
        for name in self.BOOSTER_NAMES:
            s = BoosterState()
            s.inlet_p         = self.get_parameter(f'{name}.inlet_pressure_bar').value
            s.outlet_p        = self.get_parameter(f'{name}.initial_outlet_bar').value
            s.target_pressure = self.get_parameter(f'{name}.target_pressure_bar').value
            self._state[name] = s
            self._initial[name] = {
                'inlet_p':         s.inlet_p,
                'outlet_p':        s.outlet_p,
                'target_pressure': s.target_pressure,
            }

        # Interstage SV state — owned by the coordinator, tracked separately
        self._interstage_sv: bool = False

        # ------------------------------------------------------------------
        # Services  — BoosterCmd per booster, CompressorCmd for coordinator
        # ------------------------------------------------------------------
        self._services: list = []
        for name in self.BOOSTER_NAMES:
            srv = self.create_service(
                BoosterCmd,
                f'/{name}/booster_cmd',
                self._make_booster_cmd_cb(name),
            )
            self._services.append(srv)
            self.get_logger().info(f'Advertising /{name}/booster_cmd')

        self._compressor_srv = self.create_service(
            CompressorCmd,
            '/hyfleet_compression/compressor_cmd',
            self._compressor_cmd_cb,
        )
        self.get_logger().info('Advertising /hyfleet_compression/compressor_cmd')

        # ------------------------------------------------------------------
        # Publisher  — CompressorTelemetry
        # ------------------------------------------------------------------
        telemetry_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self._telemetry_pub = self.create_publisher(
            CompressorTelemetry,
            'compressor_telemetry',
            telemetry_qos,
        )

        # ------------------------------------------------------------------
        # Timers
        # ------------------------------------------------------------------
        self._physics_timer   = self.create_timer(1.0 / self._physics_rate_hz,   self._physics_tick)
        self._telemetry_timer = self.create_timer(1.0 / self._telemetry_rate_hz, self._publish_telemetry)
        self.create_service(Trigger, '~/reset', self._reset_cb)

        self.get_logger().info(
            f'CompressorSimNode started — physics {self._physics_rate_hz} Hz, '
            f'telemetry {self._telemetry_rate_hz} Hz, '
            f'bar_per_cpm={self._bar_per_cpm}, sync_mode={self._sync_mode}, '
            f'interstage_draw={self._interstage_draw} bar/s'
        )

    # ------------------------------------------------------------------
    # BoosterCmd service callback factory
    # ------------------------------------------------------------------

    def _make_booster_cmd_cb(self, booster_name: str):
        def cb(request: BoosterCmd.Request, response: BoosterCmd.Response):
            s = self._state[booster_name]
            cmd = request.cmd

            if cmd == BoosterCmd.Request.START_VFD:
                s.vfd_running = True
                s.vfd_speed_rpm = request.setpoint
                response.success = True
                response.message = f'{booster_name}: VFD started at {request.setpoint:.1f} rpm'
                self.get_logger().info(response.message)

            elif cmd == BoosterCmd.Request.STOP_VFD:
                s.vfd_running = False
                s.vfd_speed_rpm = 0.0
                response.success = True
                response.message = f'{booster_name}: VFD stopped'
                self.get_logger().info(response.message)

            elif cmd == BoosterCmd.Request.SET_PCSV:
                s.pcsv_enabled = request.enable
                if request.enable:
                    s.cpm = request.setpoint
                response.success = True
                state_str = f'enabled at {s.cpm:.1f} cpm' if request.enable else 'disabled'
                response.message = f'{booster_name}: PCSV {state_str}'
                self.get_logger().info(response.message)

            elif cmd == BoosterCmd.Request.CONTROL_SV:
                idx = int(request.index)
                if 0 <= idx < len(s.sv):
                    s.sv[idx] = request.enable
                    response.success = True
                    response.message = f'{booster_name}: SV[{idx}] set to {request.enable}'
                else:
                    response.success = False
                    response.message = (
                        f'{booster_name}: SV index {idx} out of range (0-{len(s.sv) - 1})'
                    )
                self.get_logger().info(response.message)

            else:
                response.success = False
                response.message = f'{booster_name}: unknown cmd {cmd}'
                self.get_logger().warn(response.message)

            return response

        return cb

    # ------------------------------------------------------------------
    # CompressorCmd service callback
    # ------------------------------------------------------------------

    def _compressor_cmd_cb(self, request: CompressorCmd.Request, response: CompressorCmd.Response):
        cmd = request.cmd

        if cmd == CompressorCmd.Request.CONTROL_SV:
            idx = int(request.index)
            if idx == self._interstage_sv_index:
                self._interstage_sv = request.enable
                response.success = True
                response.message = f'compressor: interstage SV[{idx}] set to {request.enable}'
            else:
                response.success = False
                response.message = f'compressor: SV index {idx} not managed by coordinator'
            self.get_logger().info(response.message)

        elif cmd == CompressorCmd.Request.CONTROL_HEATER:
            response.success = True
            response.message = f'compressor: heater enable={request.enable} setpoint={request.setpoint}'
            self.get_logger().info(response.message)

        else:
            response.success = False
            response.message = f'compressor: unknown cmd {cmd}'
            self.get_logger().warn(response.message)

        return response

    # ------------------------------------------------------------------
    # Reset service
    # ------------------------------------------------------------------

    def _reset_cb(self, _: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        for name in self.BOOSTER_NAMES:
            s = self._state[name]
            init = self._initial[name]
            s.outlet_p        = init['outlet_p']
            s.inlet_p         = init['inlet_p']
            s.target_pressure = init['target_pressure']
            s.vfd_running     = False
            s.vfd_speed_rpm   = 0.0
            s.pcsv_enabled    = False
            s.cpm             = 0.0
            s.sv              = [False] * 5
        self._interstage_sv = False
        self.get_logger().info('Sim reset to initial conditions')
        response.success = True
        response.message = 'reset OK'
        return response

    # ------------------------------------------------------------------
    # Physics tick  (~100 Hz)
    # ------------------------------------------------------------------

    def _physics_tick(self) -> None:
        dt = 1.0 / self._physics_rate_hz

        if self._sync_mode:
            # High booster inlet tracks low booster outlet
            self._state['high_booster'].inlet_p = self._state['low_booster'].outlet_p

        for name in self.BOOSTER_NAMES:
            s = self._state[name]
            if s.pcsv_enabled and s.vfd_running:
                s.outlet_p = min(
                    s.outlet_p + self._bar_per_cpm * s.cpm * dt,
                    s.target_pressure,
                )

        # SYNC interstage draw: when high booster is compressing it draws from
        # the interstage (low booster outlet), causing pressure to decay.
        # This allows the compress_hold maintain loop to be exercised in tests.
        if self._sync_mode:
            high = self._state['high_booster']
            low  = self._state['low_booster']
            if high.pcsv_enabled and high.vfd_running:
                low.outlet_p = max(
                    low.outlet_p - self._interstage_draw * dt,
                    low.inlet_p,    # can't drop below supply
                )

    # ------------------------------------------------------------------
    # Telemetry publish  (~10 Hz)
    # ------------------------------------------------------------------

    def _publish_telemetry(self) -> None:
        msg = CompressorTelemetry()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.mode = CompressorTelemetry.AUTO

        msg.pt_bar        = [0.0] * 16
        msg.tt_celsius    = [0.0] * 12
        msg.sv            = [False] * 5
        msg.ps            = [False] * 4
        msg.vfd_state     = [CompressorTelemetry.VFD_OFFLINE] * 2
        msg.vfd_speed_rpm = [0.0] * 2
        msg.vfd_energy_kj = [0.0] * 2
        msg.vfd_power_kw  = [0.0] * 2

        for name in self.BOOSTER_NAMES:
            s   = self._state[name]
            idx = self._idx[name]

            msg.pt_bar[idx['inlet_pt']]  = s.inlet_p
            msg.pt_bar[idx['outlet_pt']] = s.outlet_p

            msg.tt_celsius[idx['inlet_tt']]  = 20.0
            msg.tt_celsius[idx['outlet_tt']] = 20.0

            vfd_i = idx['vfd']
            if s.vfd_running:
                msg.vfd_state[vfd_i]     = CompressorTelemetry.VFD_RUNNING
                msg.vfd_speed_rpm[vfd_i] = s.vfd_speed_rpm
            else:
                msg.vfd_state[vfd_i]     = CompressorTelemetry.VFD_IDLE
                msg.vfd_speed_rpm[vfd_i] = 0.0

            for i, state in enumerate(s.sv):
                msg.sv[i] = state

        # Coordinator-owned SV (interstage)
        msg.sv[self._interstage_sv_index] = self._interstage_sv

        self._telemetry_pub.publish(msg)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    rclpy.init(args=args)
    node = CompressorSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
