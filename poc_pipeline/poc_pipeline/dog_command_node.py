#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import placo
import numpy as np
from placo_utils.visualization import robot_viz, robot_frame_viz, frame_viz, points_viz
from placo_utils.tf import tf
import os
from ament_index_python import get_package_share_directory

class DogCommandNode(Node):
    def __init__(self):
        super().__init__('dog_command_node')

        self.subscription = self.create_subscription(
            String, '/dog_goal', self.goal_callback, 10
        )
        self.pub = self.create_publisher(JointState, '/joint_targets', 10)

        self.robot = placo.RobotWrapper(
            "/home/nishant/MRT/robodog_gazebo/src/rover_gazebosim/urdf/placosucks.urdf",
            placo.Flags.ignore_collisions
        )
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)
        self.solver.enable_velocity_limits(True)

        self.robot.update_kinematics()

        print("Joint names:", self.robot.joint_names())
        for leg in ['LFfoot', 'RFfoot', 'LRfoot', 'RRfoot']:
            T = self.robot.get_T_world_frame(leg)
            print(f"{leg} position: {T[:3, 3]}")

        self.step_length = 0.1
        self.step_height = 0.1
        self.gait_freq   = 0.5
        self.angular_velocity = 0.0

        self.body_center_x = 0.6678
        self.body_center_y = -1.375

        self.leg_configs = {
            'LFfoot': {
                'x': 0.56791464,
                'y_center': -1.2279557,
                'z_center': 0.67243946,
                'phase': 0.0,
                'z_offset': -0.02,
            },
            'RRfoot': {
                'x': 0.74281485,
                'y_center': -1.52181324,
                'z_center': 0.66312896,
                'phase': 0.0,
                'z_offset': -0.02,
            },
            'RFfoot': {
                'x': 0.75971348,
                'y_center': -1.26101703,
                'z_center': 0.67581485,
                'phase': np.pi,
                'z_offset': -0.02,
            },
            'LRfoot': {
                'x': 0.56492413,
                'y_center': -1.53388919,
                'z_center': 0.64826712,
                'phase': np.pi,
                'z_offset': -0.02,
            },
        }

        self.leg_turn_params = {}
        for leg, cfg in self.leg_configs.items():
            dx = cfg['x'] - self.body_center_x
            dy = cfg['y_center'] - self.body_center_y
            radius = np.sqrt(dx**2 + dy**2)
            base_angle = np.arctan2(dy, dx)
            self.leg_turn_params[leg] = {
                'radius': radius,
                'base_angle': base_angle,
            }

        self.turn_leg_order = ['LFfoot', 'RRfoot', 'RFfoot', 'LRfoot']
        self.turn_leg_index = 0
        self.turn_leg_progress = 0.0
        self.turn_step_speed = 1.0
        self.turn_step_sideways = 0.04

        self.effector_tasks = {}
        for leg, cfg in self.leg_configs.items():
            task = self.solver.add_position_task(
                leg,
                np.array([cfg['x'], cfg['y_center'], cfg['z_center']])
            )
            task.configure(leg, "soft", 5.0)
            self.effector_tasks[leg] = task

        self.foot_hold_pos = {}
        for leg, cfg in self.leg_configs.items():
            self.foot_hold_pos[leg] = np.array([
                cfg['x'],
                cfg['y_center'],
                cfg['z_center'] + cfg['z_offset']
            ])

        self.viz = robot_viz(self.robot)
        self.t = 0.0
        self.dt = 0.01
        self.solver.dt = self.dt
        self.last_targets = []
        self.last_target_t = 0.0
        self.latest_joint_angles = [0.0] * 12

        self.mode = 'walk'

        # Reset mode
        self.reset_t = 0.0
        self.reset_duration = 2.0
        self.initial_angles = [0.0] * 12
        self.reset_start_angles = [0.0] * 12

        # Custom pose mode
        self.custom_target_angles = [0.0] * 12
        self.custom_current_angles = [0.0] * 12
        self.custom_t = 0.0
        self.custom_duration = 3.0
        self.custom_start_angles = [0.0] * 12

        # Handshake mode
        self.handshake_t = 0.0
        self.handshake_duration = 6.0
        rf_cfg = self.leg_configs['RFfoot']
        self.rf_rest = [
            rf_cfg['x'],
            rf_cfg['y_center'],
            rf_cfg['z_center'] + rf_cfg['z_offset'],
        ]

        # Warmup — solver-free interpolation back to neutral before re-engaging IK
        self.needs_ik_warmup = False
        self.warmup_t = 0.0
        self.warmup_duration = 1.5

        self.timer = self.create_timer(self.dt, self.loop)
        self.get_logger().info('Dog Command Node started')

    # ─────────────────────────────────────────────
    #  Walk
    # ─────────────────────────────────────────────
    def _do_walk(self):
        gait_t = self.t * self.gait_freq * 2 * np.pi

        for leg, cfg in self.leg_configs.items():
            phase = gait_t + cfg['phase']
            t_mod = phase % (2 * np.pi)
            in_swing = (t_mod < np.pi)
            swing_progress = t_mod / np.pi

            if in_swing:
                x = cfg['x']
                y = cfg['y_center'] + self.step_length * (swing_progress - 0.5)
                z = cfg['z_center'] + cfg['z_offset'] + self.step_height * np.sin(swing_progress * np.pi)
            else:
                stance_progress = (t_mod - np.pi) / np.pi
                x = cfg['x']
                y = cfg['y_center'] + self.step_length * (0.5 - stance_progress)
                z = cfg['z_center'] + cfg['z_offset']

            self.effector_tasks[leg].target_world = np.array([x, y, z])

    # ─────────────────────────────────────────────
    #  Warmup — solver-free, writes directly to
    #  robot.state.q to avoid infeasible QP
    # ─────────────────────────────────────────────
    def _do_warmup(self):
        self.warmup_t += self.dt
        p = min(self.warmup_t / self.warmup_duration, 1.0)
        p_smooth = 0.5 - 0.5 * np.cos(p * np.pi)

        # Interpolate joint angles from current pose to neutral (0.0 rad = 90deg servo)
        # Solver is NOT called — bypass IK entirely like custom/reset modes
        interpolated = [
            float(self.latest_joint_angles[i] + p_smooth * (0.0 - self.latest_joint_angles[i]))
            for i in range(12)
        ]
        self.latest_joint_angles = interpolated
        self.robot.state.q[7:19] = np.array(interpolated)
        self.robot.update_kinematics()
        self.viz.display(self.robot.state.q)

        if p >= 1.0:
            self.warmup_t = 0.0
            self.needs_ik_warmup = False
            self.get_logger().info('Warmup complete — walking normally')

    # ─────────────────────────────────────────────
    #  Turn — sideways hops one leg at a time
    # ─────────────────────────────────────────────
    def _get_sideways_hop(self, leg):
        sign = np.sign(self.angular_velocity) if self.angular_velocity != 0 else 1.0
        if leg in ('LFfoot', 'LRfoot'):
            dx = -sign * self.turn_step_sideways
        else:
            dx = +sign * self.turn_step_sideways
        return dx, 0.0

    def _do_turn(self):
        self.turn_leg_progress += self.turn_step_speed * self.dt
        active_leg = self.turn_leg_order[self.turn_leg_index]

        if self.turn_leg_progress >= 1.0:
            dx, _ = self._get_sideways_hop(active_leg)
            self.foot_hold_pos[active_leg] = np.array([
                self.foot_hold_pos[active_leg][0] + dx,
                self.foot_hold_pos[active_leg][1],
                self.leg_configs[active_leg]['z_center'] + self.leg_configs[active_leg]['z_offset']
            ])
            self.turn_leg_index = (self.turn_leg_index + 1) % len(self.turn_leg_order)
            self.turn_leg_progress = 0.0
            active_leg = self.turn_leg_order[self.turn_leg_index]

        p = self.turn_leg_progress

        for leg, cfg in self.leg_configs.items():
            if leg == active_leg:
                dx, _ = self._get_sideways_hop(leg)
                start_x = self.foot_hold_pos[leg][0]
                end_x   = start_x + dx
                x = start_x + (end_x - start_x) * p
                y = self.foot_hold_pos[leg][1]
                z = cfg['z_center'] + cfg['z_offset'] + self.step_height * np.sin(p * np.pi)
            else:
                pos = self.foot_hold_pos[leg]
                x, y, z = pos[0], pos[1], pos[2]

            self.effector_tasks[leg].target_world = np.array([x, y, z])

    # ─────────────────────────────────────────────
    #  Handshake
    # ─────────────────────────────────────────────
    def _do_handshake(self):
        self.handshake_t += self.dt
        progress = self.handshake_t / self.handshake_duration

        raise_height = 0.12
        extend_y     = 0.05

        for leg in ['LFfoot', 'RRfoot', 'LRfoot']:
            cfg = self.leg_configs[leg]
            self.effector_tasks[leg].target_world = np.array([
                cfg['x'],
                cfg['y_center'],
                cfg['z_center'] + cfg['z_offset']
            ])

        if progress < 0.3:
            p = progress / 0.3
            z = self.rf_rest[2] + raise_height * np.sin(p * np.pi / 2)
            y = self.rf_rest[1]
        elif progress < 0.7:
            p = (progress - 0.3) / 0.4
            z = self.rf_rest[2] + raise_height
            y = self.rf_rest[1] + extend_y * np.sin(p * 2 * np.pi)
        elif progress < 1.0:
            p = (progress - 0.7) / 0.3
            z = self.rf_rest[2] + raise_height * (1.0 - p)
            y = self.rf_rest[1]
        else:
            z = self.rf_rest[2]
            y = self.rf_rest[1]
            self.handshake_t = 0.0
            self.mode = 'walk'
            self.get_logger().info('Handshake complete — resuming walk')

        self.effector_tasks['RFfoot'].target_world = np.array([
            self.rf_rest[0], y, z
        ])

    # ─────────────────────────────────────────────
    #  Reset — interpolate to neutral
    # ─────────────────────────────────────────────
    def _do_reset(self):
        self.reset_t += self.dt
        p = min(self.reset_t / self.reset_duration, 1.0)
        p_smooth = 0.5 - 0.5 * np.cos(p * np.pi)

        interpolated = [
            float(self.reset_start_angles[i] + p_smooth * (self.initial_angles[i] - self.reset_start_angles[i]))
            for i in range(12)
        ]
        self.latest_joint_angles = interpolated
        self.robot.state.q[7:19] = np.array(interpolated)
        self.robot.update_kinematics()
        self.viz.display(self.robot.state.q)

        if p >= 1.0:
            self.get_logger().info('Reset complete — starting warmup before walk')
            self.reset_t = 0.0
            self.needs_ik_warmup = True
            self.warmup_t = 0.0
            self.mode = 'walk'

    # ─────────────────────────────────────────────
    #  Custom pose — interpolate to given angles
    # ─────────────────────────────────────────────
    def _do_custom(self):
        self.custom_t += self.dt
        p = min(self.custom_t / self.custom_duration, 1.0)
        p_smooth = 0.5 - 0.5 * np.cos(p * np.pi)

        self.custom_current_angles = [
            float(self.custom_start_angles[i] + p_smooth * (self.custom_target_angles[i] - self.custom_start_angles[i]))
            for i in range(12)
        ]
        self.latest_joint_angles = self.custom_current_angles
        self.robot.state.q[7:19] = np.array(self.custom_current_angles)
        self.robot.update_kinematics()
        self.viz.display(self.robot.state.q)

        if p >= 1.0:
            self.mode = 'custom_hold'
            self.get_logger().info('Custom pose reached — holding. Send "walk" to resume.')

    # ─────────────────────────────────────────────
    #  Main loop
    # ─────────────────────────────────────────────
    def loop(self):
        self.t += self.dt

        if self.mode == 'walk':
            if self.needs_ik_warmup:
                # Solver NOT called during warmup — direct joint write only
                self._do_warmup()
            else:
                self._do_walk()
                self.solver.solve(True)
                self.robot.update_kinematics()
                self.viz.display(self.robot.state.q)
                self.latest_joint_angles = [float(a) for a in self.robot.state.q[7:19]]

        elif self.mode == 'turn':
            self._do_turn()
            self.solver.solve(True)
            self.robot.update_kinematics()
            self.viz.display(self.robot.state.q)
            self.latest_joint_angles = [float(a) for a in self.robot.state.q[7:19]]

        elif self.mode == 'handshake':
            self._do_handshake()
            self.solver.solve(True)
            self.robot.update_kinematics()
            self.viz.display(self.robot.state.q)
            self.latest_joint_angles = [float(a) for a in self.robot.state.q[7:19]]

        elif self.mode == 'reset':
            self._do_reset()

        elif self.mode == 'custom':
            self._do_custom()

        elif self.mode == 'custom_hold':
            self.robot.state.q[7:19] = np.array(self.latest_joint_angles)
            self.robot.update_kinematics()
            self.viz.display(self.robot.state.q)

        for leg in self.leg_configs:
            robot_frame_viz(self.robot, leg)

        if self.t - self.last_target_t > 0.1:
            self.last_target_t = self.t
            pos = self.effector_tasks['LFfoot'].target_world
            self.last_targets.append(pos.tolist())
            self.last_targets = self.last_targets[-50:]
            points_viz("targets", self.last_targets, color=0xaaff00)

        self.publish_joint_angles()

        if int(self.t / self.dt) % 100 == 0:
            shifted = [round(float(a) + float(np.pi / 2), 3) for a in self.latest_joint_angles]
            self.get_logger().info(f'[{self.mode}] Joint targets (0 to pi): {shifted}')

    # ─────────────────────────────────────────────
    #  Publish
    # ─────────────────────────────────────────────
    def publish_joint_angles(self):
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [
            'LFshoulder1', 'LFshoulder', 'LFknee',
            'LRshoulder1', 'LRshoulder', 'LRknee',
            'RFshoulder1', 'RFshoulder', 'RFknee',
            'RRshoulder1', 'RRshoulder', 'RRknee'
        ]
        shifted_angles = [float(a) + float(np.pi / 2) for a in self.latest_joint_angles]
        joint_msg.position = shifted_angles
        self.pub.publish(joint_msg)

    # ─────────────────────────────────────────────
    #  Goal callback
    # ─────────────────────────────────────────────
    def goal_callback(self, msg):
        command = msg.data.strip().lower()

        if command == 'handshake':
            if self.mode in ('walk', 'turn'):
                self.handshake_t = 0.0
                self.mode = 'handshake'
                self.get_logger().info('Handshake triggered')
            else:
                self.get_logger().warn(f'Cannot handshake from mode: {self.mode}')

        elif command.startswith('turn:'):
            try:
                omega = float(command.split(':')[1])
                self.angular_velocity = omega
                if omega == 0.0:
                    self.mode = 'walk'
                    self.get_logger().info('Turn stopped — resuming walk')
                else:
                    self.turn_leg_index = 0
                    self.turn_leg_progress = 0.0
                    for leg, cfg in self.leg_configs.items():
                        self.foot_hold_pos[leg] = np.array([
                            cfg['x'],
                            cfg['y_center'],
                            cfg['z_center'] + cfg['z_offset']
                        ])
                    self.mode = 'turn'
                    direction = 'left (CCW)' if omega > 0 else 'right (CW)'
                    self.get_logger().info(f'Turning {direction} at omega={omega:.2f}')
            except (ValueError, IndexError):
                self.get_logger().warn('Invalid turn command — use "turn:0.5"')

        elif command == 'reset':
            self.reset_start_angles = [float(a) for a in self.latest_joint_angles]
            self.reset_t = 0.0
            self.mode = 'reset'
            self.get_logger().info('Resetting to neutral pose...')

        elif command == 'walk':
            # Always trigger warmup when resuming from custom/reset/hold
            self.needs_ik_warmup = True
            self.warmup_t = 0.0
            self.mode = 'walk'
            self.get_logger().info('Resuming walk with warmup...')

        elif command.startswith('custom:'):
            try:
                raw = command.split(':')[1]
                angles_deg = [float(x) for x in raw.split(',')]
                if len(angles_deg) != 12:
                    self.get_logger().warn(f'Need exactly 12 angles, got {len(angles_deg)}')
                    return
                self.custom_target_angles = [float(np.radians(a)) - float(np.pi / 2) for a in angles_deg]
                self.custom_start_angles = [float(a) for a in self.latest_joint_angles]
                self.custom_t = 0.0
                self.mode = 'custom'
                self.get_logger().info(f'Moving to custom pose: {angles_deg}')
            except (ValueError, IndexError):
                self.get_logger().warn('Invalid custom command — use "custom:90,90,90,90,90,90,90,90,90,90,90,90"')

        else:
            try:
                new_freq = float(command)
                self.gait_freq = new_freq
                self.get_logger().info(f'Gait frequency updated to: {self.gait_freq} Hz')
            except ValueError:
                self.get_logger().warn(
                    f'Unknown command: "{msg.data}"\n'
                    f'  handshake       — do handshake then resume walk\n'
                    f'  turn:0.5        — turn left\n'
                    f'  turn:-0.5       — turn right\n'
                    f'  turn:0          — stop turning\n'
                    f'  reset           — neutral pose then walk\n'
                    f'  walk            — resume walking\n'
                    f'  custom:90,...   — 12 servo angles in degrees\n'
                    f'  1.0             — set gait frequency'
                )

def main(args=None):
    rclpy.init(args=args)
    node = DogCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()