#!/usr/bin/env python3

from typing import List

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_srvs.srv import Trigger

from pb_rm_interfaces.msg import ChassisCmd, GameRobotHP, GameStatus, PostureCmd, RobotStatus


class RefereeSimNode(Node):
    def __init__(self) -> None:
        super().__init__("referee_sim")

        # Simulated referee values; they can be tuned online from rqt plugin.
        self.declare_parameter("ally_base_hp", 5000)
        self.declare_parameter("ally_outpost_hp", 1500)
        self.declare_parameter("robot_hp", 400)
        self.declare_parameter("projectile_allowance_17mm", 300)

        self._ally_base_hp = 5000
        self._ally_outpost_hp = 1500
        self._robot_hp = 400
        self._projectile_allowance_17mm = 300
        self._posture = int(PostureCmd.MOVE)
        self._chassis_reset = int(ChassisCmd.DISABLE)
        self._chassis_rotate = int(ChassisCmd.DISABLE)
        self._chassis_vy_limit = int(ChassisCmd.DISABLE)

        self._last_robot_hp = self._robot_hp

        self._phase = "countdown"  # countdown -> running -> game_over
        self._remain_time = 5
        self._game_progress = int(GameStatus.COUNT_DOWN)

        self._all_robot_hp_pub = self.create_publisher(GameRobotHP, "referee/all_robot_hp", 10)
        self._robot_status_pub = self.create_publisher(RobotStatus, "referee/robot_status", 10)
        self._game_status_pub = self.create_publisher(GameStatus, "referee/game_status", 10)
        self._posture_sub = self.create_subscription(
            PostureCmd, "cmd_posture", self._posture_cb, 10
        )
        self._chassis_cmd_sub = self.create_subscription(
            ChassisCmd, "cmd_chassis", self._chassis_cmd_cb, 10
        )

        self.create_service(Trigger, "referee_sim/start_game", self._start_game_cb)

        self.add_on_set_parameters_callback(self._on_set_parameters)
        self._read_parameters()

        self.create_timer(0.1, self._publish_timer_cb)
        self.create_timer(1.0, self._second_tick_cb)
        self.create_timer(1.0, self._log_timer_cb)

        self.get_logger().info("referee_sim node started: auto enter COUNT_DOWN(3) for 5s")

    def _read_parameters(self) -> None:
        self._ally_base_hp = self._clamp_u16(int(self.get_parameter("ally_base_hp").value))
        self._ally_outpost_hp = self._clamp_u16(int(self.get_parameter("ally_outpost_hp").value))
        self._robot_hp = self._clamp_u16(int(self.get_parameter("robot_hp").value))
        self._projectile_allowance_17mm = self._clamp_u16(
            int(self.get_parameter("projectile_allowance_17mm").value)
        )

    def _on_set_parameters(self, params: List) -> SetParametersResult:
        for param in params:
            if param.name == "ally_base_hp":
                self._ally_base_hp = self._clamp_u16(int(param.value))
            elif param.name == "ally_outpost_hp":
                self._ally_outpost_hp = self._clamp_u16(int(param.value))
            elif param.name == "robot_hp":
                new_hp = self._clamp_u16(int(param.value))
                self._last_robot_hp = self._robot_hp
                self._robot_hp = new_hp
            elif param.name == "projectile_allowance_17mm":
                self._projectile_allowance_17mm = self._clamp_u16(int(param.value))

        return SetParametersResult(successful=True)

    def _posture_cb(self, msg: PostureCmd) -> None:
        self._posture = self._normalize_posture(int(msg.posture))

    def _chassis_cmd_cb(self, msg: ChassisCmd) -> None:
        self._chassis_reset = self._normalize_chassis_flag(int(msg.reset))
        self._chassis_rotate = self._normalize_chassis_flag(int(msg.rotate))
        self._chassis_vy_limit = self._normalize_chassis_flag(int(msg.vy_limit))

    def _start_game_cb(self, _request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        self._phase = "countdown"
        self._remain_time = 5
        self._game_progress = int(GameStatus.COUNT_DOWN)
        response.success = True
        response.message = "Game started: 5s countdown then 420s running"
        self.get_logger().info(response.message)
        return response

    def _second_tick_cb(self) -> None:
        if self._phase == "countdown":
            self._remain_time -= 1
            if self._remain_time <= 0:
                self._phase = "running"
                self._game_progress = int(GameStatus.RUNNING)
                self._remain_time = 420
        elif self._phase == "running":
            self._remain_time -= 1
            if self._remain_time <= 0:
                self._phase = "game_over"
                self._game_progress = int(GameStatus.GAME_OVER)
                self._remain_time = 0
        elif self._phase == "idle":
            self._game_progress = int(GameStatus.NOT_START)
            self._remain_time = 0
        elif self._phase == "game_over":
            self._game_progress = int(GameStatus.GAME_OVER)
            self._remain_time = 0

    def _publish_timer_cb(self) -> None:
        self._publish_all_robot_hp()
        self._publish_robot_status()
        self._publish_game_status()

    def _publish_all_robot_hp(self) -> None:
        msg = GameRobotHP()
        msg.ally_base_hp = self._ally_base_hp
        msg.ally_outpost_hp = self._ally_outpost_hp
        self._all_robot_hp_pub.publish(msg)

    def _publish_robot_status(self) -> None:
        msg = RobotStatus()
        msg.robot_id = 7
        msg.robot_level = 1
        msg.current_hp = self._robot_hp
        msg.maximum_hp = 400
        msg.shooter_barrel_cooling_value = 0
        msg.shooter_barrel_heat_limit = 0
        msg.shooter_17mm_1_barrel_heat = 0

        msg.robot_pos.position.x = 0.0
        msg.robot_pos.position.y = 0.0
        msg.robot_pos.position.z = 0.0
        msg.robot_pos.orientation.w = 1.0

        msg.armor_id = 0
        msg.hp_deduction_reason = int(RobotStatus.ARMOR_HIT)
        msg.projectile_allowance_17mm = self._projectile_allowance_17mm
        msg.remaining_gold_coin = 0
        msg.out_of_combat_status = 0
        msg.fire_rem_17mm = self._projectile_allowance_17mm
        msg.current_posture = self._posture
        msg.energy_mechanism_activable = 0
        msg.shoot_state = 0
        msg.is_hp_deduced = self._robot_hp < self._last_robot_hp

        self._last_robot_hp = self._robot_hp
        self._robot_status_pub.publish(msg)

    def _publish_game_status(self) -> None:
        msg = GameStatus()
        msg.game_progress = self._game_progress
        msg.stage_remain_time = int(self._remain_time)
        self._game_status_pub.publish(msg)

    def _log_timer_cb(self) -> None:
        status_text = self._progress_to_text(self._game_progress)
        self.get_logger().info(
            (
                "base_hp=%d outpost_hp=%d robot_hp=%d ammo=%d posture=%d "
                "reset=%s rotate=%s vy_limit=%s "
                "game_status=%s remain_time=%ds"
            )
            % (
                self._ally_base_hp,
                self._ally_outpost_hp,
                self._robot_hp,
                self._projectile_allowance_17mm,
                self._posture,
                self._chassis_flag_to_text(self._chassis_reset),
                self._chassis_flag_to_text(self._chassis_rotate),
                self._chassis_flag_to_text(self._chassis_vy_limit),
                status_text,
                self._remain_time,
            )
        )

    @staticmethod
    def _clamp_u16(value: int) -> int:
        if value < 0:
            return 0
        if value > 65535:
            return 65535
        return value

    @staticmethod
    def _normalize_posture(value: int) -> int:
        if value in (
            int(PostureCmd.OFF),
            int(PostureCmd.ATTACK),
            int(PostureCmd.DEFENSE),
            int(PostureCmd.MOVE),
            int(PostureCmd.SPIN),
        ):
            return value
        return int(PostureCmd.OFF)

    @staticmethod
    def _normalize_chassis_flag(value: int) -> int:
        if value == int(ChassisCmd.ENABLE):
            return int(ChassisCmd.ENABLE)
        return int(ChassisCmd.DISABLE)

    @staticmethod
    def _chassis_flag_to_text(value: int) -> str:
        if value == int(ChassisCmd.ENABLE):
            return "ENABLE(1)"
        return "DISABLE(0)"

    @staticmethod
    def _progress_to_text(progress: int) -> str:
        mapping = {
            int(GameStatus.NOT_START): "NOT_START(0)",
            int(GameStatus.PREPARATION): "PREPARATION(1)",
            int(GameStatus.SELF_CHECKING): "SELF_CHECKING(2)",
            int(GameStatus.COUNT_DOWN): "COUNT_DOWN(3)",
            int(GameStatus.RUNNING): "RUNNING(4)",
            int(GameStatus.GAME_OVER): "GAME_OVER(5)",
        }
        return mapping.get(progress, f"UNKNOWN({progress})")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RefereeSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
