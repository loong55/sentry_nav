#!/usr/bin/env python3

import importlib

try:
    qt_core = importlib.import_module("python_qt_binding.QtCore")
    qt_widgets = importlib.import_module("python_qt_binding.QtWidgets")
except ModuleNotFoundError:
    # Fallback for IDE/runtime environments without python_qt_binding on PYTHONPATH.
    qt_core = importlib.import_module("PyQt5.QtCore")
    qt_widgets = importlib.import_module("PyQt5.QtWidgets")

Qt = qt_core.Qt
QTimer = qt_core.QTimer

QFormLayout = qt_widgets.QFormLayout
QHBoxLayout = qt_widgets.QHBoxLayout
QLabel = qt_widgets.QLabel
QPushButton = qt_widgets.QPushButton
QSlider = qt_widgets.QSlider
QSpinBox = qt_widgets.QSpinBox
QVBoxLayout = qt_widgets.QVBoxLayout
QWidget = qt_widgets.QWidget
from rclpy.parameter import Parameter
import rclpy
from rclpy.node import Node
from rqt_gui_py.plugin import Plugin
from std_srvs.srv import Trigger

from pb_rm_interfaces.msg import GameStatus
from pb_rm_interfaces.msg import PostureCmd

AsyncParametersClient = importlib.import_module("rclpy.parameter_client").AsyncParametersClient


class RefereeSimPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("RefereeSimPlugin")

        if not rclpy.ok():
            rclpy.init(args=None)

        self._node = Node("referee_sim_rqt_plugin")
        self._param_client = AsyncParametersClient(self._node, "/referee_sim")
        self._start_client = self._node.create_client(Trigger, "/referee_sim/start_game")
        self._game_status_sub = self._node.create_subscription(
            GameStatus,
            "/referee/game_status",
            self._game_status_cb,
            10,
        )
        self._posture_sub = self._node.create_subscription(
            PostureCmd,
            "/cmd_posture",
            self._posture_cb,
            10,
        )

        self._widget = QWidget()
        self._widget.setWindowTitle("Referee Simulator")
        self._widget.setMinimumSize(640, 360)

        main_layout = QVBoxLayout()
        form = QFormLayout()

        self._base_hp_slider, self._base_hp_spin = self._make_slider_spin(0, 10000, 5000)
        self._outpost_hp_slider, self._outpost_hp_spin = self._make_slider_spin(0, 5000, 1500)
        self._robot_hp_slider, self._robot_hp_spin = self._make_slider_spin(0, 1000, 400)
        self._ammo_slider, self._ammo_spin = self._make_slider_spin(0, 3000, 300)

        self._posture_label = QLabel("MOVE(3)")

        self._game_progress_label = QLabel("NOT_START(0)")
        self._remain_time_label = QLabel("0")
        self._start_btn = QPushButton("比赛开始")
        self._start_btn.setMinimumHeight(36)

        form.addRow("基地血量", self._row_widget(self._base_hp_slider, self._base_hp_spin))
        form.addRow("前哨站血量", self._row_widget(self._outpost_hp_slider, self._outpost_hp_spin))
        form.addRow("机器人血量", self._row_widget(self._robot_hp_slider, self._robot_hp_spin))
        form.addRow("允许发弹量", self._row_widget(self._ammo_slider, self._ammo_spin))
        form.addRow("姿态(监听)", self._posture_label)
        form.addRow("比赛控制", self._start_btn)
        form.addRow("比赛状态", self._game_progress_label)
        form.addRow("剩余时间(s)", self._remain_time_label)

        main_layout.addLayout(form)
        self._widget.setLayout(main_layout)

        context.add_widget(self._widget)

        self._connect_signals()

        self._spin_timer = QTimer(self._widget)
        self._spin_timer.timeout.connect(self._spin_once)
        self._spin_timer.start(50)

        self._set_param("ally_base_hp", 5000)
        self._set_param("ally_outpost_hp", 1500)
        self._set_param("robot_hp", 400)
        self._set_param("projectile_allowance_17mm", 300)

    def shutdown_plugin(self):
        self._spin_timer.stop()
        self._node.destroy_node()

    def _spin_once(self):
        if rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.0)

    def _connect_signals(self):
        self._base_hp_spin.valueChanged.connect(
            lambda v: self._set_param("ally_base_hp", int(v))
        )
        self._outpost_hp_spin.valueChanged.connect(
            lambda v: self._set_param("ally_outpost_hp", int(v))
        )
        self._robot_hp_spin.valueChanged.connect(lambda v: self._set_param("robot_hp", int(v)))
        self._ammo_spin.valueChanged.connect(
            lambda v: self._set_param("projectile_allowance_17mm", int(v))
        )
        self._start_btn.clicked.connect(self._on_start_clicked)

    def _on_start_clicked(self):
        if not self._start_client.wait_for_service(timeout_sec=0.2):
            self._node.get_logger().warn("/referee_sim/start_game service not ready")
            return
        req = Trigger.Request()
        future = self._start_client.call_async(req)

        def _done_cb(fut):
            try:
                resp = fut.result()
                self._node.get_logger().info(f"start_game: {resp.message}")
            except Exception as exc:
                self._node.get_logger().error(f"start_game failed: {str(exc)}")

        future.add_done_callback(_done_cb)

    def _set_param(self, name: str, value: int):
        if not self._param_client.service_is_ready():
            self._param_client.wait_for_service(timeout_sec=0.2)
        param = Parameter(name=name, value=value)
        self._param_client.set_parameters([param])

    def _game_status_cb(self, msg: GameStatus):
        self._game_progress_label.setText(self._progress_to_text(int(msg.game_progress)))
        self._remain_time_label.setText(str(int(msg.stage_remain_time)))

    def _posture_cb(self, msg: PostureCmd):
        self._posture_label.setText(self._posture_to_text(int(msg.posture)))

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

    @staticmethod
    def _posture_to_text(posture: int) -> str:
        mapping = {
            int(PostureCmd.ATTACK): "ATTACK(1)",
            int(PostureCmd.DEFENSE): "DEFENSE(2)",
            int(PostureCmd.MOVE): "MOVE(3)",
            int(PostureCmd.SPIN): "SPIN(4)",
        }
        return mapping.get(posture, f"UNKNOWN({posture})")

    @staticmethod
    def _make_slider_spin(min_value: int, max_value: int, default_value: int):
        slider = QSlider()
        slider.setOrientation(Qt.Horizontal)
        slider.setMinimum(min_value)
        slider.setMaximum(max_value)
        slider.setValue(default_value)

        spin = QSpinBox()
        spin.setMinimum(min_value)
        spin.setMaximum(max_value)
        spin.setValue(default_value)

        slider.valueChanged.connect(spin.setValue)
        spin.valueChanged.connect(slider.setValue)

        return slider, spin

    @staticmethod
    def _row_widget(slider: QSlider, spin: QSpinBox):
        row = QWidget()
        layout = QHBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(slider)
        layout.addWidget(spin)
        row.setLayout(layout)
        return row
