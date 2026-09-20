from __future__ import annotations

import copy
import json
import os
from pathlib import Path
from typing import Any, Callable

from PyQt6.QtCore import QSettings, Qt
from PyQt6.QtGui import QAction
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMenu,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QSplitter,
    QToolBar,
    QVBoxLayout,
    QWidget,
)

from .backend import BackendClient
from .ingame_hud import InGameARHUD
from .widgets import AttitudeHUD, LocalTrajectoryMap, TrajectoryProfile


def _get(mapping: dict[str, Any], path: str, default: Any = 0) -> Any:
    value: Any = mapping
    for part in path.split("."):
        if not isinstance(value, dict):
            return default
        value = value.get(part, default)
    return value


def _set(mapping: dict[str, Any], path: str, value: Any) -> None:
    parts = path.split(".")
    cursor = mapping
    for part in parts[:-1]:
        cursor = cursor.setdefault(part, {})
    cursor[parts[-1]] = value


def _fmt(value: Any, suffix: str = "", digits: int = 1) -> str:
    try:
        return f"{float(value):.{digits}f}{suffix}"
    except (TypeError, ValueError):
        return "—"


def _signed(value: Any, suffix: str = "", digits: int = 1) -> str:
    try:
        return f"{float(value):+.{digits}f}{suffix}"
    except (TypeError, ValueError):
        return "—"


def _distance(value: Any) -> str:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return "—"
    return f"{number / 1000:.1f} km" if abs(number) >= 1000 else f"{number:.0f} m"


def _number(value: Any, default: float = 0.0) -> float:
    try:
        return default if value is None else float(value)
    except (TypeError, ValueError):
        return default


def _first_number(value: Any) -> Any:
    if isinstance(value, (list, tuple)) and value:
        return value[0]
    return None


class MainWindow(QMainWindow):
    def __init__(self, backend_path: Path) -> None:
        super().__init__()
        self.setWindowTitle("KSP Shuttle Lander")
        self.resize(1500, 920)
        project_root = Path(os.environ.get("KSP_LANDER_ROOT", Path(__file__).resolve().parents[1])).expanduser()
        runtime_dir = project_root / "Runtime" / "PyQtUI"
        runtime_dir.mkdir(parents=True, exist_ok=True)
        self.settings = QSettings(str(runtime_dir / "settings.ini"), QSettings.Format.IniFormat)
        self._restoring_layout = False
        self.config_path = runtime_dir / "config.json"
        self.config: dict[str, Any] = {}
        self.snapshot: dict[str, Any] = {}
        self.fields: dict[str, QWidget] = {}
        self.metrics: dict[str, QLabel] = {}
        self._ready = False
        self.ingame_hud = InGameARHUD()

        self.backend = BackendClient(backend_path, self)
        self.backend.ready.connect(self._on_backend_ready)
        self.backend.snapshot.connect(self._on_snapshot)
        self.backend.response.connect(self._on_response)
        self.backend.backend_error.connect(self._on_backend_error)
        self.backend.stopped.connect(self._on_backend_stopped)

        self._build_ui()
        self._restore_layout()
        self.backend.start()

    def _build_ui(self) -> None:
        self._build_toolbar()

        root = QSplitter(Qt.Orientation.Horizontal)
        root.setChildrenCollapsible(False)
        self.setCentralWidget(root)

        self.controls_scroll = QScrollArea()
        self.controls_scroll.setWidgetResizable(True)
        self.controls_scroll.setMinimumWidth(280)
        self.controls_scroll.setWidget(self._build_controls())
        root.addWidget(self.controls_scroll)

        center = QWidget()
        center_layout = QVBoxLayout(center)
        center_layout.setContentsMargins(10, 10, 10, 10)
        self.header_vessel = QLabel("—")
        self.header_vessel.setStyleSheet("font-weight: 600; font-size: 15px;")
        self.header_phase = QLabel("Idle")
        self.header_status = QLabel("Starting C landing backend…")
        self.header_status.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
        self.header_status.setWordWrap(True)
        header = QHBoxLayout()
        left_header = QVBoxLayout()
        left_header.addWidget(self.header_vessel)
        left_header.addWidget(self.header_phase)
        header.addLayout(left_header)
        header.addStretch(1)
        header.addWidget(self.header_status, 2)
        center_layout.addLayout(header)

        self.center_splitter = QSplitter(Qt.Orientation.Vertical)
        self.center_splitter.setChildrenCollapsible(False)
        self.hud = AttitudeHUD()
        self.hud_scale.valueChanged.connect(self.hud.set_instrument_scale)
        self.center_splitter.addWidget(self.hud)
        self.lower_splitter = QSplitter(Qt.Orientation.Horizontal)
        self.lower_splitter.setChildrenCollapsible(False)
        self.trajectory_map = LocalTrajectoryMap()
        self.trajectory_profile = TrajectoryProfile()
        self.lower_splitter.addWidget(self.trajectory_map)
        self.lower_splitter.addWidget(self.trajectory_profile)
        self.center_splitter.addWidget(self.lower_splitter)
        center_layout.addWidget(self.center_splitter, 1)
        root.addWidget(center)

        self.telemetry_scroll = QScrollArea()
        self.telemetry_scroll.setWidgetResizable(True)
        self.telemetry_scroll.setMinimumWidth(280)
        self.telemetry_scroll.setWidget(self._build_telemetry())
        root.addWidget(self.telemetry_scroll)
        self.root_splitter = root

    def _build_toolbar(self) -> None:
        toolbar = QToolBar("Flight")
        toolbar.setMovable(False)
        self.addToolBar(toolbar)
        self.connection_badge = QLabel("● Disconnected")
        toolbar.addWidget(self.connection_badge)
        toolbar.addSeparator()

        self.plan_action = QAction("Plan", self)
        self.plan_action.triggered.connect(self.create_plan)
        toolbar.addAction(self.plan_action)
        self.engage_action = QAction("Engage", self)
        self.engage_action.triggered.connect(self.engage)
        toolbar.addAction(self.engage_action)
        self.pause_action = QAction("Pause", self)
        self.pause_action.triggered.connect(self.toggle_pause)
        toolbar.addAction(self.pause_action)
        self.abort_action = QAction("Abort", self)
        self.abort_action.triggered.connect(self.abort)
        toolbar.addAction(self.abort_action)
        toolbar.addSeparator()

        layout_button = QPushButton("Layout")
        menu = QMenu(layout_button)
        self.telemetry_action = menu.addAction("Telemetry panel")
        self.telemetry_action.setCheckable(True)
        self.telemetry_action.setChecked(True)
        self.telemetry_action.triggered.connect(self._apply_visibility)
        self.map_action = menu.addAction("Trajectory map")
        self.map_action.setCheckable(True)
        self.map_action.setChecked(True)
        self.map_action.triggered.connect(self._apply_visibility)
        self.profile_action = menu.addAction("Range / altitude profile")
        self.profile_action.setCheckable(True)
        self.profile_action.setChecked(True)
        self.profile_action.triggered.connect(self._apply_visibility)
        menu.addSeparator()
        menu.addAction("Reset Layout", self.restore_layout_defaults)
        layout_button.setMenu(menu)
        toolbar.addWidget(layout_button)

    def _build_controls(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(12)

        connection = self._group("Connection")
        form = QFormLayout(connection)
        self._line(form, "Serial endpoint", "connection.serialPort")
        self._integer(form, "Baud rate", "connection.baudRate", 9600, 4000000)
        self._integer(form, "I/O timeout", "connection.timeoutMs", 50, 60000)
        buttons = QWidget()
        row = QHBoxLayout(buttons)
        row.setContentsMargins(0, 0, 0, 0)
        self.connect_button = QPushButton("Connect")
        self.disconnect_button = QPushButton("Disconnect")
        self.connect_button.clicked.connect(self.connect_backend)
        self.disconnect_button.clicked.connect(lambda: self.backend.send("disconnect"))
        row.addWidget(self.connect_button)
        row.addWidget(self.disconnect_button)
        form.addRow(buttons)
        layout.addWidget(connection)

        site = self._group("Landing Site")
        form = QFormLayout(site)
        self._line(form, "Name", "site.name")
        self._number(form, "Latitude", "site.latitude", -90, 90, 5, " °")
        self._number(form, "Longitude", "site.longitude", -180, 180, 5, " °")
        self._number(form, "Elevation", "site.altitude", -1000, 100000, 0, " m")
        self._number(form, "Runway heading", "site.runwayHeading", 0, 360, 2, " °")
        self._number(form, "Runway length", "site.runwayLength", 100, 50000, 0, " m")
        self._number(form, "Runway width", "site.runwayWidth", 5, 2000, 0, " m")
        layout.addWidget(site)

        vehicle = self._group("Vehicle")
        form = QFormLayout(vehicle)
        automatic_physics = QLabel(
            "Aerodynamics, mass properties, center of mass and control authority are identified automatically from live kRPC data."
        )
        automatic_physics.setWordWrap(True)
        form.addRow(automatic_physics)
        self._line(form, "Model ID", "vehicle.modelId")
        self._number(form, "Entry AoA", "vehicle.entryAngleOfAttack", 0, 60, 1, " °")
        self._number(form, "Max AoA", "vehicle.maximumAngleOfAttack", 1, 60, 1, " °")
        self._number(form, "Max bank", "vehicle.maximumBankAngle", 5, 85, 1, " °")
        self._number(form, "Approach speed", "vehicle.finalApproachSpeed", 10, 1000, 1, " m/s")
        self._number(form, "Touchdown speed", "vehicle.touchdownSpeed", 5, 1000, 1, " m/s")
        self._check(form, "Powered approach", "vehicle.allowPoweredApproach")
        self._number(form, "Max approach throttle", "vehicle.maximumApproachThrottle", 0, 1, 2)
        self._integer(form, "Airbrake action group", "vehicle.airbrakeActionGroup", 1, 10)
        layout.addWidget(vehicle)

        calibration = self._group("In-Flight Calibration")
        cal_layout = QVBoxLayout(calibration)
        self.calibration_status = QLabel("Passive calibration is ready.")
        self.calibration_status.setWordWrap(True)
        cal_layout.addWidget(self.calibration_status)
        self.calibration_progress = QProgressBar()
        self.calibration_progress.setRange(0, 1000)
        cal_layout.addWidget(self.calibration_progress)
        self.calibration_counts = QLabel("0 accepted / 0 rejected")
        self.calibration_counts.setAlignment(Qt.AlignmentFlag.AlignRight)
        cal_layout.addWidget(self.calibration_counts)
        cal_buttons = QHBoxLayout()
        self.start_cal_button = QPushButton("Start Simple Glide")
        self.stop_apply_button = QPushButton("Stop & Apply")
        self.stop_cal_button = QPushButton("Stop")
        self.start_cal_button.clicked.connect(self.start_calibration)
        self.stop_apply_button.clicked.connect(lambda: self.stop_calibration(True))
        self.stop_cal_button.clicked.connect(lambda: self.stop_calibration(False))
        cal_buttons.addWidget(self.start_cal_button)
        cal_buttons.addWidget(self.stop_apply_button)
        cal_buttons.addWidget(self.stop_cal_button)
        cal_layout.addLayout(cal_buttons)
        cal_form = QFormLayout()
        self._check(cal_form, "Passive learning", "calibration.enablePassiveCalibration")
        self._check(cal_form, "Trajectory calibration", "calibration.enableTrajectoryCalibration")
        self._check(cal_form, "Apply learned safety speeds in flight", "calibration.autoApplyInFlight")
        self._number(cal_form, "AoA sweep minimum", "calibration.minimumAngleOfAttack", 0, 25, 1, " °")
        self._number(cal_form, "AoA sweep maximum", "calibration.maximumAngleOfAttack", 1, 45, 1, " °")
        self._number(cal_form, "AoA step", "calibration.angleOfAttackStep", 0.5, 10, 1, " °")
        self._number(cal_form, "Altitude floor", "calibration.minimumCalibrationRadarAltitude", 250, 50000, 0, " m")
        self._number(cal_form, "Max dynamic pressure", "calibration.maximumCalibrationDynamicPressure", 100, 1_000_000, 0, " Pa")
        self._number(cal_form, "Max G-load", "calibration.maximumCalibrationGLoad", 1.1, 10, 2, " g")
        self._number(cal_form, "Max sink rate", "calibration.maximumCalibrationSinkRate", 5, 250, 0, " m/s")
        self._number(cal_form, "Abort stall fraction", "calibration.abortStallFraction", 0.05, 0.9, 2)
        self._number(cal_form, "Settling time", "calibration.settlingDuration", 1, 30, 1, " s")
        self._number(cal_form, "Sampling time", "calibration.samplingDuration", 2, 60, 1, " s")
        cal_layout.addLayout(cal_form)
        reset_cal = QPushButton("Reset Learned Data")
        reset_cal.clicked.connect(lambda: self.backend.send("resetCalibration"))
        cal_layout.addWidget(reset_cal)
        layout.addWidget(calibration)

        guidance = self._group("Guidance Geometry")
        form = QFormLayout(guidance)
        self._number(form, "Capture radius", "guidance.targetDeorbitCaptureRadius", 1000, 1_000_000, 0, " m")
        self._number(form, "Entry range target", "guidance.targetEntryRange", 80000, 1_500_000, 0, " m")
        self._number(form, "Entry path angle", "guidance.targetEntryFlightPathAngle", -8, -0.2, 2, " °")
        self._number(form, "Maximum entry angle", "guidance.maximumEntryFlightPathAngle", -15, -1, 2, " °")
        self._number(form, "Post-burn periapsis", "guidance.targetPostBurnPeriapsisAltitude", -5000, 65000, 0, " m")
        self._number(form, "Deorbit max throttle", "guidance.deorbitMaximumThrottle", 0.05, 1, 2)
        self._number(form, "Burn ramp duration", "guidance.deorbitThrottleRampDuration", 1, 20, 1, " s")
        self._number(form, "Burn timing uncertainty", "guidance.deorbitTimingUncertainty", 0, 20, 1, " s")
        self._number(form, "Thrust uncertainty", "guidance.deorbitThrustUncertaintyFraction", 0, 0.5, 2)
        self._number(form, "Cutoff Δv uncertainty", "guidance.deorbitDeltaVUncertainty", 0, 15, 1, " m/s")
        self._number(form, "Mass uncertainty", "guidance.deorbitMassUncertaintyFraction", 0, 0.3, 2)
        self._number(form, "Position uncertainty", "guidance.deorbitPositionUncertainty", 0, 5000, 0, " m")
        self._number(form, "Velocity uncertainty", "guidance.deorbitVelocityUncertainty", 0, 25, 1, " m/s")
        self._number(form, "Pointing uncertainty", "guidance.deorbitPointingUncertainty", 0, 10, 1, " °")
        self._number(form, "Atmosphere uncertainty", "guidance.deorbitAtmosphereUncertaintyFraction", 0, 0.5, 2)
        self._number(form, "Minimum robust pass", "guidance.deorbitRobustMinimumPassFraction", 0.5, 1, 2)
        self._number(form, "TAEM altitude", "guidance.taemInterfaceAltitude", 2000, 60000, 0, " m")
        self._number(form, "TAEM range", "guidance.taemInterfaceRange", 5000, 500000, 0, " m")
        self._number(form, "HAC radius", "guidance.hacRadius", 2000, 100000, 0, " m")
        self._number(form, "Final distance", "guidance.finalApproachDistance", 1000, 50000, 0, " m")
        self._number(form, "TAEM slope", "guidance.taemGlideSlope", 3, 30, 1, " °")
        self._number(form, "Final slope", "guidance.finalGlideSlope", 1, 12, 1, " °")
        self._number(form, "Flare altitude", "guidance.flareAltitude", 3, 500, 0, " m")
        self._number(form, "S-turn minimum leg", "guidance.sTurnMinimumLegDuration", 8, 120, 1, " s")
        self._number(form, "S-turn maximum leg", "guidance.sTurnMaximumLegDuration", 12, 240, 1, " s")
        self._number(form, "Entry roll rate", "guidance.entryRollRate", 1, 25, 1, " °/s")
        self._number(form, "Entry roll acceleration", "guidance.entryRollAcceleration", 0.5, 20, 1, " °/s²")
        self._check(form, "Automatic time warp", "guidance.useTimeWarp")
        layout.addWidget(guidance)

        ui = self._group("UI Layout")
        form = QFormLayout(ui)
        self.control_width = QSpinBox()
        self.control_width.setRange(260, 520)
        self.control_width.valueChanged.connect(self._resize_sidebars)
        form.addRow("Control width", self.control_width)
        self.telemetry_width = QSpinBox()
        self.telemetry_width.setRange(260, 560)
        self.telemetry_width.valueChanged.connect(self._resize_sidebars)
        form.addRow("Telemetry width", self.telemetry_width)
        self.hud_scale = QDoubleSpinBox()
        self.hud_scale.setRange(0.7, 1.6)
        self.hud_scale.setSingleStep(0.05)
        form.addRow("HUD scale", self.hud_scale)
        self.hud_height = QSpinBox()
        self.hud_height.setRange(220, 760)
        self.hud_height.valueChanged.connect(self._resize_center_panels)
        form.addRow("HUD height", self.hud_height)
        self.lower_height = QSpinBox()
        self.lower_height.setRange(180, 680)
        self.lower_height.valueChanged.connect(self._resize_center_panels)
        form.addRow("Lower panel height", self.lower_height)
        self.map_range = QComboBox()
        for label, value in (("Auto", 0.0), ("25 km", 25_000.0), ("50 km", 50_000.0), ("100 km", 100_000.0), ("250 km", 250_000.0), ("500 km", 500_000.0)):
            self.map_range.addItem(label, value)
        self.map_range.currentIndexChanged.connect(self._refresh_visuals)
        form.addRow("Map range", self.map_range)
        reset_layout = QPushButton("Reset Layout")
        reset_layout.clicked.connect(self.restore_layout_defaults)
        form.addRow(reset_layout)
        layout.addWidget(ui)

        actions = QHBoxLayout()
        apply_button = QPushButton("Apply Configuration")
        apply_button.clicked.connect(self.apply_configuration)
        restore_button = QPushButton("Restore Defaults")
        restore_button.clicked.connect(self.restore_defaults)
        actions.addWidget(apply_button)
        actions.addWidget(restore_button)
        layout.addLayout(actions)

        direct = self._group("Direct Controls")
        row = QHBoxLayout(direct)
        self.gear_button = QPushButton("Deploy Gear")
        self.brakes_button = QPushButton("Set Brakes")
        self.gear_button.clicked.connect(self.toggle_gear)
        self.brakes_button.clicked.connect(self.toggle_brakes)
        row.addWidget(self.gear_button)
        row.addWidget(self.brakes_button)
        layout.addWidget(direct)
        layout.addStretch(1)
        return page

    def _build_telemetry(self) -> QWidget:
        page = QWidget()
        layout = QVBoxLayout(page)
        layout.setContentsMargins(10, 10, 10, 10)
        groups: list[tuple[str, list[tuple[str, str]]]] = [
            ("Guidance", [("phase", "Phase"), ("status", "Status"), ("targetPitch", "Target pitch"), ("targetHeading", "Target heading"), ("targetRoll", "Target bank"), ("targetThrottle", "Throttle")]),
            ("Navigation", [("rangeToSite", "Range"), ("bearingToSite", "Bearing"), ("headingError", "Nose heading error"), ("groundTrackHeading", "Ground track"), ("courseToSiteError", "Course error"), ("runwayAlongTrack", "Runway along"), ("runwayCrossTrack", "Runway cross"), ("predictedMissDistance", "Predicted miss"), ("predictedTAEMDistance", "Predicted TAEM"), ("predictedTAEMRangeError", "TAEM range error"), ("energyExcessRange", "Energy margin")]),
            ("Flight", [("meanAltitude", "Altitude"), ("radarAltitude", "Radar altitude"), ("trueAirSpeed", "Airspeed"), ("verticalSpeed", "Vertical speed"), ("mach", "Mach"), ("angleOfAttack", "AoA"), ("dynamicPressure", "Dynamic pressure"), ("gForce", "G-load"), ("stallFraction", "Stall"), ("navballSpeedMode", "Navball mode"), ("pitchRate", "Pitch rate"), ("rollRate", "Roll rate"), ("yawRate", "Yaw rate"), ("autopilotError", "Autopilot error")]),
            ("Low-Level Control", [("autopilotPitchError", "AP pitch error"), ("autopilotHeadingError", "AP heading error"), ("autopilotRollError", "AP roll error"), ("controlPitch", "Pitch actuator"), ("controlRoll", "Roll actuator"), ("controlYaw", "Yaw actuator"), ("availablePitchTorque", "Pitch torque"), ("availableRollTorque", "Roll torque"), ("availableYawTorque", "Yaw torque"), ("pitchMomentOfInertia", "Pitch inertia"), ("rollMomentOfInertia", "Roll inertia"), ("yawMomentOfInertia", "Yaw inertia"), ("autopilotPitchKp", "Pitch PID Kp"), ("autopilotRollKp", "Roll PID Kp"), ("autopilotYawKp", "Yaw PID Kp"), ("telemetryLatencyMs", "Telemetry RPC"), ("guidanceComputeMs", "Guidance compute"), ("applyLatencyMs", "Apply RPC"), ("controlLoopMs", "Loop time"), ("wallTickIntervalMs", "Wall tick")]),
            ("Adaptive Model", [("estimatedLiftToDrag", "Lift / drag"), ("estimatedBallisticCoefficient", "Ballistic coeff."), ("aerodynamicConfidence", "Aero confidence"), ("trajectoryCalibrationConfidence", "Trajectory confidence"), ("trajectoryDensityScale", "Density scale"), ("trajectoryDragScale", "Drag scale"), ("trajectoryLiftScale", "Lift scale"), ("bankEffectiveness", "Bank effectiveness"), ("trajectoryAltitudeResidual", "Altitude residual"), ("trajectorySpeedResidual", "Speed residual"), ("trajectoryRangeResidual", "Range residual")]),
            ("Recommended Vehicle", [("recommendedLD", "Entry L/D"), ("recommendedBeta", "Entry β"), ("recommendedSafe", "Safe speed"), ("recommendedApproach", "Approach speed"), ("recommendedTouchdown", "Touchdown speed"), ("mass", "Mass"), ("availableThrust", "Available thrust")]),
            ("Deorbit Plan", [("planBurnUT", "Burn UT"), ("planDeltaV", "Delta-v"), ("planBurnDuration", "Burn duration"), ("planTAEM", "TAEM range"), ("planClosest", "Closest pass"), ("planEntryRange", "Entry range"), ("planEntryAngle", "Entry angle"), ("planPeriapsis", "Post-burn periapsis"), ("planRobust", "Robust cases"), ("planWorstMiss", "Worst miss"), ("planWorstPeriapsis", "Worst periapsis"), ("planAchieved", "Achieved state"), ("planConfidence", "Confidence"), ("planCapture", "Capture")]),
        ]
        for title, rows in groups:
            box = self._group(title)
            form = QFormLayout(box)
            for key, label in rows:
                value = QLabel("—")
                value.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
                value.setWordWrap(key in ("status",))
                self.metrics[key] = value
                form.addRow(label, value)
            layout.addWidget(box)
        self.warning_label = QLabel("")
        self.warning_label.setWordWrap(True)
        self.warning_label.setStyleSheet("color: #d98b2b;")
        self.error_label = QLabel("")
        self.error_label.setWordWrap(True)
        self.error_label.setStyleSheet("color: #d64c4c;")
        layout.addWidget(self.warning_label)
        layout.addWidget(self.error_label)
        layout.addStretch(1)
        return page

    def _group(self, title: str) -> QGroupBox:
        box = QGroupBox(title)
        box.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
        return box

    def _line(self, form: QFormLayout, label: str, path: str) -> None:
        widget = QLineEdit()
        self.fields[path] = widget
        form.addRow(label, widget)

    def _integer(self, form: QFormLayout, label: str, path: str, minimum: int, maximum: int) -> None:
        widget = QSpinBox()
        widget.setRange(minimum, maximum)
        self.fields[path] = widget
        form.addRow(label, widget)

    def _number(self, form: QFormLayout, label: str, path: str, minimum: float, maximum: float, decimals: int, suffix: str = "") -> None:
        widget = QDoubleSpinBox()
        widget.setRange(minimum, maximum)
        widget.setDecimals(decimals)
        widget.setSingleStep(10 ** (-decimals) if decimals else max(1, (maximum - minimum) / 200))
        widget.setSuffix(suffix)
        self.fields[path] = widget
        form.addRow(label, widget)

    def _check(self, form: QFormLayout, label: str, path: str) -> None:
        widget = QCheckBox()
        self.fields[path] = widget
        form.addRow(label, widget)

    def _on_backend_ready(self, message: dict[str, Any]) -> None:
        default_config = message.get("configuration") or {}
        saved = self._load_saved_configuration()
        self.config = saved if saved else copy.deepcopy(default_config)
        self.default_config = copy.deepcopy(default_config)
        self._set_fields_from_config(self.config)
        self.snapshot = message.get("snapshot") or {}
        self._ready = True
        self.backend.send("updateConfiguration", configuration=self._configuration_from_fields())
        self._update_snapshot_ui()

    def _on_snapshot(self, snapshot: dict[str, Any]) -> None:
        self.snapshot = snapshot
        self._update_snapshot_ui()

    def _on_response(self, message: dict[str, Any]) -> None:
        result = message.get("result") or {}
        normalized = result.get("configuration") if isinstance(result, dict) else None
        if isinstance(normalized, dict):
            self.config = normalized
            self._set_fields_from_config(normalized)
            self._save_configuration(normalized)

    def _on_backend_error(self, text: str) -> None:
        self.error_label.setText(text if hasattr(self, "error_label") else "")
        self.statusBar().showMessage(text, 8000)

    def _on_backend_stopped(self) -> None:
        self.ingame_hud.suspend()
        if self.isVisible():
            self.connection_badge.setText("● Backend stopped")

    def _configuration_from_fields(self) -> dict[str, Any]:
        configuration = copy.deepcopy(self.config or getattr(self, "default_config", {}))
        for path, widget in self.fields.items():
            if isinstance(widget, QLineEdit):
                value: Any = widget.text()
            elif isinstance(widget, QCheckBox):
                value = widget.isChecked()
            elif isinstance(widget, QSpinBox):
                value = widget.value()
            elif isinstance(widget, QDoubleSpinBox):
                value = widget.value()
            else:
                continue
            _set(configuration, path, value)
        return configuration

    def _set_fields_from_config(self, configuration: dict[str, Any]) -> None:
        for path, widget in self.fields.items():
            value = _get(configuration, path, None)
            if value is None:
                continue
            if isinstance(widget, QLineEdit):
                widget.setText(str(value))
            elif isinstance(widget, QCheckBox):
                widget.setChecked(bool(value))
            elif isinstance(widget, QSpinBox):
                widget.setValue(int(value))
            elif isinstance(widget, QDoubleSpinBox):
                widget.setValue(float(value))

    def _load_saved_configuration(self) -> dict[str, Any] | None:
        try:
            with self.config_path.open("r", encoding="utf-8") as handle:
                value = json.load(handle)
            return value if isinstance(value, dict) else None
        except (OSError, json.JSONDecodeError):
            return None

    def _save_configuration(self, configuration: dict[str, Any]) -> None:
        try:
            self.config_path.parent.mkdir(parents=True, exist_ok=True)
            with self.config_path.open("w", encoding="utf-8") as handle:
                json.dump(configuration, handle, indent=2, sort_keys=True)
                handle.write("\n")
        except OSError as exc:
            self.statusBar().showMessage(f"Could not save configuration: {exc}", 5000)

    def apply_configuration(self) -> None:
        config = self._configuration_from_fields()
        self.config = config
        self._save_configuration(config)
        self.backend.send("updateConfiguration", configuration=config)

    def connect_backend(self) -> None:
        config = self._configuration_from_fields()
        self.config = config
        self._save_configuration(config)
        self.backend.send("connect", configuration=config)

    def create_plan(self) -> None:
        config = self._configuration_from_fields()
        self.config = config
        self._save_configuration(config)
        self.backend.send("createPlan", configuration=config)

    def engage(self) -> None:
        config = self._configuration_from_fields()
        self.config = config
        self._save_configuration(config)
        self.backend.send("engage", configuration=config)

    def toggle_pause(self) -> None:
        self.backend.send("setPaused", value=not bool(self.snapshot.get("paused", False)))

    def abort(self) -> None:
        self.backend.send("abort")

    def start_calibration(self) -> None:
        config = self._configuration_from_fields()
        self.config = config
        self._save_configuration(config)
        self.backend.send("startCalibration", configuration=config)

    def stop_calibration(self, apply_results: bool) -> None:
        if apply_results:
            profile = self.snapshot.get("adaptiveVehicleProfile")
            if isinstance(profile, dict):
                self.config = self._configuration_from_fields()
                self.config["vehicle"] = copy.deepcopy(profile)
                self._set_fields_from_config(self.config)
                self._save_configuration(self.config)
                self.backend.send("updateConfiguration", configuration=self.config)
        self.backend.send("stopCalibration", applyResults=apply_results)

    def toggle_gear(self) -> None:
        current = bool(_get(self.snapshot, "telemetry.gear", False))
        self.backend.send("setGear", value=not current)

    def toggle_brakes(self) -> None:
        current = bool(_get(self.snapshot, "telemetry.brakes", False))
        self.backend.send("setBrakes", value=not current)

    def restore_defaults(self) -> None:
        if not hasattr(self, "default_config"):
            return
        self.config = copy.deepcopy(self.default_config)
        self._set_fields_from_config(self.config)
        self._save_configuration(self.config)
        self.backend.send("updateConfiguration", configuration=self.config)

    def _update_snapshot_ui(self) -> None:
        snapshot = self.snapshot or {}
        self.ingame_hud.update(snapshot, self.config)
        telemetry = snapshot.get("telemetry") or {}
        command = snapshot.get("command") or {}
        connection = str(snapshot.get("connectionStatus", "disconnected"))
        phase = str(snapshot.get("phase", "Idle"))
        status = str(snapshot.get("statusMessage", ""))
        self.connection_badge.setText(f"● {connection.capitalize()}")
        color = {"connected": "#36b96b", "connecting": "#d59f38", "failed": "#d64c4c"}.get(connection, "#8a8f9c")
        self.connection_badge.setStyleSheet(f"color: {color}; font-weight: 600;")
        self.header_vessel.setText(str(telemetry.get("vesselName", "—")))
        self.header_phase.setText(phase)
        self.header_status.setText(status)
        self.warning_label.setText(str(snapshot.get("warningMessage") or ""))
        self.error_label.setText(str(snapshot.get("lastError") or ""))
        self.hud.set_state(telemetry, command)
        self._refresh_visuals()

        calibration = snapshot.get("calibration") or {}
        self.calibration_status.setText(str(calibration.get("status", "")))
        self.calibration_progress.setValue(int(_number(calibration.get("progress")) * 1000))
        self.calibration_counts.setText(
            f"{int(calibration.get('acceptedSamples', 0))} accepted / {int(calibration.get('rejectedSamples', 0))} rejected"
        )
        cal_active = bool(calibration.get("active", False))
        engaged = bool(snapshot.get("automationEngaged", False))
        connected = connection == "connected"
        paused = bool(snapshot.get("paused", False))
        self.connect_button.setEnabled(connection not in ("connecting", "connected"))
        self.disconnect_button.setEnabled(connection != "disconnected")
        self.plan_action.setEnabled(connected and not cal_active)
        plan = snapshot.get("plan") or {}
        self.engage_action.setEnabled(bool(plan.get("executionQualified", plan.get("targetCaptureAchieved", False))) and not engaged and not cal_active)
        self.pause_action.setEnabled(engaged)
        self.pause_action.setText("Resume" if paused else "Pause")
        self.abort_action.setEnabled(engaged or cal_active or phase == "Paused")
        self.start_cal_button.setEnabled(connected and not cal_active and not engaged)
        self.stop_apply_button.setEnabled(cal_active)
        self.stop_cal_button.setEnabled(cal_active)
        gear = bool(telemetry.get("gear", False))
        brakes = bool(telemetry.get("brakes", False))
        self.gear_button.setText("Retract Gear" if gear else "Deploy Gear")
        self.brakes_button.setText("Release Brakes" if brakes else "Set Brakes")

        values: dict[str, str] = {
            "phase": phase,
            "status": status,
            "targetPitch": _fmt(command.get("targetPitch"), "°", 1),
            "targetHeading": _fmt(command.get("targetHeading"), "°", 1),
            "targetRoll": _signed(command.get("targetRoll"), "°", 1),
            "targetThrottle": _fmt(_number(command.get("targetThrottle")) * 100, "%", 0),
            "rangeToSite": _distance(telemetry.get("rangeToSite")),
            "bearingToSite": _fmt(telemetry.get("bearingToSite"), "°", 1),
            "headingError": _signed(telemetry.get("headingError"), "°", 1),
            "groundTrackHeading": _fmt(telemetry.get("groundTrackHeading"), "°", 1),
            "courseToSiteError": _signed(telemetry.get("courseToSiteError"), "°", 1),
            "runwayAlongTrack": _distance(telemetry.get("runwayAlongTrack")),
            "runwayCrossTrack": _signed(telemetry.get("runwayCrossTrack"), " m", 0),
            "predictedMissDistance": _distance(telemetry.get("predictedMissDistance")),
            "predictedTAEMDistance": _distance(telemetry.get("predictedTAEMDistance")),
            "predictedTAEMRangeError": _signed(_number(telemetry.get("predictedTAEMRangeError"), float("nan")) / 1000, " km", 1),
            "energyExcessRange": _signed(telemetry.get("energyExcessRange", 0) / 1000, " km", 1),
            "meanAltitude": _distance(telemetry.get("meanAltitude")),
            "radarAltitude": _distance(telemetry.get("radarAltitude")),
            "trueAirSpeed": _fmt(telemetry.get("trueAirSpeed"), " m/s", 1),
            "verticalSpeed": _signed(telemetry.get("verticalSpeed"), " m/s", 1),
            "mach": _fmt(telemetry.get("mach"), "", 2),
            "angleOfAttack": _signed(telemetry.get("angleOfAttack"), "°", 1),
            "dynamicPressure": _fmt(_number(telemetry.get("dynamicPressure")) / 1000, " kPa", 1),
            "gForce": _fmt(telemetry.get("gForce"), " g", 2),
            "stallFraction": _fmt(_number(telemetry.get("stallFraction")) * 100, "%", 0),
            "navballSpeedMode": str(telemetry.get("navballSpeedMode", "unchanged")).upper(),
            "pitchRate": _signed(telemetry.get("pitchRate"), "°/s", 1),
            "rollRate": _signed(telemetry.get("rollRate"), "°/s", 1),
            "yawRate": _signed(telemetry.get("bodyYawRate", telemetry.get("yawRate")), "°/s", 1),
            "autopilotError": _fmt(telemetry.get("autopilotError"), "°", 1),
            "autopilotPitchError": _signed(telemetry.get("commandPitchError", telemetry.get("autopilotPitchError")), "°", 1),
            "autopilotHeadingError": _signed(telemetry.get("commandHeadingError", telemetry.get("autopilotHeadingError")), "°", 1),
            "autopilotRollError": _signed(telemetry.get("commandRollError", telemetry.get("autopilotRollError")), "°", 1),
            "controlPitch": _signed(telemetry.get("controlPitch"), "", 3),
            "controlRoll": _signed(telemetry.get("controlRoll"), "", 3),
            "controlYaw": _signed(telemetry.get("controlYaw"), "", 3),
            "availablePitchTorque": _fmt(_number(telemetry.get("availablePitchTorque")) / 1000, " kN·m", 1),
            "availableRollTorque": _fmt(_number(telemetry.get("availableRollTorque")) / 1000, " kN·m", 1),
            "availableYawTorque": _fmt(_number(telemetry.get("availableYawTorque")) / 1000, " kN·m", 1),
            "pitchMomentOfInertia": _fmt(_number(telemetry.get("pitchMomentOfInertia")) / 1000, "k kg·m²", 1),
            "rollMomentOfInertia": _fmt(_number(telemetry.get("rollMomentOfInertia")) / 1000, "k kg·m²", 1),
            "yawMomentOfInertia": _fmt(_number(telemetry.get("yawMomentOfInertia")) / 1000, "k kg·m²", 1),
            "autopilotPitchKp": _fmt(_first_number(telemetry.get("autopilotPitchPIDGains")) or telemetry.get("autopilotPitchKp"), "", 4),
            "autopilotRollKp": _fmt(_first_number(telemetry.get("autopilotRollPIDGains")) or telemetry.get("autopilotRollKp"), "", 4),
            "autopilotYawKp": _fmt(_first_number(telemetry.get("autopilotYawPIDGains")) or telemetry.get("autopilotYawKp"), "", 4),
            "telemetryLatencyMs": _fmt(telemetry.get("telemetryLatencyMilliseconds", telemetry.get("telemetryLatencyMs")), " ms", 2),
            "guidanceComputeMs": _fmt(telemetry.get("guidanceComputeMilliseconds", telemetry.get("guidanceComputeMs")), " ms", 2),
            "applyLatencyMs": _fmt(telemetry.get("applyLatencyMilliseconds", telemetry.get("applyLatencyMs")), " ms", 2),
            "controlLoopMs": _fmt(telemetry.get("controlLoopMilliseconds", telemetry.get("controlLoopMs")), " ms", 2),
            "wallTickIntervalMs": _fmt(telemetry.get("loopWallDeltaMilliseconds", telemetry.get("wallTickIntervalMs")), " ms", 2),
            "estimatedLiftToDrag": _fmt(telemetry.get("estimatedLiftToDrag"), "", 2),
            "estimatedBallisticCoefficient": _fmt(telemetry.get("estimatedBallisticCoefficient"), " kg/m²", 0),
            "aerodynamicConfidence": _fmt(_number(telemetry.get("aerodynamicConfidence")) * 100, "%", 0),
            "trajectoryCalibrationConfidence": _fmt(_number(telemetry.get("trajectoryCalibrationConfidence")) * 100, "%", 0),
            "trajectoryDensityScale": _fmt(telemetry.get("trajectoryDensityScale"), "×", 3),
            "trajectoryDragScale": _fmt(telemetry.get("trajectoryDragScale"), "×", 3),
            "trajectoryLiftScale": _fmt(telemetry.get("trajectoryLiftScale"), "×", 3),
            "bankEffectiveness": _fmt(telemetry.get("bankEffectiveness"), "×", 3),
            "trajectoryAltitudeResidual": _signed(telemetry.get("trajectoryAltitudeResidual"), " m", 0),
            "trajectorySpeedResidual": _signed(telemetry.get("trajectorySpeedResidual"), " m/s", 1),
            "trajectoryRangeResidual": _signed(telemetry.get("trajectoryRangeResidual"), " m", 0),
        }
        recommended = snapshot.get("adaptiveVehicleProfile") or {}
        values.update({
            "recommendedLD": _fmt(recommended.get("estimatedLiftToDrag"), "", 2),
            "recommendedBeta": _fmt(recommended.get("estimatedBallisticCoefficient"), " kg/m²", 0),
            "recommendedSafe": _fmt(recommended.get("minimumSafeSpeed"), " m/s", 1),
            "recommendedApproach": _fmt(recommended.get("finalApproachSpeed"), " m/s", 1),
            "recommendedTouchdown": _fmt(recommended.get("touchdownSpeed"), " m/s", 1),
            "mass": _fmt(_number(telemetry.get("mass")) / 1000, " t", 1),
            "availableThrust": _fmt(_number(telemetry.get("availableThrust")) / 1000, " kN", 0),
        })
        plan = snapshot.get("plan") or {}
        if plan:
            execution_ready = bool(plan.get("executionQualified", plan.get("targetCaptureAchieved", False)))
            execution_degraded = bool(plan.get("executionDegraded", False))
            values.update({
                "planBurnUT": _fmt(plan.get("burnUT"), "", 1),
                "planDeltaV": _fmt(plan.get("deltaV"), " m/s", 1),
                "planBurnDuration": _fmt(plan.get("estimatedBurnDuration"), " s", 1),
                "planTAEM": _distance(plan.get("predictedTAEMDistance")),
                "planClosest": _distance(plan.get("predictedClosestDistance")),
                "planEntryRange": _distance(plan.get("predictedEntryRange")),
                "planEntryAngle": _signed(plan.get("predictedEntryFlightPathAngle"), "°", 2),
                "planPeriapsis": _distance(plan.get("predictedPostBurnPeriapsisAltitude")),
                "planRobust": f"strict {int(_number(plan.get('robustnessPassed')))} / {int(_number(plan.get('robustnessScenarios')))} ({_number(plan.get('robustnessPassFraction')) * 100:.0f}%), recovery {int(_number(plan.get('recoveryPassed')))} / {int(_number(plan.get('robustnessScenarios')))} ({_number(plan.get('recoveryPassFraction')) * 100:.0f}%)",
                "planWorstMiss": _distance(plan.get("worstCaseClosestDistance")),
                "planWorstPeriapsis": _distance(plan.get("worstCasePostBurnPeriapsisAltitude")),
                "planAchieved": ("Verified" if plan.get("achievedStateCaptureQualified") else "Outside corridor") if plan.get("achievedStateVerified") else "Pending",
                "planConfidence": _fmt(_number(plan.get("confidence")) * 100, "%", 0),
                "planCapture": "Ready (strict)" if plan.get("targetCaptureAchieved") else "Ready (guarded recovery)" if execution_ready and execution_degraded else "Preview only",
            })
        for key, label in self.metrics.items():
            label.setText(values.get(key, "—"))

    def _refresh_visuals(self) -> None:
        if not hasattr(self, "trajectory_map"):
            return
        site = self._configuration_from_fields().get("site", {}) if self.fields else self.config.get("site", {})
        manual = float(self.map_range.currentData() or 0.0) if hasattr(self, "map_range") else 0.0
        self.trajectory_map.set_state(site, self.snapshot, manual)
        self.trajectory_profile.set_state(site, self.snapshot)

    def _apply_visibility(self) -> None:
        if not self.map_action.isChecked() and not self.profile_action.isChecked():
            self.map_action.setChecked(True)
        self.telemetry_scroll.setVisible(self.telemetry_action.isChecked())
        self.trajectory_map.setVisible(self.map_action.isChecked())
        self.trajectory_profile.setVisible(self.profile_action.isChecked())
        self._save_layout()

    def _resize_sidebars(self) -> None:
        if hasattr(self, "controls_scroll"):
            self.controls_scroll.setMinimumWidth(self.control_width.value())
        if hasattr(self, "telemetry_scroll"):
            self.telemetry_scroll.setMinimumWidth(self.telemetry_width.value())
        self._save_layout()

    def _resize_center_panels(self) -> None:
        if not hasattr(self, "center_splitter"):
            return
        self.center_splitter.setSizes([self.hud_height.value(), self.lower_height.value()])
        self._save_layout()

    def _restore_layout(self) -> None:
        self._restoring_layout = True
        try:
            controls_width = int(self.settings.value("controlsWidth", 330))
            telemetry_width = int(self.settings.value("telemetryWidth", 350))
            hud_scale = float(self.settings.value("hudScale", 1.0))
            hud_height = int(self.settings.value("hudHeight", 380))
            lower_height = int(self.settings.value("lowerHeight", 310))
            self.control_width.setValue(controls_width)
            self.telemetry_width.setValue(telemetry_width)
            self.hud_scale.setValue(hud_scale)
            self.hud_height.setValue(hud_height)
            self.lower_height.setValue(lower_height)
            self.hud.set_instrument_scale(hud_scale)
            self.telemetry_action.setChecked(str(self.settings.value("showTelemetry", "true")).lower() == "true")
            self.map_action.setChecked(str(self.settings.value("showMap", "true")).lower() == "true")
            self.profile_action.setChecked(str(self.settings.value("showProfile", "true")).lower() == "true")
            root_sizes = self.settings.value("rootSplitter")
            center_sizes = self.settings.value("centerSplitter")
            lower_sizes = self.settings.value("lowerSplitter")
            if root_sizes:
                self.root_splitter.restoreState(root_sizes)
            else:
                self.root_splitter.setSizes([controls_width, 820, telemetry_width])
            if center_sizes:
                self.center_splitter.restoreState(center_sizes)
            else:
                self.center_splitter.setSizes([hud_height, lower_height])
            if lower_sizes:
                self.lower_splitter.restoreState(lower_sizes)
            else:
                self.lower_splitter.setSizes([1, 1])
            self.telemetry_scroll.setVisible(self.telemetry_action.isChecked())
            self.trajectory_map.setVisible(self.map_action.isChecked())
            self.trajectory_profile.setVisible(self.profile_action.isChecked())
        finally:
            self._restoring_layout = False
        self._save_layout()

    def _save_layout(self) -> None:
        if self._restoring_layout or not hasattr(self, "root_splitter"):
            return
        self.settings.setValue("controlsWidth", self.control_width.value())
        self.settings.setValue("telemetryWidth", self.telemetry_width.value())
        self.settings.setValue("hudScale", self.hud_scale.value())
        self.settings.setValue("hudHeight", self.hud_height.value())
        self.settings.setValue("lowerHeight", self.lower_height.value())
        self.settings.setValue("showTelemetry", self.telemetry_action.isChecked())
        self.settings.setValue("showMap", self.map_action.isChecked())
        self.settings.setValue("showProfile", self.profile_action.isChecked())
        self.settings.setValue("rootSplitter", self.root_splitter.saveState())
        self.settings.setValue("centerSplitter", self.center_splitter.saveState())
        self.settings.setValue("lowerSplitter", self.lower_splitter.saveState())

    def restore_layout_defaults(self) -> None:
        self.control_width.setValue(330)
        self.telemetry_width.setValue(350)
        self.hud_scale.setValue(1.0)
        self.hud_height.setValue(380)
        self.lower_height.setValue(310)
        self.hud.set_instrument_scale(1.0)
        self.telemetry_action.setChecked(True)
        self.map_action.setChecked(True)
        self.profile_action.setChecked(True)
        self.root_splitter.setSizes([330, 820, 350])
        self.center_splitter.setSizes([500, 300])
        self.lower_splitter.setSizes([1, 1])
        self._apply_visibility()

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self._save_layout()
        if self.config:
            self._save_configuration(self._configuration_from_fields())
        self.ingame_hud.close()
        self.backend.shutdown()
        event.accept()
