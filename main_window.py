from __future__ import annotations

import math
from typing import Optional

from constants import (
    BUFFER_NODE,
    DEFAULT_BUFFER_NODE_IDS,
    NODE_COMBO_MAX_VISIBLE_ITEMS,
    NODE_RADIUS,
    NORMAL_NODE,
    TIMER_INTERVAL_MS,
)
from graph_canvas import GraphCanvas
from graph_model import GraphModel
from models import AGV
from qt_compat import (
    NO_EDIT,
    QT_API,
    RESIZE_TO_CONTENTS,
    SELECT_ROWS,
    STRETCH,
    QAbstractItemView,
    QColor,
    QComboBox,
    QFont,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPointF,
    QPushButton,
    QSizePolicy,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QTimer,
    Qt,
    QVBoxLayout,
    QWidget,
)
from traffic_controller import TrafficController


class MainWindow(QMainWindow):
    """메인 윈도우: Graph Canvas와 오른쪽 Control Panel을 구성합니다."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(f"AGV Traffic Control Visualizer ({QT_API})")
        self.resize(1220, 760)

        self.graph = GraphModel()
        self.controller = TrafficController(self.graph)
        self.timer = QTimer(self)
        self.timer.setInterval(TIMER_INTERVAL_MS)
        self.timer.timeout.connect(self.step_simulation)

        self.canvas = GraphCanvas(self.graph, self.controller, self)

        self.clock_label = QLabel("Clock: 0")
        self.clock_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        self.status_label = QLabel("Status: EDITING")
        self.status_label.setStyleSheet(
            "padding: 8px; border: 1px solid #D1D5DB; background: #FFFFFF; color: #111827;"
        )

        self.play_button = QPushButton("Play")
        self.pause_button = QPushButton("Pause")
        self.step_button = QPushButton("Step")
        self.reset_button = QPushButton("Reset")
        self.sample_button = QPushButton("Load Sample Map")

        self.node_combo = QComboBox()
        self.add_node_button = QPushButton("Add Node")
        self.delete_node_button = QPushButton("Delete Node")

        self.start_combo = QComboBox()
        self.goal_combo = QComboBox()
        self.after_work_goal_combo = QComboBox()
        self.speed_spin = QSpinBox()
        self.speed_spin.setRange(1, 10)
        self.speed_spin.setValue(1)
        self.speed_spin.setSuffix(" tick/edge")
        self.work_ticks_spin = QSpinBox()
        self.work_ticks_spin.setRange(0, 999)
        self.work_ticks_spin.setValue(0)
        self.work_ticks_spin.setSuffix(" clk")
        self.round_trip_spin = QSpinBox()
        self.round_trip_spin.setRange(0, 50)
        self.round_trip_spin.setValue(0)
        self.round_trip_spin.setSuffix(" round")
        self.add_agv_button = QPushButton("Add AGV")

        self.edit_agv_combo = QComboBox()
        self.edit_start_combo = QComboBox()
        self.edit_goal_combo = QComboBox()
        self.edit_after_work_goal_combo = QComboBox()
        self.edit_speed_spin = QSpinBox()
        self.edit_speed_spin.setRange(1, 10)
        self.edit_speed_spin.setValue(1)
        self.edit_speed_spin.setSuffix(" tick/edge")
        self.edit_work_ticks_spin = QSpinBox()
        self.edit_work_ticks_spin.setRange(0, 999)
        self.edit_work_ticks_spin.setValue(0)
        self.edit_work_ticks_spin.setSuffix(" clk")
        self.edit_round_trip_spin = QSpinBox()
        self.edit_round_trip_spin.setRange(0, 50)
        self.edit_round_trip_spin.setValue(0)
        self.edit_round_trip_spin.setSuffix(" round")
        self.update_agv_button = QPushButton("Update AGV")
        self.delete_agv_button = QPushButton("Delete AGV")

        for combo in (
            self.node_combo,
            self.start_combo,
            self.goal_combo,
            self.after_work_goal_combo,
            self.edit_start_combo,
            self.edit_goal_combo,
            self.edit_after_work_goal_combo,
        ):
            self._configure_scrollable_node_combo(combo)

        self.agv_table = QTableWidget(0, 11)
        self.agv_table.setHorizontalHeaderLabels(
            [
                "ID",
                "Start",
                "Current",
                "Target",
                "Speed",
                "Trips",
                "Work",
                "Status",
                "Wait",
                "Replans",
                "Plan",
            ]
        )
        self.agv_table.horizontalHeader().setSectionResizeMode(STRETCH)
        self.agv_table.horizontalHeader().setSectionResizeMode(0, RESIZE_TO_CONTENTS)
        self.agv_table.horizontalHeader().setSectionResizeMode(7, RESIZE_TO_CONTENTS)
        self.agv_table.horizontalHeader().setSectionResizeMode(8, RESIZE_TO_CONTENTS)
        self.agv_table.horizontalHeader().setSectionResizeMode(9, RESIZE_TO_CONTENTS)
        self.agv_table.setSelectionBehavior(SELECT_ROWS)
        self.agv_table.setEditTriggers(NO_EDIT)

        self.log_window = QTextEdit()
        self.log_window.setReadOnly(True)
        self.log_window.setMinimumHeight(190)
        self.log_window.setStyleSheet("font-family: Consolas, monospace; font-size: 10pt;")

        self._build_layout()
        self._connect_signals()
        self.graph_changed()
        self.log("Ready. Double-click canvas to add nodes, drag node to node to add edges.")

    def _configure_scrollable_node_combo(self, combo: QComboBox) -> None:
        """노드가 많아도 Start/Goal 콤보 popup을 휠로 끝까지 탐색할 수 있게 합니다."""
        combo.setMaxVisibleItems(NODE_COMBO_MAX_VISIBLE_ITEMS)
        combo.setMinimumContentsLength(10)
        combo.setSizeAdjustPolicy(
            QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon
        )
        view = combo.view()
        view.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        view.setVerticalScrollMode(QAbstractItemView.ScrollMode.ScrollPerPixel)
        if hasattr(view, "setUniformItemSizes"):
            view.setUniformItemSizes(True)
        self._limit_node_combo_popup_height(combo)

    def _limit_node_combo_popup_height(self, combo: QComboBox) -> None:
        """Qt style이 setMaxVisibleItems를 무시해도 popup이 화면 밖으로 커지지 않게 합니다."""
        view = combo.view()
        item_count = max(1, combo.count())
        visible_items = max(1, min(NODE_COMBO_MAX_VISIBLE_ITEMS, item_count))
        row_height = view.sizeHintForRow(0) if combo.count() else -1
        if row_height <= 0:
            row_height = max(24, combo.sizeHint().height())
        popup_height = row_height * visible_items + 6
        view.setMaximumHeight(popup_height)
        if combo.count() > NODE_COMBO_MAX_VISIBLE_ITEMS:
            view.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        else:
            view.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)

    def _build_layout(self) -> None:
        central = QWidget()
        root = QHBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self.canvas, 1)

        panel = QWidget()
        panel.setFixedWidth(390)
        panel.setStyleSheet("background: #F9FAFB;")
        panel_layout = QVBoxLayout(panel)
        panel_layout.setContentsMargins(14, 14, 14, 14)
        panel_layout.setSpacing(10)

        panel_layout.addWidget(self.clock_label)
        panel_layout.addWidget(self.status_label)

        button_grid = QGridLayout()
        button_grid.addWidget(self.play_button, 0, 0)
        button_grid.addWidget(self.pause_button, 0, 1)
        button_grid.addWidget(self.step_button, 1, 0)
        button_grid.addWidget(self.reset_button, 1, 1)
        panel_layout.addLayout(button_grid)
        panel_layout.addWidget(self.sample_button)

        node_group = QGroupBox("Node Tools")
        node_layout = QGridLayout(node_group)
        node_layout.addWidget(QLabel("Selected"), 0, 0)
        node_layout.addWidget(self.node_combo, 0, 1, 1, 2)
        node_layout.addWidget(self.add_node_button, 1, 0, 1, 2)
        node_layout.addWidget(self.delete_node_button, 1, 2)
        panel_layout.addWidget(node_group)

        add_group = QGroupBox("Add AGV")
        add_layout = QGridLayout(add_group)
        add_layout.addWidget(QLabel("Start"), 0, 0)
        add_layout.addWidget(self.start_combo, 0, 1)
        add_layout.addWidget(QLabel("Work Goal"), 1, 0)
        add_layout.addWidget(self.goal_combo, 1, 1)
        add_layout.addWidget(QLabel("After Work Goal"), 2, 0)
        add_layout.addWidget(self.after_work_goal_combo, 2, 1)
        add_layout.addWidget(QLabel("Work clk"), 3, 0)
        add_layout.addWidget(self.work_ticks_spin, 3, 1)
        add_layout.addWidget(QLabel("Speed"), 4, 0)
        add_layout.addWidget(self.speed_spin, 4, 1)
        add_layout.addWidget(QLabel("Round Trips"), 5, 0)
        add_layout.addWidget(self.round_trip_spin, 5, 1)
        add_layout.addWidget(self.add_agv_button, 6, 0, 1, 2)
        panel_layout.addWidget(add_group)

        edit_group = QGroupBox("Edit AGV")
        edit_layout = QGridLayout(edit_group)
        edit_layout.addWidget(QLabel("AGV"), 0, 0)
        edit_layout.addWidget(self.edit_agv_combo, 0, 1, 1, 2)
        edit_layout.addWidget(QLabel("Start"), 1, 0)
        edit_layout.addWidget(self.edit_start_combo, 1, 1, 1, 2)
        edit_layout.addWidget(QLabel("Work Goal"), 2, 0)
        edit_layout.addWidget(self.edit_goal_combo, 2, 1, 1, 2)
        edit_layout.addWidget(QLabel("After Work Goal"), 3, 0)
        edit_layout.addWidget(self.edit_after_work_goal_combo, 3, 1, 1, 2)
        edit_layout.addWidget(QLabel("Work clk"), 4, 0)
        edit_layout.addWidget(self.edit_work_ticks_spin, 4, 1, 1, 2)
        edit_layout.addWidget(QLabel("Speed"), 5, 0)
        edit_layout.addWidget(self.edit_speed_spin, 5, 1, 1, 2)
        edit_layout.addWidget(QLabel("Round Trips"), 6, 0)
        edit_layout.addWidget(self.edit_round_trip_spin, 6, 1, 1, 2)
        edit_layout.addWidget(self.update_agv_button, 7, 0, 1, 2)
        edit_layout.addWidget(self.delete_agv_button, 7, 2)
        panel_layout.addWidget(edit_group)

        panel_layout.addWidget(QLabel("AGV State"))
        panel_layout.addWidget(self.agv_table, 1)
        panel_layout.addWidget(QLabel("Simulation Log"))
        panel_layout.addWidget(self.log_window, 1)

        root.addWidget(panel)
        self.setCentralWidget(central)

    def _connect_signals(self) -> None:
        self.play_button.clicked.connect(self.play_simulation)
        self.pause_button.clicked.connect(self.pause_simulation)
        self.step_button.clicked.connect(self.step_simulation)
        self.reset_button.clicked.connect(self.reset_simulation)
        self.sample_button.clicked.connect(self.load_sample_map)
        self.node_combo.currentIndexChanged.connect(self.node_selection_changed)
        self.add_node_button.clicked.connect(self.add_node_from_ui)
        self.delete_node_button.clicked.connect(self.delete_node_from_ui)
        self.add_agv_button.clicked.connect(self.add_agv_from_ui)
        self.edit_agv_combo.currentIndexChanged.connect(self.agv_selection_changed)
        self.update_agv_button.clicked.connect(self.update_agv_from_ui)
        self.delete_agv_button.clicked.connect(self.delete_agv_from_ui)
        self.agv_table.itemSelectionChanged.connect(self.agv_table_selection_changed)

    def graph_changed(self) -> None:
        self.refresh_node_combos()
        self.controller.recompute_conflict_zones()
        self.refresh_all()

    def refresh_all(self) -> None:
        self.clock_label.setText(f"Clock: {self.controller.clock}")
        if self.controller.deadlock_detected:
            self.status_label.setText("Status: DEADLOCK DETECTED")
            self.status_label.setStyleSheet(
                "padding: 8px; border: 1px solid #991B1B; "
                "background: #FEE2E2; color: #991B1B; font-weight: bold;"
            )
        elif self.controller.simulation_finished:
            self.status_label.setText("Status: FINISHED")
            self.status_label.setStyleSheet(
                "padding: 8px; border: 1px solid #15803D; "
                "background: #DCFCE7; color: #166534; font-weight: bold;"
            )
        elif self.timer.isActive():
            self.status_label.setText("Status: RUNNING")
            self.status_label.setStyleSheet(
                "padding: 8px; border: 1px solid #2563EB; "
                "background: #DBEAFE; color: #1D4ED8; font-weight: bold;"
            )
        else:
            self.status_label.setText("Status: PAUSED / EDITING")
            self.status_label.setStyleSheet(
                "padding: 8px; border: 1px solid #D1D5DB; background: #FFFFFF; color: #111827;"
            )
        self.refresh_agv_edit_controls()
        self.refresh_agv_table()
        self.canvas.refresh()

    def refresh_node_combos(self) -> None:
        selected_node = self.canvas.selected_node_id or self.node_combo.currentData()
        start_selected = self.start_combo.currentData()
        goal_selected = self.goal_combo.currentData()
        after_work_selected = self.after_work_goal_combo.currentData()
        edit_start_selected = self.edit_start_combo.currentData()
        edit_goal_selected = self.edit_goal_combo.currentData()
        edit_after_work_selected = self.edit_after_work_goal_combo.currentData()
        self.node_combo.blockSignals(True)
        self.start_combo.blockSignals(True)
        self.goal_combo.blockSignals(True)
        self.after_work_goal_combo.blockSignals(True)
        self.edit_start_combo.blockSignals(True)
        self.edit_goal_combo.blockSignals(True)
        self.edit_after_work_goal_combo.blockSignals(True)
        self.node_combo.clear()
        self.start_combo.clear()
        self.goal_combo.clear()
        self.after_work_goal_combo.clear()
        self.edit_start_combo.clear()
        self.edit_goal_combo.clear()
        self.edit_after_work_goal_combo.clear()
        self.after_work_goal_combo.addItem("None", None)
        self.edit_after_work_goal_combo.addItem("None", None)
        for node_id in sorted(self.graph.nodes):
            label = f"Node {node_id}"
            self.node_combo.addItem(label, node_id)
            self.start_combo.addItem(label, node_id)
            self.goal_combo.addItem(label, node_id)
            self.after_work_goal_combo.addItem(label, node_id)
            self.edit_start_combo.addItem(label, node_id)
            self.edit_goal_combo.addItem(label, node_id)
            self.edit_after_work_goal_combo.addItem(label, node_id)
        self._restore_combo_selection(self.node_combo, selected_node)
        self._restore_combo_selection(self.start_combo, start_selected)
        self._restore_combo_selection(self.goal_combo, goal_selected)
        self._restore_combo_selection(self.after_work_goal_combo, after_work_selected)
        self._restore_combo_selection(self.edit_start_combo, edit_start_selected)
        self._restore_combo_selection(self.edit_goal_combo, edit_goal_selected)
        self._restore_combo_selection(self.edit_after_work_goal_combo, edit_after_work_selected)
        for combo in (
            self.node_combo,
            self.start_combo,
            self.goal_combo,
            self.after_work_goal_combo,
            self.edit_start_combo,
            self.edit_goal_combo,
            self.edit_after_work_goal_combo,
        ):
            self._limit_node_combo_popup_height(combo)
        self.node_combo.blockSignals(False)
        self.start_combo.blockSignals(False)
        self.goal_combo.blockSignals(False)
        self.after_work_goal_combo.blockSignals(False)
        self.edit_start_combo.blockSignals(False)
        self.edit_goal_combo.blockSignals(False)
        self.edit_after_work_goal_combo.blockSignals(False)
        current_node = self.node_combo.currentData()
        self.canvas.selected_node_id = (
            current_node if current_node in self.graph.nodes else None
        )

    def refresh_agv_table(self) -> None:
        selected_agv_id = self.edit_agv_combo.currentData()
        self.agv_table.blockSignals(True)
        self.agv_table.clearSelection()
        self.agv_table.setRowCount(len(self.controller.agvs))
        for row, agv in enumerate(self.controller.agvs):
            values = [
                agv.agv_id,
                str(agv.start_node),
                str(agv.current_node),
                str(agv.goal_node),
                f"1/{max(1, agv.speed_ticks)}",
                f"{agv.completed_legs}/{agv.total_required_legs()}",
                f"{agv.work_remaining_ticks}/{agv.work_ticks}",
                agv.status,
                str(agv.wait_count),
                str(agv.replan_count),
                "->".join(str(node) for node in agv.planned_path[:12])
                + ("..." if len(agv.planned_path) > 12 else ""),
            ]
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                if agv.status == "DEADLOCK":
                    item.setBackground(QColor("#FEE2E2"))
                elif agv.status == "DONE":
                    item.setBackground(QColor("#DCFCE7"))
                elif agv.status == "MOVING":
                    item.setBackground(QColor("#E0F2FE"))
                elif agv.status == "WORKING":
                    item.setBackground(QColor("#FFFBEB"))
                elif agv.status.startswith("ESCAPE"):
                    item.setBackground(QColor("#FEF3C7"))
                self.agv_table.setItem(row, col, item)
            if selected_agv_id == agv.agv_id:
                self.agv_table.selectRow(row)
        self.agv_table.blockSignals(False)

    def refresh_agv_edit_controls(self) -> None:
        selected_agv_id = self.edit_agv_combo.currentData()
        agv_ids = [agv.agv_id for agv in self.controller.agvs]
        if selected_agv_id not in agv_ids:
            selected_agv_id = agv_ids[0] if agv_ids else None

        self.edit_agv_combo.blockSignals(True)
        self.edit_agv_combo.clear()
        for agv in self.controller.agvs:
            original_goal = agv.original_goal_node or agv.goal_node
            after_goal = (
                f"->{agv.after_work_goal_node}"
                if agv.after_work_goal_node is not None
                else ""
            )
            self.edit_agv_combo.addItem(
                f"AGV {agv.agv_id} ({agv.start_node}->{original_goal}{after_goal})",
                agv.agv_id,
            )
        self._set_combo_to_data(self.edit_agv_combo, selected_agv_id, block_signals=False)
        self.edit_agv_combo.blockSignals(False)

        agv = self._find_agv(selected_agv_id)
        editor_enabled = agv is not None and bool(self.graph.nodes)
        self.edit_start_combo.setEnabled(editor_enabled)
        self.edit_goal_combo.setEnabled(editor_enabled)
        self.edit_after_work_goal_combo.setEnabled(editor_enabled)
        self.edit_work_ticks_spin.setEnabled(editor_enabled)
        self.edit_speed_spin.setEnabled(editor_enabled)
        self.edit_round_trip_spin.setEnabled(editor_enabled)
        self.update_agv_button.setEnabled(editor_enabled)
        self.delete_agv_button.setEnabled(agv is not None)
        if agv is not None:
            self._load_agv_into_editor(agv)

    def node_selection_changed(self, _index: int = -1) -> None:
        node_id = self.node_combo.currentData()
        self.canvas.set_selected_node(node_id if node_id in self.graph.nodes else None)

    def agv_selection_changed(self, _index: int = -1) -> None:
        agv = self._find_agv(self.edit_agv_combo.currentData())
        if agv is None:
            return
        self._load_agv_into_editor(agv)
        self._select_agv_table_row(agv.agv_id)

    def agv_table_selection_changed(self) -> None:
        row = self.agv_table.currentRow()
        if row < 0:
            return
        item = self.agv_table.item(row, 0)
        if item is None:
            return
        agv = self._find_agv(item.text())
        if agv is None:
            return
        self._set_combo_to_data(self.edit_agv_combo, agv.agv_id)
        self._load_agv_into_editor(agv)

    def set_selected_node(self, node_id: Optional[int], refresh: bool = True) -> None:
        """Canvas 클릭과 Node Tools 콤보 선택을 동기화합니다."""
        selected = node_id if node_id in self.graph.nodes else None
        self.canvas.selected_node_id = selected
        self._set_combo_to_data(self.node_combo, selected)
        if refresh:
            self.refresh_all()

    def add_node_from_ui(self) -> None:
        self.timer.stop()
        pos = self._next_auto_node_position()
        node_id = self.graph.add_node(pos.x(), pos.y())
        self.canvas.selected_node_id = node_id
        self._invalidate_simulation_after_graph_edit()
        self.log(f"Node {node_id} added from Node Tools at ({pos.x():.0f}, {pos.y():.0f}).")
        self.graph_changed()

    def delete_node_from_ui(self) -> None:
        node_id = self.node_combo.currentData()
        if node_id is None or node_id not in self.graph.nodes:
            QMessageBox.warning(self, "Delete Node", "삭제할 노드를 선택하세요.")
            return

        self.timer.stop()
        removed_agvs = [
            agv.agv_id
            for agv in self.controller.agvs
            if node_id in (agv.start_node, agv.current_node, agv.goal_node)
        ]
        self.controller.agvs = [
            agv
            for agv in self.controller.agvs
            if node_id not in (agv.start_node, agv.current_node, agv.goal_node)
        ]

        removed_edges = self.graph.remove_node(node_id)
        self.canvas.selected_node_id = None
        self._invalidate_simulation_after_graph_edit()
        self.log(f"Node {node_id} deleted with {removed_edges} connected edge(s).")
        if removed_agvs:
            self.log(
                "Removed AGV(s) referencing deleted node: "
                + ", ".join(sorted(removed_agvs))
                + "."
            )
        self.graph_changed()

    def play_simulation(self) -> None:
        if self.controller.deadlock_detected or self.controller.simulation_finished:
            self.log("Simulation cannot play. Press Reset to run again.")
            return
        if not self.controller.agvs:
            self.log("Add at least one AGV before Play.")
            return
        if not self.controller.initial_plan_built:
            for message in self.controller.build_rhcr_plan("initial RHCR planning"):
                self.log(message)
        self.timer.start()
        self.refresh_all()

    def pause_simulation(self) -> None:
        self.timer.stop()
        self.log("Simulation paused.")
        self.refresh_all()

    def step_simulation(self) -> None:
        if self.controller.deadlock_detected or self.controller.simulation_finished:
            self.timer.stop()
            self.refresh_all()
            return
        result = self.controller.step()
        for message in result.logs:
            self.log(message)
        if result.deadlock or result.all_done:
            self.timer.stop()
        self.refresh_all()

    def reset_simulation(self) -> None:
        self.timer.stop()
        for message in self.controller.reset_simulation():
            self.log(message)
        self.refresh_all()

    def add_agv_from_ui(self) -> None:
        start_node = self.start_combo.currentData()
        goal_node = self.goal_combo.currentData()
        after_work_goal = self.after_work_goal_combo.currentData()
        if start_node is None or goal_node is None:
            QMessageBox.warning(self, "Add AGV", "노드를 먼저 추가하세요.")
            return
        if start_node == goal_node:
            QMessageBox.warning(self, "Add AGV", "시작 노드와 goal 노드는 달라야 합니다.")
            return
        if not self._is_start_node_available(start_node):
            QMessageBox.warning(self, "Add AGV", "해당 시작 노드에는 이미 AGV가 있습니다.")
            return

        agv_id = self._next_agv_id()
        speed_ticks = self.speed_spin.value()
        work_ticks = self.work_ticks_spin.value()
        round_trips = self.round_trip_spin.value()
        agv = AGV(
            agv_id=agv_id,
            start_node=start_node,
            current_node=start_node,
            goal_node=goal_node,
            original_goal_node=goal_node,
            after_work_goal_node=after_work_goal,
            speed_ticks=speed_ticks,
            work_ticks=work_ticks,
            round_trips_total=round_trips,
        )
        self.controller.add_agv(agv)
        self.log(
            f"AGV {agv_id} added: start Node {start_node}, work goal Node {goal_node}, "
            f"after-work goal {after_work_goal or '-'}, work={work_ticks} clk, "
            f"speed=1 edge/{speed_ticks} tick(s), round trips={round_trips}."
        )
        self.refresh_all()
        self._set_combo_to_data(self.edit_agv_combo, agv_id)
        self.agv_selection_changed()

    def update_agv_from_ui(self) -> None:
        agv_id = self.edit_agv_combo.currentData()
        agv = self._find_agv(agv_id)
        start_node = self.edit_start_combo.currentData()
        goal_node = self.edit_goal_combo.currentData()
        after_work_goal = self.edit_after_work_goal_combo.currentData()
        speed_ticks = self.edit_speed_spin.value()
        work_ticks = self.edit_work_ticks_spin.value()
        round_trips = self.edit_round_trip_spin.value()

        if agv is None:
            QMessageBox.warning(self, "Update AGV", "수정할 AGV를 선택하세요.")
            return
        if start_node is None or goal_node is None:
            QMessageBox.warning(self, "Update AGV", "start node와 goal node를 선택하세요.")
            return
        if start_node == goal_node:
            QMessageBox.warning(self, "Update AGV", "start node와 goal node는 달라야 합니다.")
            return
        if not self._is_start_node_available(start_node, excluded_agv_id=agv.agv_id):
            QMessageBox.warning(self, "Update AGV", "해당 start node에는 이미 다른 AGV가 있습니다.")
            return

        self.timer.stop()
        old_start = agv.start_node
        old_goal = agv.original_goal_node or agv.goal_node
        old_after_goal = agv.after_work_goal_node
        old_speed = agv.speed_ticks
        old_work_ticks = agv.work_ticks
        old_round_trips = agv.round_trips_total
        agv.start_node = start_node
        agv.current_node = start_node
        agv.goal_node = goal_node
        agv.original_goal_node = goal_node
        agv.after_work_goal_node = after_work_goal
        agv.speed_ticks = speed_ticks
        agv.work_ticks = work_ticks
        agv.round_trips_total = round_trips
        agv.completed_legs = 0
        agv.work_remaining_ticks = 0
        agv.work_completed_for_current_goal = False
        agv.planned_path = []
        agv.trip_turnaround = False
        agv.status = "WAITING"
        agv.wait_count = 0
        agv.replan_count = 0
        agv.plan_start_tick = self.controller.clock
        agv.last_move = None
        agv.previous_node = None
        agv.committed_path = []
        agv.commit_until_index = 0
        agv.commit_index = 0
        agv.escape_mode = False
        agv.escape_hold = False
        agv.escape_returning = False
        agv.escape_target = None
        agv.escape_attempts = 0
        agv.failed_escape_attempts = 0
        agv.last_risk_replan_tick = -9999
        agv.last_move_tick = -9999
        self._invalidate_simulation_after_graph_edit()
        self._set_combo_to_data(self.edit_agv_combo, agv.agv_id)
        self.log(
            f"AGV {agv.agv_id} updated: start Node {old_start}->{start_node}, "
            f"work goal Node {old_goal}->{goal_node}, after-work goal "
            f"{old_after_goal or '-'}->{after_work_goal or '-'}, "
            f"work {old_work_ticks}->{work_ticks} clk, speed {old_speed}->{speed_ticks}, "
            f"round trips {old_round_trips}->{round_trips}. "
            "Current node reset to start."
        )
        self.refresh_all()

    def delete_agv_from_ui(self) -> None:
        agv_id = self.edit_agv_combo.currentData()
        agv = self._find_agv(agv_id)
        if agv is None:
            QMessageBox.warning(self, "Delete AGV", "삭제할 AGV를 선택하세요.")
            return

        self.timer.stop()
        self.controller.agvs = [
            existing for existing in self.controller.agvs if existing.agv_id != agv.agv_id
        ]
        self._invalidate_simulation_after_graph_edit()
        self.log(f"AGV {agv.agv_id} deleted.")
        self.refresh_all()

    def load_sample_map(self) -> None:
        self.timer.stop()
        self.graph.clear()
        self.controller.clear()

        # 요청 예시 맵을 좌표로 옮긴 기본 샘플입니다.
        sample_nodes = {
            3: (80, 80),
            4: (180, 80),
            5: (280, 80),
            6: (380, 80),
            7: (480, 80),
            1: (580, 80),
            8: (80, 180),
            9: (280, 180),
            10: (480, 180),
            11: (80, 280),
            12: (180, 280),
            13: (280, 280),
            14: (380, 280),
            15: (480, 280),
            16: (80, 380),
            2: (180, 380),
        }
        for node_id, (x, y) in sample_nodes.items():
            node_type = BUFFER_NODE if node_id in DEFAULT_BUFFER_NODE_IDS else NORMAL_NODE
            self.graph.add_node(x, y, node_id=node_id, node_type=node_type)

        sample_edges = [
            (3, 4),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 1),
            (3, 8),
            (8, 11),
            (11, 16),
            (5, 9),
            (9, 13),
            (7, 10),
            (10, 15),
            (11, 12),
            (12, 13),
            (13, 14),
            (14, 15),
            (16, 2),
        ]
        for a, b in sample_edges:
            self.graph.add_edge(a, b)

        self.controller.add_agv(AGV("A", start_node=16, current_node=16, goal_node=1))
        self.controller.add_agv(AGV("B", start_node=13, current_node=13, goal_node=2))
        self.controller.add_agv(AGV("C", start_node=15, current_node=15, goal_node=2))
        self.controller.recompute_conflict_zones()
        self.controller.initial_plan_built = False

        self.log_window.clear()
        self.log("Sample map loaded.")
        self.log("Sample buffer nodes: Node 7, Node 10.")
        self.log("Sample AGVs: A 16->1, B 13->2, C 15->2.")
        self.graph_changed()

    def log(self, message: str) -> None:
        self.log_window.append(f"[Clock {self.controller.clock}] {message}")

    def _find_agv(self, agv_id: Optional[str]) -> Optional[AGV]:
        if agv_id is None:
            return None
        for agv in self.controller.agvs:
            if agv.agv_id == agv_id:
                return agv
        return None

    def _is_start_node_available(
        self,
        node_id: int,
        excluded_agv_id: Optional[str] = None,
    ) -> bool:
        """AGV start/current node 중복 배치를 막습니다."""
        for agv in self.controller.agvs:
            if agv.agv_id == excluded_agv_id:
                continue
            if agv.current_node == node_id:
                return False
        return True

    def _load_agv_into_editor(self, agv: AGV) -> None:
        self._set_combo_to_data(self.edit_start_combo, agv.start_node)
        self._set_combo_to_data(self.edit_goal_combo, agv.original_goal_node or agv.goal_node)
        self._set_combo_to_data(self.edit_after_work_goal_combo, agv.after_work_goal_node)
        self.edit_work_ticks_spin.setValue(max(0, agv.work_ticks))
        self.edit_speed_spin.setValue(max(1, agv.speed_ticks))
        self.edit_round_trip_spin.setValue(max(0, agv.round_trips_total))

    def _select_agv_table_row(self, agv_id: str) -> None:
        self.agv_table.blockSignals(True)
        self.agv_table.clearSelection()
        for row in range(self.agv_table.rowCount()):
            item = self.agv_table.item(row, 0)
            if item is not None and item.text() == agv_id:
                self.agv_table.selectRow(row)
                break
        self.agv_table.blockSignals(False)

    def _next_agv_id(self) -> str:
        used = {agv.agv_id for agv in self.controller.agvs}
        for code in range(ord("A"), ord("Z") + 1):
            candidate = chr(code)
            if candidate not in used:
                return candidate
        index = 1
        while f"AGV{index}" in used:
            index += 1
        return f"AGV{index}"

    def _next_auto_node_position(self) -> QPointF:
        """Node Tools에서 추가할 노드를 기존 노드와 겹치지 않는 격자 위치에 배치합니다."""
        origin_x, origin_y = 80, 80
        spacing = 100
        columns = 6
        max_attempts = max(80, len(self.graph.nodes) + 30)

        for attempt in range(max_attempts):
            x = origin_x + (attempt % columns) * spacing
            y = origin_y + (attempt // columns) * spacing
            if self._is_free_node_position(x, y):
                return QPointF(x, y)

        if not self.graph.nodes:
            return QPointF(origin_x, origin_y)
        max_x = max(pos.x() for pos in self.graph.nodes.values())
        min_y = min(pos.y() for pos in self.graph.nodes.values())
        return QPointF(max_x + spacing, min_y)

    def _is_free_node_position(self, x: float, y: float) -> bool:
        for pos in self.graph.nodes.values():
            if math.hypot(pos.x() - x, pos.y() - y) < NODE_RADIUS * 3:
                return False
        return True

    def _invalidate_simulation_after_graph_edit(self) -> None:
        """그래프 변경 후 기존 계획/예약/락을 폐기하고 다음 실행에서 다시 계획합니다."""
        self.controller.node_reservations.clear()
        self.controller.edge_reservations.clear()
        self.controller.zone_reservations.clear()
        self.controller.global_no_move_ticks = 0
        self.controller.escape_fail_ticks = 0
        self.controller.deadlock_detected = False
        self.controller.simulation_finished = False
        self.controller.initial_plan_built = False
        self.controller.rhcr_next_replan_tick = self.controller.clock
        for agv in self.controller.agvs:
            agv.planned_path = []
            agv.last_move = None
            agv.wait_count = 0
            agv.previous_node = None
            agv.committed_path = []
            agv.commit_until_index = 0
            agv.commit_index = 0
            agv.escape_mode = False
            agv.escape_hold = False
            agv.escape_returning = False
            agv.escape_target = None
            agv.failed_escape_attempts = 0
            agv.last_risk_replan_tick = -9999
            agv.last_move_tick = -9999
            agv.work_remaining_ticks = 0
            agv.work_completed_for_current_goal = False
            agv.completed_legs = 0
            agv.trip_turnaround = False
            if agv.original_goal_node is not None:
                agv.goal_node = agv.original_goal_node
            if agv.status != "DONE":
                agv.status = "WAITING"
        self.controller.recompute_conflict_zones()

    def _set_combo_to_data(
        self,
        combo: QComboBox,
        value: object,
        block_signals: bool = True,
    ) -> None:
        if block_signals:
            combo.blockSignals(True)
        if value is None:
            none_index = -1
            for index in range(combo.count()):
                if combo.itemData(index) is None:
                    none_index = index
                    break
            combo.setCurrentIndex(none_index)
        else:
            for index in range(combo.count()):
                if combo.itemData(index) == value:
                    combo.setCurrentIndex(index)
                    break
        if block_signals:
            combo.blockSignals(False)

    def _restore_combo_selection(self, combo: QComboBox, value: object) -> None:
        if value is None:
            return
        for index in range(combo.count()):
            if combo.itemData(index) == value:
                combo.setCurrentIndex(index)
                return
