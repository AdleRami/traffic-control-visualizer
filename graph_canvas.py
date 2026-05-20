from __future__ import annotations

from typing import Optional, TYPE_CHECKING

from constants import (
    AGV_RADIUS,
    BUFFER_NODE,
    CANVAS_MAX_ZOOM,
    CANVAS_MIN_ZOOM,
    CANVAS_ZOOM_STEP,
    NODE_RADIUS,
)
from graph_model import GraphModel
from qt_compat import (
    ANTIALIASING,
    DASH_LINE,
    DOT_LINE,
    LEFT_BUTTON,
    QBrush,
    QColor,
    QFont,
    QFrame,
    QGraphicsLineItem,
    QGraphicsScene,
    QGraphicsView,
    QPen,
    QRectF,
    QSizePolicy,
    SOLID_LINE,
)
from traffic_controller import TrafficController

if TYPE_CHECKING:
    from main_window import MainWindow


class GraphCanvas(QGraphicsView):
    """노드/edge 편집과 시뮬레이션 상태 렌더링을 담당하는 Canvas입니다."""

    AGV_COLORS = [
        QColor("#E4572E"),
        QColor("#17BEBB"),
        QColor("#FFC914"),
        QColor("#7D5FFF"),
        QColor("#2EAD53"),
        QColor("#D7263D"),
        QColor("#1B998B"),
        QColor("#F46036"),
    ]

    def __init__(self, graph: GraphModel, controller: TrafficController, main_window: "MainWindow") -> None:
        super().__init__()
        self.graph = graph
        self.controller = controller
        self.main_window = main_window
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self.setRenderHint(ANTIALIASING)
        self.setMouseTracking(True)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setFrameShape(QFrame.Shape.NoFrame)
        self.setBackgroundBrush(QBrush(QColor("#F6F7F9")))
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.ViewportAnchor.AnchorViewCenter)

        self.drag_start_node: Optional[int] = None
        self.drag_line: Optional[QGraphicsLineItem] = None
        self.selected_node_id: Optional[int] = None
        self.zoom_factor = 1.0
        self.refresh()

    def set_selected_node(self, node_id: Optional[int]) -> None:
        self.selected_node_id = node_id if node_id in self.graph.nodes else None
        self.refresh()

    def refresh(self) -> None:
        self.scene.clear()
        self.scene.setSceneRect(self.graph.bounding_rect())
        self._draw_conflict_zones()
        self._draw_edges()
        self._draw_planned_edges()
        self._draw_reserved_edges()
        self._draw_nodes()
        self._draw_agvs()
        self._draw_deadlock_banner()

    def mouseDoubleClickEvent(self, event) -> None:  # type: ignore[override]
        if event.button() != LEFT_BUTTON:
            super().mouseDoubleClickEvent(event)
            return

        pos = self.mapToScene(self._event_pos(event))
        if self.graph.find_node_at(pos, NODE_RADIUS * 1.8) is not None:
            return

        node_id = self.graph.add_node(pos.x(), pos.y())
        self.selected_node_id = node_id
        self.main_window._invalidate_simulation_after_graph_edit()
        self.main_window.log(f"Node {node_id} added at ({pos.x():.0f}, {pos.y():.0f}).")
        self.main_window.graph_changed()
        event.accept()

    def mousePressEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == LEFT_BUTTON:
            pos = self.mapToScene(self._event_pos(event))
            node_id = self.graph.find_node_at(pos)
            if node_id is not None:
                self.main_window.set_selected_node(node_id, refresh=False)
                self.drag_start_node = node_id
                start_pos = self.graph.nodes[node_id]
                self.drag_line = self.scene.addLine(
                    start_pos.x(),
                    start_pos.y(),
                    pos.x(),
                    pos.y(),
                    QPen(QColor("#3B82F6"), 2, DASH_LINE),
                )
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:  # type: ignore[override]
        if self.drag_start_node is not None and self.drag_line is not None:
            pos = self.mapToScene(self._event_pos(event))
            start_pos = self.graph.nodes[self.drag_start_node]
            self.drag_line.setLine(start_pos.x(), start_pos.y(), pos.x(), pos.y())
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == LEFT_BUTTON and self.drag_start_node is not None:
            pos = self.mapToScene(self._event_pos(event))
            target_node = self.graph.find_node_at(pos)
            start_node = self.drag_start_node
            if self.drag_line is not None:
                self.scene.removeItem(self.drag_line)
                self.drag_line = None
            self.drag_start_node = None

            if target_node is not None and target_node != start_node:
                if self.graph.add_edge(start_node, target_node):
                    self.main_window._invalidate_simulation_after_graph_edit()
                    self.main_window.log(f"Edge {start_node} -- {target_node} added.")
                    self.main_window.graph_changed()
                else:
                    self.refresh()
            else:
                self.refresh()
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def wheelEvent(self, event) -> None:  # type: ignore[override]
        """마우스 휠로 캔버스를 확대/축소합니다."""
        delta = event.angleDelta().y()
        if delta == 0:
            super().wheelEvent(event)
            return

        requested_factor = CANVAS_ZOOM_STEP if delta > 0 else 1.0 / CANVAS_ZOOM_STEP
        next_zoom = max(
            CANVAS_MIN_ZOOM,
            min(CANVAS_MAX_ZOOM, self.zoom_factor * requested_factor),
        )
        actual_factor = next_zoom / self.zoom_factor
        if abs(actual_factor - 1.0) < 0.0001:
            event.accept()
            return

        self.zoom_factor = next_zoom
        self.scale(actual_factor, actual_factor)
        event.accept()

    def _event_pos(self, event) -> object:
        if hasattr(event, "position"):
            return event.position().toPoint()
        return event.pos()

    def _draw_conflict_zones(self) -> None:
        for zone in self.controller.conflict_zones.values():
            points = [self.graph.nodes[node] for node in zone.nodes if node in self.graph.nodes]
            if not points:
                continue
            min_x = min(point.x() for point in points) - 34
            max_x = max(point.x() for point in points) + 34
            min_y = min(point.y() for point in points) - 34
            max_y = max(point.y() for point in points) + 34
            rect = QRectF(min_x, min_y, max_x - min_x, max_y - min_y)
            pen = QPen(QColor("#8B5CF6"), 2, DASH_LINE)
            brush = QBrush(QColor(139, 92, 246, 35))
            self.scene.addEllipse(rect, pen, brush)

            reservations = self.controller.zone_reservations.get(zone.zone_id, [])
            if reservations:
                reservation_text = ",".join(
                    f"{item.agv_id}:{item.from_node}->{item.to_node}"
                    for item in reservations[:2]
                )
                if len(reservations) > 2:
                    reservation_text += "..."
            else:
                reservation_text = "-"
            label = self.scene.addText(f"{zone.zone_id} moves:{reservation_text}", QFont("Arial", 9))
            label.setDefaultTextColor(QColor("#5B21B6"))
            label.setPos(rect.left() + 6, rect.top() + 4)

    def _draw_edges(self) -> None:
        for a, b in sorted(self.graph.edges):
            p1 = self.graph.nodes[a]
            p2 = self.graph.nodes[b]
            self.scene.addLine(
                p1.x(),
                p1.y(),
                p2.x(),
                p2.y(),
                QPen(QColor("#6B7280"), 3, SOLID_LINE),
            )

    def _draw_reserved_edges(self) -> None:
        for edge, agv_id in self.controller.edge_reservations.items():
            if edge[0] not in self.graph.nodes or edge[1] not in self.graph.nodes:
                continue
            p1 = self.graph.nodes[edge[0]]
            p2 = self.graph.nodes[edge[1]]
            pen = QPen(QColor("#F97316"), 6, DASH_LINE)
            self.scene.addLine(p1.x(), p1.y(), p2.x(), p2.y(), pen)
            mid_x = (p1.x() + p2.x()) / 2
            mid_y = (p1.y() + p2.y()) / 2
            text = self.scene.addText(f"R:{agv_id}", QFont("Arial", 8))
            text.setDefaultTextColor(QColor("#C2410C"))
            text.setPos(mid_x + 4, mid_y + 4)

    def _draw_planned_edges(self) -> None:
        for index, agv in enumerate(self.controller.agvs):
            if agv.status in ("DONE", "DEADLOCK"):
                continue
            nxt = self.controller.peek_next_node(agv)
            if nxt is None or nxt == agv.current_node:
                continue
            if agv.current_node not in self.graph.nodes or nxt not in self.graph.nodes:
                continue
            p1 = self.graph.nodes[agv.current_node]
            p2 = self.graph.nodes[nxt]
            color = self.AGV_COLORS[index % len(self.AGV_COLORS)]
            pen = QPen(color, 2, DOT_LINE)
            self.scene.addLine(p1.x(), p1.y(), p2.x(), p2.y(), pen)

    def _draw_nodes(self) -> None:
        for node_id, pos in sorted(self.graph.nodes.items()):
            reserved_by = self.controller.node_reservations.get(node_id)
            if reserved_by is not None:
                halo_rect = QRectF(
                    pos.x() - NODE_RADIUS - 7,
                    pos.y() - NODE_RADIUS - 7,
                    (NODE_RADIUS + 7) * 2,
                    (NODE_RADIUS + 7) * 2,
                )
                self.scene.addEllipse(
                    halo_rect,
                    QPen(QColor("#F59E0B"), 2, DASH_LINE),
                    QBrush(QColor(245, 158, 11, 35)),
                )

            rect = QRectF(
                pos.x() - NODE_RADIUS,
                pos.y() - NODE_RADIUS,
                NODE_RADIUS * 2,
                NODE_RADIUS * 2,
            )
            fill = QColor("#FFFFFF")
            if self.controller.get_conflict_zone(node_id):
                fill = QColor("#F5F3FF")
            outline = QColor("#111827")
            if self.graph.get_node_type(node_id) == BUFFER_NODE:
                fill = QColor("#ECFDF5")
                outline = QColor("#059669")
            self.scene.addEllipse(rect, QPen(outline, 2), QBrush(fill))

            if self.graph.get_node_type(node_id) == BUFFER_NODE:
                buffer_label = self.scene.addText("B", QFont("Arial", 8, QFont.Weight.Bold))
                buffer_label.setDefaultTextColor(QColor("#047857"))
                buffer_label.setPos(pos.x() + NODE_RADIUS - 2, pos.y() - NODE_RADIUS - 12)

            if node_id == self.selected_node_id:
                selected_rect = QRectF(
                    pos.x() - NODE_RADIUS - 5,
                    pos.y() - NODE_RADIUS - 5,
                    (NODE_RADIUS + 5) * 2,
                    (NODE_RADIUS + 5) * 2,
                )
                self.scene.addEllipse(
                    selected_rect,
                    QPen(QColor("#2563EB"), 3, DASH_LINE),
                    QBrush(QColor(37, 99, 235, 25)),
                )

            text = self.scene.addText(str(node_id), QFont("Arial", 9))
            text.setDefaultTextColor(QColor("#111827"))
            bounds = text.boundingRect()
            text.setPos(pos.x() - bounds.width() / 2, pos.y() - bounds.height() / 2)

    def _draw_agvs(self) -> None:
        for index, agv in enumerate(self.controller.agvs):
            if agv.current_node not in self.graph.nodes:
                continue
            pos = self.graph.nodes[agv.current_node]
            color = self.AGV_COLORS[index % len(self.AGV_COLORS)]
            if agv.status == "DONE":
                color = QColor("#16A34A")
            elif agv.status == "DEADLOCK":
                color = QColor("#DC2626")
            elif agv.status == "WORKING":
                color = QColor("#F59E0B")

            rect = QRectF(
                pos.x() - AGV_RADIUS,
                pos.y() - NODE_RADIUS - AGV_RADIUS - 6,
                AGV_RADIUS * 2,
                AGV_RADIUS * 2,
            )
            self.scene.addEllipse(rect, QPen(QColor("#111827"), 1), QBrush(color))
            label = self.scene.addText(agv.agv_id, QFont("Arial", 8, QFont.Weight.Bold))
            label.setDefaultTextColor(QColor("#111827"))
            bounds = label.boundingRect()
            label.setPos(
                rect.center().x() - bounds.width() / 2,
                rect.center().y() - bounds.height() / 2,
            )

    def _draw_deadlock_banner(self) -> None:
        if not self.controller.deadlock_detected:
            return
        rect = self.scene.sceneRect()
        banner = QRectF(rect.left() + 20, rect.top() + 20, min(420, rect.width() - 40), 44)
        self.scene.addRect(banner, QPen(QColor("#991B1B"), 2), QBrush(QColor("#FEE2E2")))
        text = self.scene.addText("DEADLOCK DETECTED", QFont("Arial", 17, QFont.Weight.Bold))
        text.setDefaultTextColor(QColor("#991B1B"))
        bounds = text.boundingRect()
        text.setPos(
            banner.left() + (banner.width() - bounds.width()) / 2,
            banner.top() + (banner.height() - bounds.height()) / 2,
        )
