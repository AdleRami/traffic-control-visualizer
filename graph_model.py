from __future__ import annotations

import math
from collections import deque
from typing import Dict, List, Optional, Set, Tuple

from constants import (
    BUFFER_NODE,
    DEFAULT_BUFFER_NODE_IDS,
    EDGE_SELECT_RADIUS,
    NORMAL_NODE,
    RESERVATION_NODE,
    WAIT_NODE,
)
from qt_compat import QPointF, QRectF


class GraphModel:
    """메모리 기반 무방향 그래프 모델입니다."""

    def __init__(self) -> None:
        self.nodes: Dict[int, QPointF] = {}
        self.node_types: Dict[int, str] = {}
        self.edges: Set[Tuple[int, int]] = set()
        self.next_node_id = 1

    def clear(self) -> None:
        self.nodes.clear()
        self.node_types.clear()
        self.edges.clear()
        self.next_node_id = 1

    def add_node(
        self,
        x: float,
        y: float,
        node_id: Optional[int] = None,
        node_type: Optional[str] = None,
    ) -> int:
        if node_id is None:
            node_id = self.next_node_id
            while node_id in self.nodes:
                node_id += 1
        if node_id in self.nodes:
            raise ValueError(f"Node {node_id} already exists.")

        self.nodes[node_id] = QPointF(float(x), float(y))
        if node_type is None:
            node_type = BUFFER_NODE if node_id in DEFAULT_BUFFER_NODE_IDS else NORMAL_NODE
        self.node_types[node_id] = node_type
        self.next_node_id = max(self.next_node_id, node_id + 1)
        return node_id

    def remove_node(self, node_id: int) -> int:
        """노드와 해당 노드에 연결된 모든 edge를 삭제하고, 삭제 edge 수를 반환합니다."""
        if node_id not in self.nodes:
            return 0
        incident_edges = {edge for edge in self.edges if node_id in edge}
        self.edges.difference_update(incident_edges)
        del self.nodes[node_id]
        self.node_types.pop(node_id, None)
        return len(incident_edges)

    def set_node_type(self, node_id: int, node_type: str) -> None:
        if node_id in self.nodes:
            self.node_types[node_id] = node_type

    def get_node_type(self, node_id: int) -> str:
        return self.node_types.get(node_id, NORMAL_NODE)

    def buffer_nodes(self) -> List[int]:
        return sorted(
            node_id
            for node_id in self.nodes
            if self.get_node_type(node_id) == BUFFER_NODE
        )

    def safe_wait_nodes(self) -> List[int]:
        return sorted(
            node_id
            for node_id in self.nodes
            if self.get_node_type(node_id) in (WAIT_NODE, BUFFER_NODE, RESERVATION_NODE)
        )

    def add_edge(self, a: int, b: int) -> bool:
        if a == b or a not in self.nodes or b not in self.nodes:
            return False
        edge = self.normalize_edge(a, b)
        if edge in self.edges:
            return False
        self.edges.add(edge)
        return True

    def normalize_edge(self, a: int, b: int) -> Tuple[int, int]:
        return (a, b) if a < b else (b, a)

    def has_edge(self, a: int, b: int) -> bool:
        return self.normalize_edge(a, b) in self.edges

    def neighbors(self, node_id: int) -> List[int]:
        result: List[int] = []
        for a, b in self.edges:
            if a == node_id:
                result.append(b)
            elif b == node_id:
                result.append(a)
        return sorted(result)

    def degree(self, node_id: int) -> int:
        return len(self.neighbors(node_id))

    def shortest_path(self, start: int, goal: int) -> List[int]:
        """무가중치 그래프에서 BFS shortest path를 계산합니다."""
        if start not in self.nodes or goal not in self.nodes:
            return []
        if start == goal:
            return [start]

        queue: deque[int] = deque([start])
        parent: Dict[int, Optional[int]] = {start: None}

        while queue:
            current = queue.popleft()
            for nxt in self.neighbors(current):
                if nxt in parent:
                    continue
                parent[nxt] = current
                if nxt == goal:
                    return self._reconstruct_path(parent, goal)
                queue.append(nxt)
        return []

    def _reconstruct_path(self, parent: Dict[int, Optional[int]], goal: int) -> List[int]:
        path: List[int] = []
        node: Optional[int] = goal
        while node is not None:
            path.append(node)
            node = parent[node]
        path.reverse()
        return path

    def find_node_at(self, pos: QPointF, radius: float = EDGE_SELECT_RADIUS) -> Optional[int]:
        """마우스 위치 주변의 가장 가까운 노드를 찾습니다."""
        best_node: Optional[int] = None
        best_dist = float("inf")
        for node_id, node_pos in self.nodes.items():
            dist = math.hypot(pos.x() - node_pos.x(), pos.y() - node_pos.y())
            if dist <= radius and dist < best_dist:
                best_node = node_id
                best_dist = dist
        return best_node

    def bounding_rect(self, margin: float = 120.0) -> QRectF:
        if not self.nodes:
            return QRectF(-100, -100, 800, 600)
        xs = [p.x() for p in self.nodes.values()]
        ys = [p.y() for p in self.nodes.values()]
        min_x, max_x = min(xs) - margin, max(xs) + margin
        min_y, max_y = min(ys) - margin, max(ys) + margin
        return QRectF(min_x, min_y, max_x - min_x, max_y - min_y)


