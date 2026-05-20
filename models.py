from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Set, Tuple


@dataclass
class AGV:
    agv_id: str
    start_node: int
    current_node: int
    goal_node: int
    planned_path: List[int] = field(default_factory=list)
    status: str = "WAITING"
    wait_count: int = 0
    replan_count: int = 0
    plan_start_tick: int = 0
    last_move: Optional[Tuple[int, int]] = None
    previous_node: Optional[int] = None
    committed_path: List[int] = field(default_factory=list)
    commit_until_index: int = 0
    commit_index: int = 0
    task_priority: int = 0
    original_goal_node: Optional[int] = None
    after_work_goal_node: Optional[int] = None
    escape_mode: bool = False
    escape_hold: bool = False
    escape_returning: bool = False
    escape_target: Optional[int] = None
    escape_attempts: int = 0
    failed_escape_attempts: int = 0
    last_risk_replan_tick: int = -9999
    speed_ticks: int = 1
    last_move_tick: int = -9999
    work_ticks: int = 0
    work_remaining_ticks: int = 0
    work_completed_for_current_goal: bool = False
    round_trips_total: int = 0
    completed_legs: int = 0
    trip_turnaround: bool = False

    def __post_init__(self) -> None:
        if self.original_goal_node is None:
            self.original_goal_node = self.goal_node

    def destination_sequence(self) -> List[int]:
        primary_goal = self.original_goal_node or self.goal_node
        sequence: List[int] = []
        if self.round_trips_total <= 0:
            sequence.append(primary_goal)
            if self.after_work_goal_node is not None and self.after_work_goal_node != primary_goal:
                sequence.append(self.after_work_goal_node)
            return sequence

        for _ in range(self.round_trips_total):
            sequence.append(primary_goal)
            if self.after_work_goal_node is not None and self.after_work_goal_node != primary_goal:
                sequence.append(self.after_work_goal_node)
            sequence.append(self.start_node)
        return sequence

    def total_required_legs(self) -> int:
        return max(1, len(self.destination_sequence()))

    def reset(self) -> None:
        self.current_node = self.start_node
        if self.original_goal_node is not None:
            self.goal_node = self.original_goal_node
        self.planned_path = []
        self.status = "WAITING"
        self.wait_count = 0
        self.replan_count = 0
        self.plan_start_tick = 0
        self.last_move = None
        self.previous_node = None
        self.committed_path = []
        self.commit_until_index = 0
        self.commit_index = 0
        self.escape_mode = False
        self.escape_hold = False
        self.escape_returning = False
        self.escape_target = None
        self.escape_attempts = 0
        self.failed_escape_attempts = 0
        self.last_risk_replan_tick = -9999
        self.last_move_tick = -9999
        self.work_ticks = max(0, self.work_ticks)
        self.work_remaining_ticks = 0
        self.work_completed_for_current_goal = False
        self.completed_legs = 0
        self.trip_turnaround = False


@dataclass
class ConflictZone:
    zone_id: str
    center_node: int
    nodes: Set[int]


@dataclass
class StepResult:
    logs: List[str]
    moved_count: int
    deadlock: bool
    all_done: bool


# ============================================================
# 이 부분부터 교통제어 알고리즘 부분입니다.
# RHCR + Windowed Reservation Table + Conflict Zone Lock + Replanning
# ============================================================


@dataclass(frozen=True)
class Movement:
    agv_id: str
    from_node: int
    to_node: int
    edge: Optional[Tuple[int, int]]
    time: int
    geometry: Tuple[Tuple[float, float], Tuple[float, float]]


@dataclass
class ZoneReservation:
    agv_id: str
    from_node: int
    to_node: int
    edge: Optional[Tuple[int, int]]
    time: int
    geometry: Tuple[Tuple[float, float], Tuple[float, float]]


@dataclass
class MoveCandidate:
    agv_id: str
    current_node: int
    target_node: int
    edge: Optional[Tuple[int, int]]
    zones: List[str]
    remaining_distance: int
    is_wait: bool
    movement: Movement
    reason: str = ""


@dataclass
class ConflictGroup:
    group_id: str
    agv_ids: Set[str]
    reasons: List[str]


@dataclass
class MoveCost:
    next_node: int
    path: List[int]
    total: float
    distance_component: float
    zone_component: float
    congestion_component: float
    reservation_component: float
    intersection_component: float
    backtrack_component: float
    progress_component: float
    horizon_component: float
    distance_to_goal: int
    current_distance_to_goal: int
    raw_zone_score: float
    raw_congestion_score: float
    raw_reservation_score: float
    raw_intersection_score: float
    raw_backtrack_score: float
    raw_progress_delta: int
    rejected_by_anti_oscillation: bool
    movement: Movement


@dataclass
class PathRiskReport:
    risk_nodes: Set[int] = field(default_factory=set)
    risk_edges: Set[Tuple[int, int]] = field(default_factory=set)
    risk_zones: Set[str] = field(default_factory=set)
    reasons: List[str] = field(default_factory=list)
    total_risk: float = 0.0


