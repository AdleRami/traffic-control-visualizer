from __future__ import annotations

import heapq
from dataclasses import dataclass
from typing import Any, Dict, FrozenSet, List, Optional, Set, Tuple

from models import AGV, Movement


@dataclass(frozen=True)
class PBSConflict:
    kind: str
    tick: int
    agv_id_1: str
    agv_id_2: str
    detail: str


@dataclass
class PBSSearchNode:
    priorities: FrozenSet[Tuple[str, str]]
    plans: Dict[str, List[int]]
    failures: Dict[str, str]
    conflicts: List[PBSConflict]
    order: List[str]
    path_score: float
    search_score: float
    earliest_conflict_tick: int


@dataclass
class PBSResult:
    plans: Dict[str, List[int]]
    failures: Dict[str, str]
    conflicts: List[str]
    order: List[str]
    priorities: FrozenSet[Tuple[str, str]]
    expanded_nodes: int
    search_score: float


class PBSWindowSolver:
    """Priority-Based Search window solver used by RHCR.

    RHCR itself is a framework: every h ticks it calls a Windowed MAPF solver for
    the next w ticks. This class is the in-process Python PBS solver for
    that window. It keeps the controller responsible for graph geometry, costs,
    and node/edge geometry helpers. The solver follows the standard MAPF
    collision model used by RHCR: vertex conflicts and edge conflicts only.
    """

    def __init__(self, controller: Any) -> None:
        self.controller = controller

    def solve(
        self,
        planning_agvs: List[AGV],
        fixed_plans: Dict[str, List[int]],
        window: int,
        logs: Optional[List[str]] = None,
    ) -> PBSResult:
        fixed_plans = {
            agv_id: self.controller._pad_window_path(path, window)
            for agv_id, path in fixed_plans.items()
        }
        planning_by_id = {agv.agv_id: agv for agv in planning_agvs}
        base_rank = self._base_rank(planning_agvs)
        max_nodes = max(1, getattr(self.controller, "RHCR_PBS_MAX_NODES", 200))

        root = self._make_search_node(
            planning_agvs,
            fixed_plans,
            frozenset(),
            window,
            base_rank,
        )
        if root is None:
            fallback_plans = dict(fixed_plans)
            failures: Dict[str, str] = {"PBS": "root low-level search failed"}
            for agv in planning_agvs:
                fallback_plans[agv.agv_id] = self.controller._rhcr_fallback_path(agv, window)
            conflicts = self._window_conflicts(fallback_plans, window)
            return PBSResult(
                plans=fallback_plans,
                failures=failures,
                conflicts=[conflict.detail for conflict in conflicts],
                order=[agv.agv_id for agv in planning_agvs],
                priorities=frozenset(),
                expanded_nodes=0,
                search_score=float("inf"),
            )

        # Original PBS uses a DFS container for high-level nodes. Children are
        # pushed in an order that makes the lower-cost child pop first.
        dfs: List[PBSSearchNode] = [root]
        visited: Set[FrozenSet[Tuple[str, str]]] = set()
        best = root
        expanded = 0

        while dfs and expanded < max_nodes:
            node = dfs.pop()
            if node.priorities in visited:
                continue
            visited.add(node.priorities)
            expanded += 1

            if self._is_better_best_node(node, best):
                best = node

            conflict = self._choose_conflict(node.conflicts)
            if logs is not None:
                logs.append(
                    f"[RHCR/PBS] node {expanded}: priorities={self.format_priorities(node.priorities)}, "
                    f"order={self._format_order(node.order)}, cost={node.path_score:g}, "
                    f"conflicts={len(node.conflicts)}, fallback={len(node.failures)}."
                )
                if conflict is not None:
                    logs.append(f"[RHCR/PBS] chosen conflict: {conflict.detail}.")

            if conflict is None:
                return self._result_from_node(node, expanded)

            branch_pairs = self._branch_pairs(conflict, planning_by_id)
            children: List[PBSSearchNode] = []
            for high_id, low_id in branch_pairs:
                if self._priority_reversed(node.priorities, high_id, low_id):
                    continue
                new_priorities = frozenset({*node.priorities, (high_id, low_id)})
                if new_priorities in visited:
                    continue
                child = self._make_search_node(
                    planning_agvs,
                    fixed_plans,
                    new_priorities,
                    window,
                    base_rank,
                )
                if child is not None:
                    children.append(child)

            if len(children) == 2:
                first, second = children
                if self._child_is_better(second, first):
                    first, second = second, first
                dfs.append(second)
                dfs.append(first)
            elif len(children) == 1:
                dfs.append(children[0])

        if logs is not None and best.conflicts:
            logs.append(
                f"[RHCR/PBS] search stopped after {expanded} high-level node(s); "
                "returning the best available window plan."
            )
        return self._result_from_node(best, expanded)

    def _make_search_node(
        self,
        planning_agvs: List[AGV],
        fixed_plans: Dict[str, List[int]],
        priorities: FrozenSet[Tuple[str, str]],
        window: int,
        base_rank: Dict[str, int],
    ) -> Optional[PBSSearchNode]:
        build_result = self._build_plans(planning_agvs, fixed_plans, priorities, window, base_rank)
        if build_result is None:
            return None
        plans, failures, path_score, order = build_result
        conflicts = self._window_conflicts(plans, window)
        earliest_conflict_tick = min((conflict.tick for conflict in conflicts), default=10**9)
        search_score = path_score
        return PBSSearchNode(
            priorities=priorities,
            plans=plans,
            failures=failures,
            conflicts=conflicts,
            order=order,
            path_score=path_score,
            search_score=search_score,
            earliest_conflict_tick=earliest_conflict_tick,
        )

    def _build_plans(
        self,
        planning_agvs: List[AGV],
        fixed_plans: Dict[str, List[int]],
        priorities: FrozenSet[Tuple[str, str]],
        window: int,
        base_rank: Dict[str, int],
    ) -> Optional[Tuple[Dict[str, List[int]], Dict[str, str], float, List[str]]]:
        order = self._topological_order(planning_agvs, priorities, base_rank)
        if order is None:
            return None

        planning_by_id = {agv.agv_id: agv for agv in planning_agvs}
        higher_sets = self._higher_priority_sets(set(planning_by_id), priorities)
        plans: Dict[str, List[int]] = dict(fixed_plans)
        failures: Dict[str, str] = {}
        total_score = 0.0

        for agv_id in order:
            agv = planning_by_id[agv_id]
            node_reservations: Dict[Tuple[int, int], str] = {}
            edge_reservations: Dict[Tuple[int, int, int], str] = {}
            blocking_paths: Dict[str, List[int]] = {}

            for fixed_id, fixed_path in fixed_plans.items():
                path = self.controller._pad_window_path(fixed_path, window)
                blocking_paths[fixed_id] = path
                self.controller._rhcr_insert_path_reservations(
                    fixed_id,
                    path,
                    node_reservations,
                    edge_reservations,
                    window,
                )

            missing_higher: List[str] = []
            for high_id in sorted(higher_sets.get(agv_id, set()), key=lambda item: (base_rank.get(item, 9999), item)):
                high_path = plans.get(high_id)
                if high_path is None:
                    missing_higher.append(high_id)
                    continue
                path = self.controller._pad_window_path(high_path, window)
                blocking_paths[high_id] = path
                self.controller._rhcr_insert_path_reservations(
                    high_id,
                    path,
                    node_reservations,
                    edge_reservations,
                    window,
                )

            if missing_higher:
                failures[agv_id] = f"higher-priority path missing: {', '.join(missing_higher)}"
                path = self.controller._rhcr_fallback_path(agv, window)
                plans[agv_id] = path
                total_score += self.controller._rhcr_score_window_path(agv, path)
                continue

            path, cost, _reached = self._state_time_a_star(
                agv,
                node_reservations,
                edge_reservations,
                blocking_paths,
                window,
            )
            if not path:
                failures[agv_id] = "PBS low-level state-time A* found no feasible window path"
                return None

            path = self.controller._pad_window_path(path, window)
            plans[agv_id] = path
            total_score += cost

        return plans, failures, total_score, order

    def _state_time_a_star(
        self,
        agv: AGV,
        node_reservations: Dict[Tuple[int, int], str],
        edge_reservations: Dict[Tuple[int, int, int], str],
        blocking_paths: Dict[str, List[int]],
        window: int,
    ) -> Tuple[List[int], float, bool]:
        start = agv.current_node
        goal = agv.goal_node
        if start not in self.controller.graph.nodes or goal not in self.controller.graph.nodes:
            return [], float("inf"), False
        if start == goal:
            hold_path = [start] * (window + 1)
            return hold_path, 0.0, True

        distance_cache: Dict[int, int] = {}

        def distance(node_id: int) -> int:
            if node_id not in distance_cache:
                distance_cache[node_id] = self.controller._distance_between(node_id, goal)
            return distance_cache[node_id]

        frontier: List[Tuple[float, float, int, int, int, int, List[int]]] = []
        counter = 0
        start_distance = distance(start)
        heapq.heappush(frontier, (float(start_distance), 0.0, start_distance, 0, counter, start, [start]))
        best_cost: Dict[Tuple[int, int], float] = {(start, 0): 0.0}
        best_partial: Tuple[int, int, float, List[int]] = (start_distance, 0, 0.0, [start])
        expansions = 0
        max_expansions = max(200, len(self.controller.graph.nodes) * (window + 1) * 8)

        while frontier and expansions < max_expansions:
            _f_score, cost_so_far, _dist, wait_count, _counter, node_id, path = heapq.heappop(frontier)
            expansions += 1
            time_offset = len(path) - 1
            partial_key = (distance(node_id), wait_count, cost_so_far)
            if partial_key < best_partial[:3]:
                best_partial = (partial_key[0], partial_key[1], partial_key[2], list(path))

            if node_id == goal and self._can_hold_goal(
                agv.agv_id,
                goal,
                time_offset,
                window,
                node_reservations,
                edge_reservations,
                blocking_paths,
            ):
                return self.controller._pad_window_path(path, window), cost_so_far, True

            if time_offset >= window:
                continue

            neighbors = sorted(self.controller.graph.neighbors(node_id), key=lambda item: (distance(item), item))
            actions = neighbors + [node_id]
            next_offset = time_offset + 1

            for next_node in actions:
                if next_node != node_id and not self.controller.graph.has_edge(node_id, next_node):
                    continue
                if self.controller._rhcr_transition_blocked(
                    agv.agv_id,
                    node_id,
                    next_node,
                    next_offset,
                    node_reservations,
                    edge_reservations,
                ):
                    continue
                # Original RHCR/PBS uses standard MAPF constraints here:
                # vertex reservations and edge/swap reservations only.

                next_wait_count = wait_count + (1 if next_node == node_id else 0)
                step_cost = self.controller._rhcr_step_cost(agv, node_id, next_node, next_offset)
                new_cost = cost_so_far + step_cost
                state = (next_node, next_offset)
                if new_cost >= best_cost.get(state, float("inf")):
                    continue
                best_cost[state] = new_cost
                next_path = path + [next_node]

                if next_node == goal and self._can_hold_goal(
                    agv.agv_id,
                    goal,
                    next_offset,
                    window,
                    node_reservations,
                    edge_reservations,
                    blocking_paths,
                ):
                    return self.controller._pad_window_path(next_path, window), new_cost, True

                counter += 1
                heuristic = float(distance(next_node))
                heapq.heappush(
                    frontier,
                    (
                        new_cost + heuristic,
                        new_cost,
                        distance(next_node),
                        next_wait_count,
                        counter,
                        next_node,
                        next_path,
                    ),
                )

        partial_path = self.controller._pad_window_path(best_partial[3], window)
        return partial_path, self.controller._rhcr_score_window_path(agv, partial_path), False

    def _can_hold_goal(
        self,
        agv_id: str,
        goal: int,
        reached_offset: int,
        window: int,
        node_reservations: Dict[Tuple[int, int], str],
        edge_reservations: Dict[Tuple[int, int, int], str],
        blocking_paths: Dict[str, List[int]],
    ) -> bool:
        for offset in range(reached_offset + 1, window + 1):
            if self.controller._rhcr_transition_blocked(
                agv_id,
                goal,
                goal,
                offset,
                node_reservations,
                edge_reservations,
            ):
                return False
        return True

    def _window_conflicts(self, plans: Dict[str, List[int]], window: int) -> List[PBSConflict]:
        conflicts: List[PBSConflict] = []
        if not plans:
            return conflicts

        agv_ids = sorted(plans)
        for tick in range(window + 1):
            node_seen: Dict[int, str] = {}
            for agv_id in agv_ids:
                node = self.controller._node_at_tick(plans[agv_id], tick)
                owner = node_seen.get(node)
                if owner is not None:
                    conflicts.append(
                        PBSConflict(
                            kind="node",
                            tick=tick,
                            agv_id_1=owner,
                            agv_id_2=agv_id,
                            detail=f"node conflict tick={tick}, node={node}, AGV {owner} vs AGV {agv_id}",
                        )
                    )
                else:
                    node_seen[node] = agv_id

        for tick in range(1, window + 1):
            for index, agv_id_1 in enumerate(agv_ids):
                movement_1 = self._movement_from_path(agv_id_1, plans[agv_id_1], tick)
                for agv_id_2 in agv_ids[index + 1:]:
                    movement_2 = self._movement_from_path(agv_id_2, plans[agv_id_2], tick)
                    if self.controller.is_edge_swap_conflict(movement_1, movement_2):
                        conflicts.append(
                            PBSConflict(
                                kind="edge_swap",
                                tick=tick,
                                agv_id_1=agv_id_1,
                                agv_id_2=agv_id_2,
                                detail=(
                                    f"edge swap conflict tick={tick}, "
                                    f"{self.controller._movement_label(movement_1)} vs "
                                    f"{self.controller._movement_label(movement_2)}"
                                ),
                            )
                        )
                        continue
                    if self.controller.is_same_edge_conflict(movement_1, movement_2):
                        conflicts.append(
                            PBSConflict(
                                kind="same_edge",
                                tick=tick,
                                agv_id_1=agv_id_1,
                                agv_id_2=agv_id_2,
                                detail=(
                                    f"same edge conflict tick={tick}, edge={movement_1.edge}, "
                                    f"{self.controller._movement_label(movement_1)} vs "
                                    f"{self.controller._movement_label(movement_2)}"
                                ),
                            )
                        )
                        continue
        return conflicts

    def _movement_from_path(self, agv_id: str, path: List[int], tick: int) -> Movement:
        from_node = self.controller._node_at_tick(path, tick - 1)
        to_node = self.controller._node_at_tick(path, tick)
        return self._movement(agv_id, from_node, to_node, tick)

    def _movement(self, agv_id: str, from_node: int, to_node: int, tick: int) -> Movement:
        edge = None if from_node == to_node else self.controller.graph.normalize_edge(from_node, to_node)
        return Movement(
            agv_id=agv_id,
            from_node=from_node,
            to_node=to_node,
            edge=edge,
            time=tick,
            geometry=self.controller._movement_geometry(from_node, to_node),
        )

    def _branch_pairs(
        self,
        conflict: PBSConflict,
        planning_by_id: Dict[str, AGV],
    ) -> List[Tuple[str, str]]:
        a = conflict.agv_id_1
        b = conflict.agv_id_2
        if a not in planning_by_id and b not in planning_by_id:
            return []
        if a not in planning_by_id:
            return [(a, b)] if b in planning_by_id else []
        if b not in planning_by_id:
            return [(b, a)] if a in planning_by_id else []
        # Original PBS creates child priorities (a1 lower than a2) and
        # (a2 lower than a1). This solver stores them as high>low, so the
        # first branch is b>a and the second branch is a>b.
        return [(b, a), (a, b)]

    def _topological_order(
        self,
        agvs: List[AGV],
        priorities: FrozenSet[Tuple[str, str]],
        base_rank: Dict[str, int],
    ) -> Optional[List[str]]:
        ids = {agv.agv_id for agv in agvs}
        outgoing: Dict[str, Set[str]] = {agv_id: set() for agv_id in ids}
        incoming_count: Dict[str, int] = {agv_id: 0 for agv_id in ids}

        for high_id, low_id in priorities:
            if high_id not in ids or low_id not in ids:
                continue
            if low_id in outgoing[high_id]:
                continue
            outgoing[high_id].add(low_id)
            incoming_count[low_id] += 1

        ready = [agv_id for agv_id, count in incoming_count.items() if count == 0]
        ready.sort(key=lambda item: (base_rank.get(item, 9999), item))
        order: List[str] = []

        while ready:
            agv_id = ready.pop(0)
            order.append(agv_id)
            for low_id in sorted(outgoing[agv_id], key=lambda item: (base_rank.get(item, 9999), item)):
                incoming_count[low_id] -= 1
                if incoming_count[low_id] == 0:
                    ready.append(low_id)
                    ready.sort(key=lambda item: (base_rank.get(item, 9999), item))

        if len(order) != len(ids):
            return None
        return order

    def _higher_priority_sets(
        self,
        ids: Set[str],
        priorities: FrozenSet[Tuple[str, str]],
    ) -> Dict[str, Set[str]]:
        higher: Dict[str, Set[str]] = {agv_id: set() for agv_id in ids}
        for high_id, low_id in priorities:
            if high_id in ids and low_id in ids:
                higher[low_id].add(high_id)

        changed = True
        while changed:
            changed = False
            for agv_id in ids:
                expanded = set(higher[agv_id])
                for high_id in list(higher[agv_id]):
                    expanded.update(higher.get(high_id, set()))
                if expanded != higher[agv_id]:
                    higher[agv_id] = expanded
                    changed = True
        return higher

    def _base_rank(self, agvs: List[AGV]) -> Dict[str, int]:
        return {agv.agv_id: index for index, agv in enumerate(agvs)}

    def _choose_conflict(self, conflicts: List[PBSConflict]) -> Optional[PBSConflict]:
        if not conflicts:
            return None
        return min(
            conflicts,
            key=lambda conflict: (
                conflict.tick,
                min(conflict.agv_id_1, conflict.agv_id_2),
                max(conflict.agv_id_1, conflict.agv_id_2),
                conflict.kind,
            ),
        )

    def _priority_reversed(
        self,
        priorities: FrozenSet[Tuple[str, str]],
        high_id: str,
        low_id: str,
    ) -> bool:
        higher = self._higher_priority_sets({high_id, low_id}, priorities)
        return low_id in higher.get(high_id, set())

    def _child_is_better(self, node: PBSSearchNode, other: PBSSearchNode) -> bool:
        return (
            node.path_score,
            len(node.conflicts),
            len(node.failures),
            len(node.priorities),
        ) < (
            other.path_score,
            len(other.conflicts),
            len(other.failures),
            len(other.priorities),
        )

    def _is_better_best_node(self, node: PBSSearchNode, best: PBSSearchNode) -> bool:
        return (
            len(node.conflicts),
            node.earliest_conflict_tick,
            len(node.failures),
            node.path_score,
        ) < (
            len(best.conflicts),
            best.earliest_conflict_tick,
            len(best.failures),
            best.path_score,
        )

    def _result_from_node(self, node: PBSSearchNode, expanded: int) -> PBSResult:
        return PBSResult(
            plans=node.plans,
            failures=node.failures,
            conflicts=[conflict.detail for conflict in node.conflicts],
            order=node.order,
            priorities=node.priorities,
            expanded_nodes=expanded,
            search_score=node.search_score,
        )

    def format_priorities(self, priorities: FrozenSet[Tuple[str, str]]) -> str:
        if not priorities:
            return "none"
        return ", ".join(f"{high}>{low}" for high, low in sorted(priorities))

    def _format_order(self, order: List[str]) -> str:
        return " > ".join(order) if order else "-"
