from __future__ import annotations

from typing import Dict, List, Optional, Set, Tuple

from constants import BUFFER_NODE, INTERSECTION_NODE, RESERVATION_NODE, WAIT_NODE
from graph_model import GraphModel
from models import AGV, ConflictZone, MoveCandidate, Movement, StepResult, ZoneReservation
from pbs_window_solver import PBSWindowSolver


class TrafficController:
    """RHCR controller backed by a PBS window solver.

    The active traffic-control logic follows the RHCR shape used by the
    reference project: every h ticks, solve a Windowed MAPF problem for the next
    w ticks, then execute the first part of that plan. Conflict-zone coloring is
    kept only for the UI; it is not part of the solver constraints.
    """

    RHCR_SIMULATION_WINDOW = 5
    RHCR_PLANNING_WINDOW = 25
    RHCR_PBS_MAX_NODES = 200
    RHCR_WAIT_PENALTY = 0.1
    RHCR_CORRIDOR_MIN_SHARED_NODES = 2
    DEADLOCK_WAIT_LIMIT = 5

    def __init__(self, graph: GraphModel) -> None:
        self.graph = graph
        self.agvs: List[AGV] = []
        self.clock = 0
        self.node_reservations: Dict[int, str] = {}
        self.edge_reservations: Dict[Tuple[int, int], str] = {}
        self.conflict_zones: Dict[str, ConflictZone] = {}
        self.zone_reservations: Dict[str, List[ZoneReservation]] = {}
        self.global_no_move_ticks = 0
        self.escape_fail_ticks = 0
        self.deadlock_detected = False
        self.initial_plan_built = False
        self.simulation_finished = False
        self.rhcr_next_replan_tick = 0
        self.window_solver = PBSWindowSolver(self)
        self.corridor_locks: Dict[str, Dict[str, object]] = {}

    def clear(self) -> None:
        self.agvs.clear()
        self.clock = 0
        self.node_reservations.clear()
        self.edge_reservations.clear()
        self.conflict_zones.clear()
        self.zone_reservations.clear()
        self.global_no_move_ticks = 0
        self.escape_fail_ticks = 0
        self.deadlock_detected = False
        self.initial_plan_built = False
        self.simulation_finished = False
        self.rhcr_next_replan_tick = 0
        self.corridor_locks.clear()

    def add_agv(self, agv: AGV) -> None:
        self.agvs.append(agv)
        self.initial_plan_built = False
        self.rhcr_next_replan_tick = self.clock
        self.simulation_finished = False
        self.deadlock_detected = False

    def reset_simulation(self) -> List[str]:
        self.clock = 0
        self.node_reservations.clear()
        self.edge_reservations.clear()
        self.zone_reservations.clear()
        self.global_no_move_ticks = 0
        self.escape_fail_ticks = 0
        self.deadlock_detected = False
        self.initial_plan_built = False
        self.simulation_finished = False
        self.rhcr_next_replan_tick = 0
        self.corridor_locks.clear()
        for agv in self.agvs:
            agv.reset()
        self.recompute_conflict_zones()
        return ["Simulation reset. AGVs returned to their start nodes."]

    def recompute_conflict_zones(self) -> None:
        """Build visual-only conflict zones around degree >= 3 intersections."""
        zones: Dict[str, ConflictZone] = {}
        zone_index = 1
        for node_id in sorted(self.graph.nodes):
            if self.graph.degree(node_id) < 3:
                continue
            if self.graph.get_node_type(node_id) not in (WAIT_NODE, BUFFER_NODE, RESERVATION_NODE):
                self.graph.set_node_type(node_id, INTERSECTION_NODE)
            zone_id = f"Z{zone_index}"
            zones[zone_id] = ConflictZone(zone_id, node_id, {node_id, *self.graph.neighbors(node_id)})
            zone_index += 1
        self.conflict_zones = zones
        valid_zone_ids = set(zones)
        self.zone_reservations = {
            zone_id: reservations
            for zone_id, reservations in self.zone_reservations.items()
            if zone_id in valid_zone_ids
        }

    def build_rhcr_plan(self, reason: str = "rolling RHCR replan") -> List[str]:
        """Solve the current rolling window with PBS and store each AGV path."""
        logs: List[str] = []
        self.recompute_conflict_zones()
        window = max(self.RHCR_SIMULATION_WINDOW, self.RHCR_PLANNING_WINDOW)

        fixed_plans: Dict[str, List[int]] = {}
        planning_agvs: List[AGV] = []
        for agv in self.agvs:
            if agv.status in ("DONE", "DEADLOCK"):
                continue
            self._clear_runtime_mode_state(agv)
            if agv.work_remaining_ticks > 0:
                fixed_plans[agv.agv_id] = self._rhcr_fixed_window_path(agv, window)
            else:
                planning_agvs.append(agv)

        logs.append(
            f"[RHCR] {reason}: h={self.RHCR_SIMULATION_WINDOW}, "
            f"w={window}, active={len(planning_agvs)}, fixed={len(fixed_plans)}."
        )

        if not planning_agvs:
            self.initial_plan_built = True
            self.rhcr_next_replan_tick = self.clock + self.RHCR_SIMULATION_WINDOW
            self._refresh_corridor_locks(logs)
            logs.append("[RHCR] no movable AGV requires planning; next window scheduled.")
            return logs

        pbs_result = self.window_solver.solve(planning_agvs, fixed_plans, window, logs)
        plans = pbs_result.plans

        for agv in planning_agvs:
            path = self._pad_window_path(plans.get(agv.agv_id, [agv.current_node]), window)
            agv.planned_path = path
            agv.plan_start_tick = self.clock
            agv.last_risk_replan_tick = self.clock
            self._clear_commitment(agv)
            if agv.status not in ("WORKING", "DONE", "DEADLOCK"):
                agv.status = "WAITING"
            suffix = "reaches goal within window" if agv.goal_node in path else "window path"
            logs.append(
                f"[RHCR] AGV {agv.agv_id}: plan_start={self.clock}, "
                f"path={path} ({suffix})."
            )

        for agv_id, failure in pbs_result.failures.items():
            logs.append(f"[RHCR/PBS] AGV {agv_id}: {failure}.")

        if pbs_result.conflicts:
            logs.extend(f"[RHCR/PBS] remaining window conflict: {conflict}" for conflict in pbs_result.conflicts)
            logs.append(
                "[RHCR] unresolved PBS conflicts will be handled by stop-before-conflict "
                "execution and the next rolling replan."
            )
        else:
            logs.append(
                f"[RHCR/PBS] selected priority constraints: "
                f"{self.window_solver.format_priorities(pbs_result.priorities)}."
            )
            logs.append(
                f"[RHCR/PBS] selected topological order: "
                f"{' > '.join(pbs_result.order) if pbs_result.order else '-'}."
            )
            logs.append("[RHCR] window plans have no detected node/edge conflicts.")

        self.initial_plan_built = True
        self.rhcr_next_replan_tick = self.clock + self.RHCR_SIMULATION_WINDOW
        self._rebuild_corridor_locks(pbs_result.order, logs)
        logs.append(f"[RHCR] next rolling replan scheduled at tick {self.rhcr_next_replan_tick}.")
        return logs

    def _rhcr_transition_blocked(
        self,
        agv_id: str,
        from_node: int,
        to_node: int,
        next_offset: int,
        node_reservations: Dict[Tuple[int, int], str],
        edge_reservations: Dict[Tuple[int, int, int], str],
    ) -> bool:
        node_owner = node_reservations.get((to_node, next_offset))
        if node_owner is not None and node_owner != agv_id:
            return True
        if from_node == to_node:
            return False
        reverse_owner = edge_reservations.get((to_node, from_node, next_offset))
        if reverse_owner is not None and reverse_owner != agv_id:
            return True
        same_owner = edge_reservations.get((from_node, to_node, next_offset))
        if same_owner is not None and same_owner != agv_id:
            return True
        return False

    def _rhcr_insert_path_reservations(
        self,
        agv_id: str,
        path: List[int],
        node_reservations: Dict[Tuple[int, int], str],
        edge_reservations: Dict[Tuple[int, int, int], str],
        window: int,
    ) -> None:
        path = self._pad_window_path(path, window)
        for offset in range(window + 1):
            node_id = self._node_at_tick(path, offset)
            node_reservations.setdefault((node_id, offset), agv_id)
            if offset == 0:
                continue
            prev_node = self._node_at_tick(path, offset - 1)
            if prev_node != node_id:
                edge_reservations.setdefault((prev_node, node_id, offset), agv_id)

    def _rhcr_fixed_window_path(self, agv: AGV, window: int) -> List[int]:
        return [agv.current_node] * (window + 1)

    def _rhcr_fallback_path(self, agv: AGV, window: int) -> List[int]:
        path = self.graph.shortest_path(agv.current_node, agv.goal_node) or [agv.current_node]
        return self._pad_window_path(path, window)

    def _rhcr_score_window_path(self, agv: AGV, path: List[int]) -> float:
        if not path:
            return float("inf")
        score = 0.0
        for index in range(1, len(path)):
            score += self._rhcr_step_cost(agv, path[index - 1], path[index], index)
        if agv.goal_node not in path:
            score += self._distance_between(path[-1], agv.goal_node) * 100.0
        return score

    def _rhcr_step_cost(self, agv: AGV, from_node: int, to_node: int, tick_offset: int) -> float:
        cost = 1.0
        if from_node == to_node:
            cost += self.RHCR_WAIT_PENALTY
        cost += tick_offset * 0.001
        return cost

    def _pad_window_path(self, path: List[int], window: int) -> List[int]:
        if not path:
            return []
        padded = list(path[: window + 1])
        if len(padded) < window + 1:
            padded.extend([padded[-1]] * (window + 1 - len(padded)))
        return padded

    def detect_conflicts(self, plans: Optional[Dict[str, List[int]]] = None) -> List[str]:
        if plans is None:
            plans = {agv.agv_id: agv.planned_path for agv in self.agvs if agv.planned_path}
        conflicts: List[str] = []
        if not plans:
            return conflicts
        max_len = max(len(path) for path in plans.values())
        agv_ids = sorted(plans)

        for tick in range(max_len):
            node_seen: Dict[int, str] = {}
            for agv_id in agv_ids:
                node_id = self._node_at_tick(plans[agv_id], tick)
                owner = node_seen.get(node_id)
                if owner is not None:
                    conflicts.append(f"node conflict tick={tick}, node={node_id}, AGV {owner} vs AGV {agv_id}")
                else:
                    node_seen[node_id] = agv_id

        for tick in range(1, max_len):
            for index, agv_id_1 in enumerate(agv_ids):
                movement_1 = self._movement_from_path(agv_id_1, plans[agv_id_1], tick)
                for agv_id_2 in agv_ids[index + 1:]:
                    movement_2 = self._movement_from_path(agv_id_2, plans[agv_id_2], tick)
                    if self.is_edge_swap_conflict(movement_1, movement_2):
                        conflicts.append(
                            f"edge swap conflict tick={tick}: "
                            f"{self._movement_label(movement_1)} vs {self._movement_label(movement_2)}"
                        )
                    elif self.is_same_edge_conflict(movement_1, movement_2):
                        conflicts.append(
                            f"same edge conflict tick={tick}, edge={movement_1.edge}: "
                            f"{self._movement_label(movement_1)} vs {self._movement_label(movement_2)}"
                        )
        return conflicts

    def reserve_node(self, node_id: int, agv_id: str) -> bool:
        owner = self.node_reservations.get(node_id)
        if owner is not None and owner != agv_id:
            return False
        self.node_reservations[node_id] = agv_id
        return True

    def reserve_edge(self, a: int, b: int, agv_id: str) -> bool:
        edge = self.graph.normalize_edge(a, b)
        owner = self.edge_reservations.get(edge)
        if owner is not None and owner != agv_id:
            return False
        self.edge_reservations[edge] = agv_id
        return True

    def get_conflict_zone(self, node_id: int) -> Optional[str]:
        zones = self.get_conflict_zones_for_node(node_id)
        return zones[0] if zones else None

    def get_conflict_zones_for_node(self, node_id: int) -> List[str]:
        return sorted(zone_id for zone_id, zone in self.conflict_zones.items() if node_id in zone.nodes)

    def is_edge_swap_conflict(self, m1: Movement, m2: Movement) -> bool:
        return m1.from_node == m2.to_node and m1.to_node == m2.from_node and m1.from_node != m1.to_node

    def is_same_edge_conflict(self, m1: Movement, m2: Movement) -> bool:
        return m1.edge is not None and m1.edge == m2.edge

    def step(self) -> StepResult:
        logs: List[str] = []
        if self.deadlock_detected:
            return StepResult(["Simulation is stopped because deadlock was detected."], 0, True, False)
        if self.simulation_finished:
            return StepResult(["Simulation already finished."], 0, False, True)
        if not self.agvs:
            return StepResult(["No AGV. Add AGVs before simulation."], 0, False, False)

        if not self.initial_plan_built:
            logs.extend(self.build_rhcr_plan("initial RHCR planning"))
        elif self.clock >= self.rhcr_next_replan_tick:
            logs.extend(self.build_rhcr_plan("rolling horizon replan"))

        self.recompute_conflict_zones()
        self._reset_runtime_reservations(logs)
        self._refresh_corridor_locks(logs)

        next_tick = self.clock + 1
        active_agvs = [agv for agv in self.agvs if agv.status not in ("DONE", "DEADLOCK")]
        candidates = self._build_rhcr_runtime_candidates(active_agvs, next_tick, logs)
        conflicts = self._rhcr_runtime_conflicts(candidates)

        if conflicts:
            logs.extend(f"[RHCR] runtime plan conflict detected: {conflict}" for conflict in conflicts)
            logs.append("[RHCR] rebuilding the current rolling window before executing moves.")
            logs.extend(self.build_rhcr_plan("runtime conflict repair"))
            active_agvs = [agv for agv in self.agvs if agv.status not in ("DONE", "DEADLOCK")]
            candidates = self._build_rhcr_runtime_candidates(active_agvs, next_tick, logs)
            conflicts = self._rhcr_runtime_conflicts(candidates)

        moved_count = 0
        if conflicts:
            logs.extend(f"[RHCR] unresolved runtime conflict after repair: {conflict}" for conflict in conflicts)
            for agv_id, candidate in sorted(candidates.items()):
                agv = self._agv_by_id(agv_id)
                if agv is not None and agv.status not in ("DONE", "DEADLOCK"):
                    self._approve_wait(agv, next_tick, logs, "RHCR unresolved runtime conflict")
        else:
            moving_ids = {agv_id for agv_id, candidate in candidates.items() if not candidate.is_wait}
            for agv_id in moving_ids:
                agv = self._agv_by_id(agv_id)
                if agv is not None:
                    self._release_node_reservation(agv.current_node, agv.agv_id)
            for agv_id in sorted(moving_ids):
                agv = self._agv_by_id(agv_id)
                if agv is None:
                    continue
                self._execute_rhcr_move(agv, candidates[agv_id], next_tick, logs)
                moved_count += 1
            for agv_id, candidate in sorted(candidates.items()):
                if agv_id in moving_ids:
                    continue
                agv = self._agv_by_id(agv_id)
                if agv is not None and agv.status not in ("DONE", "DEADLOCK"):
                    self._approve_wait(agv, next_tick, logs, candidate.reason or "RHCR planned wait")

        self.clock = next_tick
        unfinished = [agv for agv in self.agvs if agv.status not in ("DONE", "DEADLOCK")]
        all_done = bool(self.agvs) and not unfinished
        deadlock = False

        if all_done:
            self.simulation_finished = True
            self.global_no_move_ticks = 0
            self.escape_fail_ticks = 0
            logs.append(f"[Tick {self.clock}] All AGVs are DONE. Simulation stopped.")
        elif moved_count == 0 and unfinished:
            if any(agv.work_remaining_ticks > 0 for agv in unfinished):
                self.global_no_move_ticks = 0
            else:
                self.global_no_move_ticks += 1
                limit = max(self.RHCR_PLANNING_WINDOW, self.DEADLOCK_WAIT_LIMIT)
                logs.append(
                    f"[RHCR] no unfinished AGV moved for {self.global_no_move_ticks} "
                    f"tick(s); deadlock guard={limit}."
                )
                if self.global_no_move_ticks >= limit:
                    self.deadlock_detected = True
                    deadlock = True
                    for agv in unfinished:
                        agv.status = "DEADLOCK"
                    logs.append(
                        "[RHCR] FINAL DEADLOCK DETECTED: PBS rolling-window planning could not "
                        "produce executable progress within the guard horizon."
                    )
        else:
            self.global_no_move_ticks = 0
            self.escape_fail_ticks = 0

        return StepResult(logs, moved_count, deadlock, all_done)

    def _build_rhcr_runtime_candidates(
        self,
        active_agvs: List[AGV],
        next_tick: int,
        logs: List[str],
    ) -> Dict[str, MoveCandidate]:
        candidates: Dict[str, MoveCandidate] = {}
        for agv in active_agvs:
            agv.last_move = None
            self._clear_runtime_mode_state(agv)

            if agv.work_remaining_ticks <= 0 and agv.current_node == agv.goal_node:
                if self._start_work_if_needed(agv, next_tick, logs):
                    self.reserve_node(agv.current_node, agv.agv_id)
                    continue
                self._complete_current_leg_if_ready(agv, next_tick, logs)
                self.reserve_node(agv.current_node, agv.agv_id)
                if agv.status == "DONE":
                    continue

            candidate = self._rhcr_move_candidate(agv)
            candidates[agv.agv_id] = candidate
            logs.append(
                f"[Candidate] AGV {agv.agv_id}: Node {candidate.current_node} -> "
                f"Node {candidate.target_node}; reason={candidate.reason or 'RHCR window plan'}."
            )
        return candidates

    def _rhcr_move_candidate(self, agv: AGV) -> MoveCandidate:
        if agv.work_remaining_ticks > 0:
            return self._make_wait_candidate(
                agv,
                f"work: Node {agv.current_node} remaining {agv.work_remaining_ticks} tick(s)",
            )

        speed_ticks = max(1, agv.speed_ticks)
        next_tick = self.clock + 1
        if agv.last_move_tick > -9999 and next_tick - agv.last_move_tick < speed_ticks:
            return self._make_wait_candidate(
                agv,
                f"speed limit: speed=1 edge per {speed_ticks} tick(s), "
                f"next move available at tick {agv.last_move_tick + speed_ticks}",
            )

        desired_next = self._planned_next_node_without_traffic(agv)
        if desired_next is None:
            desired_next = agv.current_node
        if desired_next != agv.current_node and not self.graph.has_edge(agv.current_node, desired_next):
            desired_next = agv.current_node
        if desired_next == agv.current_node:
            return self._make_wait_candidate(agv, "RHCR planned wait")

        corridor_block = self._corridor_lock_block_reason(agv, desired_next)
        if corridor_block:
            return self._make_wait_candidate(agv, corridor_block)

        movement = Movement(
            agv_id=agv.agv_id,
            from_node=agv.current_node,
            to_node=desired_next,
            edge=self.graph.normalize_edge(agv.current_node, desired_next),
            time=self.clock + 1,
            geometry=self._movement_geometry(agv.current_node, desired_next),
        )
        return MoveCandidate(
            agv_id=agv.agv_id,
            current_node=agv.current_node,
            target_node=desired_next,
            edge=movement.edge,
            zones=[],
            remaining_distance=self._remaining_distance(agv),
            is_wait=False,
            movement=movement,
            reason="RHCR window plan",
        )

    def _rhcr_runtime_conflicts(self, candidates: Dict[str, MoveCandidate]) -> List[str]:
        moving = {agv_id: candidate for agv_id, candidate in candidates.items() if not candidate.is_wait}
        conflicts: List[str] = []

        target_to_ids: Dict[int, List[str]] = {}
        for agv_id, candidate in moving.items():
            target_to_ids.setdefault(candidate.target_node, []).append(agv_id)
        for node_id, agv_ids in target_to_ids.items():
            if len(agv_ids) > 1:
                conflicts.append(f"same target Node {node_id}: {', '.join(sorted(agv_ids))}")

        current_occupancy = self._current_occupancy()
        for agv_id, candidate in moving.items():
            occupant = current_occupancy.get(candidate.target_node)
            if occupant is None or occupant == agv_id:
                continue
            occupant_candidate = candidates.get(occupant)
            occupant_moves_away = (
                occupant_candidate is not None
                and not occupant_candidate.is_wait
                and occupant_candidate.target_node != candidate.current_node
                and occupant_candidate.target_node != occupant_candidate.current_node
            )
            if not occupant_moves_away:
                conflicts.append(
                    f"AGV {agv_id} targets occupied Node {candidate.target_node} held by AGV {occupant}"
                )

        moving_ids = sorted(moving)
        for index, agv_id_1 in enumerate(moving_ids):
            c1 = moving[agv_id_1]
            for agv_id_2 in moving_ids[index + 1:]:
                c2 = moving[agv_id_2]
                if self.is_edge_swap_conflict(c1.movement, c2.movement):
                    conflicts.append(f"edge swap {self._movement_label(c1.movement)} vs {self._movement_label(c2.movement)}")
                elif self.is_same_edge_conflict(c1.movement, c2.movement):
                    conflicts.append(
                        f"same edge {c1.edge}: {self._movement_label(c1.movement)} vs "
                        f"{self._movement_label(c2.movement)}"
                    )
        return conflicts

    def _rebuild_corridor_locks(self, order: List[str], logs: List[str]) -> None:
        self.corridor_locks.clear()
        active_agvs = [agv for agv in self.agvs if agv.status not in ("DONE", "DEADLOCK")]
        order_rank = {agv_id: index for index, agv_id in enumerate(order)}

        for index, agv_1 in enumerate(active_agvs):
            path_1 = self._planned_route_nodes(agv_1)
            for agv_2 in active_agvs[index + 1:]:
                path_2 = self._planned_route_nodes(agv_2)
                shared_nodes = self._opposing_shared_nodes(path_1, path_2)
                if len(shared_nodes) < self.RHCR_CORRIDOR_MIN_SHARED_NODES:
                    continue

                owner = self._corridor_lock_owner(agv_1, agv_2, set(shared_nodes), order_rank)
                lock_id = self._corridor_lock_id(shared_nodes)
                self.corridor_locks[lock_id] = {
                    "owner": owner.agv_id,
                    "nodes": tuple(shared_nodes),
                    "members": tuple(sorted((agv_1.agv_id, agv_2.agv_id))),
                }
                logs.append(
                    f"[CorridorLock] {lock_id}: owner=AGV {owner.agv_id}; "
                    f"nodes={shared_nodes}; members={agv_1.agv_id},{agv_2.agv_id}."
                )

    def _refresh_corridor_locks(self, logs: Optional[List[str]] = None) -> None:
        for lock_id, lock in list(self.corridor_locks.items()):
            owner_id = str(lock["owner"])
            owner = self._agv_by_id(owner_id)
            lock_nodes = set(lock["nodes"])
            if owner is None or owner.status in ("DONE", "DEADLOCK"):
                del self.corridor_locks[lock_id]
                if logs is not None:
                    logs.append(f"[CorridorLock] {lock_id} released: owner AGV {owner_id} is inactive.")
                continue

            owner_route = self._planned_route_nodes(owner)
            owner_still_needs_lock = owner.current_node in lock_nodes or any(
                node_id in lock_nodes for node_id in owner_route[1:]
            )
            if not owner_still_needs_lock:
                del self.corridor_locks[lock_id]
                if logs is not None:
                    logs.append(f"[CorridorLock] {lock_id} released: AGV {owner_id} cleared the corridor.")

    def _corridor_lock_block_reason(self, agv: AGV, desired_next: int) -> Optional[str]:
        for lock_id, lock in self.corridor_locks.items():
            owner_id = str(lock["owner"])
            if owner_id == agv.agv_id:
                continue

            lock_nodes = set(lock["nodes"])
            if agv.current_node in lock_nodes:
                continue
            if desired_next not in lock_nodes:
                continue

            owner = self._agv_by_id(owner_id)
            owner_node = owner.current_node if owner is not None else "unknown"
            return (
                f"corridor {lock_id} reserved by AGV {owner_id}; "
                f"owner currently at Node {owner_node}"
            )
        return None

    def _planned_route_nodes(self, agv: AGV) -> List[int]:
        route = [agv.current_node]
        if agv.planned_path:
            offset = self._plan_offset(agv)
            for node_id in agv.planned_path[offset + 1:]:
                if node_id != route[-1]:
                    route.append(node_id)
        if len(route) == 1 or route[-1] != agv.goal_node:
            fallback = self.graph.shortest_path(agv.current_node, agv.goal_node)
            if len(fallback) > 1:
                route = fallback
        return route

    def _opposing_shared_nodes(self, path_1: List[int], path_2: List[int]) -> List[int]:
        compact_1 = self._without_repeated_nodes(path_1)
        compact_2 = self._without_repeated_nodes(path_2)
        index_2 = {node_id: index for index, node_id in enumerate(compact_2)}
        shared = [node_id for node_id in compact_1 if node_id in index_2]
        if len(shared) < self.RHCR_CORRIDOR_MIN_SHARED_NODES:
            return []

        indices_2 = [index_2[node_id] for node_id in shared]
        if indices_2[0] <= indices_2[-1]:
            return []
        return shared

    def _without_repeated_nodes(self, path: List[int]) -> List[int]:
        result: List[int] = []
        for node_id in path:
            if not result or result[-1] != node_id:
                result.append(node_id)
        return result

    def _corridor_lock_owner(
        self,
        agv_1: AGV,
        agv_2: AGV,
        lock_nodes: Set[int],
        order_rank: Dict[str, int],
    ) -> AGV:
        agv_1_inside = agv_1.current_node in lock_nodes
        agv_2_inside = agv_2.current_node in lock_nodes
        if agv_1_inside and not agv_2_inside:
            return agv_1
        if agv_2_inside and not agv_1_inside:
            return agv_2
        return min(
            (agv_1, agv_2),
            key=lambda agv: (order_rank.get(agv.agv_id, 9999), agv.agv_id),
        )

    def _corridor_lock_id(self, nodes: List[int]) -> str:
        return "C:" + "-".join(str(node_id) for node_id in sorted(set(nodes)))

    def _execute_rhcr_move(self, agv: AGV, candidate: MoveCandidate, tick: int, logs: List[str]) -> None:
        old_node = agv.current_node
        self.reserve_node(candidate.target_node, agv.agv_id)
        if candidate.edge is not None:
            self.reserve_edge(old_node, candidate.target_node, agv.agv_id)

        agv.current_node = candidate.target_node
        agv.last_move = (old_node, candidate.target_node)
        agv.previous_node = old_node
        agv.last_move_tick = tick
        agv.wait_count = 0
        agv.failed_escape_attempts = 0
        agv.status = "MOVING"

        if agv.trip_turnaround and old_node in (agv.start_node, agv.original_goal_node):
            logs.append(
                f"[Trip] AGV {agv.agv_id} completed turnaround departure from Node {old_node}."
            )
            agv.trip_turnaround = False

        if not self._start_work_if_needed(agv, tick, logs):
            self._complete_current_leg_if_ready(agv, tick, logs)

        logs.append(
            f"[Tick {tick}] AGV {agv.agv_id} MOVE Node {old_node} -> Node {candidate.target_node}; "
            f"reserved node {candidate.target_node}, edge {candidate.edge}."
        )

    def _make_wait_candidate(self, agv: AGV, reason: str) -> MoveCandidate:
        movement = Movement(
            agv_id=agv.agv_id,
            from_node=agv.current_node,
            to_node=agv.current_node,
            edge=None,
            time=self.clock + 1,
            geometry=self._movement_geometry(agv.current_node, agv.current_node),
        )
        return MoveCandidate(
            agv_id=agv.agv_id,
            current_node=agv.current_node,
            target_node=agv.current_node,
            edge=None,
            zones=[],
            remaining_distance=self._remaining_distance(agv),
            is_wait=True,
            movement=movement,
            reason=reason,
        )

    def _start_work_if_needed(self, agv: AGV, tick: int, logs: List[str]) -> bool:
        primary_goal = agv.original_goal_node or agv.goal_node
        at_primary_goal = agv.current_node == primary_goal
        if not at_primary_goal or agv.work_completed_for_current_goal:
            return False
        if agv.work_ticks <= 0:
            agv.work_completed_for_current_goal = True
            return False
        agv.work_remaining_ticks = agv.work_ticks
        agv.status = "WORKING"
        agv.wait_count = 0
        agv.planned_path = [agv.current_node] * (self.RHCR_PLANNING_WINDOW + 1)
        agv.plan_start_tick = self.clock
        logs.append(
            f"[Work] AGV {agv.agv_id} starts work at Node {agv.current_node}; "
            f"duration={agv.work_ticks} tick(s)."
        )
        return True

    def _complete_current_leg_if_ready(self, agv: AGV, tick: int, logs: List[str]) -> bool:
        if agv.current_node != agv.goal_node or agv.work_remaining_ticks > 0:
            return False

        sequence = agv.destination_sequence()
        if agv.completed_legs >= len(sequence):
            agv.status = "DONE"
            return True

        agv.completed_legs += 1
        logs.append(
            f"[Trip] AGV {agv.agv_id} completed leg {agv.completed_legs}/{len(sequence)} "
            f"at Node {agv.current_node}."
        )

        if agv.completed_legs >= len(sequence):
            agv.status = "DONE"
            agv.planned_path = [agv.current_node]
            self._clear_commitment(agv)
            logs.append(f"[Tick {tick}] AGV {agv.agv_id} completed all trip leg(s); final node Node {agv.current_node}.")
            return True

        next_goal = sequence[agv.completed_legs]
        agv.goal_node = next_goal
        agv.work_completed_for_current_goal = False
        agv.trip_turnaround = True
        agv.status = "WAITING"
        agv.planned_path = []
        agv.plan_start_tick = self.clock
        self._clear_commitment(agv)
        self.initial_plan_built = False
        self.rhcr_next_replan_tick = self.clock
        logs.append(f"[Trip] AGV {agv.agv_id} next goal is Node {next_goal}; RHCR replan required.")
        return False

    def _current_occupancy(self) -> Dict[int, str]:
        occupancy: Dict[int, str] = {}
        for agv in self.agvs:
            if agv.status != "DONE":
                occupancy[agv.current_node] = agv.agv_id
        return occupancy

    def _remaining_distance(self, agv: AGV) -> int:
        return self._distance_between(agv.current_node, agv.goal_node)

    def _distance_between(self, start: int, goal: int) -> int:
        path = self.graph.shortest_path(start, goal)
        if not path:
            return 9999
        return max(0, len(path) - 1)

    def peek_next_node(self, agv: AGV) -> Optional[int]:
        return self._planned_next_node_without_traffic(agv)

    def _plan_offset(self, agv: AGV) -> int:
        return max(0, self.clock - agv.plan_start_tick)

    def _planned_next_node_without_traffic(self, agv: AGV) -> Optional[int]:
        if agv.status == "DONE":
            return agv.current_node
        if agv.planned_path:
            offset = self._plan_offset(agv)
            next_index = min(offset + 1, len(agv.planned_path) - 1)
            planned_next = agv.planned_path[next_index]
            if planned_next == agv.current_node:
                return planned_next
            if self.graph.has_edge(agv.current_node, planned_next):
                return planned_next
        path = self.graph.shortest_path(agv.current_node, agv.goal_node)
        if len(path) >= 2:
            return path[1]
        return agv.current_node

    def _reset_runtime_reservations(self, logs: Optional[List[str]] = None) -> None:
        if logs is not None and self.edge_reservations:
            logs.append(f"[LockRelease] cleared previous tick edge reservation(s): {dict(self.edge_reservations)}.")
        self.node_reservations.clear()
        self.edge_reservations.clear()
        self.zone_reservations.clear()
        for agv in self.agvs:
            if agv.status != "DONE":
                self.reserve_node(agv.current_node, agv.agv_id)

    def _movement_from_path(self, agv_id: str, path: List[int], tick: int) -> Movement:
        from_node = self._node_at_tick(path, tick - 1)
        to_node = self._node_at_tick(path, tick)
        edge = None if from_node == to_node else self.graph.normalize_edge(from_node, to_node)
        return Movement(
            agv_id=agv_id,
            from_node=from_node,
            to_node=to_node,
            edge=edge,
            time=tick,
            geometry=self._movement_geometry(from_node, to_node),
        )

    def _movement_geometry(self, from_node: int, to_node: int) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        from_pos = self.graph.nodes.get(from_node)
        to_pos = self.graph.nodes.get(to_node)
        if from_pos is None or to_pos is None:
            return ((0.0, 0.0), (0.0, 0.0))
        return ((float(from_pos.x()), float(from_pos.y())), (float(to_pos.x()), float(to_pos.y())))

    def _movement_label(self, movement: Movement) -> str:
        return f"{movement.agv_id}({movement.from_node}->{movement.to_node})"

    def _approve_wait(self, agv: AGV, tick: int, logs: List[str], reason: str) -> None:
        if agv.work_remaining_ticks > 0:
            agv.work_remaining_ticks -= 1
            agv.status = "WORKING" if agv.work_remaining_ticks > 0 else "WAITING"
            if agv.work_remaining_ticks == 0:
                agv.work_completed_for_current_goal = True
                logs.append(f"[Work] AGV {agv.agv_id} finished work at Node {agv.current_node}.")
        elif reason.startswith("speed limit"):
            agv.status = "WAITING"
        else:
            agv.wait_count += 1
            agv.status = "WAITING"

        self.reserve_node(agv.current_node, agv.agv_id)
        self._release_non_current_claims(agv)
        logs.append(f"[Tick {tick}] AGV {agv.agv_id} WAIT at Node {agv.current_node}: {reason}.")

    def _release_node_reservation(self, node_id: int, agv_id: str) -> None:
        if self.node_reservations.get(node_id) == agv_id:
            del self.node_reservations[node_id]

    def _release_non_current_claims(self, agv: AGV) -> None:
        for node_id, owner in list(self.node_reservations.items()):
            if owner == agv.agv_id and node_id != agv.current_node:
                del self.node_reservations[node_id]
        for edge, owner in list(self.edge_reservations.items()):
            if owner == agv.agv_id:
                del self.edge_reservations[edge]

    def _clear_commitment(self, agv: AGV) -> None:
        agv.committed_path = []
        agv.commit_index = 0
        agv.commit_until_index = 0

    def _clear_runtime_mode_state(self, agv: AGV) -> None:
        agv.escape_mode = False
        agv.escape_hold = False
        agv.escape_returning = False
        agv.escape_target = None
        agv.escape_attempts = 0
        agv.failed_escape_attempts = 0

    def _agv_by_id(self, agv_id: str) -> Optional[AGV]:
        for agv in self.agvs:
            if agv.agv_id == agv_id:
                return agv
        return None

    def _node_at_tick(self, path: List[int], tick: int) -> int:
        if not path:
            return -1
        if tick < 0:
            return path[0]
        if tick >= len(path):
            return path[-1]
        return path[tick]
