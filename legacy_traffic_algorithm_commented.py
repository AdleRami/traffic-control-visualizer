# Existing pre-RHCR traffic-control planner code.
#
# This file is intentionally commented out. It preserves the old CBS / risk-aware A*
# planning code for reference while traffic_controller.py runs the RHCR-style planner.
#

# ---- legacy _legacy_build_initial_cbs_plan ----
#     def _legacy_build_initial_cbs_plan(self) -> List[str]:
#         """
#         LEGACY - 기존 CBS/risk-aware 초기 계획입니다.
#         RHCR 적용 후 호출 경로에서 제외했습니다. 요청대로 삭제하지 않고
#         비활성 reference 구현으로 보존합니다.
#
#         CBS 스타일 초기 계획 생성:
#         - AGV별 shortest path를 구합니다.
#         - tick별 node/edge reservation을 만들면서 conflict가 있으면 wait를 삽입합니다.
#         - 먼저 계획된 AGV가 더 높은 우선순위를 가지는 prioritized CBS 형태입니다.
#         """
#         logs: List[str] = []
#         self.recompute_conflict_zones()
#         temp_node_res: Dict[Tuple[int, int], str] = {}
#         temp_edge_res: Dict[Tuple[int, Tuple[int, int]], str] = {}
#         plans: Dict[str, List[int]] = {}
#
#         for agv in sorted(self.agvs, key=lambda item: item.agv_id):
#             agv.status = "WAITING"
#             agv.wait_count = 0
#             agv.last_move = None
#             agv.plan_start_tick = self.clock
#
#             route = self.traffic_aware_astar(
#                 agv.current_node,
#                 agv.goal_node,
#                 self.clock,
#                 agv=agv,
#                 logs=logs,
#                 reason="initial CBS planning",
#             )
#             if not route:
#                 agv.planned_path = [agv.current_node]
#                 agv.status = "DEADLOCK"
#                 logs.append(
#                     f"[CBS] AGV {agv.agv_id}: no path from Node {agv.current_node} "
#                     f"to Node {agv.goal_node}."
#                 )
#                 continue
#
#             path = [agv.current_node]
#             current = agv.current_node
#             tick = self.clock
#             wait_insertions = 0
#             start_key = (tick, current)
#             if start_key not in temp_node_res:
#                 temp_node_res[start_key] = agv.agv_id
#             else:
#                 logs.append(
#                     f"[CBS] initial node conflict at tick {tick}, Node {current}: "
#                     f"AGV {agv.agv_id} conflicts with AGV {temp_node_res[start_key]}."
#                 )
#
#             for next_node in route[1:]:
#                 guard = 0
#                 while guard < self.MAX_INITIAL_PLAN_TICKS:
#                     next_tick = tick + 1
#                     edge = self.graph.normalize_edge(current, next_node)
#                     node_key = (next_tick, next_node)
#                     edge_key = (next_tick, edge)
#                     node_owner = temp_node_res.get(node_key)
#                     edge_owner = temp_edge_res.get(edge_key)
#                     can_move = node_owner in (None, agv.agv_id) and edge_owner in (
#                         None,
#                         agv.agv_id,
#                     )
#                     if can_move:
#                         temp_node_res[node_key] = agv.agv_id
#                         temp_edge_res[edge_key] = agv.agv_id
#                         path.append(next_node)
#                         current = next_node
#                         tick = next_tick
#                         break
#
#                     # CBS constraint를 wait action으로 반영합니다.
#                     wait_tick = tick + 1
#                     wait_key = (wait_tick, current)
#                     if wait_key not in temp_node_res:
#                         temp_node_res[wait_key] = agv.agv_id
#                     else:
#                         # 이미 더 높은 우선순위 AGV가 해당 tick의 current node를
#                         # 예약한 경우입니다. 완전 CBS라면 constraint tree를 확장하지만,
#                         # 여기서는 시각화 목적상 unresolved wait로 남기고 runtime 제어에 맡깁니다.
#                         logs.append(
#                             f"[CBS] AGV {agv.agv_id}: unresolved wait conflict at "
#                             f"tick {wait_tick}, Node {current}."
#                         )
#                     path.append(current)
#                     tick = wait_tick
#                     wait_insertions += 1
#                     guard += 1
#
#                 if guard >= self.MAX_INITIAL_PLAN_TICKS:
#                     logs.append(
#                         f"[CBS] AGV {agv.agv_id}: initial planning stopped at "
#                         f"{self.MAX_INITIAL_PLAN_TICKS} ticks."
#                     )
#                     break
#
#             agv.planned_path = path
#             plans[agv.agv_id] = path
#             logs.append(
#                 f"[CBS] AGV {agv.agv_id}: traffic-aware route {route} -> "
#                 f"time plan {path} ({wait_insertions} wait inserted)."
#             )
#
#         conflicts = self.detect_conflicts(plans)
#         if conflicts:
#             logs.extend(f"[CBS] remaining conflict: {conflict}" for conflict in conflicts)
#         else:
#             logs.append("[CBS] initial time-expanded plans have no detected conflicts.")
#
#         self.initial_plan_built = True
#         return logs
#
#
# ---- legacy traffic_aware_astar ----
#     def traffic_aware_astar(
#         self,
#         start: int,
#         goal: int,
#         current_tick: int,
#         agv: Optional[AGV] = None,
#         logs: Optional[List[str]] = None,
#         reason: str = "",
#     ) -> List[int]:
#         """
#         risk-aware / reservation-aware A* 경로 탐색입니다.
#
#         shortest hop count만 보지 않고, 앞으로의 node/edge/zone 예약과
#         다른 AGV의 planned_path, 현재 혼잡도를 비용으로 반영합니다.
#         단, 우회가 지나치게 길어지면 병목 구간은 WAIT 정책으로 처리하도록
#         shortest path를 유지합니다.
#         """
#         self.recompute_conflict_zones()
#         agv_id = agv.agv_id if agv is not None else None
#         shortest_path = self.graph.shortest_path(start, goal)
#         if not shortest_path:
#             return []
#         if start == goal:
#             return [start]
#
#         shortest_edges = max(1, len(shortest_path) - 1)
#         max_steps = max(
#             shortest_edges,
#             int(math.ceil(shortest_edges * self.MAX_DETOUR_RATIO)) + 2,
#         )
#
#         frontier: List[Tuple[float, float, int, int, List[int]]] = []
#         heapq.heappush(
#             frontier,
#             (float(self._distance_between(start, goal)), 0.0, 0, start, [start]),
#         )
#         best_state_cost: Dict[Tuple[int, int], float] = {(start, 0): 0.0}
#         complete_paths: List[Tuple[float, List[int]]] = []
#         expansion_guard = 0
#         max_expansions = max(200, len(self.graph.nodes) * max_steps * 4)
#
#         while frontier and expansion_guard < max_expansions:
#             _priority, cost_so_far, steps, node_id, path = heapq.heappop(frontier)
#             expansion_guard += 1
#
#             if node_id == goal:
#                 complete_paths.append((cost_so_far, path))
#                 if len(complete_paths) >= 6:
#                     break
#                 continue
#             if steps >= max_steps:
#                 continue
#
#             for next_node in self.graph.neighbors(node_id):
#                 # A* 경로 자체에는 cycle을 넣지 않습니다. 대기와 escape는 step()에서 처리합니다.
#                 if next_node in path:
#                     continue
#                 next_tick = current_tick + steps + 1
#                 step_cost, _components = self._movement_risk_cost(
#                     agv_id,
#                     node_id,
#                     next_node,
#                     next_tick,
#                 )
#                 new_cost = cost_so_far + step_cost
#                 state = (next_node, steps + 1)
#                 if new_cost >= best_state_cost.get(state, float("inf")):
#                     continue
#                 best_state_cost[state] = new_cost
#                 heuristic = float(self._distance_between(next_node, goal))
#                 heapq.heappush(
#                     frontier,
#                     (new_cost + heuristic, new_cost, steps + 1, next_node, path + [next_node]),
#                 )
#
#         shortest_cost, _shortest_risk = self._score_path_risk(
#             agv_id,
#             shortest_path,
#             current_tick,
#         )
#         complete_paths.append((shortest_cost, shortest_path))
#
#         unique_candidates: Dict[Tuple[int, ...], Tuple[float, List[int]]] = {}
#         for cost, path in complete_paths:
#             key = tuple(path)
#             if key not in unique_candidates or cost < unique_candidates[key][0]:
#                 unique_candidates[key] = (cost, path)
#
#         candidates = sorted(
#             unique_candidates.values(),
#             key=lambda item: (item[0], len(item[1]), item[1]),
#         )
#         best_cost, best_path = candidates[0]
#         selected_path = best_path
#         selected_cost = best_cost
#         selected_reason = "lowest predicted conflict risk"
#
#         detour_too_long = (len(best_path) - 1) > shortest_edges * self.MAX_DETOUR_RATIO
#         detour_too_expensive = best_cost > shortest_cost * self.MAX_DETOUR_RATIO
#         if best_path != shortest_path and (detour_too_long or detour_too_expensive):
#             selected_path = shortest_path
#             selected_cost = shortest_cost
#             selected_reason = (
#                 "detour exceeded MAX_DETOUR_RATIO; keep shortest path and wait near bottleneck"
#             )
#         elif best_path == shortest_path:
#             selected_reason = "shortest path has acceptable predicted risk"
#
#         if logs is not None:
#             label = agv_id if agv_id is not None else "-"
#             reason_text = f" ({reason})" if reason else ""
#             logs.append(
#                 f"[AGV {label}] traffic-aware A* from Node {start} to Node {goal}{reason_text}."
#             )
#             for index, (cost, path) in enumerate(candidates[:4], start=1):
#                 report = self.predict_path_conflicts(
#                     agv,
#                     path,
#                     current_tick,
#                     self.RISK_LOOKAHEAD_HORIZON,
#                 )
#                 risk_text = self._format_path_risk(report)
#                 logs.append(
#                     f"candidate path {index}: {'-'.join(str(node) for node in path)}; "
#                     f"risk: {risk_text}; total_cost: {cost:g}."
#                 )
#             if selected_path != shortest_path:
#                 logs.append(
#                     f"[AGV {label}] selected detour path due to lower predicted conflict risk: "
#                     f"{selected_path} (cost={selected_cost:g})."
#                 )
#             else:
#                 logs.append(
#                     f"[AGV {label}] selected path {selected_path}: {selected_reason} "
#                     f"(cost={selected_cost:g})."
#                 )
#
#         return selected_path
#
#
# ---- legacy calculate_node_risk ----
#     def calculate_node_risk(
#         self,
#         node_id: int,
#         tick: int,
#         agv_id: Optional[str] = None,
#     ) -> float:
#         """미래 path/예약/node type을 기준으로 node risk penalty를 계산합니다."""
#         horizon = max(self.RISK_LOOKAHEAD_HORIZON, tick - self.clock + 2)
#         node_table, _edge_table, _directed_edge_table, _zone_table = (
#             self._time_based_reservation_tables(agv_id, self.clock, horizon)
#         )
#         risk = 0.0
#
#         owners = node_table.get((node_id, tick), set())
#         if owners:
#             risk += self.NODE_FUTURE_CONFLICT_PENALTY * len(owners)
#
#         # 정확히 같은 tick이 아니더라도 다른 AGV의 가까운 미래 path에 포함되면 약한 penalty를 둡니다.
#         for (reserved_node, reserved_tick), reserved_owners in node_table.items():
#             if reserved_node == node_id and reserved_tick != tick and reserved_owners:
#                 risk += self.NODE_FUTURE_CONFLICT_PENALTY * 0.25
#                 break
#
#         node_owner = self.node_reservations.get(node_id)
#         if node_owner is not None and node_owner != agv_id:
#             risk += self.NODE_RESERVATION_PENALTY
#
#         node_type = self.graph.get_node_type(node_id)
#         if node_type == INTERSECTION_NODE:
#             risk += self.NODE_TYPE_INTERSECTION_PENALTY
#         elif node_type == RESERVATION_NODE:
#             risk += self.NODE_TYPE_RESERVATION_PENALTY
#
#         return risk
#
#
# ---- legacy calculate_edge_risk ----
#     def calculate_edge_risk(
#         self,
#         from_node: int,
#         to_node: int,
#         tick: int,
#         agv_id: Optional[str] = None,
#     ) -> float:
#         """동일 edge, 반대방향 head-on, narrow passage 위험도를 계산합니다."""
#         if from_node == to_node:
#             return 0.0
#         horizon = max(self.RISK_LOOKAHEAD_HORIZON, tick - self.clock + 2)
#         _node_table, edge_table, directed_edge_table, _zone_table = (
#             self._time_based_reservation_tables(agv_id, self.clock, horizon)
#         )
#         edge = self.graph.normalize_edge(from_node, to_node)
#         risk = 0.0
#
#         same_edge_owners = edge_table.get((edge, tick), set())
#         if same_edge_owners:
#             risk += self.EDGE_FUTURE_CONFLICT_PENALTY * len(same_edge_owners)
#
#         reverse_owners = directed_edge_table.get((to_node, from_node, tick), set())
#         if reverse_owners:
#             risk += self.HEAD_ON_CONFLICT_PENALTY * len(reverse_owners)
#
#         edge_owner = self.edge_reservations.get(edge)
#         if edge_owner is not None and edge_owner != agv_id:
#             risk += self.EDGE_RESERVATION_PENALTY
#
#         if self._is_narrow_passage_edge(from_node, to_node):
#             risk += self.NARROW_EDGE_PENALTY
#
#         return risk
#
#
# ---- legacy calculate_zone_risk ----
#     def calculate_zone_risk(
#         self,
#         zone_id: str,
#         movement: Movement,
#         tick: int,
#         agv_id: Optional[str] = None,
#     ) -> float:
#         """
#         conflict zone risk를 movement-aware 방식으로 계산합니다.
#         같은 zone을 공유해도 geometry가 compatible하면 낮은 penalty만 부여합니다.
#         """
#         horizon = max(self.RISK_LOOKAHEAD_HORIZON, tick - self.clock + 2)
#         _node_table, _edge_table, _directed_edge_table, zone_table = (
#             self._time_based_reservation_tables(agv_id, self.clock, horizon)
#         )
#         risk = 0.0
#
#         for other_movement in zone_table.get((zone_id, tick), []):
#             if other_movement.agv_id == agv_id:
#                 continue
#             if self.are_movements_compatible_in_zone(movement, other_movement, zone_id):
#                 risk += self.ZONE_COMPATIBLE_PENALTY
#             else:
#                 risk += self.ZONE_CONFLICT_PENALTY
#
#         for reservation in self.zone_reservations.get(zone_id, []):
#             if reservation.agv_id == agv_id:
#                 continue
#             existing = Movement(
#                 agv_id=reservation.agv_id,
#                 from_node=reservation.from_node,
#                 to_node=reservation.to_node,
#                 edge=reservation.edge,
#                 time=reservation.time,
#                 geometry=reservation.geometry,
#             )
#             if reservation.time == tick and not self.are_movements_compatible_in_zone(
#                 movement,
#                 existing,
#                 zone_id,
#             ):
#                 risk += self.ZONE_RESERVATION_PENALTY
#
#         zone = self.conflict_zones.get(zone_id)
#         if zone is not None:
#             occupants = sum(
#                 1
#                 for other in self.agvs
#                 if other.agv_id != agv_id
#                 and other.status not in ("DONE", "DEADLOCK")
#                 and other.current_node in zone.nodes
#             )
#             risk += occupants * self.ZONE_CONGESTION_PENALTY
#
#         return risk
#
#
# ---- legacy calculate_congestion_risk ----
#     def calculate_congestion_risk(
#         self,
#         node_id: int,
#         agv_id: Optional[str] = None,
#     ) -> float:
#         """현재 AGV 분포 기준 1-hop/2-hop 혼잡 penalty를 계산합니다."""
#         one_hop = set(self.graph.neighbors(node_id))
#         two_hop: Set[int] = set()
#         for near_node in one_hop:
#             two_hop.update(self.graph.neighbors(near_node))
#         two_hop.discard(node_id)
#         two_hop.difference_update(one_hop)
#
#         risk = 0.0
#         for other in self.agvs:
#             if other.agv_id == agv_id or other.status == "DEADLOCK":
#                 continue
#             if other.current_node == node_id:
#                 risk += self.NODE_RESERVATION_PENALTY
#             elif other.current_node in one_hop:
#                 risk += self.CONGESTION_ONE_HOP_PENALTY
#             elif other.current_node in two_hop:
#                 risk += self.CONGESTION_TWO_HOP_PENALTY
#         return risk
#
#
# ---- legacy predict_path_conflicts ----
#     def predict_path_conflicts(
#         self,
#         agv: Optional[AGV],
#         path: List[int],
#         current_tick: int,
#         horizon: int,
#     ) -> PathRiskReport:
#         """
#         planned_path를 앞으로 N tick까지 검사하고 위험 node/edge/zone 목록을 반환합니다.
#         완전 차단이 아니라 A* 비용으로 쓰기 위한 risk report입니다.
#         """
#         agv_id = agv.agv_id if agv is not None else None
#         report = PathRiskReport()
#         if len(path) < 2:
#             return report
#
#         max_step = min(len(path) - 1, horizon)
#         node_table, edge_table, directed_edge_table, zone_table = (
#             self._time_based_reservation_tables(agv_id, current_tick, horizon)
#         )
#
#         for step_index in range(1, max_step + 1):
#             from_node = path[step_index - 1]
#             to_node = path[step_index]
#             tick = current_tick + step_index
#             edge = self.graph.normalize_edge(from_node, to_node)
#             movement = Movement(
#                 agv_id=agv_id or "",
#                 from_node=from_node,
#                 to_node=to_node,
#                 edge=edge,
#                 time=tick,
#                 geometry=self._movement_geometry(from_node, to_node),
#             )
#
#             components = self._movement_risk_components(agv_id, from_node, to_node, tick)
#             report.total_risk += sum(components.values())
#
#             node_owners = node_table.get((to_node, tick), set())
#             if node_owners:
#                 report.risk_nodes.add(to_node)
#                 report.reasons.append(
#                     f"node {to_node} planned/reserved by AGV {','.join(sorted(node_owners))}"
#                 )
#
#             if self.graph.get_node_type(to_node) in (INTERSECTION_NODE, RESERVATION_NODE):
#                 report.risk_nodes.add(to_node)
#                 report.reasons.append(
#                     f"node {to_node} type={self.graph.get_node_type(to_node)}"
#                 )
#
#             edge_owners = edge_table.get((edge, tick), set())
#             if edge_owners:
#                 report.risk_edges.add(edge)
#                 report.reasons.append(
#                     f"edge {edge[0]}-{edge[1]} planned by AGV {','.join(sorted(edge_owners))}"
#                 )
#
#             reverse_owners = directed_edge_table.get((to_node, from_node, tick), set())
#             if reverse_owners:
#                 report.risk_edges.add(edge)
#                 report.reasons.append(
#                     f"head-on edge {to_node}->{from_node} by AGV {','.join(sorted(reverse_owners))}"
#                 )
#
#             for zone_id in self.get_movement_zones(movement):
#                 for other_movement in zone_table.get((zone_id, tick), []):
#                     if other_movement.agv_id == agv_id:
#                         continue
#                     if self.are_movements_compatible_in_zone(movement, other_movement, zone_id):
#                         continue
#                     report.risk_zones.add(zone_id)
#                     report.reasons.append(
#                         f"{zone_id} conflict with AGV {other_movement.agv_id} "
#                         f"{other_movement.from_node}->{other_movement.to_node}"
#                     )
#
#             if components["congestion"] >= self.CONGESTION_ONE_HOP_PENALTY:
#                 report.risk_nodes.add(to_node)
#                 report.reasons.append(f"node {to_node} has nearby AGV congestion")
#
#         # 로그가 과도하게 길어지지 않도록 같은 사유는 한 번만 남깁니다.
#         report.reasons = list(dict.fromkeys(report.reasons))
#         return report
#
#
# ---- legacy _maybe_traffic_aware_replan ----
#     def _maybe_traffic_aware_replan(self, agv: AGV, logs: List[str]) -> None:
#         """정해진 조건에서만 risk-aware A* 재계획을 수행합니다."""
#         if (
#             agv.escape_mode
#             or agv.escape_hold
#             or agv.escape_returning
#             or agv.status in ("DONE", "DEADLOCK")
#         ):
#             return
#
#         current_plan = self._current_plan_from_current(agv)
#         if len(current_plan) <= 1:
#             logs.extend(self.replan_agv(agv, "departure/no active traffic-aware plan"))
#             return
#
#         if agv.wait_count >= self.WAIT_THRESHOLD:
#             if self.clock - agv.last_risk_replan_tick < self.RISK_REPLAN_COOLDOWN:
#                 return
#             agv.last_risk_replan_tick = self.clock
#             logs.extend(
#                 self.replan_agv(
#                     agv,
#                     f"wait_count={agv.wait_count} reached WAIT_THRESHOLD",
#                 )
#             )
#             return
#
#         if self.clock - agv.plan_start_tick < self.RISK_REPLAN_COOLDOWN:
#             return
#         if self.clock - agv.last_risk_replan_tick < self.RISK_REPLAN_COOLDOWN:
#             return
#
#         report = self.predict_path_conflicts(
#             agv,
#             current_plan,
#             self.clock,
#             self.RISK_LOOKAHEAD_HORIZON,
#         )
#         if report.total_risk < self.PATH_RISK_REPLAN_THRESHOLD:
#             return
#         agv.last_risk_replan_tick = self.clock
#
#         old_plan = list(current_plan)
#         old_risk = report.total_risk
#         new_logs: List[str] = []
#         new_route = self.traffic_aware_astar(
#             agv.current_node,
#             agv.goal_node,
#             self.clock,
#             agv=agv,
#             logs=new_logs,
#             reason=f"predicted path risk {old_risk:g}",
#         )
#         if not new_route:
#             logs.append(
#                 f"[Replan] AGV {agv.agv_id}: predicted risk is high "
#                 f"({self._format_path_risk(report)}), but no alternative route exists."
#             )
#             return
#
#         new_report = self.predict_path_conflicts(
#             agv,
#             new_route,
#             self.clock,
#             self.RISK_LOOKAHEAD_HORIZON,
#         )
#         if (
#             tuple(new_route) != tuple(old_plan)
#             and new_report.total_risk + self.PATH_RISK_REPLAN_MARGIN < old_risk
#         ):
#             logs.extend(new_logs)
#             agv.planned_path = new_route
#             agv.plan_start_tick = self.clock
#             agv.replan_count += 1
#             self._clear_commitment(agv)
#             logs.append(
#                 f"[Replan] AGV {agv.agv_id}: current path risk "
#                 f"{old_risk:g} -> {new_report.total_risk:g}; "
#                 f"traffic-aware path selected: {new_route}."
#             )
#         else:
#             logs.append(
#                 f"[Replan] AGV {agv.agv_id}: predicted path risk is high "
#                 f"({self._format_path_risk(report)}), but no lower-risk detour "
#                 "within MAX_DETOUR_RATIO was found. Keep current path and wait when needed."
#             )
#
#
# ---- legacy _movement_risk_cost ----
#     def _movement_risk_cost(
#         self,
#         agv_id: Optional[str],
#         from_node: int,
#         to_node: int,
#         tick: int,
#     ) -> Tuple[float, Dict[str, float]]:
#         components = self._movement_risk_components(agv_id, from_node, to_node, tick)
#         return 1.0 + sum(components.values()), components
#
#
# ---- legacy _movement_risk_components ----
#     def _movement_risk_components(
#         self,
#         agv_id: Optional[str],
#         from_node: int,
#         to_node: int,
#         tick: int,
#     ) -> Dict[str, float]:
#         edge = None if from_node == to_node else self.graph.normalize_edge(from_node, to_node)
#         movement = Movement(
#             agv_id=agv_id or "",
#             from_node=from_node,
#             to_node=to_node,
#             edge=edge,
#             time=tick,
#             geometry=self._movement_geometry(from_node, to_node),
#         )
#         zone_risk = sum(
#             self.calculate_zone_risk(zone_id, movement, tick, agv_id)
#             for zone_id in self.get_movement_zones(movement)
#         )
#         return {
#             "node": self.calculate_node_risk(to_node, tick, agv_id),
#             "edge": self.calculate_edge_risk(from_node, to_node, tick, agv_id),
#             "zone": zone_risk,
#             "reservation": self._calculate_reservation_risk(movement, tick, agv_id),
#             "congestion": self.calculate_congestion_risk(to_node, agv_id),
#         }
#
#
# ---- legacy _score_path_risk ----
#     def _score_path_risk(
#         self,
#         agv_id: Optional[str],
#         path: List[int],
#         current_tick: int,
#     ) -> Tuple[float, float]:
#         total_cost = 0.0
#         total_risk = 0.0
#         for step_index in range(1, len(path)):
#             tick = current_tick + step_index
#             step_cost, components = self._movement_risk_cost(
#                 agv_id,
#                 path[step_index - 1],
#                 path[step_index],
#                 tick,
#             )
#             total_cost += step_cost
#             total_risk += sum(components.values())
#         return total_cost, total_risk
#
#
# ---- legacy _calculate_reservation_risk ----
#     def _calculate_reservation_risk(
#         self,
#         movement: Movement,
#         tick: int,
#         agv_id: Optional[str] = None,
#     ) -> float:
#         horizon = max(self.RISK_LOOKAHEAD_HORIZON, tick - self.clock + 2)
#         node_table, edge_table, _directed_edge_table, zone_table = (
#             self._time_based_reservation_tables(agv_id, self.clock, horizon)
#         )
#         risk = 0.0
#
#         node_owners = node_table.get((movement.to_node, tick), set())
#         if node_owners:
#             risk += self.NODE_RESERVATION_PENALTY
#
#         if movement.edge is not None and edge_table.get((movement.edge, tick), set()):
#             risk += self.EDGE_RESERVATION_PENALTY
#
#         for zone_id in self.get_movement_zones(movement):
#             for other_movement in zone_table.get((zone_id, tick), []):
#                 if other_movement.agv_id == agv_id:
#                     continue
#                 if not self.are_movements_compatible_in_zone(movement, other_movement, zone_id):
#                     risk += self.ZONE_RESERVATION_PENALTY
#                     break
#
#         return risk
#
#
# ---- legacy _time_based_reservation_tables ----
#     def _time_based_reservation_tables(
#         self,
#         excluded_agv_id: Optional[str],
#         current_tick: int,
#         horizon: int,
#     ) -> Tuple[
#         Dict[Tuple[int, int], Set[str]],
#         Dict[Tuple[Tuple[int, int], int], Set[str]],
#         Dict[Tuple[int, int, int], Set[str]],
#         Dict[Tuple[str, int], List[Movement]],
#     ]:
#         node_table: Dict[Tuple[int, int], Set[str]] = {}
#         edge_table: Dict[Tuple[Tuple[int, int], int], Set[str]] = {}
#         directed_edge_table: Dict[Tuple[int, int, int], Set[str]] = {}
#         zone_table: Dict[Tuple[str, int], List[Movement]] = {}
#
#         for other in self.agvs:
#             # DONE AGV도 node를 점유한 상태이므로 미래 예약 테이블에는 남깁니다.
#             if other.agv_id == excluded_agv_id or other.status == "DEADLOCK":
#                 continue
#             future_path = self._future_path_from_current(other, horizon)
#             for offset, node_id in enumerate(future_path[: horizon + 1]):
#                 tick = current_tick + offset
#                 node_table.setdefault((node_id, tick), set()).add(other.agv_id)
#                 if offset == 0:
#                     continue
#                 prev_node = future_path[offset - 1]
#                 if prev_node == node_id:
#                     continue
#                 edge = self.graph.normalize_edge(prev_node, node_id)
#                 edge_table.setdefault((edge, tick), set()).add(other.agv_id)
#                 directed_edge_table.setdefault((prev_node, node_id, tick), set()).add(other.agv_id)
#                 movement = Movement(
#                     agv_id=other.agv_id,
#                     from_node=prev_node,
#                     to_node=node_id,
#                     edge=edge,
#                     time=tick,
#                     geometry=self._movement_geometry(prev_node, node_id),
#                 )
#                 for zone_id in self.get_movement_zones(movement):
#                     zone_table.setdefault((zone_id, tick), []).append(movement)
#
#         for node_id, owner in self.node_reservations.items():
#             if owner != excluded_agv_id:
#                 node_table.setdefault((node_id, current_tick), set()).add(owner)
#
#         for edge, owner in self.edge_reservations.items():
#             if owner != excluded_agv_id:
#                 edge_table.setdefault((edge, current_tick + 1), set()).add(owner)
#
#         for zone_id, reservations in self.zone_reservations.items():
#             for reservation in reservations:
#                 if reservation.agv_id == excluded_agv_id:
#                     continue
#                 movement = Movement(
#                     agv_id=reservation.agv_id,
#                     from_node=reservation.from_node,
#                     to_node=reservation.to_node,
#                     edge=reservation.edge,
#                     time=reservation.time,
#                     geometry=reservation.geometry,
#                 )
#                 zone_table.setdefault((zone_id, reservation.time), []).append(movement)
#
#         return node_table, edge_table, directed_edge_table, zone_table
#
#
# ---- legacy _current_plan_from_current ----
#     def _current_plan_from_current(self, agv: AGV) -> List[int]:
#         if agv.planned_path:
#             offset = min(self._plan_offset(agv), max(0, len(agv.planned_path) - 1))
#             if agv.planned_path[offset] == agv.current_node:
#                 return list(agv.planned_path[offset:])
#             for index in range(offset, len(agv.planned_path)):
#                 node_id = agv.planned_path[index]
#                 if node_id == agv.current_node:
#                     return list(agv.planned_path[index:])
#         return [agv.current_node]
#
#
# ---- legacy _is_narrow_passage_edge ----
#     def _is_narrow_passage_edge(self, from_node: int, to_node: int) -> bool:
#         if from_node == to_node:
#             return False
#         return self.graph.degree(from_node) <= 2 and self.graph.degree(to_node) <= 2
#
#
# ---- legacy _format_path_risk ----
#     def _format_path_risk(self, report: PathRiskReport) -> str:
#         if not report.reasons:
#             return "low"
#         preview = ", ".join(report.reasons[:3])
#         if len(report.reasons) > 3:
#             preview += ", ..."
#         return preview
#
#
# ---- legacy _shortest_next_if_clear ----
#     def _shortest_next_if_clear(
#         self,
#         agv: AGV,
#         logs: Optional[List[str]] = None,
#     ) -> Optional[int]:
#         """
#         목표 방향 shortest path의 다음 노드가 실제로 clear이면 local sidestep
#         평가보다 우선합니다.
#         """
#         if agv.escape_mode or agv.escape_hold or agv.escape_returning:
#             return None
#
#         route = self.graph.shortest_path(agv.current_node, agv.goal_node)
#         if not route or len(route) < 2:
#             return None
#
#         next_node = route[1]
#         if next_node == agv.previous_node and not self._is_backtrack_allowed(agv):
#             if logs is not None:
#                 logs.append(
#                     f"[ShortestPath] AGV {agv.agv_id} shortest next "
#                     f"{agv.current_node}->{next_node} points to previous node; "
#                     "U-turn is blocked."
#                 )
#             return None
#
#         movement = Movement(
#             agv_id=agv.agv_id,
#             from_node=agv.current_node,
#             to_node=next_node,
#             edge=self.graph.normalize_edge(agv.current_node, next_node),
#             time=self.clock + 1,
#             geometry=self._movement_geometry(agv.current_node, next_node),
#         )
#
#         occupant = self._current_occupancy().get(next_node)
#         if occupant is not None and occupant != agv.agv_id:
#             if logs is not None:
#                 logs.append(
#                     f"[ShortestPath] AGV {agv.agv_id} cannot use shortest next "
#                     f"{agv.current_node}->{next_node}: Node {next_node} occupied by AGV {occupant}."
#                 )
#             return None
#
#         node_owner = self.node_reservations.get(next_node)
#         if node_owner is not None and node_owner != agv.agv_id:
#             if logs is not None:
#                 logs.append(
#                     f"[ShortestPath] AGV {agv.agv_id} cannot use shortest next "
#                     f"{agv.current_node}->{next_node}: Node {next_node} reserved by AGV {node_owner}."
#                 )
#             return None
#
#         edge_owner = self.edge_reservations.get(movement.edge)
#         if edge_owner is not None and edge_owner != agv.agv_id:
#             if logs is not None:
#                 logs.append(
#                     f"[ShortestPath] AGV {agv.agv_id} cannot use shortest next "
#                     f"{agv.current_node}->{next_node}: edge {movement.edge} reserved by AGV {edge_owner}."
#                 )
#             return None
#
#         can_enter, reason = self.can_enter_zone_with_movement(agv, movement)
#         if not can_enter:
#             if logs is not None:
#                 logs.append(
#                     f"[ShortestPath] AGV {agv.agv_id} cannot use shortest next "
#                     f"{agv.current_node}->{next_node}: {reason}."
#                 )
#             return None
#
#         components = self._movement_risk_components(
#             agv.agv_id,
#             agv.current_node,
#             next_node,
#             self.clock + 1,
#         )
#         risk = sum(components.values())
#         if risk >= self.PATH_RISK_REPLAN_THRESHOLD:
#             if logs is not None:
#                 logs.append(
#                     f"[ShortestPath] AGV {agv.agv_id} shortest next "
#                     f"{agv.current_node}->{next_node} has high risk={risk:g} "
#                     f"components={components}; fall back to local traffic-aware planner."
#                 )
#             return None
#
#         if logs is not None:
#             logs.append(
#                 f"[ShortestPath] AGV {agv.agv_id} uses clear shortest next "
#                 f"{agv.current_node}->{next_node}; local sidestep skipped "
#                 f"(risk={risk:g})."
#             )
#         return next_node
#
#
# ---- legacy _select_traffic_aware_next_node ----
#     def _select_traffic_aware_next_node(
#         self,
#         agv: AGV,
#         logs: Optional[List[str]] = None,
#         update_plan: bool = False,
#     ) -> Optional[int]:
#         """distance만 보지 않고 traffic-aware cost가 가장 낮은 next node를 선택합니다."""
#         if agv.current_node == agv.goal_node:
#             return agv.current_node
#
#         neighbor_nodes = self.graph.neighbors(agv.current_node)
#         if not neighbor_nodes:
#             if logs is not None:
#                 logs.append(f"[AGV {agv.agv_id}] no neighbor from Node {agv.current_node}; wait.")
#             return agv.current_node
#
#         shortest_next = self._shortest_next_if_clear(agv, logs)
#         if shortest_next is not None:
#             if update_plan:
#                 route = self.graph.shortest_path(agv.current_node, agv.goal_node)
#                 agv.planned_path = route
#                 agv.plan_start_tick = self.clock
#                 self._clear_commitment(agv)
#             return shortest_next
#
#         predicted_movements = self._predict_other_movements(agv)
#         committed_next = self._committed_next_node(agv)
#         if committed_next is not None:
#             committed_cost = self._evaluate_path_cost(
#                 agv,
#                 [agv.current_node, committed_next],
#                 predicted_movements,
#             )
#             if committed_cost.raw_reservation_score <= 0 and committed_cost.raw_zone_score < 4.0:
#                 if logs is not None:
#                     logs.append(
#                         f"[AGV {agv.agv_id}] commitment keeps next node {committed_next}; "
#                         f"commit_index={agv.commit_index}, until={agv.commit_until_index}."
#                     )
#                     logs.append(
#                         f"[AGV {agv.agv_id}] selected {agv.current_node}->{committed_next} "
#                         "due to active commitment and no immediate conflict."
#                     )
#                 return committed_next
#             if logs is not None:
#                 logs.append(
#                     f"[AGV {agv.agv_id}] commitment {agv.current_node}->{committed_next} "
#                     "interrupted by conflict risk."
#                 )
#             self._clear_commitment(agv)
#
#         paths = self._generate_horizon_paths(agv)
#         costs = [
#             self._evaluate_path_cost(agv, path, predicted_movements)
#             for path in paths
#             if len(path) >= 2
#         ]
#         if not costs:
#             return agv.current_node
#
#         has_safe_non_backtrack = any(
#             not cost.rejected_by_anti_oscillation
#             and cost.raw_reservation_score <= 0
#             and cost.raw_zone_score < 4.0
#             for cost in costs
#         )
#         if has_safe_non_backtrack:
#             selectable_costs = [
#                 cost for cost in costs if not cost.rejected_by_anti_oscillation
#             ]
#         else:
#             selectable_costs = costs
#
#         costs.sort(
#             key=lambda item: (
#                 item.total,
#                 item.raw_reservation_score,
#                 item.raw_zone_score,
#                 item.distance_to_goal,
#                 item.next_node,
#             )
#         )
#         selectable_costs.sort(
#             key=lambda item: (
#                 item.total,
#                 item.raw_reservation_score,
#                 item.raw_zone_score,
#                 item.distance_to_goal,
#                 item.next_node,
#             )
#         )
#
#         if logs is not None:
#             logs.append(f"[AGV {agv.agv_id}] evaluating moves from Node {agv.current_node}:")
#             for cost in costs:
#                 anti_note = ""
#                 if cost.rejected_by_anti_oscillation:
#                     anti_note = ", rejected_by_anti_oscillation"
#                 progress_note = (
#                     "progress_reward"
#                     if cost.raw_progress_delta > 0
#                     else ("no_progress_penalty" if cost.raw_progress_delta <= 0 else "")
#                 )
#                 backtrack_note = (
#                     ", backtrack_penalty"
#                     if cost.raw_backtrack_score > 0
#                     else ""
#                 )
#                 logs.append(
#                     f"- path {'->'.join(str(node) for node in cost.path)} : "
#                     f"next={cost.next_node}, cost={cost.total:g} "
#                     f"(distance={cost.distance_component:g}, zone={cost.zone_component:g}, "
#                     f"congestion={cost.congestion_component:g}, "
#                     f"reservation={cost.reservation_component:g}, "
#                     f"intersection={cost.intersection_component:g}, "
#                     f"backtrack={cost.backtrack_component:g}, "
#                     f"progress={cost.progress_component:g}, "
#                     f"horizon={cost.horizon_component:g}, "
#                     f"progress_delta={cost.raw_progress_delta}"
#                     f"{backtrack_note}, {progress_note}{anti_note})"
#                 )
#             if has_safe_non_backtrack:
#                 for cost in costs:
#                     if cost.rejected_by_anti_oscillation:
#                         logs.append(
#                             f"[AGV {agv.agv_id}] evaluate {agv.current_node} -> previous node "
#                             "rejected by anti-oscillation."
#                         )
#
#         best = selectable_costs[0]
#         if logs is not None:
#             reason = (
#                 "progress toward goal"
#                 if best.raw_progress_delta > 0
#                 else "lowest traffic-aware horizon cost"
#             )
#             if best.raw_backtrack_score > 0:
#                 reason += "; backtrack allowed because no safer candidate exists"
#             logs.append(
#                 f"[AGV {agv.agv_id}] selected {agv.current_node}->{best.next_node} due to {reason}."
#             )
#
#         if update_plan:
#             agv.planned_path = list(best.path)
#             agv.plan_start_tick = self.clock
#             self._set_commitment(agv, best.path)
#
#         return best.next_node
#
#
# ---- legacy _evaluate_move_cost ----
#     def _evaluate_move_cost(
#         self,
#         agv: AGV,
#         next_node: int,
#         predicted_movements: List[Movement],
#     ) -> MoveCost:
#         return self._evaluate_path_cost(
#             agv,
#             [agv.current_node, next_node],
#             predicted_movements,
#         )
#
#
# ---- legacy _generate_horizon_paths ----
#     def _generate_horizon_paths(self, agv: AGV) -> List[List[int]]:
#         """현재 노드에서 2~4 step 후보 경로를 생성합니다."""
#         paths: List[List[int]] = []
#         max_depth = max(1, self.SHORT_HORIZON_STEPS)
#
#         def dfs(path: List[int], depth: int) -> None:
#             current = path[-1]
#             if len(path) >= 2:
#                 paths.append(list(path))
#             if depth >= max_depth or current == agv.goal_node:
#                 return
#
#             neighbors = self.graph.neighbors(current)
#             for nxt in neighbors:
#                 # 짧은 horizon 안에서 불필요한 cycle을 줄입니다.
#                 if nxt in path[:-1]:
#                     continue
#                 path.append(nxt)
#                 dfs(path, depth + 1)
#                 path.pop()
#
#         dfs([agv.current_node], 0)
#         if not paths:
#             for nxt in self.graph.neighbors(agv.current_node):
#                 paths.append([agv.current_node, nxt])
#         return paths
#
#
# ---- legacy _evaluate_path_cost ----
#     def _evaluate_path_cost(
#         self,
#         agv: AGV,
#         path: List[int],
#         predicted_movements: List[Movement],
#     ) -> MoveCost:
#         next_node = path[1]
#         movement = Movement(
#             agv_id=agv.agv_id,
#             from_node=agv.current_node,
#             to_node=next_node,
#             edge=self.graph.normalize_edge(agv.current_node, next_node),
#             time=self.clock + 1,
#             geometry=self._movement_geometry(agv.current_node, next_node),
#         )
#         current_distance_to_goal = self._distance_between(agv.current_node, agv.goal_node)
#         distance_to_goal = self._distance_between(next_node, agv.goal_node)
#         raw_zone_score = self._zone_conflict_score(agv, movement, predicted_movements)
#         raw_congestion_score = self._congestion_score(agv, next_node)
#         raw_reservation_score = self._reservation_conflict_score(agv, movement)
#         raw_intersection_score = self._intersection_penalty(movement)
#         raw_backtrack_score = 1.0 if next_node == agv.previous_node else 0.0
#         raw_progress_delta = current_distance_to_goal - distance_to_goal
#
#         progress_component = (
#             -self.PROGRESS_REWARD
#             if raw_progress_delta > 0
#             else self.NO_PROGRESS_PENALTY
#         )
#         if raw_progress_delta < 0:
#             progress_component += self.NO_PROGRESS_PENALTY + self.REGRESSION_PENALTY
#         horizon_distance = self._distance_between(path[-1], agv.goal_node)
#         horizon_component = self.HORIZON_DISTANCE_WEIGHT * horizon_distance
#
#         distance_component = self.DISTANCE_WEIGHT * distance_to_goal
#         zone_component = self.ZONE_CONFLICT_WEIGHT * raw_zone_score
#         congestion_component = self.CONGESTION_WEIGHT * raw_congestion_score
#         reservation_component = self.RESERVATION_WEIGHT * raw_reservation_score
#         intersection_component = self.INTERSECTION_WEIGHT * raw_intersection_score
#         backtrack_component = self.BACKTRACK_PENALTY * raw_backtrack_score
#         total = (
#             distance_component
#             + zone_component
#             + congestion_component
#             + reservation_component
#             + intersection_component
#             + backtrack_component
#             + progress_component
#             + horizon_component
#         )
#
#         return MoveCost(
#             next_node=next_node,
#             path=path,
#             total=total,
#             distance_component=distance_component,
#             zone_component=zone_component,
#             congestion_component=congestion_component,
#             reservation_component=reservation_component,
#             intersection_component=intersection_component,
#             backtrack_component=backtrack_component,
#             progress_component=progress_component,
#             horizon_component=horizon_component,
#             distance_to_goal=distance_to_goal,
#             current_distance_to_goal=current_distance_to_goal,
#             raw_zone_score=raw_zone_score,
#             raw_congestion_score=raw_congestion_score,
#             raw_reservation_score=raw_reservation_score,
#             raw_intersection_score=raw_intersection_score,
#             raw_backtrack_score=raw_backtrack_score,
#             raw_progress_delta=raw_progress_delta,
#             rejected_by_anti_oscillation=(
#                 raw_backtrack_score > 0 and len(self.graph.neighbors(agv.current_node)) > 1
#             ),
#             movement=movement,
#         )
#
#
# ---- legacy _zone_conflict_score ----
#     def _zone_conflict_score(
#         self,
#         agv: AGV,
#         movement: Movement,
#         predicted_movements: List[Movement],
#     ) -> float:
#         score = 0.0
#         movement_zones = self.get_movement_zones(movement)
#         for zone_id in movement_zones:
#             score += len(self.zone_reservations.get(zone_id, [])) * 2.0
#
#             zone = self.conflict_zones.get(zone_id)
#             if zone is not None:
#                 for other in self.agvs:
#                     if other.agv_id == agv.agv_id or other.status in ("DONE", "DEADLOCK"):
#                         continue
#                     if other.current_node in zone.nodes:
#                         score += 0.75
#
#             for other_movement in predicted_movements:
#                 if zone_id not in self.get_movement_zones(other_movement):
#                     continue
#                 if self.are_movements_compatible_in_zone(movement, other_movement, zone_id):
#                     score += 0.5
#                 else:
#                     score += 4.0
#         return score
#
#
# ---- legacy _congestion_score ----
#     def _congestion_score(self, agv: AGV, next_node: int) -> float:
#         one_hop = set(self.graph.neighbors(next_node))
#         two_hop: Set[int] = set()
#         for node_id in one_hop:
#             two_hop.update(self.graph.neighbors(node_id))
#         two_hop.discard(next_node)
#         two_hop.difference_update(one_hop)
#
#         score = 0.0
#         for other in self.agvs:
#             if other.agv_id == agv.agv_id or other.status == "DEADLOCK":
#                 continue
#             if other.current_node == next_node:
#                 score += 4.0
#             elif other.current_node in one_hop:
#                 score += 2.0
#             elif other.current_node in two_hop:
#                 score += 1.0
#         return score
#
#
# ---- legacy _reservation_conflict_score ----
#     def _reservation_conflict_score(self, agv: AGV, movement: Movement) -> float:
#         score = 0.0
#         node_owner = self.node_reservations.get(movement.to_node)
#         if node_owner is not None and node_owner != agv.agv_id:
#             score += 5.0
#
#         if movement.edge is not None:
#             edge_owner = self.edge_reservations.get(movement.edge)
#             if edge_owner is not None and edge_owner != agv.agv_id:
#                 score += 5.0
#
#         can_enter, _reason = self.can_enter_zone_with_movement(agv, movement)
#         if not can_enter:
#             score += 5.0
#         return score
#
#
# ---- legacy _intersection_penalty ----
#     def _intersection_penalty(self, movement: Movement) -> float:
#         penalty = 0.0
#         if self.graph.degree(movement.to_node) >= 3:
#             penalty += 1.0
#         for zone in self.conflict_zones.values():
#             if movement.to_node == zone.center_node:
#                 penalty += 2.0
#             elif movement.from_node == zone.center_node:
#                 penalty += 1.0
#         return penalty
#
#
# ---- legacy _predict_other_movements ----
#     def _predict_other_movements(self, agv: AGV) -> List[Movement]:
#         movements: List[Movement] = []
#         for other in self.agvs:
#             if other.agv_id == agv.agv_id or other.status in ("DONE", "DEADLOCK"):
#                 continue
#             next_node = self._planned_next_node_without_traffic(other) or other.current_node
#             if next_node != other.current_node and not self.graph.has_edge(other.current_node, next_node):
#                 next_node = other.current_node
#             edge = None if next_node == other.current_node else self.graph.normalize_edge(other.current_node, next_node)
#             movements.append(
#                 Movement(
#                     agv_id=other.agv_id,
#                     from_node=other.current_node,
#                     to_node=next_node,
#                     edge=edge,
#                     time=self.clock + 1,
#                     geometry=self._movement_geometry(other.current_node, next_node),
#                 )
#             )
#         return movements
#
#
