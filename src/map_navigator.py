# map_navigator.py
# import rospy

import json
import networkx as nx
import math
from itertools import permutations

import requests

DOMAIN = "https://hackathon2025-dev.fpt.edu.vn"
url = f"{DOMAIN}/api/maps/get_active_map/"
token = "7437f6b784f59029d38b71799c713c72"

class MapNavigator:
    def __init__(self, map_type):
        """
        Khởi tạo bộ điều hướng bản đồ.
        - Tải bản đồ từ file JSON.
        - Xây dựng một đồ thị có hướng (directed graph) bằng networkx.
        - Xác định điểm bắt đầu và kết thúc.
        """
        self.graph = nx.DiGraph()
        self.nodes_data = {}
        self.start_node = None
        self.end_node = None
        self._opposite_direction = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
        self._load_map(map_type)

    # TODO - NOTE: khi tạo các file problem khác nhau
    #  thì truyền tham số map_type khác nhau (a, b, c)
    def _load_map(self, map_type):
        try:
            # URL: /api/maps/get_active_map/?token=[Team’s
            # Token]&map_type=[Map type]
            # Method: GET
            # Content-Type: application/json

            headers = {
                "Content-Type": "application/json",
                "User-Agent": "Python-requests/2.28.1"
            }
            response = requests.get(url, params={"map_type": map_type, "token": token}, headers=headers)
            response.raise_for_status()  # Raise error for non-200 status codes

            data = response.json()

            if not data:
                raise ValueError("No data found in the API response.")

            for node in data['nodes']:
                self.nodes_data[node['id']] = node
                self.graph.add_node(node['id'], **node)

                if node['type'].lower() == 'start':
                    self.start_node = node['id']
                elif node['type'].lower() == 'end':
                    self.end_node = node['id']

            for edge in data['edges']:
                self.graph.add_edge(edge['source'], edge['target'], label=edge['label'])
                opposite_label = self._opposite_direction.get(edge['label'])
                if opposite_label:
                    self.graph.add_edge(edge['target'], edge['source'], label=opposite_label)

        except requests.exceptions.RequestException as e:
            raise RuntimeError(f"Failed to fetch map data: {e}")
        except ValueError as e:
            raise RuntimeError(f"Invalid map data: {e}")

    def _heuristic(self, node1_id, node2_id):
        """Hàm heuristic (khoảng cách Euclid) cho thuật toán A*."""
        pos1 = self.nodes_data[node1_id]
        pos2 = self.nodes_data[node2_id]
        return math.sqrt((pos1['x'] - pos2['x'])**2 + (pos1['y'] - pos2['y'])**2)

    def get_direction_label_between(self, u, v):
        """Trả về nhãn hướng (N/E/S/W) của cạnh u->v, hoặc None nếu không có."""
        data = self.graph.get_edge_data(u, v)
        return data.get('label') if data else None

    def find_path(
    self,
    start_node_id,
    end_node_id,
    banned_edges=None,
    *,
    no_reuse_edges=True,
    forbid_reverse=True
):
        """
        Tìm đường đi từ start -> tất cả các load nodes (nếu có) -> end bằng A*,
        và tránh đi lại các đoạn đã đi hoặc quay đầu.

        :param start_node_id: node bắt đầu
        :param end_node_id: node kết thúc
        :param banned_edges: list [(u, v)] cạnh cấm ngay từ đầu
        :param no_reuse_edges: nếu True, không dùng lại cạnh vừa đi (cả 2 chiều)
        :param forbid_reverse: nếu True, tối thiểu cấm cạnh ngược chiều vừa đi (chống quay đầu)
        :return: list node id tạo thành đường đi, hoặc None nếu không có
        """
        import copy
        from itertools import permutations
        import networkx as nx

        # ---- helpers ----
        def _apply_bans(g, edges):
            if edges:
                g.remove_edges_from(edges)

        def _remove_traversed_edges(g, path):
            """Sau khi có sub_path, loại bỏ cạnh vừa đi để tránh retrace/quay đầu."""
            for i in range(len(path) - 1):
                u, v = path[i], path[i+1]
                # luôn loại tối thiểu cạnh ngược chiều để tránh quay đầu
                if forbid_reverse and g.has_edge(v, u):
                    g.remove_edge(v, u)
                # nếu cấm dùng lại đoạn vừa đi, loại cả 2 chiều (xem như đường đã đi rồi)
                if no_reuse_edges:
                    if g.has_edge(u, v):
                        g.remove_edge(u, v)
                    if g.has_edge(v, u):
                        g.remove_edge(v, u)

        # ---- chuẩn bị đồ thị cơ sở (copy để không đụng self.graph) ----
        base_graph = self.graph.copy()
        if banned_edges:
            _apply_bans(base_graph, banned_edges)

        # ---- lấy danh sách load nodes ----
        load_nodes = [n for n, d in self.nodes_data.items() if d["type"].lower() == "load"]

        # ---- không có load: chạy A* một phát, vẫn áp ràng buộc không quay đầu khi cần ----
        if not load_nodes:
            try:
                path = nx.astar_path(base_graph, start_node_id, end_node_id, heuristic=self._heuristic)
                # tuỳ chọn: nếu muốn đảm bảo không quay đầu trong path đơn này, có thể kiểm tra pattern (u,v)->(v,u) liền kề
                # nhưng A* trên đồ thị có hướng hiếm khi tạo U-turn liền kề. Bỏ qua để giữ nguyên tốc độ.
                return path
            except nx.NetworkXNoPath:
                return None

        # ---- có load: thử mọi hoán vị load order, mỗi order chạy trên 1 bản sao đồ thị còn lại ----
        best_path = None
        best_length = float("inf")

        for order in permutations(load_nodes):
            # đồ thị làm việc cho order này (sẽ bị cắt cạnh dần)
            g_work = base_graph.copy()
            candidate_path = []
            valid = True

            # 1) start -> load đầu
            try:
                sub_path = nx.astar_path(g_work, start_node_id, order[0], heuristic=self._heuristic)
            except nx.NetworkXNoPath:
                continue
            candidate_path.extend(sub_path)
            _remove_traversed_edges(g_work, sub_path)

            # 2) đi qua các load tiếp theo
            for i in range(len(order) - 1):
                try:
                    sub_path = nx.astar_path(g_work, order[i], order[i+1], heuristic=self._heuristic)
                    # nối, bỏ node trùng
                    candidate_path.extend(sub_path[1:])
                    _remove_traversed_edges(g_work, sub_path)
                except nx.NetworkXNoPath:
                    valid = False
                    break
            if not valid:
                continue

            # 3) load cuối -> end
            try:
                sub_path = nx.astar_path(g_work, order[-1], end_node_id, heuristic=self._heuristic)
                candidate_path.extend(sub_path[1:])
                _remove_traversed_edges(g_work, sub_path)
            except nx.NetworkXNoPath:
                continue

            # đánh giá (dùng độ dài node hoặc có thể dùng tổng trọng số nếu bạn có weight)
            if len(candidate_path) < best_length:
                best_length = len(candidate_path)
                best_path = candidate_path

        return best_path


    def get_next_direction_label(self, current_node_id, path):
        """
        Từ đường đi đã cho, xác định hướng đi tiếp theo (N, E, S, W) từ node hiện tại.
        """
        if not path or current_node_id not in path:
            return None
        
        current_index = path.index(current_node_id)
        if current_index + 1 >= len(path):
            return None # Đã đến đích

        next_node_id = path[current_index + 1]
        edge_data = self.graph.get_edge_data(current_node_id, next_node_id)
        # rospy.loginfo(f"Next direction from {current_node_id} to {next_node_id}: {edge_data.get('label', None)}")
        print("Next direction from", current_node_id, "to", next_node_id, ":", edge_data.get('label', None))
        return edge_data.get('label', None)
    
    def get_neighbor_by_direction(self, current_node_id, direction_label):
        """
        Tìm ID của node hàng xóm từ node hiện tại theo một hướng cho trước.
        :param current_node_id: ID của node hiện tại.
        :param direction_label: Hướng đi ('N', 'E', 'S', 'W').
        :return: ID của node hàng xóm, hoặc None nếu không có.
        """
        for neighbor in self.graph.neighbors(current_node_id):
            edge_data = self.graph.get_edge_data(current_node_id, neighbor)
            if edge_data and edge_data.get('label') == direction_label:
                # rospy.loginfo(f"Neighbor of {current_node_id} in direction {direction_label}: {neighbor}")
                print("Neighbor of", current_node_id, "in direction", direction_label, ":", neighbor)
                return neighbor
        return None