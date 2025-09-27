# map_navigator.py
# import rospy

import json
import networkx as nx
import math
from itertools import permutations


class MapNavigator:
    def __init__(self, map_file_path):
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
        self._load_map(map_file_path)

    def _load_map(self, map_file_path):
        """Tải và phân tích cú pháp file JSON của bản đồ."""
        with open(map_file_path, 'r') as f:
            data = json.load(f)

        for node in data['nodes']:
            self.nodes_data[node['id']] = node
            self.graph.add_node(node['id'], **node)
            if node['type'] == 'start':
                self.start_node = node['id']
            elif node['type'] == 'end':
                self.end_node = node['id']

        for edge in data['edges']:
            # Thêm cạnh xuôi và cạnh ngược để robot có thể đi hai chiều
            self.graph.add_edge(edge['source'], edge['target'], label=edge['label'])
            opposite_label = self._opposite_direction.get(edge['label'])
            if opposite_label:
                self.graph.add_edge(edge['target'], edge['source'], label=opposite_label)
    
    def _heuristic(self, node1_id, node2_id):
        """Hàm heuristic (khoảng cách Euclid) cho thuật toán A*."""
        pos1 = self.nodes_data[node1_id]
        pos2 = self.nodes_data[node2_id]
        return math.sqrt((pos1['x'] - pos2['x'])**2 + (pos1['y'] - pos2['y'])**2)

    def get_direction_label_between(self, u, v):
        """Trả về nhãn hướng (N/E/S/W) của cạnh u->v, hoặc None nếu không có."""
        data = self.graph.get_edge_data(u, v)
        return data.get('label') if data else None

    def find_path(self, start_node_id, end_node_id, banned_edges=None):
        """
        Tìm đường đi từ start -> tất cả các load nodes (nếu có) -> end bằng A*.
        Nếu không có load node, hoạt động như tìm đường ngắn nhất bình thường.
        :param start_node_id: ID của node bắt đầu.
        :param end_node_id: ID của node kết thúc.
        :param banned_edges: List các cạnh (u, v) bị cấm, dùng để tìm đường lại.
        :return: List các ID node trên đường đi, hoặc None nếu không có đường.
        """
        # Chuẩn bị graph tạm (có thể loại bỏ banned_edges nếu có)
        graph_to_search = self.graph.copy()
        if banned_edges:
            graph_to_search.remove_edges_from(banned_edges)

        # Lấy danh sách load nodes
        load_nodes = [n for n, d in self.nodes_data.items() if d["type"] == "load"]

        # Trường hợp không có load node -> chạy A* như cũ
        if not load_nodes:
            try:
                return nx.astar_path(
                    graph_to_search,
                    start_node_id,
                    end_node_id,
                    heuristic=self._heuristic
                )
            except nx.NetworkXNoPath:
                return None

        # Có load nodes -> phải đi qua tất cả
        best_path = None
        best_length = float("inf")

        for order in permutations(load_nodes):
            candidate_path = []
            valid = True

            # 1. start -> load đầu tiên
            try:
                sub_path = nx.astar_path(graph_to_search, start_node_id, order[0], heuristic=self._heuristic)
            except nx.NetworkXNoPath:
                continue
            candidate_path.extend(sub_path)

            # 2. đi qua các load tiếp theo
            for i in range(len(order) - 1):
                try:
                    sub_path = nx.astar_path(graph_to_search, order[i], order[i+1], heuristic=self._heuristic)
                    candidate_path.extend(sub_path[1:])  # bỏ node trùng
                except nx.NetworkXNoPath:
                    valid = False
                    break
            if not valid:
                continue

            # 3. load cuối -> end
            try:
                sub_path = nx.astar_path(graph_to_search, order[-1], end_node_id, heuristic=self._heuristic)
                candidate_path.extend(sub_path[1:])
            except nx.NetworkXNoPath:
                continue

            # Đánh giá độ dài
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
                return neighbor
        return None