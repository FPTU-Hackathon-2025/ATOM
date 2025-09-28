# map_navigator.py
# import rospy

import json
import networkx as nx
import heapq
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
            
              
            # with open("./map.json", "r", encoding="utf-8") as f:
            #     data = json.load(f)
            # if not data:
            #     raise ValueError("No data found in the API response.")
          
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

    def find_path(self, start_node_id, end_node_id, banned_edges=None):
        """
        Tìm đường đi từ start -> tất cả các load nodes (nếu có) -> end bằng A* liên tục.
        Không quay lại node vừa đi.
        
        :param start_node_id: ID của node bắt đầu.
        :param end_node_id: ID của node kết thúc.
        :param banned_edges: List các cạnh (u, v) bị cấm, dùng để tìm đường lại.
        :return: List các ID node trên đường đi, hoặc None nếu không có đường.
        """
        # Chuẩn bị graph tạm (loại bỏ banned_edges nếu có)
        graph_to_search = self.graph.copy()
        if banned_edges:
            graph_to_search.remove_edges_from(banned_edges)
        
        # Lấy danh sách load nodes
        load_nodes = {n for n, d in self.nodes_data.items() if d["type"].lower() == "load"}

        # Trường hợp không có load node -> dùng A* bình thường
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

        # A* liên tục với trạng thái (current, parent, path, visited_loads)
        frontier = []
        heapq.heappush(frontier, (0, start_node_id, None, [start_node_id], set()))
        visited_states = set()  # để tránh vòng lặp, lưu (current, frozenset(visited_loads))

        while frontier:
            f_score, current, parent, path, visited_loads = heapq.heappop(frontier)

            # Cập nhật visited_loads nếu node hiện tại là load node
            if current in load_nodes:
                visited_loads = visited_loads | {current}

            # Kiểm tra điều kiện kết thúc
            if current == end_node_id and visited_loads == load_nodes:
                return path

            state_id = (current, frozenset(visited_loads))
            if state_id in visited_states:
                continue
            visited_states.add(state_id)

            # Duyệt các neighbor
            for neighbor in graph_to_search.neighbors(current):
                # Không đi lùi: neighbor không phải parent
                if neighbor == parent:
                    continue

                # Tạo path mới
                new_path = path + [neighbor]

                # Tính f_score = g + h, g = len path, h = heuristic
                g = len(new_path)
                h = self._heuristic(neighbor, end_node_id)
                heapq.heappush(frontier, (g + h, neighbor, current, new_path, visited_loads))

        # Không tìm được đường hợp lệ
        return None


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