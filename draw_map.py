import json
import networkx as nx
import matplotlib.pyplot as plt
from map_navigator import MapNavigator


def draw_map(json_file):
    # Load bản đồ bằng MapNavigator
    nav = MapNavigator(json_file)
    G = nav.graph

    # Lấy vị trí node từ dữ liệu
    pos = {node_id: (data["x"], data["y"]) for node_id, data in nav.nodes_data.items()}

    # Vẽ tất cả các node
    node_colors = []
    for node_id, data in nav.nodes_data.items():
        if data["type"] == "start":
            node_colors.append("green")
        elif data["type"] == "end":
            node_colors.append("red")
        else:
            node_colors.append("lightblue")

    nx.draw(G, pos, with_labels=True, node_color=node_colors, node_size=600, font_size=10)

    # Vẽ nhãn cạnh (hướng đi N/E/S/W)
    edge_labels = nx.get_edge_attributes(G, "label")
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=8)

    # Tìm đường từ start đến end
    path = nav.find_path(nav.start_node, nav.end_node)

    if path:
        # Highlight đường đi
        path_edges = list(zip(path, path[1:]))
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color="orange", width=2)

    plt.title("Bản đồ và đường đi (Map Navigator)")
    plt.show()


if __name__ == "__main__":
    draw_map("map.json")
