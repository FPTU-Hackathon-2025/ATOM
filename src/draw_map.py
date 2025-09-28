
import json
import networkx as nx
import matplotlib.pyplot as plt
import logging
from map_navigator import MapNavigator

# Cấu hình logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S"
)

def draw_map(map_type):
    logging.info(f"Đang tải bản đồ từ file: {map_type}")

    # Load bản đồ bằng MapNavigator
    nav = MapNavigator(map_type)
    G = nav.graph
    logging.info(f"Số node: {len(G.nodes)}, số cạnh: {len(G.edges)}")

    # Lấy vị trí node từ dữ liệu
    pos = {node_id: (data["x"], -data["y"]) for node_id, data in nav.nodes_data.items()}
    logging.debug(f"Vị trí node: {pos}")

    # Vẽ tất cả các node
    node_colors = []
    for node_id, data in nav.nodes_data.items():
        if data["type"] == "start":
            node_colors.append("green")
            logging.info(f"Node START: {node_id} tại ({data['x']}, {data['y']})")
        elif data["type"] == "end":
            node_colors.append("red")
            logging.info(f"Node END: {node_id} tại ({data['x']}, {data['y']})")
        elif data["type"] == "load":
            node_colors.append("purple")
            logging.info(f"Node LOAD: {node_id} tại ({data['x']}, {data['y']})")
        else:
            node_colors.append("lightblue")
            logging.debug(f"Node thường: {node_id}")

    nx.draw(G, pos, with_labels=True, node_color=node_colors, node_size=600, font_size=10)

    # Vẽ nhãn cạnh (hướng đi N/E/S/W)
    edge_labels = nx.get_edge_attributes(G, "label")
    logging.info(f"Các cạnh với nhãn: {edge_labels}")
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels, font_size=8)

    # Tìm đường từ start đến end
    path = nav.find_path(nav.start_node, nav.end_node)

    if path:
        logging.info(f"Đường đi tìm được: {path}")
        # Highlight đường đi
        path_edges = list(zip(path, path[1:]))
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, edge_color="orange", width=2)
    else:
        logging.warning("Không tìm thấy đường đi từ start đến end!")

    plt.title("Bản đồ và đường đi (Map Navigator)")
    plt.show()

if __name__ == "__main__":
    map_type = "map_z"
    draw_map(map_type)
