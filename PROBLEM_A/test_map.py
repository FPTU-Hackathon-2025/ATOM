from map_navigator import MapNavigator

def main():
    # Load map từ file JSON (bạn lưu JSON thành map.json)
    nav = MapNavigator("map.json")

    # Tìm đường từ node start -> end
    path = nav.find_path(nav.start_node, nav.end_node)

    if path:
        print("\nĐường đi tìm được:")
        print(path)

        print("\nChi tiết hướng đi:")
        for i in range(len(path) - 1):
            current = path[i]
            direction = nav.get_next_direction_label(current, path)
            print(f"Từ node {current} đi {direction} -> node {path[i+1]}")
    else:
        print("Không tìm thấy đường đi!")

if __name__ == "__main__":
    main()
