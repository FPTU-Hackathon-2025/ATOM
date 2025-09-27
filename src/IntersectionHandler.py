# intersection_handler.py
import time
import rospy

class IntersectionHandler:
    def __init__(self, robot, latest_image, yolo_detector, navigator, robot_utils,
                 DIRECTIONS, current_direction_index, ANGLE_TO_FACE_SIGN_MAP,
                 PRESCRIPTIVE_SIGNS, PROHIBITIVE_SIGNS, DATA_ITEMS,
                 LABEL_TO_DIRECTION_ENUM, planned_path, banned_edges,
                 target_node_id, current_node_id, set_state_callback, turn_robot_callback, publish_data_callback):
        """
        Khởi tạo IntersectionHandler với tất cả các tham số cần thiết từ controller.
        """
        self.robot = robot
        self.latest_image = latest_image
        self.yolo_detector = yolo_detector
        self.navigator = navigator
        self.robot_utils = robot_utils

        self.DIRECTIONS = DIRECTIONS
        self.current_direction_index = current_direction_index
        self.ANGLE_TO_FACE_SIGN_MAP = ANGLE_TO_FACE_SIGN_MAP

        self.PRESCRIPTIVE_SIGNS = PRESCRIPTIVE_SIGNS
        self.PROHIBITIVE_SIGNS = PROHIBITIVE_SIGNS
        self.DATA_ITEMS = DATA_ITEMS
        self.LABEL_TO_DIRECTION_ENUM = LABEL_TO_DIRECTION_ENUM

        self.planned_path = planned_path
        self.banned_edges = banned_edges
        self.target_node_id = target_node_id
        self.current_node_id = current_node_id

        self._set_state = set_state_callback
        self.turn_robot = turn_robot_callback
        self.publish_data = publish_data_callback

    def handle_intersection(self):
        rospy.loginfo("\n[GIAO LỘ] Dừng lại và xử lý...")
        self.robot.stop()
        time.sleep(0.5)

        current_direction = self.DIRECTIONS[self.current_direction_index]
        angle_to_sign = self.ANGLE_TO_FACE_SIGN_MAP.get(current_direction, 0)
        self.turn_robot(angle_to_sign, False)
        image_info = self.latest_image
        detections = self.yolo_detector.detect(image_info)
        self.turn_robot(-angle_to_sign, False)

        prescriptive_cmds = {det['class_name'] for det in detections if det['class_name'] in self.PRESCRIPTIVE_SIGNS}
        prohibitive_cmds = {det['class_name'] for det in detections if det['class_name'] in self.PROHIBITIVE_SIGNS}
        data_items = [det for det in detections if det['class_name'] in self.DATA_ITEMS]

        # Xử lý các mục dữ liệu (QR, Math) và Publish
        for item in data_items:
            if item['class_name'] == 'qr_code':
                rospy.loginfo("Found QR Code. Publishing data...")
                self.publish_data({'type': 'QR_CODE', 'value': 'simulated_data_123'})
            elif item['class_name'] == 'math_problem':
                rospy.loginfo("Found Math Problem. Solving and publishing...")
                self.publish_data({'type': 'MATH_PROBLEM', 'value': '2+2=4'})

        # Lập kế hoạch điều hướng
        final_decision = None
        is_deviation = False

        while True:
            planned_direction_label = self.navigator.get_next_direction_label(self.current_node_id, self.planned_path)
            if not planned_direction_label:
                rospy.logerr("Lỗi kế hoạch: Không tìm thấy bước tiếp theo.")
                self._set_state('DEAD_END')
                return

            planned_action = self.robot_utils.map_absolute_to_relative(planned_direction_label, current_direction)
            rospy.loginfo(f"Kế hoạch A* đề xuất: Đi {planned_action} (hướng {planned_direction_label})")

            # Ưu tiên 1: Biển báo bắt buộc
            intended_action = None
            if 'L' in prescriptive_cmds: intended_action = 'left'
            elif 'R' in prescriptive_cmds: intended_action = 'right'
            elif 'F' in prescriptive_cmds: intended_action = 'straight'

            # Ưu tiên 2: Plan
            if intended_action is None:
                intended_action = planned_action
            else:
                if intended_action != planned_action:
                    is_deviation = True
                    rospy.logwarn(f"CHỆCH HƯỚNG! Biển báo bắt buộc ({intended_action}) khác kế hoạch ({planned_action}).")

            # Kiểm tra biển cấm
            is_prohibited = (intended_action == 'straight' and 'NF' in prohibitive_cmds) or \
                            (intended_action == 'right' and 'NR' in prohibitive_cmds) or \
                            (intended_action == 'left' and 'NL' in prohibitive_cmds)
            if is_prohibited:
                rospy.logwarn(f"Hành động dự định '{intended_action}' bị CẤM!")
                if is_deviation:
                    rospy.logerr("LỖI BẢN ĐỒ! Biển báo bắt buộc mâu thuẫn với biển báo cấm.")
                    self._set_state('DEAD_END')
                    return
                banned_edge = (self.current_node_id, self.planned_path[self.planned_path.index(self.current_node_id) + 1])
                if banned_edge not in self.banned_edges:
                    self.banned_edges.append(banned_edge)
                new_path = self.navigator.find_path(self.current_node_id, self.navigator.end_node, self.banned_edges)
                if new_path:
                    self.planned_path = new_path
                    continue
                else:
                    self._set_state('DEAD_END')
                    return

            final_decision = intended_action
            break

        # Thực thi quyết định
        if final_decision == 'straight':
            rospy.loginfo("[FINAL] Decision: Go STRAIGHT.")
        elif final_decision == 'right':
            rospy.loginfo("[FINAL] Decision: Turn RIGHT.")
            self.turn_robot(90, True)
        elif final_decision == 'left':
            rospy.loginfo("[FINAL] Decision: Turn LEFT.")
            self.turn_robot(-90, True)
        else:
            rospy.logwarn("[!!!] DEAD END! No valid paths found.")
            self._set_state('DEAD_END')
            return

        # Cập nhật target_node_id
        next_node_id = None
        if not is_deviation:
            next_node_id = self.planned_path[self.planned_path.index(self.current_node_id) + 1]
        else:
            new_robot_direction = self.DIRECTIONS[self.current_direction_index]
            executed_direction_label = None
            for label, direction_enum in self.LABEL_TO_DIRECTION_ENUM.items():
                if direction_enum == new_robot_direction:
                    executed_direction_label = label
                    break
            if executed_direction_label is None:
                rospy.logerr("Lỗi logic: Không tìm thấy label cho hướng mới.")
                self._set_state('DEAD_END')
                return
            next_node_id = self.navigator.get_neighbor_by_direction(self.current_node_id, executed_direction_label)
            if next_node_id is None:
                rospy.logerr("LỖI BẢN ĐỒ! Node tương ứng không tồn tại.")
                self._set_state('DEAD_END')
                return
            # Lập kế hoạch lại từ node mới
            new_path = self.navigator.find_path(next_node_id, self.navigator.end_node, self.banned_edges)
            if new_path:
                self.planned_path = new_path
            else:
                self._set_state('DEAD_END')
                return

        self.target_node_id = next_node_id
        rospy.loginfo(f"==> Đang di chuyển đến node tiếp theo: {self.target_node_id}")
        self._set_state('LEAVING_INTERSECTION')
