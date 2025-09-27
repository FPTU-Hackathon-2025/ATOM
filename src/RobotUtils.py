# utils.py
import rospy


class RobotUtils:
    """
    Class tiện ích để:
    1. Quản lý tài nguyên robot (cleanup)
    2. Chuyển đổi hướng giữa tuyệt đối và tương đối
    """

    def __init__(self, label_to_direction_enum, robot=None, video_writer=None, detector=None, mqtt_client=None):
        self.LABEL_TO_DIRECTION_ENUM = label_to_direction_enum
        self.robot = robot
        self.video_writer = video_writer
        self.detector = detector
        self.mqtt_client = mqtt_client

    # =========================
    # Mapping hướng
    # =========================
    def map_absolute_to_relative(self, target_direction_label, current_robot_direction):
        """
        Chuyển đổi hướng tuyệt đối ('N', 'E', 'S', 'W') thành hành động tương đối
        ('straight', 'left', 'right', 'turn_around').
        """
        target_dir = self.LABEL_TO_DIRECTION_ENUM.get(target_direction_label)
        if target_dir is None:
            return None

        current_idx = current_robot_direction.value
        target_idx = target_dir.value
        diff = (target_idx - current_idx + 4) % 4

        if diff == 0:
            return 'straight'
        elif diff == 1:
            return 'right'
        elif diff == 3:
            return 'left'
        else:
            return 'turn_around'

    def map_relative_to_absolute(self, relative_action, current_robot_direction):
        """
        Chuyển đổi hành động tương đối ('straight', 'left', 'right') thành hướng tuyệt đối ('N', 'E', 'S', 'W').
        """
        current_idx = current_robot_direction.value
        if relative_action == 'straight':
            target_idx = current_idx
        elif relative_action == 'right':
            target_idx = (current_idx + 1) % 4
        elif relative_action == 'left':
            target_idx = (current_idx - 1 + 4) % 4
        else:
            return None

        for label, direction in self.LABEL_TO_DIRECTION_ENUM.items():
            if direction.value == target_idx:
                return label
        return None

    # =========================
    # Cleanup resources
    # =========================
    def cleanup(self):
        """
        Dừng robot, đóng video, dừng detector, ngắt MQTT và giải phóng tài nguyên.
        """
        rospy.loginfo("Dừng robot và giải phóng tài nguyên...")

        if hasattr(self, 'robot') and self.robot is not None:
            self.robot.stop()

        if hasattr(self, 'video_writer') and self.video_writer is not None:
            self.video_writer.release()
            rospy.loginfo("Đã lưu và đóng file video.")

        if hasattr(self, 'detector') and self.detector is not None:
            self.detector.stop_scanning()

        if hasattr(self, 'mqtt_client') and self.mqtt_client is not None:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

        rospy.loginfo("Đã giải phóng tài nguyên. Chương trình kết thúc.")
