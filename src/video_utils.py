# video_utils.py
import cv2
import numpy as np


def draw_debug_info(controller, image):
    """Vẽ các thông tin gỡ lỗi lên một khung hình."""
    if image is None:
        return None

    debug_frame = image.copy()

    # 1. Vẽ các ROI
    cv2.rectangle(debug_frame, (0, controller.ROI_Y),
                  (controller.WIDTH - 1, controller.ROI_Y + controller.ROI_H), (0, 255, 0), 1)
    cv2.rectangle(debug_frame, (0, controller.LOOKAHEAD_ROI_Y),
                  (controller.WIDTH - 1, controller.LOOKAHEAD_ROI_Y + controller.LOOKAHEAD_ROI_H), (0, 255, 255), 1)

    # 2. Vẽ trạng thái hiện tại
    state_text = f"State: {controller.current_state.name}"
    cv2.putText(debug_frame, state_text, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    # 3. Vẽ line và trọng tâm (nếu robot đang bám line)
    if controller.current_state == controller.RobotState.DRIVING_STRAIGHT:
        line_center = controller._get_line_center(image, controller.ROI_Y, controller.ROI_H)
        if line_center is not None:
            cv2.line(debug_frame, (line_center, controller.ROI_Y),
                     (line_center, controller.ROI_Y + controller.ROI_H), (0, 0, 255), 2)

    return debug_frame


def initialize_video_writer(controller):
    try:
        frame_size = (controller.WIDTH, controller.HEIGHT)
        video_writer = cv2.VideoWriter(controller.VIDEO_OUTPUT_FILENAME,
                                       controller.VIDEO_FOURCC,
                                       controller.VIDEO_FPS,
                                       frame_size)
        if video_writer.isOpened():
            print(f"Bắt đầu ghi video vào file '{controller.VIDEO_OUTPUT_FILENAME}'")
        else:
            print("Không thể mở file video để ghi.")
            video_writer = None
    except Exception as e:
        print(f"Lỗi khi khởi tạo VideoWriter: {e}")
        video_writer = None
    return video_writer



def camera_callback(controller, image_msg):
    """Xử lý dữ liệu ảnh từ camera."""
    try:
        if image_msg.encoding.endswith('compressed'):
            np_arr = np.frombuffer(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:
            cv_image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, -1)
        if 'rgb' in image_msg.encoding:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        controller.latest_image = cv2.resize(cv_image, (controller.WIDTH, controller.HEIGHT))
    except Exception as e:
        print(f"Lỗi chuyển đổi ảnh: {e}")
