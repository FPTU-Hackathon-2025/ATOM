import onnxruntime as ort
import numpy as np
import cv2
import rospy

class YOLODetector:
    def __init__(self, model_path, class_names, input_size=(640, 640), conf_threshold=0.6):
        self.model_path = model_path
        self.class_names = class_names
        self.input_size = input_size
        self.conf_threshold = conf_threshold
        self.session = self._load_model()

    def _load_model(self):
        try:
            session = ort.InferenceSession(self.model_path, providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
            rospy.loginfo(f"Tải mô hình YOLO từ {self.model_path} thành công.")
            return session
        except Exception as e:
            rospy.logerr(f"Không thể tải mô hình YOLO từ '{self.model_path}': {e}")
            return None

    def numpy_nms(self, boxes, scores, iou_threshold):
        """
        Thực hiện Non-Maximum Suppression (NMS) bằng NumPy.
        :param boxes: list các bounding box, mỗi box là [x1, y1, x2, y2]
        :param scores: list các điểm tin cậy tương ứng
        :param iou_threshold: ngưỡng IoU để loại bỏ các box trùng lặp
        :return: list các chỉ số (indices) của các box được giữ lại
        """
        # Chuyển đổi sang NumPy array để tính toán vector hóa
        x1 = np.array([b[0] for b in boxes])
        y1 = np.array([b[1] for b in boxes])
        x2 = np.array([b[2] for b in boxes])
        y2 = np.array([b[3] for b in boxes])

        areas = (x2 - x1 + 1) * (y2 - y1 + 1)
        # Sắp xếp các box theo điểm tin cậy giảm dần
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)

            # Tính toán IoU (Intersection over Union)
            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            w = np.maximum(0.0, xx2 - xx1 + 1)
            h = np.maximum(0.0, yy2 - yy1 + 1)
            intersection = w * h

            iou = intersection / (areas[i] + areas[order[1:]] - intersection)

            # Giữ lại các box có IoU nhỏ hơn ngưỡng
            inds = np.where(iou <= iou_threshold)[0]
            order = order[inds + 1]

        return np.array(keep)

    def detect(self, image):
        """
        Thực hiện nhận diện đối tượng bằng YOLOv8 và hậu xử lý kết quả đúng cách.
        """
        if self.yolo_session is None: return []

        original_height, original_width = image.shape[:2]

        img_resized = cv2.resize(image, self.YOLO_INPUT_SIZE)
        img_data = np.array(img_resized, dtype=np.float32) / 255.0
        img_data = np.transpose(img_data, (2, 0, 1))  # HWC to CHW
        input_tensor = np.expand_dims(img_data, axis=0)  # Add batch dimension

        input_name = self.yolo_session.get_inputs()[0].name
        outputs = self.yolo_session.run(None, {input_name: input_tensor})

        # Lấy output thô, output của YOLOv8 thường có shape (1, 84, 8400) hoặc tương tự
        # Chúng ta cần transpose nó thành (1, 8400, 84) để dễ xử lý
        predictions = np.squeeze(outputs[0]).T

        # Lọc các box có điểm tin cậy (objectness score) thấp
        # Cột 4 trong predictions là điểm tin cậy tổng thể của box
        scores = np.max(predictions[:, 4:], axis=1)
        predictions = predictions[scores > self.YOLO_CONF_THRESHOLD, :]
        scores = scores[scores > self.YOLO_CONF_THRESHOLD]

        if predictions.shape[0] == 0:
            rospy.loginfo("YOLO không phát hiện đối tượng nào vượt ngưỡng tin cậy.")
            return []

        # Lấy class_id có điểm cao nhất
        class_ids = np.argmax(predictions[:, 4:], axis=1)

        # Lấy tọa độ box và chuyển đổi về ảnh gốc
        x, y, w, h = predictions[:, 0], predictions[:, 1], predictions[:, 2], predictions[:, 3]

        # Tính toán tỷ lệ scale để chuyển đổi tọa độ
        x_scale = original_width / self.YOLO_INPUT_SIZE[0]
        y_scale = original_height / self.YOLO_INPUT_SIZE[1]

        # Chuyển từ [center_x, center_y, width, height] sang [x1, y1, x2, y2]
        x1 = (x - w / 2) * x_scale
        y1 = (y - h / 2) * y_scale
        x2 = (x + w / 2) * x_scale
        y2 = (y + h / 2) * y_scale

        # Chuyển thành list các box và scores
        boxes = np.column_stack((x1, y1, x2, y2)).tolist()

        # 4. Thực hiện Non-Maximum Suppression (NMS)
        # Đây là một bước cực kỳ quan trọng để loại bỏ các box trùng lặp
        # OpenCV cung cấp một hàm NMS hiệu quả
        nms_threshold = 0.45  # Ngưỡng IOU để loại bỏ box
        indices = self.numpy_nms(np.array(boxes), scores, nms_threshold)

        if len(indices) == 0:
            rospy.loginfo("YOLO: Sau NMS, không còn đối tượng nào.")
            return []

        # 5. Tạo danh sách kết quả cuối cùng
        final_detections = []
        for i in indices.flatten():
            final_detections.append({
                'class_name': self.YOLO_CLASS_NAMES[class_ids[i]],
                'confidence': float(scores[i]),
                'box': [int(coord) for coord in boxes[i]]  # Chuyển tọa độ sang int
            })

        rospy.loginfo(f"YOLO đã phát hiện {len(final_detections)} đối tượng cuối cùng.")
        return final_detections