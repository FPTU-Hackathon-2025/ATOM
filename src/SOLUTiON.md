# Mô tả giải pháp – ATOM
## Tổng quan
- Ngôn ngữ sử dụng: Python
- Thư viện chính: 
+ rospy, sensor_msgs, std_msgs, std_srvs – giao tiếp ROS.
+ opencv-python – xử lý ảnh (line, sign, camera).
+ numpy – xử lý dữ liệu số, ma trận.
+ jetbot – điều khiển robot JetBot.
+ onnxruntime – chạy mô hình nhận diện (YOLO).
+ paho-mqtt – giao tiếp MQTT.
+ networkx – xây dựng và tìm đường trong graph.
+ requests – gửi kết quả về server.
- Cấu trúc tổng thể: Tách file vì:
+ Dễ bảo trì & debug: Mỗi file phụ trách một chức năng riêng (xử lý node, OCR, QR, YOLO, điều hướng map). Khi lỗi xảy ra sẽ dễ khoanh vùng.
+ Chia việc cho nhiều thành viên: Người phụ trách OCR chỉ cần làm việc trong ocr_text.py, người khác có thể làm node_handler.py, không bị chồng chéo.
+ Tái sử dụng module: Một số module như qr_reader.py hoặc ocr_text.py có thể dùng độc lập để test mà không cần chạy cả hệ thống.
+ Phù hợp với yêu cầu nhiều round: Round 1 chỉ cần map_navigator, Round 3 cần thêm OCR/QR/YOLO. Tách file giúp kích hoạt đúng module mà không ảnh hưởng các phần khác.
## Bài 1

Ý tưởng/thuật toán:

Bài toán được mô hình hóa thành đồ thị (graph), trong đó:

Node = các giao điểm (phát hiện bằng LiDAR + camera).

Edge = đoạn đường thẳng giữa 2 node.

Sử dụng thuật toán A* để tìm đường đi ngắn nhất từ node xuất phát đến node đích.

Robot vừa đi vừa xây dựng đồ thị (nếu bản đồ chưa có sẵn), sau đó tính đường ngắn nhất và điều hướng theo kết quả.

Độ phức tạp thời gian/bộ nhớ (Big-O):

Thuật toán A* trên đồ thị có V node và E cạnh: O(E log V).

Bộ nhớ lưu trữ đồ thị: O(V + E).

Các trường hợp biên đã xử lý:

Trường hợp dead-end (ngõ cụt) → backtrack để tìm đường khác.

Lý do chọn giải pháp này thay vì phương án khác:

A* là thuật toán chuẩn cho tìm đường tối ưu, dễ cài đặt với thư viện networkx.

Đảm bảo tìm đường ngắn nhất thay vì đi mò theo heuristic.

Có thể mở rộng để xử lý bản đồ động hoặc thêm trọng số (ví dụ: đường hẹp, robot đối thủ).
## Bài 2

Ý tưởng/thuật toán:

Sử dụng thuật toán tìm đường A* để tính toán đường đi ngắn nhất như ở Bài 1.

Tích hợp thêm chức năng nhận diện biển thông tin bằng camera (OpenCV + pyzbar hoặc OCR như Tesseract).

Khi robot di chuyển, nó sẽ dừng lại khi gặp biển, quét và giải mã nội dung biển (ví dụ: cấm đi, rẽ trái, rẽ phải, hoặc thông báo gói hàng).

Nội dung biển sẽ được chuyển thành ràng buộc mới cho thuật toán tìm đường (ví dụ: nếu biển “cấm đi thẳng” thì cạnh/đường đó bị loại khỏi đồ thị, robot phải tìm đường thay thế).

Kết hợp ROS topic/subscriber để xử lý đồng bộ:

Node Camera → publish hình ảnh.

Node Vision → decode biển thông tin.

Node Navigator → cập nhật bản đồ/đồ thị theo thông tin mới và tính lại đường đi.

Độ phức tạp:

Phần tìm đường: O(V log V + E) với A* (V = số node, E = số cạnh).

Phần nhận diện biển: độ phức tạp phụ thuộc vào thuật toán OCR/decode, thường O(n) theo số pixel/hình ảnh.

Tổng thể vẫn chấp nhận được vì số node trong bản đồ robot thường không lớn.

Trường hợp biên:

Biển bị mờ/che khuất → không đọc được → cần fallback (ví dụ: giả sử đường cấm).

Biển đưa thông tin xung đột với bản đồ cũ → cần ưu tiên thông tin biển vì đó là tình huống mới.

Nhiều biển liên tiếp → phải xử lý ưu tiên, không bỏ sót.

Trường hợp robot vừa tính xong đường mới nhưng phát hiện thêm biển → phải hủy và tính lại đường ngay.

Ghi chú tối ưu hoá:

Thay vì tính lại toàn bộ đường đi từ đầu, có thể dùng thuật toán incremental search (ví dụ: D* Lite) để cập nhật đường đi khi phát hiện thay đổi.

Bộ giải mã biển nên chạy trên thread riêng để không làm chậm navigation.

Có thể cache kết quả biển đã quét để không lặp lại khi quay lại cùng một node.

## Bài 3

Ý tưởng/thuật toán:

Robot không có sẵn bản đồ hoặc JSON node list.

Di chuyển dựa hoàn toàn vào nhận diện biển báo:

Biển có thể có thông tintin, thông tin cấm, hoặc đích đến (Finish).

Khi gặp ngã rẽ, robot dừng lại quét biển báo và thực hiện theo hướng dẫn.

Quá trình lặp lại cho đến khi phát hiện biển “Finish” hoặc dấu hiệu kết thúc hành trình.

Có thể kết hợp Line Following (dùng sensor dải đen) để đi theo đường kẻ, còn quyết định rẽ dựa vào biển báo.

Tư duy: thay vì tìm đường ngắn nhất, robot làm theo một thuật toán tuần tự định hướng (rule-based navigation) dựa vào dữ liệu từ camera.

Độ phức tạp:

Phần nhận diện biển: O(n) theo số pixel/hình ảnh mỗi lần quét.

Logic điều hướng: O(k) với k là số biển báo trên đường (mỗi biển xử lý đúng một lần).

Tổng thể rất nhẹ, độ phức tạp phụ thuộc vào số biển báo cần đọc.

Trường hợp biên:

Biển báo bị mờ, không đọc được → robot có thể dừng lại thử quét nhiều lần hoặc báo lỗi.

Có nhiều biển cùng lúc.

Đường giao cắt nhưng không có biển báo → robot không biết chọn hướng nào (trường hợp xấu).

Hạn chế còn lại:

Không có bản đồ → robot không thể tối ưu quãng đường, chỉ đi “mù” theo hướng dẫn.

Dễ bị ảnh hưởng bởi lỗi OCR/nhận dạng biển.

Không xử lý được tình huống đường cụt nếu không có biển báo.

Không thể lên kế hoạch tổng thể, chỉ có thể phản ứng cục bộ theo từng biển.

## Khác

Hướng cải tiến nếu có thêm thời gian:

Tích hợp SLAM (Simultaneous Localization and Mapping): Robot vừa đi vừa tự xây dựng bản đồ, không cần phụ thuộc hoàn toàn vào JSON hoặc biển báo.

Cải thiện nhận diện thị giác:

Sử dụng deep learning model nhẹ (MobileNet, YOLOv5n) để tăng độ chính xác khi nhận diện biển báo, kể cả trong điều kiện ánh sáng yếu hoặc biển mờ.

Kết hợp OCR và shape detection để tránh lỗi đọc sai ký tự.

Tối ưu điều hướng:

Thay vì chỉ dựa vào biển, robot có thể kết hợp với cảm biến khoảng cách (LiDAR, siêu âm) để tránh vật cản và tìm đường thay thế.

Khi có nhiều đường đi hợp lệ, robot chọn hướng tối ưu (ví dụ: ngắn hơn, ít rẽ hơn).

Song song xử lý: Tách riêng thread cho camera, line following và decision making để tăng tốc độ phản ứng.

Khả năng học tập: Dùng reinforcement learning để robot học cách đi tốt hơn theo thời gian.

Môi trường mô phỏng: Nếu có thêm thời gian, có thể dựng mô phỏng Gazebo/Unity để test trước khi đưa robot chạy thật.