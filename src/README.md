## Hướng dẫn chạy nhanh

Hãy kết nối vào jetbot, sau đó thực hiện lần lượt 3 câu lệnh sau trong terminal:

```sh
roslaunch jetbot_pro lidar.launch
roslaunch jetbot_pro csi_camera.launch
python3 ros_lidar_follower.py
```

---

sudo nmcli connection modify "FPT Hackathon 2025" connection.permissions ""
nmcli connection show "FPT Hackathon 2025" | grep permissions
sudo nmcli connection modify "FPT Hackathon 2025" connection.autoconnect yes
