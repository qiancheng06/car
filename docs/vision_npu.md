# Atlas NPU 视觉运行

模型：`~/racecar/vision_data/models/runs/v8_exp/weights/best_310b.om`

单独运行视觉：

```bash
cd ~/racecar
bash vision-npu-one.sh
```

MobaXterm 新终端查看框选画面：

```bash
set +u
source /opt/ros/humble/setup.bash
source ~/racecar/install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

选择 `/racecar/vision/debug_image`。

检查性能：

```bash
ros2 topic hz /racecar/vision/debug_image
ros2 topic echo /racecar/vision/status
```

当前实测约 19--20 FPS，端到端处理延迟约 44--51 ms。

CPU回退：

```bash
cd ~/racecar
bash vision-one.sh debug_image:=true
```
