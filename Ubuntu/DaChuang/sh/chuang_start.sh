#!/bin/bash
# 这是一个运行 ROS 命令的 Shell 脚本


# 运行第一个命令：rosrun camera detection.py
echo "正在启动 detection.py..."
rosrun camera detection.py &

# 等待第一个命令启动完成
sleep 10  # 根据实际情况调整等待时间

# 运行第二个命令：roslaunch chuang_navigation diff_launch
echo "正在启动 diff_launch..."
roslaunch chuang_navigation diff_navigation

# 脚本结束
echo "所有命令已执行完成！"