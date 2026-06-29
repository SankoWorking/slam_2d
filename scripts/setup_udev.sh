#!/bin/bash

# 1. 将规则文件复制到系统目录（假设你的规则文件存放在代码仓库的 udev/ 文件夹下）
sudo cp $(pwd)/robot_usb.rules /etc/udev/rules.d/

# 2. 刷新规则
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Udev rules installed and triggered successfully!"