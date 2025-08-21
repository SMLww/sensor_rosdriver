#!/bin/bash

# 定义颜色和格式化输出
GREEN="\033[0;32m"
RED="\033[0;31m"
YELLOW="\033[1;33m"
NC="\033[0m" # 重置颜色

# 启动MPMGS201节点并显示状态
echo -e "${YELLOW}正在启动 MPMGS201 Node...${NC}"
gnome-terminal --title="MPMGS201 Node" -- bash -c "
echo -e '${YELLOW}MPMGS201 Node 初始化中...${NC}';
if cd ~/Documents/sensor_rosdriver; then
    cd devel && source setup.bash && cd ..;
    echo -e '${GREEN}MPMGS201 环境准备完成，启动中...${NC}';
    roslaunch memsplus_sensor_ros mpmgs201.launch;
else
    echo -e '${RED}错误：无法进入 ~/Documents/sensor_rosdriver 目录${NC}';
    read -p '按Enter键退出...'
fi;
exec bash
"

# 短暂延迟，避免同时启动冲突
sleep 2

# 启动MPRID1356节点并显示状态
echo -e "${YELLOW}正在启动 MPRID1356 Node...${NC}"
gnome-terminal --title="MPRID1356 Node" -- bash -c "
echo -e '${YELLOW}MPRID1356 Node 初始化中...${NC}';
if cd ~/Documents/sensor_rosdriver; then
    cd devel && source setup.bash && cd ..;
    echo -e '${GREEN}MPRID1356 环境准备完成，启动中...${NC}';
    roslaunch memsplus_sensor_ros mprid1356.launch;
else
    echo -e '${RED}错误：无法进入 ~/Documents/sensor_rosdriver 目录${NC}';
    read -p '按Enter键退出...'
fi;
exec bash
"

# 主终端显示启动完成信息
echo -e "${GREEN}所有节点启动命令已发送${NC}"
echo -e "${YELLOW}请查看各终端窗口了解详细启动状态${NC}"