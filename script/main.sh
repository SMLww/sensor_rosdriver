#!/bin/bash

# custom colors and reset
GREEN="\033[0;32m"
RED="\033[0;31m"
YELLOW="\033[1;33m"
NC="\033[0m" # No Color

./kill.sh

sleep 5;

# launch MPMGS201 Node and show status
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

# shot delay to avoid conflict
sleep 2

# launch MPRID1356 Node and show status
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

echo -e "${GREEN}所有节点启动命令已发送${NC}"
echo -e "${YELLOW}请查看history/loc/Rfid 是否生成最新的日志文件${NC}"