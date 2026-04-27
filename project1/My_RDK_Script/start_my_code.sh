#!/bin/bash

set -e

# 等设备初始化
sleep 8

# 加载 conda（必须）
source /home/sunrise/miniconda3/etc/profile.d/conda.sh

# 激活环境
conda activate dian_sai

# 进入项目目录
cd /home/sunrise/rdk_linux_dian_sai/project1/Threads

# 运行程序
/home/sunrise/miniconda3/envs/dian_sai/bin/python my_send2gimbal_systemd.py >> /home/sunrise/my_code.log 2>&1