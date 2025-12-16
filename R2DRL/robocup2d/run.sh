#!/bin/bash
#SBATCH -p ubuntu
#SBATCH -c 8
#SBATCH -w tuatara

# ========== 环境变量配置 ==========
# 设置 LD_LIBRARY_PATH，确保能找到 librcsc 等 so 库
export LD_LIBRARY_PATH=$HOME/local/lib:$HOME/.local/lib:/fsws1/h_qin/robocup/robocup/librcsc/.libs:/fsws1/h_qin/robocup/robocup/librcssclangparser/.libs:$LD_LIBRARY_PATH

# 设置 PATH（如果你将可执行文件安装到了 $HOME/local/bin）
export PATH=$HOME/local/bin:$PATH

# ========== Conda 激活 ==========
# 加载 Anaconda 环境（注意路径与 conda 版本）
source /home/h_qin/workspace6/anaconda3/etc/profile.d/conda.sh
conda activate marl

# ========== 运行主程序 ==========
echo "🚀 开始执行 test.py ..."
# python test.py
python -u test.py