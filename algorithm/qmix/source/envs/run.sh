#!/bin/bash
#SBATCH -p ubuntu
#SBATCH -c 2
#SBATCH -w huracan


# ========== Conda 激活 ==========
# 加载 Anaconda 环境（注意路径与 conda 版本）
source /home/h_qin/workspace7/anaconda3/etc/profile.d/conda.sh
conda activate marl

# ========== 运行主程序 ==========
echo "🚀 开始执行 main.py ..."
# python run_random.py
# python -u main.py --config=qmix --env-config=robocup
python -u draw.py 
