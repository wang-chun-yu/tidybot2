# 安装
1. 安装Miniforge（包含mamba）：
```
cd ~/Downloads
wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh

# 安装到标准位置
bash Miniforge3-Linux-x86_64.sh

# 重新加载配置
source ~/.bashrc

# 验证
mamba --version
conda --version
```
2. 创建环境
```
mamba create -n tidybot2 python=3.10.14
mamba activate tidybot2
pip install -r requirements.txt
```

## 启动
```
eval "$(mamba shell hook --shell bash)"
mamba activate
mamba activate tidybot2
```