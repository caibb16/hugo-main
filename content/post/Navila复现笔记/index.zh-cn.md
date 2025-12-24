+++
date = '2025-12-24T12:39:23+08:00'
draft = true
title = 'Navila复现笔记'
categories = [
    "Notes",
    "VLA",
]
+++
## 论文解读
Navila: A High-Performance and Energy-Efficient Vector Length Agnostic SIMD Architecture
待定

## 实验复现(评估部分)  
### 环境配置
1. 创建模型评估的虚拟环境
```bash
conda create -n navila-eval python=3.10
conda activate navila-eval  # 后续pip安装均在该环境下进行
```

2. 安装 Habitat-Sim and Lab(v0.1.7),参考[VLA-CE设置](https://github.com/jacobkrantz/VLN-CE?tab=readme-ov-file#setup)  

由于Habitat-Sim(0.1.7)只支持python3.6~3.9，在3.10环境下需要采用源码安装

```bash
# 安装Habitat-Lab
git clone --branch v0.1.7 https://github.com/facebookresearch/habitat-lab.git
cd habitat-lab
# installs both habitat and habitat_baselines
python -m pip install -r requirements.txt
python -m pip install -r habitat_baselines/rl/requirements.txt
python -m pip install -r habitat_baselines/rl/ddppo/requirements.txt
python setup.py develop --all
```
安装Habitat-sim(v0.1.7),参考[官方源码安装指南](https://github.com/facebookresearch/habitat-sim/blob/v0.1.7/BUILD_FROM_SOURCE.md)

```bash
git clone https://github.com/facebookresearch/habitat-sim.git #（默认就是v0.1.7）
cd habitat-sim
git submodule update --init --recursive
git checkout v0.1.7
git submodule update --init --recursive #注意切换分支后可能导致部分submodule无效

python -m pip install -r requirements.txt
# 如果出现路径问题编译不成功，可能因为之前编译过了，进入到habitat-sim目录删除build(rm -rf build)

python -m pip install cmake
sudo apt-get update || true
sudo apt-get install -y --no-install-recommends \
     libjpeg-dev libglm-dev libgl1-mesa-glx libegl1-mesa-dev mesa-utils xorg-dev freeglut3-dev

# 可能出现安装libgl1-mesa-glx不成功，可以尝试单独安装以下两个依赖
# sudo apt-get install libgl1-mesa-dev
# sudo apt-get install libegl1-mesa-dev

python -m pip install --upgrade pybind11

# 注意，编译要采用headless模式
python setup.py install --headless --cmake-args="-DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_CXX_STANDARD=11"
```
编译时若遇到error: ‘uint16_t’ in namespace ‘std’ does not name a type; did you mean ‘wint_t’?报错,在编译时添加 cstdint 头文件：
```
python setup.py install --headless --cmake-args="-DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_CXX_STANDARD=11 -DCMAKE_CXX_FLAGS=-include\ cstdint"
```
解决 NumPy 兼容性问题
```bash
cd Navila
python evaluation/scripts/habitat_sim_autofix.py # replace habitat_sim/utils/common.py
```
3. 安装VLN-CE依赖
```bash
python -m pip install -r evaluation/requirements.txt
```
4. 安装VILA依赖
```bash
# 安装 FlashAttention2
python -m pip install https://github.com/Dao-AILab/flash-attention/releases/download/v2.5.8/flash_attn-2.5.8+cu122torch2.3cxx11abiFALSE-cp310-cp310-linux_x86_64.whl

# 安装 VILA (假设在项目根目录下)
python -m pip install -e .
python -m pip install -e ".[train]"
python -m pip install -e ".[eval]"

# 安装 HF 的 Transformers
python -m pip install git+https://github.com/huggingface/transformers@v4.37.2
# 替换部分包以兼容
site_pkg_path=$(python -c 'import site; print(site.getsitepackages()[0])')
cp -rv ./llava/train/transformers_replace/* $site_pkg_path/transformers/
cp -rv ./llava/train/deepspeed_replace/* $site_pkg_path/deepspeed/
```
5.修复 WebDataset 版本以实现 VLN-CE 兼容性
```bash
python -m pip install webdataset==0.1.103
```
### 数据集下载
1. 参考[VLA-CE](https://github.com/jacobkrantz/VLN-CE?tab=readme-ov-file#setup)下载R2R和RxR数据集，并解压到`evaluation/data`路径
2. 下载Matterport3D数据集，可通过[官网](https://niessner.github.io/Matterport/)申请获取，也可参考[mp3D 数据集](https://blog.csdn.net/qq_41204464/article/details/149549133)

数据应具备如下结构：`evaluation/data`
```
data/datasets
├─ RxR_VLNCE_v0
|   ├─ train
|   |    ├─ train_guide.json.gz
|   |    ├─ ...
|   ├─ val_unseen
|   |    ├─ val_unseen_guide.json.gz
|   |    ├─ ...
|   ├─ ...
├─ R2R_VLNCE_v1-3_preprocessed
|   ├─ train
|   |    ├─ train.json.gz
|   |    ├─ ...
|   ├─ val_unseen
|   |    ├─ val_unseen.json.gz
|   |    ├─ ...
data/scene_datasets
├─ mp3d
|   ├─ 17DRP5sb8fy
|   |    ├─ 17DRP5sb8fy.glb
|   |    ├─ ...
|   ├─ ...
```
### 评估运行
1. 下载checkpoint  

从[a8cheng/navila-llama3-8b-8f](https://huggingface.co/a8cheng/navila-llama3-8b-8f)下载预训练模型，并解压到`evaluation/models/navila-llama3-8b-8f`路径下,可通过以下命令下载：
```bash
# 安装 huggingface_hub
python -m pip install huggingface_hub
```
创建下载脚本`download_huggingface.py`并运行,脚本内容如下：
```bash
from huggingface_hub import snapshot_download

local_dir = snapshot_download(
    repo_id="a8cheng/navila-llama3-8b-8f",
    local_dir="~/NaVILA/navila-llama3-8b-8f",  
    cache_dir="~/NaVILA/navila-llama3-8b-8f/cache", # 改成自己的项目路径
    token="hf_******",     #  这里填写你的 HuggingFace 访问token
    endpoint="https://hf-mirror.com"   # 如果需要走镜像
)

print("模型下载到本地路径:", local_dir)
```
2. 在r2r上运行评估
```bash
# bash scripts/eval/r2r.sh CKPT_PATH NUM_CHUNKS CHUNK_START_IDX "GPU_IDS"
bash scripts/eval/r2r.sh /data/code/seu004/czd/NaVILA/navila-llama3-8b-8f 1 0 "0"
```