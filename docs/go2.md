# Walk-These-Ways Go2 设置

## 1. 机器人模型 (MJCF XML)

The Go2 robot model comes from MuJoCo Menagerie (not walk-these-ways):

```bash
git clone https://github.com/google-deepmind/mujoco_menagerie.git
# Import into Unreal: drag mujoco_menagerie/unitree_go2/go2.xml into Content Browser
```

## 2. 策略检查点

在该目录下放置这些文件：

- `body_latest.jit` — 主策略网络
- `adaptation_module_latest.jit` — Observation history adaptation module

```bash
git clone https://github.com/Teddy-Liao/walk-these-ways-go2.git
cd walk-these-ways-go2

# 拷贝预训练检查点
cp runs/gait-conditioned-agility/pretrain-go2/train/checkpoints/body_latest.jit <this_directory>/
cp runs/gait-conditioned-agility/pretrain-go2/train/checkpoints/adaptation_module_latest.jit <this_directory>/
```

## 3. 运行

```bash
cd synthy_bridge
python src/run.py --policy go2_wtw --prefix go2
```

## 资源

- Robot XML: https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_go2
- Policy: https://github.com/Teddy-Liao/walk-these-ways-go2
- Original paper: https://github.com/Improbable-AI/walk-these-ways (MIT License)
