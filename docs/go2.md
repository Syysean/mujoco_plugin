# Walk-These-Ways Go2 设置

## 1. 机器人模型 (MJCF XML)

来自 [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menageri) 的 Go2 机器人 (不是 walk-these-ways):

```bash
git clone https://github.com/google-deepmind/mujoco_menagerie.git
# Import into Unreal: drag mujoco_menagerie/unitree_go2/go2.xml into Content Browser
```

## 2. 策略检查点

在该目录下放置这些文件：

- `body_latest.jit` — 主策略网络
- `adaptation_module_latest.jit` — 观测历史适应模块

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

- 机器人 XML: https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_go2
- 策略: https://github.com/Teddy-Liao/walk-these-ways-go2
- 原始论文: https://github.com/Improbable-AI/walk-these-ways (MIT 许可证)
