"""
水下机器人力学模拟 (MuJoCo) 示例
=====================================
验证目标：MuJoCo 能否正确模拟水下浮力 + 阻力行为
参考论文：Simple Models, Real Swimming (ETH Zurich, 2025)
参考文档：https://docs.mujoco.cn/en/stable/computation/fluid.html
"""

import os
import json
import mujoco
import mujoco.viewer
import numpy as np
import matplotlib.pyplot as plt

# ── 解决 Windows 中文字体显示问题 ──────────────────────────
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

# ── 路径锚定：相对脚本位置定位资源文件 ────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_CONFIG = os.path.join(_HERE, "data/config.json")
_DEFAULT_MODEL  = os.path.join(_HERE, "data/rov_base.xml")


class UnderwaterSimulation:
    def __init__(self, config_path=_DEFAULT_CONFIG, model_path=_DEFAULT_MODEL):
        # 读取参数化配置文件
        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = json.load(f)
        self.model_path = model_path

        # 预先计算几何特征
        rov_cfg = self.config["rov_body"]
        self.volume = np.pi * (rov_cfg["radius"] ** 2) * (2 * rov_cfg["half_length"])
        self.mass   = rov_cfg["mass"]
        self.g      = self.config["gravity"]

    def load_and_setup_model(self, density, viscosity):
        """动态加载 XML 并注入配置文件中的流体力学参数"""
        model = mujoco.MjModel.from_xml_path(self.model_path)

        # 动态注入全局环境参数
        model.opt.gravity    = np.array([0, 0, -self.g])
        model.opt.timestep   = self.config["simulation_timestep"]
        model.opt.density    = density
        model.opt.viscosity  = viscosity

        # 动态重载几何体物理特征
        rov_cfg = self.config["rov_body"]
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, rov_cfg["geom_name"])
        model.geom_size[geom_id]       = [rov_cfg["radius"], rov_cfg["half_length"], 0]
        model.body_mass[model.geom_bodyid[geom_id]] = self.mass
        model.geom_fluid[geom_id, :5] = np.array(rov_cfg["fluid_coef"])

        return model

    def run_scenario(self, density, viscosity, apply_buoyancy):
        """执行特定场景的仿真并记录数据"""
        model   = self.load_and_setup_model(density, viscosity)
        data    = mujoco.MjData(model)
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rov")

        # 核心发现：计算静态阿基米德浮力 F = ρ × g × V
        buoyancy_force = density * self.g * self.volume if apply_buoyancy else 0.0

        log_t, log_z, log_vz = [], [], []
        steps = int(self.config["duration_seconds"] / model.opt.timestep)

        for i in range(steps):
            # 关键步骤：在每个仿真步手动向 Z 轴正方向注入浮力
            if apply_buoyancy and density > 0:
                data.xfrc_applied[body_id, 2] = buoyancy_force

            mujoco.mj_step(model, data)

            # 每隔 10ms 记录一次数据用于绘图
            if i % 5 == 0:
                log_t.append(data.time)
                log_z.append(data.sensor("pos").data[2])
                log_vz.append(data.sensor("vel").data[2])

        return np.array(log_t), np.array(log_z), np.array(log_vz)

    def generate_report(self, results):
        """自动绘制折线图"""
        plt.figure(figsize=(12, 6))

        # 子图 1：Z 轴垂直位移曲线
        plt.subplot(1, 2, 1)
        for label, (t, z, _) in results.items():
            plt.plot(t, z, label=label, linewidth=2)
        plt.title("ROV 垂直位置变化 (Position Z)", fontsize=12)
        plt.xlabel("时间 (Time / s)")
        plt.ylabel("深度 (Depth / m)")
        plt.grid(True, linestyle="--", alpha=0.6)
        plt.legend()

        # 子图 2：Z 轴垂直速度曲线
        plt.subplot(1, 2, 2)
        for label, (t, _, vz) in results.items():
            plt.plot(t, vz, label=label, linewidth=2)
        plt.title("ROV 垂直速度变化 (Velocity Z)", fontsize=12)
        plt.xlabel("时间 (Time / s)")
        plt.ylabel("速度 (Velocity / m/s)")
        plt.grid(True, linestyle="--", alpha=0.6)
        plt.legend()

        plt.tight_layout()

        # 保存至 docs/img 目录
        graph_dir  = os.path.join(_HERE, "../../docs/img")
        os.makedirs(graph_dir, exist_ok=True)
        graph_path = os.path.join(graph_dir, "underwater_test_results.png")
        plt.savefig(graph_path, dpi=300)
        print(f"\n[数据反馈] 量化对比折线图已保存至: {os.path.abspath(graph_path)}")
        plt.show()

    def start_interactive_viewer(self):
        """启动带有完整浮力补偿的水下互动可视化界面"""
        ans = input("\n是否启动 MuJoCo 互动可视化查看器？(y/n): ").strip().lower()
        if ans != 'y':
            return

        print("正在启动查看器（包含手动浮力机制）...")
        model   = self.load_and_setup_model(
            self.config["fluid_density"],
            self.config["fluid_viscosity"]
        )
        data    = mujoco.MjData(model)
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rov")
        buoyancy_force = self.config["fluid_density"] * self.g * self.volume

        def buoyancy_callback(m, d):
            d.xfrc_applied[body_id, 2] = buoyancy_force

        # 注册手动外力回调并打开查看器
        mujoco.set_mjcb_control(buoyancy_callback)
        mujoco.viewer.launch(model, data)


# ─────────────────────────────────────────────
# 主程序入口
# ─────────────────────────────────────────────
if __name__ == "__main__":
    sim = UnderwaterSimulation()

    print("=" * 60)
    print("  水下机器人 MuJoCo 流体仿真验证 Demo")
    print("  OpenHUTB/mujoco_plugin")
    print("=" * 60)

    rov_cfg = sim.config["rov_body"]
    print(f"\n  ROV 参数：")
    print(f"    质量     = {sim.mass} kg")
    print(f"    体积     = {sim.volume:.4f} m³")
    print(f"    等效密度 = {sim.mass / sim.volume:.1f} kg/m³  （水：1000 kg/m³）")
    buoyancy = sim.config["fluid_density"] * sim.g * sim.volume
    print(f"\n  理论浮力   = {buoyancy:.1f} N")
    print(f"  重力       = {sim.mass * sim.g:.1f} N")
    print(f"  净浮力     = {buoyancy - sim.mass * sim.g:.1f} N  → 预期上浮")

    results = {}
    for _, sc in sim.config["test_scenarios"].items():
        print(f"\n▶ 正在执行: {sc['label']}")
        t, z, vz = sim.run_scenario(sc["density"], sc["viscosity"], sc["apply_buoyancy"])
        results[sc['label']] = (t, z, vz)
        dz  = z[-1] - z[0]
        tag = "↑ 上浮" if dz > 0.05 else ("↓ 下沉" if dz < -0.05 else "→ 近似悬停")
        print(f"   初始Z：{z[0]:.3f} m  →  最终Z：{z[-1]:.3f} m")
        print(f"   位移：{dz:+.3f} m  {tag}  |  末速度：{vz[-1]:+.4f} m/s")

    # 生成折线图报告
    sim.generate_report(results)

    # 可选：拉起 GUI 查看器
    sim.start_interactive_viewer()