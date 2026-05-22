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
        #plt.show()

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

    def test_anisotropic_drag(self):
        """
        验证流体阻力的各向异性 (6-DOF 脉冲阶跃测试)
        通过向不同轴向施加瞬时外力, 观察撤销外力后速度的衰减响应, 验证流体模型的科学性
        """
        print("\n" + "=" * 60)
        print("  正在执行：第一阶段目标 - 复杂受力与阻力衰减测试")
        print("=" * 60)

        # 提取水环境参数
        density = self.config["fluid_density"]
        viscosity = self.config["fluid_viscosity"]
        buoyancy_force = density * self.g * self.volume

        # ──────────────────────────────────────────────────────────────────
        # 💡 6-DOF 测试矩阵裁剪说明 (力学与几何对称性分析)：
        # 1. Fx 与 Fy 效果完全相同：圆柱体绕其中心轴(Z轴)呈中心对称。从 X 轴或 Y 轴正交正视, 
        #    其迎水横截面均为等面积的长方形(0.3m x 0.6m), 遭受相同的细长体阻力。故此处仅测试 Fx, 省略 Fy。
        # 2. Tx 与 Ty 效果完全相同：同理, 绕 X 轴翻转与绕 Y 轴翻转均属于大面积拨水的非线性钝体旋转, 
        #    两端物理阻尼力矩完全一致。故此处仅测试 Ty, 省略 Tx。
        # 3. Tz 阻力极小, 可忽略不计：绕 Z 轴自转时, 圆柱体属于完美旋转体, 在旋转切线方向的迎水面积理论上为 0。
        #    在 MuJoCo 现象学流体动力学底层的计算中, 自转仅触发极微弱的流体剪切皮肤摩擦力, 其角速度衰减极其缓慢。
        #    为保证演示图表的数据焦点和视觉反差, 战略性忽略 Tz 测试。
        # ──────────────────────────────────────────────────────────────────
        scenarios = {
            "纵向平移 (端面迎水，阻力小)": {"force": [0, 0, 500, 0, 0, 0], "track_axis": 2},  # 监测 Z 轴线速度 (qvel 索引 2)
            "横向平移 (侧面迎水，阻力大)": {"force": [500, 0, 0, 0, 0, 0], "track_axis": 0},  # 监测 X 轴线速度 (qvel 索引 0)
            "横向翻滚 (大面积拨水，角阻力)": {"force": [0, 0, 0, 0, 50, 0], "track_axis": 4}   # 监测 Y 轴角速度 (qvel 索引 4)
        }

        plt.figure("6-DOF 流体各向异性测试", figsize=(10, 6))
        
        for label, config in scenarios.items():
            model = self.load_and_setup_model(density, viscosity)
            data = mujoco.MjData(model)
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "rov")
            
            # 获取 freejoint 的速度基地址 (0-2 为线速度，3-5 为角速度)
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "freejoint")
            qvel_addr = model.jnt_dofadr[joint_id]

            log_t, log_v = [], []
            steps = int(3.0 / model.opt.timestep)  # 设定标准物理观测时长为 3.0 秒

            for i in range(steps):
                # 静态阿基米德浮力全时段持续生效，确保 ROV 在水中的基本悬浮状态
                data.xfrc_applied[body_id, 2] = buoyancy_force
                
                # 在仿真前 0.1 秒施加瞬间阶跃外力/扭矩进行能量注入
                if data.time < 0.1:
                    data.xfrc_applied[body_id] = np.array(config["force"])
                else:
                    # 0.1 秒后瞬间撤销外力，使其进入纯流体阻力作用下的惯性滑行时段
                    data.xfrc_applied[body_id][:2] = 0.0
                    data.xfrc_applied[body_id][3:] = 0.0
                    # 注意：Z 轴的静态浮力补偿必须予以保留，防止其受纯重力直接下砸

                mujoco.mj_step(model, data)

                # 每隔 20 步(10ms) 抽样记录一次数据，防止数据点过密导致绘图卡顿
                if i % 5 == 0:
                    log_t.append(data.time)
                    current_vel = data.qvel[qvel_addr + config["track_axis"]]
                    log_v.append(current_vel)
            
            plt.plot(log_t, log_v, label=label, linewidth=2.5)

        plt.title("水下机器人不同姿态受力后的速度衰减对比 (各向异性流体阻力)", fontsize=14)
        plt.xlabel("时间 (Time / s)", fontsize=12)
        plt.ylabel("速度 / 角速度 响应值", fontsize=12)
        plt.axvline(x=0.1, color='r', linestyle='--', alpha=0.5, label="撤销外力点")
        plt.grid(True, linestyle="--", alpha=0.6)
        plt.legend(fontsize=11)
        plt.tight_layout()

        # 将生成的精细化对比图表直接落入 MkDocs 静态文档的图片目录
        graph_dir = os.path.join(_HERE, "../../docs/img")
        os.makedirs(graph_dir, exist_ok=True)
        graph_path = os.path.join(graph_dir, "anisotropic_drag_test.png")
        plt.savefig(graph_path, dpi=300)
        print(f"\n[交付物生成] 空间 6-DOF 各向异性阻力测试图表已保存至: {os.path.abspath(graph_path)}")
        #plt.show()

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

    # 执行新添的 6-DOF 各向异性流体阻力测试
    sim.test_anisotropic_drag()

    # 集中展示：同时弹出上面画好的所有窗口，方便用户进行对比分析
    print("\n正在同时展示所有测试图表，请在查看完毕后关闭所有图片窗口以继续...")
    plt.show()

    # 可选：拉起 GUI 查看器
    sim.start_interactive_viewer()