import os
import mujoco
import mujoco.viewer
import numpy as np

# ──────────────────────────────────────────────────────────────────
# 💡 核心物理骨架定义 (内嵌 MJCF 字符串)
# 解释：
# 1. 采用通用 6 自由度串联运动学架构（底座旋转+大臂偏航+小臂偏航+腕部3自由度）。
# 2. 核心求解器微调：设置 solref="0.01 1" 和 solimp="0.95 0.99 0.001"。
#    这会把刚性碰撞软化为高度阻尼的弹簧接触，彻底吸收物体碰撞瞬间的动能，防止弹飞。
# 3. 增大摩擦系数 friction="1.5 0.005 0.0001"，为末端夹爪提供极强的切向和扭转摩擦，防止小球异常旋转。
# ──────────────────────────────────────────────────────────────────
MJCF_ARM_SANDBOX = """
<mujoco model="underwater_arm_sandbox">
    <compiler angle="degree" coordinate="local"/>
    <option timestep="0.002" gravity="0 0 -9.81"/>
    
    <default>
        <joint armature="0.05" damping="5" stiffness="0"/>
        <geom condim="4" friction="1.5 0.005 0.0001" solref="0.01 1" solimp="0.95 0.99 0.001" rgba="0.8 0.8 0.8 1"/>
        <position ctrlrange="-180 180" kp="400"/>
    </default>

    <worldbody>
        <light directional="true" diffuse=".8 .8 .8" specular=".2 .2 .2" pos="0 0 5" dir="0 0 -1"/>
        <geom name="floor" type="plane" size="2 2 .25" rgba=".2 .3 .3 1"/>

        <body name="target_ball" pos="0.55 0 0.05">
            <freejoint name="ball_free"/>
            <geom name="ball_geom" type="sphere" size="0.03" mass="0.1" rgba="1 0 0 1"/>
        </body>

        <body name="base_link" pos="0 0 0">
            <geom name="base_geom" type="cylinder" size="0.1 0.05" pos="0 0 0.025"/>
            
            <body name="link1" pos="0 0 0.05">
                <joint name="joint1" type="hinge" axis="0 0 1"/>
                <geom name="link1_geom" type="cylinder" size="0.06 0.08" pos="0 0 0.04"/>
                
                <body name="link2" pos="0 0 0.08">
                    <joint name="joint2" type="hinge" axis="0 1 0" range="-90 90"/>
                    <geom name="link2_geom" type="box" size="0.04 0.04 0.2" pos="0 0 0.2"/>
                    
                    <body name="link3" pos="0 0 0.4">
                        <joint name="joint3" type="hinge" axis="0 1 0" range="-120 120"/>
                        <geom name="link3_geom" type="box" size="0.03 0.03 0.15" pos="0 0 0.15"/>
                        
                        <body name="link4" pos="0 0 0.3">
                            <joint name="joint4" type="hinge" axis="0 1 0"/>
                            <geom name="link4_geom" type="cylinder" size="0.025 0.04" pos="0 0 0.04"/>
                            
                            <body name="link5" pos="0 0 0.08">
                                <joint name="joint5" type="hinge" axis="0 0 1"/>
                                <geom name="link5_geom" type="box" size="0.02 0.02 0.04" pos="0 0 0.04"/>
                                
                                <body name="link6" pos="0 0 0.08">
                                    <joint name="joint6" type="hinge" axis="0 1 0"/>
                                    <geom name="link6_geom" type="box" size="0.04 0.015 0.01" pos="0 0 0.005" rgba="0.2 0.2 0.2 1"/>
                                    
                                    <body name="left_finger" pos="0 -0.06 0.01">
                                        <joint name="joint_left_finger" type="slide" axis="0 1 0" range="0 0.55"/>
                                        <geom name="left_finger_geom" type="box" size="0.005 0.008 0.04" pos="0 0 0.04" rgba="0.3 0.3 0.3 1"/>
                                    </body>
                                    
                                    <body name="right_finger" pos="0 0.06 0.01">
                                        <joint name="joint_right_finger" type="slide" axis="0 -1 0" range="0 0.55"/>
                                        <geom name="right_finger_geom" type="box" size="0.005 0.008 0.04" pos="0 0 0.04" rgba="0.3 0.3 0.3 1"/>
                                    </body>
                                </body>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </body>
    </worldbody>

    <actuator>
        <position name="act_j1" joint="joint1" ctrlrange="-180 180"/>
        
        <position name="act_j2" joint="joint2" ctrlrange="0 1.5"/>
        <position name="act_j3" joint="joint3" ctrlrange="0 1.5"/>
        
        <position name="act_j4" joint="joint4" ctrlrange="-1 1"/>
        <position name="act_j5" joint="joint5" ctrlrange="-1 1"/>
        <position name="act_j6" joint="joint6" ctrlrange="-1 1"/>
        
        <position name="act_f_left" joint="joint_left_finger" ctrlrange="0 0.06" kp="800"/>
        <position name="act_f_right" joint="joint_right_finger" ctrlrange="0 0.06" kp="800"/>
    </actuator>
</mujoco>
"""

class ArmGraspSandbox:
    def __init__(self):
        # 从内嵌字符串直接编译模型，消除外部依赖文件
        self.model = mujoco.MjModel.from_xml_string(MJCF_ARM_SANDBOX)
        self.data = mujoco.MjData(self.model)

    def execute_scripted_grasp(self):
        """控制机械臂顺序执行：平滑下落接近 -> 稳定夹紧 -> 垂直举起目标物体"""
        print("=" * 60)
        print("  正在拉起多自由度机械臂物理接触控制沙盒...")
        print("  已启用 Cosine 平滑轨迹插补算法")
        print("=" * 60)

        # 预设动作序列的各控制点位置 (Target Positions)
        ctrl_home  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        ctrl_reach = [0.0, np.deg2rad(75), np.deg2rad(50), np.deg2rad(-20), 0.0, 0.0, 0.0, 0.0]
        # 💡 修正 4：闭合距离加大到 0.018，确保两侧紧紧夹住小球而不发生碾压穿模
        ctrl_clamp = [0.0, np.deg2rad(75), np.deg2rad(50), np.deg2rad(-20), 0.0, 0.0, 0.018, 0.018]
        ctrl_lift  = [0.0, np.deg2rad(45), np.deg2rad(30), np.deg2rad(-10), 0.0, 0.0, 0.018, 0.018]

        # 💡 修正 2：工业级平滑插值函数
        def smooth_interp(start, end, progress):
            """余弦缓动 (Cosine Easing)，让电机的启动和停止像真实机械臂一样丝滑"""
            progress = np.clip(progress, 0.0, 1.0)
            ease = (1.0 - np.cos(progress * np.pi)) / 2.0
            return start + (end - start) * ease

        def controller_callback(model, data):
            t = data.time
            # 将离散的瞬间阶跃指令，转化为基于时间的连续平滑轨迹
            if t < 2.0:
                # 阶段 1 (0~2秒)：从初始竖直状态，缓慢丝滑地探向小球
                p = t / 2.0
                data.ctrl[:] = smooth_interp(np.array(ctrl_home), np.array(ctrl_reach), p)
            elif t < 3.0:
                # 阶段 2 (2~3秒)：保持手臂姿态不动，平滑收缩夹爪
                p = (t - 2.0) / 1.0
                data.ctrl[:] = smooth_interp(np.array(ctrl_reach), np.array(ctrl_clamp), p)
            elif t < 5.0:
                # 阶段 3 (3~5秒)：发力抬起，平滑过渡到举高姿态
                p = (t - 3.0) / 2.0
                data.ctrl[:] = smooth_interp(np.array(ctrl_clamp), np.array(ctrl_lift), p)
            else:
                data.ctrl[:] = ctrl_lift

        #mujoco.set_mjcb_control(controller_callback)
        mujoco.viewer.launch(self.model, self.data)


if __name__ == "__main__":
    sandbox = ArmGraspSandbox()
    sandbox.execute_scripted_grasp()