# 迁移到UE4

1.将xml文件转成 _ue.xml 文件

2.通过内容浏览器导入虚幻编辑器，并生成蓝图 -> 通过拖拽 Actor 的方式（参考 [mujoco_plugin](https://github.com/OpenHUTB/hutb/commit/2cc693c1248f3f65d71a5d95c23231f9dfa928a1) ）

-> Source/URLab/Public/MuJoCo/Core/AMjManager.h 是一个参与者，通过它进行xml文件的加载

3.在场景中指定位置将蓝图实例化


## 问题

* 打包报错：`UnrealBuildTool: ERROR: Non-editor build cannot depend on non-redistributable modules.`

    原因：将一些编辑器相关的内容打包进来了，参考[链接](https://imzlp.com/posts/9050/)。

    解决：打开 Build/engine/UE4.sln，右键项目 UnrealBuildTool 进行`生成`。


    资产打包报错：
    ```text
    LogAssetRegistry: Error: Package ../../../../../Unreal/CarlaUE4/Plugins/UnrealRoboticsLab/Content/Input/IA_TwistMove.uasset is too old
    LogAssetRegistry: Error: Package ../../../../../Unreal/CarlaUE4/Plugins/UnrealRoboticsLab/Content/Input/IMC_TwistControl.uasset is too old
    LogAssetRegistry: Error: Package ../../../../../Unreal/CarlaUE4/Plugins/UnrealRoboticsLab/Content/Materials/M_MuJoCo_Master.uasset is too old
    LogAssetRegistry: Error: Package ../../../../../Unreal/CarlaUE4/Plugins/UnrealRoboticsLab/Content/Demo/example_BP/M_HoverGlow.uasset is too old
    LogAssetRegistry: Error: Package ../../../../../Unreal/CarlaUE4/Plugins/UnrealRoboticsLab/Content/UI/WBP_MjCameraFeedEntry.uasset is too old
    LogAssetRegistry: Error: Package ../../../../../Unreal/CarlaUE4/Plugins/UnrealRoboticsLab/Content/UI/WBP_MjPropertyRow.uasset is too old
    LogAssetRegistry: Error: Package ../../../../../Unreal/CarlaUE4/Plugins/UnrealRoboticsLab/Content/UI/WBP_MjSimulate.uasset is too old
    LogAssetRegistry: Error: Package ../../../../../Unreal/CarlaUE4/Plugins/UnrealRoboticsLab/Content/Input/IA_TwistTurn.uasset is too old
    LogAssetRegistry: Error: Package ../../../../../Unreal/CarlaUE4/Plugins/UnrealRoboticsLab/Content/Demo/example_BP/BP_RandomActions.uasset is too old
    ```
    解决：在资源浏览器中删除资产。



* error C2039: "byte": is not a member "std" C++14
原因：mujoco 和 CoACD 使用的是 C++ 17 进行编译，而虚幻工程使用 C++ 14

    解决：将 C++ 17 的一些语法改为 C++ 14
    ```cpp
    // Unreal\CarlaUE4\Plugins\UnrealRoboticsLab\third_party\install\MuJoCo\include\mujoco\mjspec.h
    // using mjByteVec     = std::vector<std:byte>;
    using mjByteVec     = std::vector<unsigned char>;

    // Unreal\CarlaUE4\Plugins\UnrealRoboticsLab\third_party\install\CoACD\include\CoACD\coacd.h
    void set_log_level(); // void set_log_level(std::string_view level);
    ```

* UnrealBuildTool: ERROR: Could not find definition for module 'GeometryFramework', (referenced via Target -> URLab.Build.cs)

    模块 GeometryFramework 位于 UE5 的`Engine/Source/Runtime/GeometryFramework`


