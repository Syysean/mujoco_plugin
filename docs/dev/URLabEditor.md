# URLab 编辑器

编辑器（[URLabEditor](https://github.com/OpenHUTB/hutb/tree/hutb/Unreal/CarlaUE4/Plugins/UnrealRoboticsLab/Source/URLabEditor)）模块提供了将 MuJoCo 的 MJCF XML 定义与虚幻引擎的参与者（Actor）和组件（Component）系统相桥接所需的专用工具。该模块负责处理资源导入流程、网格预处理以及编辑器端的自定义设置，使用户能够在虚幻编辑器环境中管理复杂的机器人资源。


编辑工具的主要目标是自动将层次化的 XML 机器人描述转换为功能性的 [AMjArticulation](https://github.com/OpenHUTB/hutb/blob/f49dd4dd8c0effaa4a07b81a4a53248682fe7e5c/Unreal/CarlaUE4/Plugins/UnrealRoboticsLab/Source/URLab/Public/MuJoCo/Core/MjPhysicsEngine.h#L34) 蓝图，该蓝图包含物理就绪的组件和优化的几何体。

## 系统概述：从MJCF到蓝图

下图展示了从原始MJCF文件到虚幻编辑器中生成机器人的高级流程。

### MJCF 导入管线

导入流程是一个多阶段的过程，当将 .xml 文件拖入虚幻内容浏览器时，该流程即开始。UMujocoImportFactory会拦截此操作，并协调多个专用类将 MuJoCo 场景图重建为虚幻参与者。

* XML解析：MujocoXmlParser读取MJCF结构，并将MuJoCo元素（物体、关节、几何体）映射到其对应的UMjComponent子类。

* 网格处理：由于MuJoCo经常使用可能未针对虚幻引擎优化的STL或OBJ文件，因此MujocoMeshImporter（由UMjPythonHelper支持）负责将其转换为GLB格式，并进行三角网格清理。

* 蓝图生成：UMujocoGenerationAction接收解析后的组件树，并通过编程方式构建一个从AMjArticulation派生的蓝图类。

有关这些类和数据映射逻辑的详细说明，请参阅MJCF导入流程。


## 
