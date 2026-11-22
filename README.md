# RoboStudio 6-DOF · 六轴机械臂 Three.js 网页操控平台

基于 **Three.js** 构建的纯前端六轴工业机械臂 3D 仿真与操控平台，零构建依赖，开箱即用。默认内置官方开源精细模型 **Universal Robots UR5e**（ROS 2 CAD `.dae` 网格与精确运动学参数），并支持导入自定义机械臂与末端工具模型。

🌐 **在线预览 (GitHub Pages)**：https://gewaltc.github.io/six-axis-robot-arm/

---

## 核心特性

- **默认开源精细模型（UR5e）**：内置 Universal Robots UR5e 完整 6 轴 URDF 与 7 组官方高精 `.dae` 视觉网格，搭配 PBR 工业材质优化与可控电动二指平行夹爪。
- **多格式模型导入**（支持按钮选择或直接拖拽到 3D 视窗）：
  - **ROS URDF 包**：支持选择包含 `.urdf` 与 `meshes/` 的文件夹、多选文件或 `.zip` 压缩包，自动解析关节链与限位。
  - **GLB / GLTF**：自动归一化尺寸与层级识别，并提供可视化 **「关节映射」** 面板手动绑定 J1–J6 节点及局部旋转轴（X / Y / Z）。
  - **DAE / STL / OBJ**：支持作为静态网格导入，或作为 **自定义末端工具/夹爪** 挂载到当前机械臂法兰。
- **完整正/逆运动学操控**：
  - **关节控制 (FK)**：J1–J6 滑块、精确数值输入、±5° 点动、6 组预设工业位姿与一键归零。
  - **笛卡尔控制 (IK)**：内置通用 6-DOF 阻尼最小二乘雅可比逆运动学求解器，支持 **3D 拖拽 Gizmo** 实时牵引、目标 `X/Y/Z (mm)` + `RX/RY/RZ (°)` 位姿求解及 ±25mm 空间步进点动。
  - **示教与轨迹回放**：内置 UR5e 抓取搬运演示 Demo；支持自由记录示教点位、平滑 S 曲线插值播放、调速/循环及 JSON 导入导出。
- **工业仿真视窗工具**：末端 TCP 基座标系（ROS Z-up）实时遥测 HUD、3D TCP 运动轨迹绘制、各关节局部坐标系显示、工作空间包络球、线框模式与深/浅主题切换。

---

## 本地运行

项目为纯静态 ES Module 应用，使用任意静态 HTTP 服务器即可运行（推荐直接使用内置的 Python 脚本）：

```bash
python server.py 8080
```

随后在浏览器打开 `http://127.0.0.1:8080/` 即可使用。

---

## 目录结构

```text
six-axis-robot-arm/
├── index.html                  # 主页面与控制面板
├── css/style.css               # 双主题工业 UI 样式
├── js/
│   ├── main.js                 # 主控逻辑、渲染循环与事件绑定
│   ├── scene.js                # Three.js 展厅光照、控制器、IK Gizmo 与轨迹渲染
│   ├── urdfParser.js           # 原生 URDF 解析与 Three.js 运动链构建
│   ├── modelImporter.js        # URDF/ZIP/GLB/GLTF/STL/OBJ 导入与夹爪模块
│   ├── kinematics.js           # FK 位姿计算与 6-DOF DLS 雅可比 IK 求解器
│   └── presetsAndTrajectory.js # 预设位姿与轨迹插值控制器
├── models/ur5e/                # 官方 Universal Robots UR5e URDF 与 DAE 网格
└── server.py                   # 本地静态 HTTP 服务启动脚本
```

---

## 开源协议与模型致谢

- 默认机械臂模型（`models/ur5e/`）来源于 [UniversalRobots/Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description)，遵循 **BSD-3-Clause** 协议（详见 [models/ur5e/LICENSE.txt](models/ur5e/LICENSE.txt)）。
- 3D 渲染引擎基于 [Three.js](https://threejs.org/)（MIT License）。
