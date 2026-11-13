# URDForge Studio

> **下一代跨平台、免 CAD 强绑定的现代化机器人建模与仿真就绪（Simulation-Ready）URDF 工业级生成软件**

---

## 1. 软件概述与定位
传统的 ROS 官方 `solidworks_urdf_exporter` 工具高度强绑定 Windows 平台的 SolidWorks 商业闭源软件，且界面老旧、坐标系反人类、依赖外部宿主环境。

**URDForge** 作为一款**纯原生、独立运行的跨平台桌面应用软件**（非浏览器启动）：
* **原生独立桌面应用**：基于 PySide6 (Qt) + Chromium 高性能 3D 渲染视口开发，双击即启动独立原生窗口，自带 Windows 原生菜单栏与快捷键。
* **支持打开任意自定义 CAD 文件**：支持原生文件选择弹窗、顶部文件浏览按钮以及**全窗口文件拖拽（Drag & Drop）**，自由载入本地任意 `.step` / `.stp` 机器人装配体。
* **高精度几何与物性计算**：基于 OpenCASCADE (OCP) XCAF，自动递归提取装配体层级树、计算部件精确体积、质心（CoM）与 3×3 惯性张量（Inertia Tensor）。
* **装配体树与特征折叠管理（Collapsible Hierarchy & Features）**：面向包含 50+ 零件的工业级大型装配体，CAD 部件树节点支持一键折叠/展开、按名称快速搜索过滤；检测到的几何特征面板支持折叠与按零件过滤，彻底告别冗长滚动条。
* **从 base_link 开始交互构建 URDF 树（Hierarchical URDF Builder）**：右侧属性面板以 `base_link` 为根节点直观呈现机器人层级结构，支持通过 `➕ 添加子连杆` 层层添加子 Link。
* **多零件归属一个 Link（Multi-Part per Link & Composite Physics）**：支持在左侧零件树中批量多选零件，一键归属至当前 Link。几何内核自动根据平行轴定理（Steiner's Parallel Axis Theorem）合成复合刚体总质量、质心以及精确的 3×3 惯性张量，并在 URDF 中生成标准多 `<visual>` 与 `<collision>` 节点。
* **圆柱中心点建立坐标系（Cylinder Midpoint & Center Snapping）**：精准求解圆柱体参数化轴线几何中点（Midpoint）、顶端圆心（Top）与底端圆心（Bottom），支持一键将 Joint 坐标原点放置在圆柱几何中心点！
* **圆面/圆柱中心轴为旋转轴（Axis Alignment）**：自动提取圆孔边缘法向量或圆柱中心轴线，利用 Rodrigues 旋转公式自动对齐 Joint Z 轴朝向，并提供 180° 一键反转轴向。
* **物理有效性验证器（Simulation-Ready Check）**：遵循 Isaac Sim / MuJoCo / Gazebo 仿真求解器物理标准，严格校验正定性（Positive-Definiteness）与主惯性矩三角不等式（Triangle Inequalities）。
* **现代工业级设计风格（Clean Studio Theme 重构）**：参考 `B1_Robotic_Arm_App`，采用现代工作室灰（`0x68727a`）、高对比度网格与高清晰度控件重构，彻底解决界面闪屏（去除 `backdrop-filter` 混合冲突，关闭动态阴影闪烁，确保 60FPS 确定性稳定渲染）。
* **全景 3D 粗箭头坐标系预览（Thick RGB Triad Preview）**：在设置关节坐标系、吸附几何特征或调整 Origin XYZ / RPY 欧拉角时，视口中实时呈现工业标准的 3D 粗箭头坐标系（X 轴为红 Red、Y 轴为绿 Green、Z 轴为蓝 Blue），并提供“坐标轴”开关可同时呈现全机器人所有关节坐标系，与参考项目效果完全一致。
* **SolidWorks Y-Up 到 URDF Z-Up 一键坐标系转换**：完美解决 SolidWorks 默认以 ZX 平面为基准面与 ROS/URDF 以 XY 为上视基准面的冲突，提供一键旋转转换与姿态校准，支持全姿态变换矩阵及惯性张量坐标转换。
* **Link 独立隐藏/显示与动态重命名**：每个 Link 节点配备独立可视性切换开关（👁 眼睛图标），支持快速重命名 Link 并自动同步更新关联 Joint 与所有子连杆关系树。
* **运动学树断开与孤儿保护（Kinematic Integrity Guard）**：删除或修改连杆连接时，智能检测是否会导致孤儿连杆（Orphan Links）或运动链断开，并在界面中给出清晰的确认与预警保护。
* **高精微米导轨与 0° 旋转限位支持**：平移导轨关节（Prismatic）支持最小 0.0000001 (1e-7m) 步进与微小限位，前向运动学滑块精准联动；旋转关节支持上下限位同时设为 0。
* **3D 视口拖拽与硬件加速深度优化**：解除 Chromium 软解渲染瓶颈，全面开启原生 GPU 光栅化与零拷贝显存缓冲；引入 OrbitControls 平滑阻尼与主动渲染管线，实现 60~144 FPS 流畅交互。
* **全平台免安装便携版与打包工具链**：提供 Windows 免安装便携版（`.zip` 解压即用，内置完整环境与一键启动脚本）、Windows 安装向导脚本（Inno Setup），以及 macOS（`.app` / `.dmg`）和 Linux（`.tar.gz` / `.deb`）自动化打包流水线。
* **一键生成生产级 ROS / ROS 2 功能包**：自动生成 `package.xml`、`CMakeLists.txt`、`display.launch.py`、`urdf.rviz` 及高精度 Visual/Collision STL 网格，完整补全 `package://` 相对路径。

---

## 2. 软件运行指南

### 方式 1：双击批处理一键启动（推荐）
在项目根目录下，直接双击运行：
```bash
URDForge.bat
```
或在终端执行：
```bash
& "D:\Anaconda3\envs\robotarm\python.exe" app.py
```
> **说明**：系统将直接唤起 **URDForge Studio 原生桌面应用程序窗口**（无需打开外部浏览器），窗口尺寸 1440×900，居中呈现。

---

### 方式 2：使用独立命令行（Headless CLI）批量导出
如需在服务器或流水线中无界面运行：
```bash
& "D:\Anaconda3\envs\robotarm\python.exe" step_urdf_parser.py \
  -i tests/fixtures/sample_nested_robot.step \
  --export-all -o output/meshes \
  --json output/assembly.json \
  --urdf output/robot.urdf
```

---

## 3. 核心功能操作说明

### 3.1 载入自定义 STEP 装配体文件
1. **菜单栏打开**：点击顶部菜单 `文件` -> `打开本地 STEP 装配体...` 或快捷键 `Ctrl+O`，弹出 Windows 原生文件浏览对话框，可选择电脑任意磁盘位置的模型。
2. **工具栏点击**：点击顶部导航栏的 `📁 打开本地 STEP 文件...` 按钮。
3. **直接拖拽**：直接从 Windows 资源管理器将任意 `.step` / `.stp` 文件拖入软件三维视口，松开鼠标即可自动完成几何剖分与层级树构建。

### 3.2 特征吸附创建 Joint（Feature Snapping）
1. 点击顶部栏 `🎯 特征吸附模式` 按钮切换至吸附状态。
2. 3D 视口中所有零件的圆柱轴线与圆孔边缘将浮现高亮青色导引环与轴线矢量。
3. 点击目标圆孔，系统自动计算并填入高精度关节原点 $(x, y, z)$ 与对齐的 RPY 旋转角度。
4. 若需反向运动，点击 `↺ 反转 Z 轴` 即可瞬时翻转。

### 3.3 实时 FK 运动学滑块调试
在右侧 `FK Sliders` 面板中，拖动对应活动关节的滑块，3D 视口中的机器人连杆将以 60 FPS 实时跟随旋转或平移，支持快速点击 `回零位` 恢复初始姿态。

### 3.4 导出 ROS 2 功能包与 URDF
* 点击右上角 `🚀 导出 ROS 2 功能包` 或快捷键 `Ctrl+E`，选择保存目录，即可一键生成包含 `CMakeLists.txt`、`package.xml`、`launch/display.launch.py`、`rviz/urdf.rviz` 与 `meshes/` 的标准 ROS 2 功能包！
* 点击 `📄 查看 URDF` 可弹出代码查看窗口并一键复制 XML。

---

## 4. 运行全量自动化测试
```bash
$env:PYTHONPATH="src"
& "D:\Anaconda3\envs\robotarm\python.exe" -m pytest -v
```
**测试覆盖率**：24 项单元与集成测试全部通过（100% 通过率，覆盖 B-Rep 圆柱轴线与中心点求解、平行轴定理多刚体惯性张量合成、SolidWorks Y-Up 到 Z-Up 坐标变换矩阵、运动学树孤儿连杆检测、高精度微小导轨运动学与限位、URDF 导出、ROS 2 打包及 API 端点）。
