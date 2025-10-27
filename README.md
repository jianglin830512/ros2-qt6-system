
# COLCON BUILD

1. 指令如下：
```bash
colcon build --merge-install
```

2. 如果出错需要日志：
```bash
colcon build --merge-install --event-handlers console_direct+
```

# ROS2 LAUNCH
```bash
ros2 lauch qt_node run_qt_node.launch.py
```

# PIXI PACK
1. 开发环境已经装有pixi，还需要下载 pixi-pack.exe
```bash
pixi global install pixi-pack
```

2. 打包环境（指定缓存，避免重复下载）
```bash
pixi-pack --use-cache .\pixi-cache\
```

# 安装到目标机器
1. 目标机器安装 pixi （https://pixi.sh）

2. 下载 pixi-unpack.exe
```bash
pixi global install pixi-unpack
```

3. 把 evironment.tar 拷贝到目标文件夹，然后执行
```bash
pixi-unpack evironment.tar
```
> **注：**此时会多出 env 文件夹以及 activate.bat 文件

4. 把开发环境中的 install 目录（由COLCON生成）拷贝到目标文件夹

5. 打开 "Developer Command Prompt for VS2022"，进入目标文件夹
```bash
call activate.bat
call install\setup.bat
ros2 launch qt_node run_qt_node.launch.py
```

6. 编写一个自动运行的PowerShell脚本: 请见 ros_auto_launch.ps1
	- 右键点击ros_auto_launch.ps1，选择 “使用PowerShell运行”。
	- 或者双击ros_auto_launch.bat，间接运行 ros_auto_launch.ps1。
	
# PACKAGE 说明
1. ros2_interfaces 用于定义公共接口（msg, serv, action)
2. qt_node 是一个用户交互界面
3. control_node 是整个系统运行的核心逻辑

所有单纯的 ros2 node 应该用以下命令创建：
ros2 pkg create control_node --build-type ament_cmake --dependencies rclcpp std_msgs ros2_interfaces --license "Apache-2.0"