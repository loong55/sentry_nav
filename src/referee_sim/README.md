### 裁判系统模拟器

#### 功能
模拟裁判系统发布消息，用于给行为树提供决策条件。

#### 使用方法
##### 1.编译并加载环境
```bash
colcon build
source install/setup.bash
```
##### 2.启动模拟节点：
```bash
source install/setup.bash
ros2 launch referee_sim referee_sim.launch.py
```
打开 rqt 插件：
```bash
rqt --force-discover
```
在插件列表中找到 “Plugins -> Referee -> Referee Simulator”。

如果 referee_sim 以 red_standard_robot1 命名空间启动，插件会自动连接到
/red_standard_robot1/referee_sim；如果是全局启动，则自动连接到 /referee_sim。

在插件内调节血量、弹量等数值，观察话题和终端日志变化。