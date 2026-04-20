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
在插件列表中找到 “plugins--->dynamic_config”
在插件内调节数值，观察话题和终端日志变化。