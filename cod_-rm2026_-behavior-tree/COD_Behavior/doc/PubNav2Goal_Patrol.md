# PubNav2Goal 巡逻方案使用文档

## 概述

本方案使用 `PubNav2Goal`（Topic 发布）替代原来的 `FollowWaypointsAction`（Action 调用）实现多点循环巡逻。

原方案链路：`BT → waypoint_follower → bt_navigator`（双层 Action，cancel/重发易出问题）

新方案链路：`BT → /goal_pose Topic → bt_navigator`（无状态，简单可靠）

## 新增节点说明

### LoadWaypoints

从 CSV 文件加载航点列表和每个航点的等待时间到黑板。每次巡逻启动时执行一次。

| 端口 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `waypoint_file` | 输入 | string | 航点 CSV 文件的绝对路径 |
| `frame_id` | 输入 | string | 坐标系，默认 `"map"` |
| `waypoints` | 输出 | vector\<PoseStamped\> | 航点列表（写入黑板） |
| `wait_times` | 输出 | vector\<double\> | 每个航点的等待时间（秒） |
| `total_waypoints` | 输出 | int | 航点总数 |

### GetCurrentWaypoint

根据当前索引 `wp_idx` 从航点列表中取出目标点和对应的等待时间。

| 端口 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `waypoints` | 输入 | vector\<PoseStamped\> | 航点列表 |
| `wait_times` | 输入 | vector\<double\> | 等待时间列表 |
| `wp_idx` | 输入 | int | 当前航点索引 |
| `current_goal` | 输出 | PoseStamped | 当前目标点 |
| `current_wait_sec` | 输出 | double | 当前航点的等待时间 |

### WaitUntilReached

通过 TF2 查询 `map → base_link` 变换，计算机器人与目标点的距离。未到达返回 `RUNNING`，到达返回 `SUCCESS`。

| 端口 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `goal_pose` | 输入 | PoseStamped | 目标位姿 |
| `tolerance` | 输入 | double | 到达判定距离阈值（米），默认 `0.5` |

### WaitDuration

到达航点后原地等待指定时间。`duration_sec` 为 0 时直接跳过。

| 端口 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `duration_sec` | 输入 | double | 等待时长（秒），默认 `0`（不等待） |

### NextWaypoint

将航点索引加 1 并循环：`wp_idx = (wp_idx + 1) % total_waypoints`。

| 端口 | 方向 | 类型 | 说明 |
|------|------|------|------|
| `wp_idx` | 双向 | int | 当前航点索引（读取后更新） |
| `total_waypoints` | 输入 | int | 航点总数 |

## 行为树结构

```
ReactiveSequence
├── Inverter(ZsCondition)              ← 守门条件（zone_status 为 false 时放行）
└── Sequence
    ├── LoadWaypoints                  ← 加载 CSV（每次巡逻启动执行一次）
    └── KeepRunningUntilFailure
        └── Sequence                   ← 循环体
            ├── GetCurrentWaypoint     ← 取当前目标点 + 等待时间
            ├── ReactiveSequence
            │   ├── PubNav2Goal        ← 每 500ms 发布目标点到 /goal_pose
            │   └── WaitUntilReached   ← 等待到达（RUNNING）
            ├── WaitDuration           ← 到达后等待指定时间
            └── NextWaypoint           ← 切换下一个航点
```

**执行流程：**

1. `LoadWaypoints` 从 CSV 加载航点和等待时间 → SUCCESS
2. 进入循环：
   - `GetCurrentWaypoint` 取出 `waypoints[wp_idx]` 和 `wait_times[wp_idx]` → SUCCESS
   - `PubNav2Goal` 发布目标点到 `/goal_pose` → SUCCESS（每 500ms 重发）
   - `WaitUntilReached` 查 TF 判断距离 → RUNNING（未到达）/ SUCCESS（已到达）
   - `WaitDuration` 原地等待 `current_wait_sec` 秒 → RUNNING / SUCCESS
   - `NextWaypoint` 切换索引 → SUCCESS
3. 循环回到步骤 2，前往下一个航点

## 配置方法

### 1. 准备航点 CSV 文件

```csv
id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command,wait_sec
0,5.00463,-3.12219,0,0,0,0,1,,3.0
1,0.587131,-2.9942,0,0,0,0,1,,0
2,-0.577313,0.210904,0,0,0,0,1,,5.0
```

| 列 | 索引 | 含义 |
|----|------|------|
| id | 0 | 航点编号（仅标识用） |
| pose_x, pose_y, pose_z | 1-3 | 位置坐标（map 坐标系） |
| rot_x, rot_y, rot_z, rot_w | 4-7 | 姿态四元数 |
| command | 8 | 预留字段（当前未使用） |
| wait_sec | 9 | 到达该航点后等待时间（秒），可选，缺省为 0 |

`wait_sec` 列是**可选的**，与旧版 CSV 文件完全兼容：
- 没有第 10 列 → 所有航点等待时间为 0
- 某行该列为空 → 该航点等待时间为 0

CSV 文件可通过 `waypoint_editor` 工具录制后手动添加 `wait_sec` 列。

### 2. 修改行为树 XML

在 `singlenav_tree.xml` 中的可配置参数：

```xml
<LoadWaypoints waypoint_file="/path/to/patrol.csv"
               frame_id="map"
               waypoints="{waypoints}"
               wait_times="{wait_times}"
               total_waypoints="{total_wp}"/>
```

```xml
<WaitUntilReached goal_pose="{current_goal}"
                  tolerance="0.5"/>
```

**可调参数汇总：**

| 参数 | 位置 | 说明 | 建议值 |
|------|------|------|--------|
| `waypoint_file` | LoadWaypoints | CSV 文件绝对路径 | 按部署路径配置 |
| `frame_id` | LoadWaypoints | 坐标系 | `"map"` |
| `tolerance` | WaitUntilReached | 到达判定距离 | `0.3` ~ `0.8`，越小越精确但可能反复抖动 |
| `min_pub_interval_ms` | PubNav2Goal | 目标点发布间隔 | `500`（ms） |
| `wait_sec` | CSV 每行 | 每个航点到达后的等待时间 | 按需求配置（秒） |

### 3. 编译部署

```bash
colcon build --packages-select cod_behavior
source install/setup.bash
```

## Halt 与恢复行为

巡逻被打断（如 HP 低回家）后恢复时：

- `wp_idx` 保留在黑板中，**从上次的航点继续巡逻**
- `LoadWaypoints` 重新读取 CSV（快速），支持运行中更换航点文件
- `WaitUntilReached` 重新初始化 TF 距离判定
- 如果打断发生在 `WaitDuration` 期间，等待被中断，恢复后重新导航到该航点并重新计时

如需每次恢复后从第一个航点重新开始，可在回家逻辑中手动重置 `wp_idx = 0`。

## 依赖

在原有依赖基础上新增：

- `tf2_ros` — 用于 `WaitUntilReached` 查询机器人位姿

已在 `CMakeLists.txt` 和 `package.xml` 中添加。

## 日志输出

正常运行时的关键日志：

```
[LoadWaypoints]: 已加载 3 个航点
[GetCurrentWaypoint]: 当前目标航点[0]: [5.00, -3.12], 等待 3.0s
[PubNav2Goal]: 发布目标点 [5.00, -3.12, 0.00] frame: map
[WaitUntilReached]: 等待到达 [5.00, -3.12], 阈值=0.50
[WaitUntilReached]: 已到达目标点 (距离=0.35)
[WaitDuration]: 在航点等待 3.0 秒
[WaitDuration]: 等待完成
[NextWaypoint]: 航点切换: 0 -> 1 (共 3 个)
```

异常情况日志：

```
[LoadWaypoints]: 无法从文件加载航点: /path/to/patrol.csv     ← 文件不存在
[WaitUntilReached]: TF 查询失败: ...                         ← TF 树未就绪（节点会继续重试）
[GetCurrentWaypoint]: 航点索引越界: idx=5, size=3            ← wp_idx 异常
[WaitDuration]: 等待被中断                                    ← 巡逻被打断
```
