# Joint State Publisher

This contains a package for publishing `sensor_msgs/msg/JointState` messages for a robot described with URDF.
Given a URDF (either passed on the command-line or via the `/robot_description` topic), this node
will continually publish values for all of the movable joints in the URDF to the `/joint_states` topic.
In combination with `robot_state_publisher`, this ensures that there is a valid transform for all joints
even when the joint doesn't have encoder data.

Published Topics
----------------
* `/joint_states` (`sensor_msgs/msg/JointState`) - The state of all of the movable joints in the system.

Subscribed Topics
-----------------
* (optional) `/robot_description` (`std_msgs/msg/String`) - If no URDF is given on the command-line, then this node will listen on the `/robot_description` topic for the URDF to be published.  Once it has been received at least once, this node will start to publish joint values to `/joint_states`.
* (optional) `/any_topic` (`sensor_msgs/msg/JointState`) - If the `sources_list` parameter is not empty (see Parameters below), then every named topic in this parameter will be subscribed to for joint state updates.  Do *not* add the default `/joint_states` topic to this list, as it will end up in an endless loop!

Parameters
----------
* `rate` (int) - The rate at which to publish updates to the `/joint_states` topic.  Defaults to 10.
* `offset_timestamp`(double) - The offset to add to the timestamp of the published message, in seconds. Defaults to 0.0.
* `publish_default_positions` (bool) - Whether to publish a default position for each movable joint to the `/joint_states` topic.  Defaults to True.
* `publish_default_velocities` (bool) - Whether to publish a default velocity for each movable joint to the `/joint_states` topic.  Defaults to False.
* `publish_default_efforts` (bool) - Whether to publish a default effort for each movable joint to the `/joint_states` topic.  Defaults to False.
* `use_mimic_tags` (bool) - Whether to honor `<mimic>` tags in the URDF.  Defaults to True.
* `use_smallest_joint_limits` (bool) - Whether to honor `<safety_controller>` tags in the URDF.  Defaults to True.
* `source_list` (array of strings) - Each string in this array represents a topic name.  For each string, create a subscription to the named topic of type `sensor_msgs/msg/JointStates`.  Publication to that topic will update the joints named in the message.  Defaults to an empty array.
* `delta` (double) - How much to automatically move joints during each iteration.  Defaults to 0.0.

#### Mapped Parameters

These parameters map from joint_names to values. The format to use these parameters is `<parameter>.<key>:=<value>`, where a new parameter is defined for each key. See below for examples.

* `zeros` (map from string -> float) - A map of joint_names to initial starting values for the joint. For example, in Eloquent and beyond, this parameter can be used from the command-line with `ros2 run joint_state_publisher joint_state_publisher --ros-args --param zeros.joint_name1:=value1 --param zeros.joint_name2:=value2`. This parameter is not set by default, so all joints start at zero. For joints where zero isn't within the joint range, it uses the range's (max + min) / 2.
* `dependent_joints` (map from string -> map from 'parent', 'factor', 'offset' -> float) - A map of joint_names to the joints that they mimic; compare to the `<mimic>` tag in URDF.  A joint listed here will mimic the movements of the 'parent' joint, subject to the 'factor' and 'offset' provided.  The 'parent' name must be provided, while the 'factor' and 'offset' parameters are optional (they default to 1.0 and 0.0, respectively).  For example, in Eloquent and beyond, this parameter can be used from the command-line with `ros2 run joint_state_publisher joint_state_publisher --ros-args --param dependent_joints.left_leg.parent:=right_leg --param dependent_joints.left_leg.offset:=0.0 --param dependent_joints.left_leg.factor:=2.0`. This parameter is not set by default, in which case only joints that are marked as `<mimic>` in the URDF are mimiced.

翻译
# 联合状态发布者

这包含了一个用于为使用URDF描述的机器人发布`sensor_msgs/msg/JointState`消息的包。
给定一个 URDF（通过命令行或 `/robot_description` 主题传递），此节点
将不断向 `/joint_states` 主题发布 URDF 中所有可移动关节的值。
结合`robot_state_publisher`，这确保了所有关节都有一个有效的变换
即使关节没有编码器数据。

已发布的话题
----------------
* `/joint_states` (`sensor_msgs/msg/JointState`) - 系统中所有可移动关节的状态。

已订阅的主题
-----------------
* (可选) `/robot_description` (`std_msgs/msg/String`) - 如果命令行上没有给出 URDF，那么此节点将侦听 `/robot_description` 主题以发布 URDF。一旦至少收到一次，此节点将开始向 `/joint_states` 发布关节值。
* (可选) `/any_topic` (`sensor_msgs/msg/JointState`) - 如果 `sources_list` 参数不为空（见下面的参数），那么此参数中的每个命名主题都将被订阅以获取关节状态更新。 不要将默认的 `/joint_states` 主题添加到此列表中，因为它将导致无限循环！

参数：
----------
* `rate` (int) - 发布 `/joint_states` 主题更新的频率。默认为 10。
* `offset_timestamp`(double) - 发布消息的时间戳偏移量，单位为秒。默认为0.0。
* `publish_default_positions` (bool) - 是否为每个可移动关节发布一个默认位置到 `/joint_states` 主题。默认为 True。
* `publish_default_velocities` (bool) - 是否为每个可移动关节发布默认速度到 `/joint_states` 主题。默认为 False。
* `publish_default_effort` (bool) - 是否为每个可移动关节发布默认的力到 `/joint_states` 主题。默认为 False。
* `use_mimic_tags` (布尔) - 是否在URDF中遵循`<mimic>`标签。默认为True。
* `use_smallest_joint_limits` (bool) - 是否遵循URDF中的`<safety_controller>`标签。默认为True。
* `source_list` (字符串数组) - 此数组中的每个字符串代表一个主题名称。对于每个字符串，创建一个名为 `sensor_msgs/msg/JointStates` 的订阅。发布到该主题将更新消息中命名的关节。默认为空数组。
* `delta` (double) - 每次迭代期间关节自动移动的量。默认为 0.0。

#### 映射参数

这些参数从 joint_names 映射到值。使用这些参数的格式为 `<parameter>.<key>:=<value>`，其中为每个键定义了一个新参数。请参阅下面的示例。

* `zeros` (字符串 -> 浮点数映射) - 这是一个从关节名称到关节初始值的映射。例如，在Eloquent及更高版本中，可以通过命令行使用此参数，例如 `ros2 run joint_state_publisher joint_state_publisher --ros-args --param zeros.joint_name1:=value1 --param zeros.joint_name2:=value2`。默认情况下，此参数未设置，因此所有关节都从零开始。对于那些零不在关节范围内的关节，它将使用范围的最大值和最小值的平均值。
* `dependent_joints` (从字符串映射到由 'parent', 'factor', 'offset' 映射到浮点数) - 一个从关节名称到它们所模仿的关节的映射；与 URDF 中的 `<mimic>` 标签相对应。此处列出的关节将根据所提供的 'factor' 和 'offset' 模仿 'parent' 关节的运动。必须提供 'parent' 名称，而 'factor' 和 'offset' 参数是可选的（它们分别默认为 1.0 和 0.0）。例如，在 Eloquent 和更高版本中，此参数可以通过命令行使用，例如 `ros2 run joint_state_publisher joint_state_publisher --ros-args --param dependent_joints.left_leg.parent:=right_leg --param dependent_joints.left_leg.offset:=0.0 --param dependent_joints.left_leg.factor:=2.0`。默认情况下，此参数未设置，在这种情况下，只有 URDF 中标记为 `<mimic>` 的关节才会被模仿。
