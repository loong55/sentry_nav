# BehaviorTree.ROS2
[![Test](https://github.com/BehaviorTree/BehaviorTree.ROS2/actions/workflows/test.yml/badge.svg)](https://github.com/BehaviorTree/BehaviorTree.ROS2/actions/workflows/test.yml)

This repository contains useful wrappers to use ROS2 and BehaviorTree.CPP together.

In particular, it provides a standard way to implement:

- Behavior Tree Executor with ROS Action interface.
- Action clients.
- Service Clients.
- Topic Subscribers.
- Topic Publishers.

Our main goals are:

- to minimize the amount of boilerplate.
- to make asynchronous Actions non-blocking.

# Documentation

- [ROS Behavior Wrappers](behaviortree_ros2/ros_behavior_wrappers.md)
- [TreeExecutionServer](behaviortree_ros2/tree_execution_server.md)
- [Sample Behaviors](btcpp_ros2_samples/README.md)

Note that this library is compatible **only** with:

- **BT.CPP** 4.6 or newer.
- **ROS2** Humble or newer.

Additionally, check **plugins.hpp** to see how to learn how to
wrap your Nodes into plugins that can be loaded at run-time.


## Acknowledgements

A lot of code is either inspired or copied from [Nav2](https://docs.nav2.org/).

For this reason, we retain the same license and copyright.

翻译：

# 行为树.ROS2
[![测试](https://github.com/BehaviorTree/BehaviorTree.ROS2/actions/workflows/test.yml/badge.svg)](https://github.com/BehaviorTree/BehaviorTree.ROS2/actions/workflows/test.yml)

此存储库包含有用的包装器，可将ROS2和BehaviorTree.CPP一起使用。

特别是，它提供了一种标准的方式来实现：

- 带有ROS动作接口的行为树执行器。
- 行动客户。
- 服务客户。
- 主题订阅者。
- 主题发布者。

我们的主要目标是：

- 尽量减少样板文件数量。
- 使异步操作非阻塞。

# 文档

- [ROS行为封装器](behaviortree_ros2/ros_behavior_wrappers.md)
- [行为树执行服务器](behaviortree_ros2/tree_execution_server.md)
- [行为示例](btcpp_ros2_samples/README.md)

请注意，此库仅与以下版本兼容：

- **BT.CPP** 4.6 或更新版本。
- **ROS2** 谦逊版或更新版本。

此外，请查看 **plugins.hpp** 以了解如何学习如何
将你的节点封装成可以在运行时加载的插件。


## 致谢

很多代码都是从[Nav2](https://docs.nav2.org/)中获取灵感或直接复制过来的。

因此，我们保留相同的许可和版权。
