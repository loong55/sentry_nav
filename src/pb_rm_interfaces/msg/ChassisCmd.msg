# 底盘控制命令
# reset: 1=复位, 0=关闭复位
# rotate: 1=旋转, 0=关闭旋转
# vy_limit: 1=限制底盘y轴速度, 0=不限制

uint8 DISABLE = 0
uint8 ENABLE = 1

builtin_interfaces/Time stamp
uint8 reset
uint8 rotate
uint8 vy_limit