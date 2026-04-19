# 姿态切换命令
# posture: 1=攻击 2=防御 3=移动 4=小陀螺
# reason: 0=默认 1=脱战 2=自瞄锁定 3=扣血 

uint8 ATTACK = 1
uint8 DEFENSE = 2
uint8 MOVE = 3
uint8 SPIN = 4

uint8 REASON_DEFAULT = 0
uint8 REASON_OUT_OF_COMBAT = 1
uint8 REASON_AIM_LOCKED = 2
uint8 REASON_HP_DEDUCED = 3

builtin_interfaces/Time stamp
uint8 posture
uint8 reason
bool force_switch
