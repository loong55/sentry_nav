#!/bin/bash

# --- 配置参数 ---
TOPIC="/livox/imu"
DURATION=10
YAML_FILE="src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/nav2_params_mppi.yaml"
TEMP_DATA="imu_raw_output.log"

echo ">>> 开始采集话题 $TOPIC 数据，持续 $DURATION 秒..."

# 1. 采集原始数据流并保存
# 使用 timeout 10秒，重定向输出到临时文件
timeout ${DURATION}s ros2 topic echo $TOPIC > $TEMP_DATA 2>/dev/null

# 2. 从原始文本中提取 linear_acceleration 后的 x, y, z 数值
# 逻辑：找到 linear_acceleration 后的行，提取其数值并计算平均
echo ">>> 正在计算平均值（并取负号）..."

AVGS=$(awk '
    /linear_acceleration:/ {flag=1; next}
    flag && /x:/ {sumX += $2; countX++; next}
    flag && /y:/ {sumY += $2; countY++; next}
    flag && /z:/ {sumZ += $2; countZ++; flag=0; next}
    END {
        if (countX > 0) {
            # 计算平均值并乘以 -1
            printf "%.16f, %.16f, %.16f", -sumX/countX, -sumY/countY, -sumZ/countZ
        }
    }
' $TEMP_DATA)

# 检查是否成功获取数据
if [ -z "$AVGS" ]; then
    echo "错误：未能解析到数据。请确保话题正在发送且格式包含 linear_acceleration。"
    rm -f $TEMP_DATA
    exit 1
fi

# 3. 赋值变量
VAL_X=$(echo $AVGS | cut -d',' -f1 | tr -d ' ')
VAL_Y=$(echo $AVGS | cut -d',' -f2 | tr -d ' ')
VAL_Z=$(echo $AVGS | cut -d',' -f3 | tr -d ' ')

echo ">>> 计算结果 (Avg * -1):"
echo "X: $VAL_X"
echo "Y: $VAL_Y"
echo "Z: $VAL_Z"

# 4. 精准修改 YAML 文件
# 替换包含 gravity 和 gravity_init 的行，保留前面的缩进
if [ -f "$YAML_FILE" ]; then
    sed -i "s/\(gravity: \).*/\1[$VAL_X, $VAL_Y, $VAL_Z]/" "$YAML_FILE"
    sed -i "s/\(gravity_init: \).*/\1[$VAL_X, $VAL_Y, $VAL_Z]/" "$YAML_FILE"
    echo ">>> YAML 文件已更新: $YAML_FILE"
else
    echo "错误：找不到文件 $YAML_FILE"
fi

# 清理
rm -f $TEMP_DATA