#!/bin/bash
# 发布返程最终位置 → /task_planner/return_goal
# 用法: bash pub_return_goal.sh [x] [y]

X=${1:-5.966083}
Y=${2:--39.846901}

echo "发布返程最终位置: ($X, $Y) → /task_planner/return_goal"
echo "Ctrl+C 停止"

trap 'echo "正在停止..."; docker exec autoware-humble-4.0.2 bash -c "pkill -f \"ros2 topic pub.*return_goal\"" 2>/dev/null; exit 0' INT TERM

docker exec autoware-humble-4.0.2 bash -c "
  cd .. && \
  source install/setup.bash && \
  ros2 topic pub --rate 10 /task_planner/return_goal geometry_msgs/msg/PoseStamped \
    \"{
      header: {frame_id: 'map'},
      pose: {
        position: {x: $X, y: $Y, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    }\"
" &

wait
