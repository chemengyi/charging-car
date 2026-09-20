#!/bin/bash
# 发布充电完成信号 → /task_planner/charge_complete
# 用法: bash pub_charge_complete.sh

echo "发布充电完成信号 → /task_planner/charge_complete"

docker exec autoware-humble-4.0.2 bash -c "
  cd .. && \
  source install/setup.bash && \
  ros2 topic pub --once /task_planner/charge_complete std_msgs/msg/Bool '{data: true}'
"

echo "已发送!"
