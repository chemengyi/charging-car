#!/bin/bash
# 发布虚拟目标车障碍物 → /task_planner/target_vehicle
# 用法: bash pub_target_vehicle.sh [x] [y]

X=${1:-5.72}
Y=${2:--9.2}

echo "发布虚拟目标车: ($X, $Y) → /task_planner/target_vehicle"
echo "Ctrl+C 停止"

# 捕获 Ctrl+C，杀掉容器内的 ros2 topic pub 进程
trap 'echo "正在停止..."; docker exec autoware-humble-4.0.2 bash -c "pkill -f \"ros2 topic pub.*target_vehicle\"" 2>/dev/null; exit 0' INT TERM

docker exec autoware-humble-4.0.2 bash -c "
  cd .. && \
  source install/setup.bash && \
  ros2 topic pub --rate 100 /task_planner/target_vehicle \
    autoware_auto_perception_msgs/msg/PredictedObjects \
    \"{
      header: {frame_id: 'map'},
      objects: [{
        classification: [{label: 1, probability: 1.0}],
        kinematics: {
          initial_pose_with_covariance: {
            pose: {
              position: {x: $X, y: $Y, z: 0.0},
              orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
            }
          }
        }
      }]
    }\"
" &

wait
