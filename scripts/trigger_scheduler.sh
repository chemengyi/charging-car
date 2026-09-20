#!/bin/bash
# 手动触发调度器重新规划
ros2 topic pub --once /charge_scheduler/trigger std_msgs/msg/Bool "{data: true}"
