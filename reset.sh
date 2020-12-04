#!/bin/bash
script_name=$0
script_full_path=$(dirname "$0")
rostopic pub /gazebo/rwheel_traction_controller/command std_msgs/Float64 "data: 0"&

rostopic pub /gazebo/lwheel_traction_controller/command std_msgs/Float64 "data: 0"&
sleep 1
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'mr'
  pose:
    position:
      x: 0.0
      y: 0.5
      z: 0.1
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: 'map'"

