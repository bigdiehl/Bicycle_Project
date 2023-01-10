"""
Return from /gazebo/get_model_properties bicycle
parent_model_name: ''
canonical_body_name: ''
body_names:
  - back_frame
  - back_wheel
  - front_frame
  - front_wheel
geom_names:
  - back_frame_collision
  - back_wheel_collision
  - front_frame_collision
  - front_wheel_collision
joint_names:
  - back_wheel_joint
  - steering_joint
  - front_wheel_joint
child_model_names: []
is_static: False



rosservice call /gazebo/get_link_properties back_frame
com:
  position:
    x: 0.439
    y: 0.0
    z: 0.579
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
gravity_mode: True
mass: 12.0
ixx: 0.476
ixy: 0.0
ixz: 0.0
iyy: 1.033
iyz: 0.0
izz: 0.527
success: True
status_message: "GetLinkProperties: got properties"

rosservice call /gazebo/get_joint_properties back_wheel_joint
type: 0
damping: []
position: [3863.1365657102865]
rate: [-0.005644390322604271]
success: True
status_message: "GetJointProperties: got properties"

"""



rosservice call /gazebo/set_link_properties "link_name : back_frame
com:
  position:
    x: 0.43
    y: 0.2
    z: 0.579
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
gravity_mode: True
mass: 12.0
ixx: 0.476
ixy: 0.0
ixz: 0.0
iyy: 1.033
iyz: 0.0
izz: 0.527
"