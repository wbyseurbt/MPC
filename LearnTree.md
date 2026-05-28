one_start_gazebo.launch
├── empty_world.launch (legged_dm_description)
│   ├── empty_world.launch (legged_gazebo)
│   │   ├── /gazebo          [gazebo_ros / gzserver]
│   │   └── /gazebo_gui      [gazebo_ros / gzclient]     (gui=true)
│   └── /spawn_urdf          [gazebo_ros / spawn_model]
│
├── /timedelay_launch        [legged_controllers / timedelay_launch.sh]  ← 包装节点
│   └── (8s 后) load_controller_sim.launch
│       ├── /controller_loader           [controller_manager / spawner]
│       │   └── 插件: joint_state_controller, legged_controller
│       └── /legged_robot_target         [legged_controllers / legged_target_trajectories_publisher]
│
├── /rviz                    [rviz / rviz]                 (rviz=true)
│
├── record_data.launch       (record_data=true，文件缺失)
│
└── /timedelay_rqt           [legged_controllers / timedelay_launch.sh]  ← 包装节点
    └── (12s 后) start_rqt_reconfigure.launch
        └── /rqt_reconfigure [rqt_reconfigure / rqt_reconfigure]



one_start_real.launch
├── legged_dm_hw.launch (legged_dm_hw)
│   ├── /legged_dm_hw         [legged_dm_hw / legged_dm_hw]
│   └── /dm_imu_node          [dm_imu / dm_imu_node]
│
├── /timedelay_launch         [legged_controllers / timedelay_launch.sh]  ← 包装节点
│   └── (2s 后) load_controller_real.launch
│       ├── /controller_loader           [controller_manager / spawner]
│       │   └── 插件: legged_controller (cheater=true 时额外加载 legged_cheater_controller)
│       └── /legged_robot_target         [legged_controllers / legged_target_trajectories_publisher]
│
├── /rviz                     [rviz / rviz]                 (rviz=true)
│
├── record_data.launch        (record_data=true，文件缺失)
│
└── /rqt_reconfigure          [rqt_reconfigure / rqt_reconfigure]

load_controller_sim.launch
├── /controller_loader        [controller_manager / spawner]
│   └── 插件: joint_state_controller, legged_controller
└── /legged_robot_target      [legged_controllers / legged_target_trajectories_publisher]

load_controller_real.launch
├── joy_teleop.launch
│   ├── /joy_node             [joy / joy_node]
│   └── /joy_teleop           [joy_teleop / joy_teleop.py]
├── /controller_loader        [controller_manager / spawner]
│   └── 插件: legged_controller (cheater=true 时额外加载 legged_cheater_controller)
└── /legged_robot_target      [legged_controllers / legged_target_trajectories_publisher]

start_rqt_reconfigure.launch
└── /rqt_reconfigure          [rqt_reconfigure / rqt_reconfigure]

joy_teleop.launch
├── /joy_node                 [joy / joy_node]
└── /joy_teleop               [joy_teleop / joy_teleop.py]

test_joy.launch
├── /joy_node                 [joy / joy_node]
└── /teleop_node              [teleop_twist_joy / teleop_node]

rosbag_topic.launch
└── /rosbag_record            [rosbag / record]



leg_l1_joint / leg_r1_joint：axis="0 1 0"，绕 +Y 为正转，绕 -Y 为反转
leg_l2_joint / leg_r2_joint：axis="1 0 0"，绕 +X 为正转，绕 -X 为反转
leg_l3_joint / leg_r3_joint：axis="0 0 1"，绕 +Z 为正转，绕 -Z 为反转
leg_l4_joint / leg_r4_joint：axis="0 1 0"，绕 +Y 为正转，绕 -Y 为反转
leg_l5_joint / leg_r5_joint：axis="0 1 0"，绕 +Y 为正转，绕 -Y 为反转