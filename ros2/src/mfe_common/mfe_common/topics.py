"""Topic names used by the planner, control, validation and simulation bridge nodes.

The values are the exact strings the nodes used before, so renaming a constant here is the only change needed
to re-route a topic. Parameter-driven topics (for example the YOLO cone topic) are not listed here.
"""

AS_SUPERVISOR_STATUS = '/as/supervisor/status'
CONTROL_COMMAND = '/control/command'
CONTROL_COMMAND_MPC = '/control/command_mpc'
EKF_OUTPUT = '/ekf/output'
GPS = '/gps'
GROUND_TRUTH_STATE = '/ground_truth/state'
GROUND_TRUTH_STATE_ODOM = '/ground_truth/state_odom'
GROUND_TRUTH_TRACK_COLORED = '/ground_truth/track_colored'
IMAGE_CONES = '/image/cones'
LIDAR_POINTS_RAW = '/lidar/points_raw'
MFE_EMERGENCY_BRAKE = '/mfe/emergency_brake'
PERCEPTION_CONES_UNCOLORED = '/perception/cones_uncolored'
PLANNING_CENTERLINE = '/planning/centerline'
PLANNING_CONES = '/planning/cones'
PLANNING_CONES_CAMERA_COLOURS = '/planning/cones_camera_colours'
PLANNING_LAPS_COMPLETED = '/planning/laps_completed'
PLANNING_MISSION_FINISHED = '/planning/mission_finished'
PLANNING_ORANGE_CONES = '/planning/orange_cones'
PLANNING_PATH_HORIZON = '/planning/path_horizon'
PLANNING_TARGET_SPEEDS = '/planning/target_speeds'
PLANNING_TRACK_LEFT = '/planning/track_left'
PLANNING_TRACK_RIGHT = '/planning/track_right'
ROS_CAN_MISSION_COMPLETED = '/ros_can/mission_completed'
SIM_XSENS_STATE_ODOM = '/sim/xsens/state_odom'
SKIDPAD_CONES = '/skidpad/cones'
SLAM_ODOM = '/slam/odom'
VALIDATION_CENTERLINE = '/validation/centerline'
VALIDATION_CONES = '/validation/cones'
