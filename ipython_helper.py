
## creation of a simulated robot controller and viewer, for sim2sim

from robot.sim_robot import SimBipedalRobotController

robot = SimBipedalRobotController(control_hz=200.0, fixed_base=False)
robot.start(mode="control", auto_enable=True)
robot.start_viewer()



### Creation of a real robot controller, IMU, and RL agent, and connection to gamepad

from apps.gamepad_controller import GamepadController
from control.RL_agent_isolated import RLAgent
from robot.bipedal_robot import BipedalRobotController
from imu.IMU_integration import IMU


imu = IMU(sensor="bno055", i2c_bus=1, address=0x28, rate_hz=100.0, frame_yaw_deg=-180.0)

robot = BipedalRobotController(control_hz=100.0, imu=imu)
robot.attach_default_meshcat()   # optional
robot.set_max_command_delta(80.0)

robot.start(mode="state_only", auto_enable=False)

## CHECK MESHCAT VISUALIZATION, JOINT LIMITS, AND IMU ORIENTATION


robot._viz_hz = 20.0            # reduce lag a lot


robot.set_mode("control")


robot.enable_all()  # can be done several time to ensure evry motor is enabled



robot.set_action(
left={
    "hipz": 0.0,
    "hipx": 0.0,
    "hipy": 0,
    "knee": 0.,
    "ankle_pitch": 0.0,  # from ankley_left
    "ankle_roll": 0.0,        # from anklex_left
},
right={
    "hipz": 0.0,
    "hipx": 0.0,
    "hipy": 0.0,
    "knee": 00.,
    "ankle_pitch": 0.0,   # from ankley_right
    "ankle_roll": 0.0,        # from anklex_right
},
)


#CHECK thaht evry joint is actionned


for mid in [1,7]:
    robot.set_joint_gains(mid, kp=30, kd=3.0) 

for mid in [5,6,11,12]:
    robot.set_joint_gains(mid, kp=10, kd=0.75) 


robot.set_joint_gains(2, kp=40, kd=3.0) 
robot.set_joint_gains(3, kp=6, kd=4/20) 
robot.set_joint_gains(4, kp=6, kd=4/20) 
robot.set_joint_gains(8, kp=40, kd=3.0) 
robot.set_joint_gains(9, kp=6, kd=4/20)
robot.set_joint_gains(10, kp=6, kd=4/20)


pad = GamepadController(
    name_substring="8bitdo",
    deadzone=0.12,
    max_lin_x=0.75,
    max_lin_y=0.5,
    max_yaw_rate=0.8,
)
pad.connect()
pad.start()

agent = RLAgent.from_files(
    robot,
    config_path="control/policy/codex_iteration_history/config.yaml",
    policy_path="control/policy/codex_iteration_history/policy.onnx", #2026-03-04_17-28-46.onnx",
    log_path="control/policy/codex_iteration_history/debug_ctrl_sim.csv",
    log_observation=True,
    log_action=True,
    log_every_n=1,
)

agent.spec.joint_vel_source = "finite_diff"  # use explicit finite-diff mode

# manual global scaling remains available
agent.spec.action_scale = 0.
# pad provides (lin_x, lin_y, yaw_rate) commands
agent.set_command_source(pad)
agent.start()
