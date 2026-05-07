#!/usr/bin/env python

from dataclasses import dataclass, field

from lerobot.robots import RobotConfig



@RobotConfig.register_subclass("lerobot_humanoid")
@dataclass
class LeRobotHumanoidConfig(RobotConfig):
    # Kinematic/base metadata kept for compatibility with sim-side config usage.
    # These fields are not used by the real hardware controller.
    fixed_base: bool = False
    fixed_base_height_m: float = 0.77

    # Two CAN buses. ID-to-bus routing is defined in `lerobot_humanoid.py`.
    can0_port: str = "can0"
    can1_port: str = "can1"
    can_interface: str = "socketcan"
    use_can_fd: bool = True
    can_bitrate: int = 1_000_000
    can_data_bitrate: int = 5_000_000
    handshake: bool = True

    # Use local robots.mock_bus.MockBus instead of real CAN hardware.
    use_mock_bus: bool = False
    mock_bus_default_temp_c: float = 30.0
    mock_bus_send_sleep_s: float = 0.0

    # Background control loop
    control_hz: float = 100.0
    auto_enable_control: bool = True

    # Torque behavior
    enable_torque_on_connect: bool = True
    disable_torque_on_disconnect: bool = True

    # IMU backend from robots/IMU_integration.py (BNO055 only)
    imu_enabled: bool = True
    imu_bno055_i2c_bus: int = 1
    imu_bno055_address: int = 0x28
    imu_poll_hz: float = 100.0
    imu_mock: bool = False
    imu_mock_quaternion_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    imu_mock_gyro_rads: tuple[float, float, float] = (0.0, 0.0, 0.0)
    imu_mock_linear_velocity_mps: tuple[float, float, float] = (0.0, 0.0, 0.0)
    imu_mock_linear_acceleration_mps2: tuple[float, float, float] = (0.0, 0.0, 0.0)

    # Safety margins in raw motor degrees
    clamp_raw_targets: bool = True
    command_margin_deg: float = 1.0
    state_margin_deg: float = 3.0
    startup_wrap_correction_enabled: bool = True
    startup_wrap_shift_limits: bool = False
    # Match the default real-robot deployment guard used by bipedal controller scripts.
    max_command_delta_deg: float = 60.0
    ankle_guard_enabled: bool = True
    ankle_guard_abs_deg: float = 90.0

    # Hardcoded per-motor raw limits (deg), used directly for safety clamping.
    # No manual calibration workflow is required.
    joint_limits_deg: dict[int, tuple[float, float]] = field(
        default_factory=lambda: {
            1: (-209.123, -47.685),
            2: (-129.887, 12.934),
            3: (-168.065, 1.967),
            4: (0.978, 112.172),
            5: (-89.753, 11.542),
            6: (-18.14, 285.830),
            7: (64.894, 215.255),
            8: (-55.311, 40.409),
            9: (-0.055, 162.438),
            10: (-98.105, -0.693),
            11: (-12.50, 87.972),
            12: (-78.191, 13.9),
        }
    )

    # Per-motor default gains (indexed by motor id 1..12)
    position_kp: dict[int, float] = field(
        default_factory=lambda: {
            1: 5.0,
            2: 5.0,
            3: 15.0,
            4: 15.0,
            5: 8.0,
            6: 8.0,
            7: 5.0,
            8: 5.0,
            9: 15.0,
            10: 15.0,
            11: 8.0,
            12: 8.0,
        }
    )
    position_kd: dict[int, float] = field(
        default_factory=lambda: {
            1: 0.5,
            2: 0.5,
            3: 1.5,
            4: 1.5,
            5: 0.5,
            6: 0.5,
            7: 0.5,
            8: 0.5,
            9: 1.5,
            10: 1.5,
            11: 0.5,
            12: 0.5,
        }
    )
