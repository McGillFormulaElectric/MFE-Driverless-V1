"""
AMK DD5-14-10-POW in-hub drivetrain model (4 motors, datasheet A2370DD-18600-B5).

Vehicle numbers (gear ratio, wheel radius, mass, power cap, tire mu) are placeholders, exposed as bridge ROS params.
"""
import math

# AMK DD5-14-10-POW datasheet constants, not tunable.
RATED_TORQUE_NM = 9.8
CONTINUOUS_STALL_TORQUE_NM = 13.8
PEAK_TORQUE_NM = 21.0
PEAK_TORQUE_MAX_DURATION_S = 1.24
RATED_POWER_W = 12_300.0
NO_LOAD_SPEED_RPM = 18617.0
MAX_MECH_SPEED_RPM = 20000.0
MOTOR_MASS_KG = 3.55
N_MOTORS = 4

# Read off the datasheet power curve (not tabulated): ~21 Nm at ~16000 rpm.
PEAK_POWER_W = 35_000.0

_RPM_TO_RAD_S = 2.0 * math.pi / 60.0


def _rpm_to_rad_s(rpm: float) -> float:
    return rpm * _RPM_TO_RAD_S


def _torque_speed_curve(motor_rpm: float, flat_torque_nm: float, power_w: float) -> float:
    """Flat torque up to the speed where it would exceed power_w, then constant power down to zero at no-load speed."""
    motor_rpm = max(0.0, motor_rpm)
    if motor_rpm >= NO_LOAD_SPEED_RPM:
        return 0.0
    omega = _rpm_to_rad_s(motor_rpm)
    if omega < 1e-6:
        return flat_torque_nm
    power_limited_torque = power_w / omega
    return min(flat_torque_nm, power_limited_torque)


class AMKDrivetrainModel:
    """
    Maps throttle/brake in [0, 1] and speed to one longitudinal acceleration (m/s^2).

    The EUFS single-track model has no per-axle force input, so brake_bias_front is reported only.
    """

    def __init__(self, gear_ratio: float, wheel_radius_m: float, vehicle_mass_kg: float,
                 max_system_power_w: float, tire_mu: float, brake_bias_front: float,
                 regen_enabled: bool):
        self.gear_ratio = gear_ratio
        self.wheel_radius_m = wheel_radius_m
        self.vehicle_mass_kg = vehicle_mass_kg
        self.max_system_power_w = max_system_power_w
        self.tire_mu = tire_mu
        self.brake_bias_front = brake_bias_front
        self.regen_enabled = regen_enabled

        # Peak-torque budget (s): drains above continuous torque, refills below it.
        self._peak_budget_s = PEAK_TORQUE_MAX_DURATION_S

    def _wheel_speed_to_motor_rpm(self, vehicle_speed_ms: float) -> float:
        wheel_rad_s = max(0.0, vehicle_speed_ms) / self.wheel_radius_m
        wheel_rpm = wheel_rad_s * 60.0 / (2.0 * math.pi)
        return wheel_rpm * self.gear_ratio

    def _available_motor_torque(self, motor_rpm: float, dt: float) -> float:
        """Per-motor available torque at this instant, respecting the 1.24s peak duration."""
        continuous = _torque_speed_curve(motor_rpm, CONTINUOUS_STALL_TORQUE_NM, RATED_POWER_W)
        if self._peak_budget_s <= 0.0:
            return continuous
        peak = _torque_speed_curve(motor_rpm, PEAK_TORQUE_NM, PEAK_POWER_W)
        return max(continuous, peak)

    def _consume_peak_budget(self, torque_used_nm: float, continuous_nm: float, dt: float) -> None:
        if torque_used_nm > continuous_nm + 1e-6:
            self._peak_budget_s = max(0.0, self._peak_budget_s - dt)
        else:
            # Refill at 5x real time so short throttle blips do not exhaust the budget.
            self._peak_budget_s = min(PEAK_TORQUE_MAX_DURATION_S, self._peak_budget_s + 5.0 * dt)

    def compute_acceleration(self, throttle: float, vehicle_speed_ms: float, dt: float) -> float:
        """Positive longitudinal acceleration (m/s^2) from throttle in [0, 1]."""
        throttle = max(0.0, min(1.0, throttle))
        motor_rpm = self._wheel_speed_to_motor_rpm(vehicle_speed_ms)
        continuous_nm = _torque_speed_curve(motor_rpm, CONTINUOUS_STALL_TORQUE_NM, RATED_POWER_W)
        available_nm = self._available_motor_torque(motor_rpm, dt)

        requested_nm = throttle * available_nm
        self._consume_peak_budget(requested_nm, continuous_nm, dt)

        # System (accumulator/inverter) power cap, split evenly across 4 motors.
        motor_omega = _rpm_to_rad_s(motor_rpm)
        requested_power_w = requested_nm * N_MOTORS * motor_omega
        if requested_power_w > self.max_system_power_w and motor_omega > 1e-6:
            requested_nm *= self.max_system_power_w / requested_power_w

        wheel_torque_total = requested_nm * N_MOTORS * self.gear_ratio
        tractive_force_n = wheel_torque_total / self.wheel_radius_m
        accel = tractive_force_n / self.vehicle_mass_kg

        # Traction circle limit.
        max_traction_accel = self.tire_mu * 9.81
        return max(0.0, min(accel, max_traction_accel))

    def compute_deceleration(self, brake: float, vehicle_speed_ms: float) -> float:
        """Positive deceleration magnitude (m/s^2) from brake in [0, 1]."""
        brake = max(0.0, min(1.0, brake))
        max_decel = self.tire_mu * 9.81
        if brake <= 0.0 or vehicle_speed_ms <= 0.0:
            return 0.0

        regen_decel = 0.0
        if self.regen_enabled:
            motor_rpm = self._wheel_speed_to_motor_rpm(vehicle_speed_ms)
            continuous_nm = _torque_speed_curve(motor_rpm, CONTINUOUS_STALL_TORQUE_NM, RATED_POWER_W)
            regen_force_n = (continuous_nm * N_MOTORS * self.gear_ratio) / self.wheel_radius_m
            regen_decel = min(regen_force_n / self.vehicle_mass_kg, max_decel)

        # Regen first; friction brakes cover the rest up to max_decel.
        requested_decel = brake * max_decel
        if requested_decel <= regen_decel:
            return requested_decel
        return min(max_decel, requested_decel)
