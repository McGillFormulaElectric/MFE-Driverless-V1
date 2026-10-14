"""
4x AMK DD5-14-10-POW in-hub motor drivetrain model.

Source: AMK official datasheet, "DD5-14-10-POW - Formula Student", part A2370DD-18600-B5,
motor_data_sheet_A2370DD_DD5.pdf (amk-motion.com), dated 27.11.2018.

Real datasheet numbers used here:
    Rated (continuous, S1) torque     "Mn": 9.8 Nm
    Continuous stall torque           "Mo": 13.8 Nm
    Peak torque                       "Mmax": 21.0 Nm  (thermally limited — see below)
    Peak current max duration                1.24 s at Imax=105 Arms
    Rated power                       "Pn": 12.3 kW
    Rated speed                       "Nn": 12000 rpm
    Theoretical no-load speed         "No": 18617 rpm
    Mechanical speed limit            "Nmax": 20000 rpm
    Motor mass                              3.55 kg

NOT from the datasheet (repo/vehicle assumptions — confirm and override via ROS params):
    gear_ratio        14.0   — motor:wheel = 14:1 reduction (torque x14 at the wheel),
                                in line with typical AMK-based FS in-hub builds (12.5:1-
                                14.8:1 seen across several published team writeups).
    wheel_radius_m    0.2525 — taken from EUFS's configDry.yaml tire.radius, NOT a real
                                MFE wheel/tire measurement.
    vehicle_mass_kg   225.0  — taken from EUFS's configDry.yaml inertia.m (its reference
                                vehicle), NOT the real MFE car's mass. Also doesn't yet
                                include the ~4x3.55=14.2kg of added unsprung hub-motor mass.
    max_system_power_w 80000 — generic FSAE combustion-equivalent accumulator power cap.
                                Confirm against MFE's actual accumulator/inverter rating.
    tire_mu           1.4    — generic FSAE slick-tire combined-grip placeholder, used only
                                to bound achievable longitudinal accel/decel (traction
                                circle), NOT a measured tire coefficient.

All of the above (except the motor's own datasheet numbers) are placeholders pending real
vehicle specs — they are exposed as ROS parameters on the bridge node specifically so they
can be corrected without touching this file.
"""
import math

# ---- AMK DD5-14-10-POW datasheet constants (do not treat as tunable) ----
RATED_TORQUE_NM = 9.8
CONTINUOUS_STALL_TORQUE_NM = 13.8
PEAK_TORQUE_NM = 21.0
PEAK_TORQUE_MAX_DURATION_S = 1.24
RATED_POWER_W = 12_300.0
NO_LOAD_SPEED_RPM = 18617.0
MAX_MECH_SPEED_RPM = 20000.0
MOTOR_MASS_KG = 3.55
N_MOTORS = 4

# Peak power read off the datasheet's torque/power-vs-speed curve (not a tabulated value):
# ~21 Nm x ~16000 rpm (field-weakening region) ~= 35 kW.
PEAK_POWER_W = 35_000.0

_RPM_TO_RAD_S = 2.0 * math.pi / 60.0


def _rpm_to_rad_s(rpm: float) -> float:
    return rpm * _RPM_TO_RAD_S


def _torque_speed_curve(motor_rpm: float, flat_torque_nm: float, power_w: float) -> float:
    """
    Generic PMSM torque-speed envelope: flat (constant-torque) region up to the speed
    where that torque would exceed `power_w`, then constant-power (field-weakening)
    down to zero at the motor's no-load speed.
    """
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
    Converts a normalized throttle/brake command + current vehicle speed into a single
    longitudinal acceleration (m/s^2), using 4x AMK DD5-14-10-POW in-hub motors.

    This intentionally mirrors the EUFS `gazebo_race_car_model` plugin's single-track
    (bicycle) vehicle model: it produces ONE scalar acceleration, the same interface the
    plugin's `commandMode:=acceleration` already accepts (`AckermannDriveStamped.drive.
    acceleration`). It does NOT implement genuine per-axle brake bias physics — the
    underlying sim vehicle model has no separate front/rear force inputs to apply that to.
    `brake_bias_front` is computed/exposed for reporting (e.g. sizing real brake hardware)
    but does not change the single combined deceleration this class returns.
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

        # Thermal "boost timer" for the 1.24s peak-torque duration limit: counts down
        # while drawing more than continuous-stall torque, recovers while below it.
        self._peak_budget_s = PEAK_TORQUE_MAX_DURATION_S

    # ------------------------------------------------------------------ #

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
            # Recover slowly while not drawing peak — 5x real-time, arbitrary but keeps
            # short throttle blips from permanently exhausting the budget.
            self._peak_budget_s = min(PEAK_TORQUE_MAX_DURATION_S, self._peak_budget_s + 5.0 * dt)

    # ------------------------------------------------------------------ #

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

        # Traction circle: can't exceed tire-limited longitudinal accel regardless of torque.
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

        # Below the brake fraction where regen alone covers the request, use regen only
        # (proportionally); beyond that, friction brakes make up the rest up to max_decel.
        requested_decel = brake * max_decel
        if requested_decel <= regen_decel:
            return requested_decel
        return min(max_decel, requested_decel)
