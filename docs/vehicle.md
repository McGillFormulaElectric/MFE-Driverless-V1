# MFE25 Vehicle Constants

Single source of truth for MFE25 geometry, mass, powertrain, and battery parameters.

File: `ros2/src/mfe_bringup/config/vehicle.yaml`

---

## Why a central YAML?

All nodes that need vehicle dimensions (pure pursuit, EKF, path planner) load the same file at launch rather than hard-coding values. Change one number in `vehicle.yaml` — every node picks it up on next launch.

---

## Geometry

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| `wheelbase` | 1.56 | m | Front-to-rear axle distance |
| `track_width` | 1.15 | m | Left-to-right wheel centre |
| `cg_height` | 0.25 | m | CG height above ground |
| `cg_x` | 0.44 | — | Longitudinal CG: 44% front / 56% rear |

---

## Mass

| Parameter | Value | Unit |
|-----------|-------|------|
| `mass_vehicle` | 198.0 | kg |
| `mass_driver` | 70.0 | kg |
| `mass_total` | 268.0 | kg |
| `mass_sprung` | 125.0 | kg |
| `mass_unsprung_f` | 30.0 | kg (front axle) |
| `mass_unsprung_r` | 30.0 | kg (rear axle) |
| `mass_wheel` | 4.1 | kg (rim + tyre) |

Static corner loads (no aero, 268 kg × 9.81 m/s²):
- Front per wheel ≈ **581 N** (44% × 268 × 9.81 / 2)
- Rear per wheel ≈ **739 N** (56% × 268 × 9.81 / 2)

---

## Powertrain

MFE25 is **not** a hub-motor car. Four Fisher motors drive through a **13.39:1 chain reduction** — one motor per wheel, 4WD.

| Parameter | Value | Unit |
|-----------|-------|------|
| `motor_type` | Fisher | — |
| `motor_count` | 4 | — |
| `gear_ratio` | 13.39 | — |
| `motor_peak_torque_Nm` | 29.0 | Nm per motor |
| `motor_peak_power_kW` | 41.5 | kW per motor (166 kW total) |
| `motor_max_rpm` | 20 000 | RPM |
| `max_pack_power_kW` | 66.7 | kW (80 kW / 1.2 scaling) |
| `inverter_eta` | 0.98 | — |
| `torque_split_rear` | 0.65 | 65% rear / 35% front |
| `tyre_radius` | 0.2023 | m (rolling radius Re0) |

Derived values at peak torque:
- Wheel torque: 29 × 13.39 = **388 Nm/wheel**
- Tractive force: 388 / 0.2023 ≈ **1 919 N/wheel**
- v_max: 20 000 RPM → **≈ 31 m/s** at wheel

---

## Battery

ThunderPower **AE8040160**, 144s2p configuration (288 cells total).

| Parameter | Value | Unit |
|-----------|-------|------|
| `battery_series` | 144 | cells |
| `battery_parallel` | 2 | cells |
| `battery_voltage_V` | 600.0 | V nominal |
| `battery_energy_kWh` | 6.396 | kWh usable |
| `battery_mass_kg` | 46.9 | kg |
| `battery_max_current_A` | 150.0 | A continuous |
| `regen_enabled` | false | — |

Cell specs: 6 Ah, 3.7 V nominal, 4.2 V max, 150 A peak discharge.

---

## Aerodynamics

| Parameter | Value | Unit |
|-----------|-------|------|
| `CL` | −4.26 | — (negative = downforce) |
| `CD` | 1.77 | — |
| `Aref` | 1.106 | m² |
| `aero_balance` | 0.42 | 42% front / 58% rear |

---

## Suspension

| Parameter | Value | Unit |
|-----------|-------|------|
| `spring_rate_front_lb_in` | 450 | lb/in |
| `spring_rate_rear_lb_in` | 750 | lb/in |
| `roll_center_height_f` | 0.106 | m |
| `roll_center_height_r` | 0.135 | m |
| `chassis_torsional_stiffness_Nm_deg` | 1 800 | Nm/deg |
| `front_brake_bias` | 0.55 | 55% front / 45% rear |

---

## Control Limits

These are the **software defaults** loaded by `pure_pursuit_node`. Physical URDF steering joint limit is ±29.8° (0.52 rad).

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| `max_steering_deg` | 28.0 | deg | Stay under URDF joint limit of ±29.8° |
| `max_lateral_accel` | 8.0 | m/s² | Corner speed: v = √(a × R) |
| `max_deceleration` | 10.0 | m/s² | |

Mission-specific overrides applied on top of these defaults:

| Mission | max_speed | lookahead | Notes |
|---------|-----------|-----------|-------|
| autocross / trackdrive | 10.0 m/s | 7.0 m | max_lateral_accel overridden to 5.0 m/s² |
| skidpad | 4.5 m/s | 3.0 m | Tight 9.1 m radius circles |
| acceleration | 13.0 m/s | 8.0 m | Full throttle straight |

---

## How vehicle.yaml is loaded

```python
# In bringup.launch.py
_VEHICLE_YAML = os.path.join(
    get_package_share_directory('mfe_bringup'), 'config', 'vehicle.yaml'
)

# Node using it (e.g. pure_pursuit)
Node(
    package='mfe_control',
    executable='pure_pursuit_node',
    parameters=[_VEHICLE_YAML, mission_params],  # mission_params override
)
```

`CMakeLists.txt` installs the config directory so it ships with the built package:
```cmake
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
```
