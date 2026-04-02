# isaac_sim/robot_config.py
# Physical parameters for the TurtleBot-like differential drive robot.
# These values are used by setup_scene.py and must stay consistent with
# the URDF in urdf/turtlebot_like.urdf.

ROBOT_CONFIG = {
    # ── Chassis ─────────────────────────────────────────────────────────────
    "base_width":  0.30,    # m, left-right extent
    "base_depth":  0.28,    # m, front-back extent
    "base_height": 0.14,    # m, vertical extent
    "base_mass":   2.0,     # kg

    # ── Drive wheels (left & right) ──────────────────────────────────────────
    "wheel_radius":     0.033,  # m
    "wheel_thickness":  0.018,  # m
    "wheel_mass":       0.1,    # kg
    # Lateral offset from robot center to wheel centerline
    "wheel_separation": 0.16,   # m (each side)
    # Longitudinal position (0 = centered on base)
    "wheel_x_offset":   0.0,    # m
    # Height of wheel axle above ground
    "wheel_z":          0.033,  # m  (= wheel_radius, sits on ground)

    # ── Caster wheels (passive, front & rear) ───────────────────────────────
    "caster_radius":  0.015,  # m
    "caster_mass":    0.05,   # kg
    "caster_x_front": 0.10,   # m (forward)
    "caster_x_rear": -0.10,   # m (rearward)
    "caster_y":       0.0,    # m (on center-line)
    "caster_z":       0.015,  # m (= caster_radius)

    # ── 2-D LiDAR ────────────────────────────────────────────────────────────
    "lidar_z_offset":  0.18,   # m above base bottom
    "lidar_range_min": 0.12,   # m
    "lidar_range_max": 3.5,    # m
    "lidar_fov_deg":   360.0,
    "lidar_num_rays":  360,
    "lidar_frequency": 10.0,   # Hz

    # ── Differential drive controller limits ─────────────────────────────────
    "max_linear_vel":  0.5,    # m/s
    "max_angular_vel": 1.0,    # rad/s
}
