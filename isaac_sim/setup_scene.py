"""
isaac_sim/setup_scene.py
========================
TurtleBot-like robot in Isaac Sim 4.x with Nav2 ROS2 bridge.

Launch with:
    $ISAAC_SIM_ROOT/python.sh isaac_sim/setup_scene.py [--headless]

The script:
  1. Boots SimulationApp (must happen before any omni.* imports)
  2. Creates a flat 10 × 10 m arena with box obstacles
  3. Builds a differential-drive robot programmatically via USD API
  4. Attaches an RTX 2-D LiDAR
  5. Wires up OmniGraph ROS2 bridge nodes for:
       /scan        (sensor_msgs/LaserScan)
       /odom        (nav_msgs/Odometry)
       /tf          (geometry_msgs/TransformStamped)
       /cmd_vel     (geometry_msgs/Twist  →  DifferentialController)
  6. Runs the simulation loop
"""

import argparse
import sys
import os

# ── 0. CLI args (must parse before SimulationApp) ────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true",
                    help="Run without GUI (for CI / remote servers)")
args, _ = parser.parse_known_args()

# ── 1. Boot SimulationApp ────────────────────────────────────────────────────
from omni.isaac.kit import SimulationApp  # noqa: E402  (must be first omni import)

simulation_app = SimulationApp({
    "renderer": "RayTracedLighting",
    "headless": args.headless,
    "width":  1280,
    "height":  720,
})

# ── 2. Post-boot imports ─────────────────────────────────────────────────────
import numpy as np
import carb                                          # noqa: F401

import omni
import omni.graph.core as og
from omni.isaac.core import World
from omni.isaac.core.objects import GroundPlane, DynamicCuboid
from omni.isaac.core.utils.prims import get_prim_at_path, is_prim_path_valid
from omni.isaac.wheeled_robot.robots import WheeledRobot
from omni.isaac.wheeled_robot.controllers import DifferentialController

# USD / Physics Python APIs
from pxr import (
    Usd, UsdGeom, UsdPhysics, PhysxSchema,
    Gf, Sdf, Vt,
)

# Project-local config (same directory as this script)
sys.path.insert(0, os.path.dirname(__file__))
from robot_config import ROBOT_CONFIG as RC

# ── Constants ────────────────────────────────────────────────────────────────
ROBOT_ROOT   = "/World/TurtleBot"
LIDAR_PATH   = ROBOT_ROOT + "/base_link/lidar"
GROUND_PATH  = "/World/Ground"

TOPIC_SCAN      = "/scan"
TOPIC_ODOM      = "/odom"
TOPIC_CMD_VEL   = "/cmd_vel"
TOPIC_TF        = "/tf"
TOPIC_TF_STATIC = "/tf_static"

FRAME_ODOM  = "odom"
FRAME_BASE  = "base_footprint"
FRAME_LIDAR = "base_scan"


# ─────────────────────────────────────────────────────────────────────────────
# Helper: build robot prim tree on USD stage
# ─────────────────────────────────────────────────────────────────────────────

def _xform(stage, path, translate=(0, 0, 0), orient_quat=None):
    """Define an Xform prim and set an optional translation."""
    prim = stage.DefinePrim(path, "Xform")
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(*translate))
    if orient_quat is not None:
        # orient_quat = (w, x, y, z)
        xf.AddOrientOp().Set(Gf.Quatd(*orient_quat))
    return prim


def _add_box_collision(stage, parent_path, half_extents, translate=(0, 0, 0)):
    col_path = parent_path + "/collision"
    cube = UsdGeom.Cube.Define(stage, col_path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*translate))
    xf.AddScaleOp().Set(Gf.Vec3d(*half_extents))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def _add_cylinder_collision(stage, parent_path, radius, height):
    col_path = parent_path + "/collision"
    cyl = UsdGeom.Cylinder.Define(stage, col_path)
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetHeightAttr().Set(height)
    cyl.GetAxisAttr().Set("Z")
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    return cyl


def build_robot(stage):
    """
    Construct differential-drive robot as USD articulation.

    Tree:
      /World/TurtleBot            (ArticulationRoot + RigidBody)
        /base_link                (chassis geometry)
        /left_wheel               (revolute joint, driven)
        /right_wheel              (revolute joint, driven)
        /front_caster             (sphere, free)
        /rear_caster              (sphere, free)
        /base_link/lidar          (Xform placeholder — sensor attached later)
    """
    # ── Articulation root ────────────────────────────────────────────────────
    robot_prim = _xform(stage, ROBOT_ROOT, translate=(0, 0, RC["wheel_z"]))
    UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
    art_api = PhysxSchema.PhysxArticulationAPI.Apply(robot_prim)
    art_api.GetEnabledSelfCollisionsAttr().Set(False)

    # ── base_link ────────────────────────────────────────────────────────────
    base_path = ROBOT_ROOT + "/base_link"
    base_prim = _xform(stage, base_path)
    rb = UsdPhysics.RigidBodyAPI.Apply(base_prim)
    mass_api = UsdPhysics.MassAPI.Apply(base_prim)
    mass_api.GetMassAttr().Set(RC["base_mass"])

    # Chassis visual + collision box
    chassis_z = RC["base_height"] / 2.0
    vis_path = base_path + "/chassis_vis"
    chassis = UsdGeom.Cube.Define(stage, vis_path)
    chassis.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(chassis.GetPrim())
    xf.AddScaleOp().Set(Gf.Vec3d(
        RC["base_depth"] / 2.0,
        RC["base_width"] / 2.0,
        RC["base_height"] / 2.0,
    ))
    xf.AddTranslateOp().Set(Gf.Vec3d(0, 0, chassis_z))
    _add_box_collision(
        stage, base_path,
        half_extents=(RC["base_depth"] / 2.0, RC["base_width"] / 2.0, RC["base_height"] / 2.0),
        translate=(0, 0, chassis_z),
    )

    # LiDAR mount point
    lidar_prim = _xform(stage, LIDAR_PATH,
                        translate=(0, 0, RC["lidar_z_offset"]))

    # ── Drive wheels ─────────────────────────────────────────────────────────
    r = RC["wheel_radius"]
    t = RC["wheel_thickness"]
    for side, y_sign in [("left_wheel", 1), ("right_wheel", -1)]:
        y = y_sign * RC["wheel_separation"]
        wpath = ROBOT_ROOT + f"/{side}"
        wprim = _xform(stage, wpath, translate=(RC["wheel_x_offset"], y, 0))

        # Cylinder visual (axis = Y → axle direction)
        cyl = UsdGeom.Cylinder.Define(stage, wpath + "/visual")
        cyl.GetRadiusAttr().Set(r)
        cyl.GetHeightAttr().Set(t)
        cyl.GetAxisAttr().Set("Y")

        # Collision
        col = UsdGeom.Cylinder.Define(stage, wpath + "/collision")
        col.GetRadiusAttr().Set(r)
        col.GetHeightAttr().Set(t)
        col.GetAxisAttr().Set("Y")
        UsdPhysics.CollisionAPI.Apply(col.GetPrim())

        # Rigid body + mass
        UsdPhysics.RigidBodyAPI.Apply(wprim)
        m = UsdPhysics.MassAPI.Apply(wprim)
        m.GetMassAttr().Set(RC["wheel_mass"])

        # Revolute joint (wheel spins around Y axis)
        jpath = ROBOT_ROOT + f"/{side}_joint"
        joint = UsdPhysics.RevoluteJoint.Define(stage, jpath)
        joint.GetAxisAttr().Set("Y")
        joint.GetBody0Rel().SetTargets([Sdf.Path(base_path)])
        joint.GetBody1Rel().SetTargets([Sdf.Path(wpath)])
        joint.GetLocalPos0Attr().Set(Gf.Vec3f(RC["wheel_x_offset"], y, 0))
        joint.GetLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

        # Drive API on the joint
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
        drive.GetTypeAttr().Set("force")
        drive.GetMaxForceAttr().Set(1000.0)
        drive.GetDampingAttr().Set(1e4)
        drive.GetStiffnessAttr().Set(0.0)

    # ── Caster wheels (passive spheres) ──────────────────────────────────────
    cr = RC["caster_radius"]
    for name, cx in [("front_caster", RC["caster_x_front"]),
                     ("rear_caster",  RC["caster_x_rear"])]:
        cpath = ROBOT_ROOT + f"/{name}"
        cprim = _xform(stage, cpath,
                       translate=(cx, RC["caster_y"], -RC["wheel_z"] + cr))
        sph = UsdGeom.Sphere.Define(stage, cpath + "/visual")
        sph.GetRadiusAttr().Set(cr)
        csph = UsdGeom.Sphere.Define(stage, cpath + "/collision")
        csph.GetRadiusAttr().Set(cr)
        UsdPhysics.CollisionAPI.Apply(csph.GetPrim())

        UsdPhysics.RigidBodyAPI.Apply(cprim)
        m = UsdPhysics.MassAPI.Apply(cprim)
        m.GetMassAttr().Set(RC["caster_mass"])

        # Fixed joint to base (casters don't steer — they slide freely)
        jpath = ROBOT_ROOT + f"/{name}_joint"
        fj = UsdPhysics.FixedJoint.Define(stage, jpath)
        fj.GetBody0Rel().SetTargets([Sdf.Path(base_path)])
        fj.GetBody1Rel().SetTargets([Sdf.Path(cpath)])
        fj.GetLocalPos0Attr().Set(Gf.Vec3f(cx, RC["caster_y"], -RC["wheel_z"] + cr))
        fj.GetLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))

    return robot_prim


# ─────────────────────────────────────────────────────────────────────────────
# Helper: add obstacles
# ─────────────────────────────────────────────────────────────────────────────

OBSTACLE_SPECS = [
    # (x,  y,  half_w, half_d, half_h)
    ( 2.0,  1.0, 0.20, 0.20, 0.30),
    (-2.0,  1.5, 0.15, 0.40, 0.30),
    ( 0.5, -2.5, 0.30, 0.15, 0.30),
    (-1.5, -1.0, 0.25, 0.25, 0.30),
    ( 3.0, -1.5, 0.15, 0.50, 0.30),
]


def add_obstacles(world):
    for i, (ox, oy, hw, hd, hh) in enumerate(OBSTACLE_SPECS):
        world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/Obstacle_{i}",
                name=f"obstacle_{i}",
                position=np.array([ox, oy, hh]),
                scale=np.array([hw * 2, hd * 2, hh * 2]),
                mass=1000.0,  # effectively static
                color=np.array([0.6, 0.3, 0.1]),
            )
        )


# ─────────────────────────────────────────────────────────────────────────────
# Helper: attach RTX LiDAR sensor
# ─────────────────────────────────────────────────────────────────────────────

def attach_lidar(simulation_app):
    """
    Create an RTX LiDAR sensor at LIDAR_PATH.
    Uses the generic 2-D lidar profile shipped with Isaac Sim.
    """
    import omni.kit.commands
    _, sensor = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path=LIDAR_PATH,
        parent=None,
        config="Example_Rotary",   # 360° rotary profile included in Isaac Sim
        translation=Gf.Vec3f(0, 0, 0),
        orientation=Gf.Quatd(1, 0, 0, 0),
    )
    return sensor


# ─────────────────────────────────────────────────────────────────────────────
# Helper: wire up ROS2 OmniGraph bridge
# ─────────────────────────────────────────────────────────────────────────────

def setup_ros2_graph():
    """
    Build an OmniGraph action graph that:
      - Publishes /scan  (LaserScan)  from the RTX lidar
      - Publishes /odom  (Odometry)   from the articulation
      - Publishes /tf                 (odom→base_footprint transform)
      - Publishes /tf_static          (base_footprint→base_scan)
      - Subscribes /cmd_vel and drives the DifferentialController
    """
    keys = og.Controller.Keys

    (graph, nodes, _, _) = og.Controller.edit(
        {
            "graph_path": "/World/ROS2Graph",
            "evaluator_name": "execution",
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick",  "omni.graph.action.OnPlaybackTick"),

                # ── Clock / simulation time ──────────────────────────────────
                ("SimTime",        "omni.isaac.core_nodes.IsaacReadSimulationTime"),

                # ── Odometry + TF ────────────────────────────────────────────
                ("ComputeOdom",    "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                ("ComputeTF",      "omni.isaac.ros2_bridge.ROS2PublishTransformTree"),

                # ── LiDAR → /scan ────────────────────────────────────────────
                ("LidarHelper",    "omni.isaac.ros2_bridge.ROS2RTXLidarHelper"),

                # ── /cmd_vel subscriber ──────────────────────────────────────
                ("SubCmdVel",      "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
                ("DiffCtrl",       "omni.isaac.wheeled_robot.DifferentialController"),
                ("ArticCtrl",      "omni.isaac.core_nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                # Sim time
                ("SimTime.inputs:resetOnStop", True),

                # Odometry node
                ("ComputeOdom.inputs:odomFrameId",       FRAME_ODOM),
                ("ComputeOdom.inputs:chassisFrameId",    FRAME_BASE),
                ("ComputeOdom.inputs:topicName",         TOPIC_ODOM),
                ("ComputeOdom.inputs:robotPath",         ROBOT_ROOT),

                # TF publisher
                ("ComputeTF.inputs:targetPrims",  [ROBOT_ROOT]),

                # LiDAR helper
                ("LidarHelper.inputs:lidarPrimPath",    LIDAR_PATH),
                ("LidarHelper.inputs:topicName",        TOPIC_SCAN),
                ("LidarHelper.inputs:frameId",          FRAME_LIDAR),
                ("LidarHelper.inputs:renderProductPath", ""),  # auto-detected

                # cmd_vel subscriber
                ("SubCmdVel.inputs:topicName",      TOPIC_CMD_VEL),

                # Differential controller (converts Twist → wheel velocities)
                ("DiffCtrl.inputs:wheelRadius",      RC["wheel_radius"]),
                ("DiffCtrl.inputs:wheelDistance",    RC["wheel_separation"] * 2.0),
                ("DiffCtrl.inputs:maxLinearSpeed",   RC["max_linear_vel"]),
                ("DiffCtrl.inputs:maxAngularSpeed",  RC["max_angular_vel"]),

                # Articulation controller
                ("ArticCtrl.inputs:robotPath",       ROBOT_ROOT),
                ("ArticCtrl.inputs:jointNames",      ["left_wheel_joint",
                                                      "right_wheel_joint"]),
                ("ArticCtrl.inputs:usePath",         True),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick",  "SimTime.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick",  "ComputeOdom.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick",  "ComputeTF.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick",  "LidarHelper.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick",  "SubCmdVel.inputs:execIn"),

                ("SimTime.outputs:simulationTime", "ComputeOdom.inputs:timeStamp"),
                ("SimTime.outputs:simulationTime", "ComputeTF.inputs:timeStamp"),

                ("SubCmdVel.outputs:linearVelocity",  "DiffCtrl.inputs:linearVelocity"),
                ("SubCmdVel.outputs:angularVelocity", "DiffCtrl.inputs:angularVelocity"),

                ("DiffCtrl.outputs:velocityCommand",  "ArticCtrl.inputs:velocityCommand"),
                ("OnPlaybackTick.outputs:tick",        "ArticCtrl.inputs:execIn"),
            ],
        },
    )
    return graph


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    # World with 60 Hz physics
    world = World(physics_dt=1.0 / 60.0, rendering_dt=1.0 / 30.0,
                  stage_units_in_meters=1.0)

    # Ground plane
    world.scene.add_default_ground_plane()

    # Obstacles
    add_obstacles(world)

    # Build robot on USD stage
    stage = omni.usd.get_context().get_stage()
    build_robot(stage)

    # LiDAR sensor
    attach_lidar(simulation_app)

    # ROS2 OmniGraph bridge
    world.reset()          # initialize physics before building graph
    setup_ros2_graph()

    # ── Simulation loop ───────────────────────────────────────────────────────
    world.reset()
    print("[isaac_nav2] Simulation running. Press Ctrl-C or close the window to stop.")

    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
