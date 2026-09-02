#!/usr/bin/env python3
"""Render an OIM open_table Franka rollout from native sim telemetry.

Consumes the joint-count-generic telemetry the native sim writes with
--record_dir (summary.json / steps.jsonl / contacts.jsonl) plus the planner
log, builds the real Franka+stick scene from the same URDF the run used, and
writes a single-pane mp4 with a HUD (step, time, errors, contact) and an
aggregate trial-stats overlay. Trial-ledger/statistics helpers are reused
from scripts/render_oim_xarm_rollout.py, which is joint-model agnostic.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image, ImageDraw

REPO = Path(__file__).resolve().parents[3]
ADMM = Path("/root/push_anything_ADMM")
sys.path.insert(0, str(ADMM / "scripts"))
import render_oim_xarm_rollout as xr  # noqa: E402  (stats/planner helpers)

from pydrake.geometry import (  # noqa: E402
    Box, ClippingRange, ColorRenderCamera, DepthRange, DepthRenderCamera,
    MakeRenderEngineVtk, RenderCameraCore, RenderEngineVtkParams, Rgba,
)
from pydrake.math import RigidTransform, RollPitchYaw  # noqa: E402
from pydrake.multibody.parsing import Parser  # noqa: E402
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph  # noqa: E402
from pydrake.multibody.tree import BodyIndex  # noqa: E402
from pydrake.systems.framework import DiagramBuilder  # noqa: E402
from pydrake.systems.sensors import CameraInfo, RgbdSensor  # noqa: E402


def build_scene(robot_urdf: Path, object_sdf: Path, width=1280, height=720):
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    renderer = "franka_render"
    scene_graph.AddRenderer(renderer,
                            MakeRenderEngineVtk(RenderEngineVtkParams()))
    parser = Parser(plant, scene_graph)
    robot = parser.AddModels(str(robot_urdf))[0]
    obj = parser.AddModels(str(object_sdf))[0]
    # Same programmatic table the native sim registers.
    X_WTable = RigidTransform([0.35, 0.0, -0.455])
    plant.RegisterVisualGeometry(plant.world_body(), X_WTable,
                                 Box(0.8, 1.523, 0.91), "open_table_visual",
                                 [0.93, 0.93, 0.90, 1.0])
    plant.Finalize()

    camera_core = RenderCameraCore(
        renderer, CameraInfo(width=width, height=height, fov_y=np.pi / 4.5),
        ClippingRange(0.05, 10.0), RigidTransform())
    color_camera = ColorRenderCamera(camera_core, show_window=False)
    depth_camera = DepthRenderCamera(camera_core, DepthRange(0.05, 10.0))
    # Look from the +x/-y front quarter, down at the table center.
    eye = np.array([1.55, -1.05, 0.85])
    target = np.array([0.35, 0.05, 0.05])
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, np.array([0.0, 0.0, 1.0]))
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    from pydrake.math import RotationMatrix
    X_WC = RigidTransform(RotationMatrix(R), eye)
    camera = builder.AddSystem(RgbdSensor(
        scene_graph.world_frame_id(), X_WC, color_camera, depth_camera))
    builder.Connect(scene_graph.get_query_output_port(),
                    camera.query_object_input_port())
    diagram = builder.Build()
    return diagram, plant, camera, robot, obj


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run-dir", type=Path, required=True,
                   help="telemetry dir (summary.json/steps.jsonl)")
    p.add_argument("--robot-urdf", type=Path, required=True)
    p.add_argument("--object-sdf", type=Path, required=True)
    p.add_argument("--controlled-joints", type=str, required=True,
                   help="comma-separated ordered joint names")
    p.add_argument("--stride", type=int, default=4)
    p.add_argument("--fps", type=int, default=20)
    p.add_argument("--output", type=Path)
    p.add_argument("--trial-stats-ledger", type=Path)
    args = p.parse_args()

    summary = json.loads((args.run_dir / "summary.json").read_text())
    records = [json.loads(line) for line in
               (args.run_dir / "steps.jsonl").read_text().splitlines()]
    if not records:
        raise SystemExit("no recorded steps")
    joints = args.controlled_joints.split(",")

    result_dir = args.run_dir.parent
    planner_summary = xr.load_planner_summary(result_dir / "planner.log")
    trial_stats = None
    if args.trial_stats_ledger is not None:
        trial = xr.build_trial_record(result_dir, planner_summary)
        trials = xr.update_trial_ledger(args.trial_stats_ledger, trial)
        trial_stats = xr.aggregate_trial_stats(trials)
        stats_path = result_dir / "trial_stats.json"
        stats_path.write_text(json.dumps(trial_stats, indent=2))
        print("trial stats: " + json.dumps(trial_stats, sort_keys=True))

    diagram, plant, camera, robot, obj = build_scene(
        args.robot_urdf, args.object_sdf)
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyMutableContextFromRoot(context)
    camera_context = camera.GetMyContextFromRoot(context)
    block = plant.GetBodyByName("block")

    selected = records[::args.stride]
    if selected[-1] is not records[-1]:
        selected.append(records[-1])
    output = args.output or result_dir / "franka_rollout.mp4"
    if output.exists():
        raise SystemExit(f"refusing to overwrite existing video: {output}")

    hud_font = xr.font(16)
    with tempfile.TemporaryDirectory(prefix="oim_franka_frames_") as tmp:
        tmp = Path(tmp)
        for index, record in enumerate(selected):
            for name, value in zip(joints, record["arm_positions"]):
                plant.GetJointByName(name).set_angle(
                    plant_context, float(value))
            pose = record["object_pose"]
            zpos = record.get("object_position_W", [0, 0, 0.0298])[2]
            plant.SetFreeBodyPose(
                plant_context, block,
                RigidTransform(RollPitchYaw(0, 0, float(pose[2])),
                               [float(pose[0]), float(pose[1]), zpos]))
            diagram.ForcedPublish(context)
            rgb = camera.color_image_output_port().Eval(
                camera_context).data[:, :, :3]
            view = Image.fromarray(rgb.copy()).convert("RGB")
            draw = ImageDraw.Draw(view)
            lines = [
                f"franka open_table  step {record['step']}"
                f"  t={record['sim_time']:.2f}s",
                f"pos_err {record['position_error']:.4f} m   "
                f"yaw_err {record['orientation_error']:.4f} rad",
                f"goal ({record['goal_pose'][0]:.3f}, "
                f"{record['goal_pose'][1]:.3f}, {record['goal_pose'][2]:.3f})",
            ]
            for li, text in enumerate(lines):
                draw.text((12, 10 + 22 * li), text, font=hud_font,
                          fill=(20, 20, 30))
            if trial_stats is not None:
                view = xr.add_trial_stats_overlay(view, trial_stats)
            view.save(tmp / f"frame_{index:06d}.png")
        subprocess.run([
            "ffmpeg", "-loglevel", "error", "-n", "-framerate", str(args.fps),
            "-i", str(tmp / "frame_%06d.png"), "-c:v", "libx264",
            "-pix_fmt", "yuv420p", str(output)], check=True)
    print(f"wrote {output}: {len(selected)} frames, real OIM Franka geometry")


if __name__ == "__main__":
    main()
