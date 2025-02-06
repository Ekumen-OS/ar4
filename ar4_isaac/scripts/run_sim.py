#!/usr/bin/env python3

# BSD 3-Clause License
#
# Copyright 2025 Ekumen, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Run Sim.

This script runs a simulation in isaac sim with ROS2 enabled.
It spawns an AR4 robot in the simulation.
"""

import argparse
from pathlib import Path

from isaacsim import SimulationApp

# Note(Fran): There are many imports inside each of the functions,
# this is required because the simulation app must be created first


def create_world(
    _: SimulationApp,
    physics_dt: float,
    rendering_dt: float,
    stage_units_per_meter: float,
):
    """Create a world with a ground plane and a dome light."""
    from omni.isaac.core import World
    import omni.isaac.core.utils.prims as prims_utils

    world = World(
        physics_dt=physics_dt,
        rendering_dt=rendering_dt,
        stage_units_in_meters=stage_units_per_meter,
    )
    world.scene.add_default_ground_plane()
    prims_utils.create_prim("/World/Light/Dome", "DomeLight", translation=(0, 0, 10.0))
    return world


def set_rate(_: SimulationApp, sim_dt: float):
    from omni.isaac.cortex.tools import SteadyRate

    rate_hz = 1.0 / sim_dt
    rate = SteadyRate(rate_hz)
    return rate


def enable_ros2_ext(_: SimulationApp):
    from omni.isaac.core.utils.extensions import enable_extension

    enable_extension("omni.isaac.ros2_bridge")


def spawn_ar4(_: SimulationApp, robot_model_path: Path):
    import omni.isaac.core.utils.prims as prims_utils

    prims_utils.create_prim(
        prim_path="/World/ar4",
        usd_path=str(robot_model_path.absolute()),
        translation=(0.0, 0.0, 0.0),
        orientation=(0.0, 0.0, 0.0, 1.0),
    )


def main(args):
    config = {
        "headless": args.headless,
        "renderer": "RayTracedLighting",
        "subframes": 8,
        "samples_per_pixel_per_frame": 8,
        "fast_shutdown": True,
    }

    simulation_app = SimulationApp(config, "")

    enable_ros2_ext(simulation_app)
    # Update once to prevent a SIGSEV
    # See: https://forums.developer.nvidia.com/t/seg-fault-when-activating-ros2-bridge-in-python/324908/3 # noqa: E501
    simulation_app.update()

    assert (
        args.physics_dt <= args.rendering_dt
    ), "Physics dt must be less than rendering dt"
    assert (
        args.rendering_dt % args.physics_dt == 0
    ), "Rendering dt must be a multiple of physics dt"

    world = create_world(
        simulation_app, args.physics_dt, args.rendering_dt, args.stage_units_per_meter
    )

    spawn_ar4(simulation_app, args.robot_model_path)

    # Keep the RTF at 1.0
    rate = set_rate(simulation_app, args.sim_dt)

    world.play()
    while simulation_app.is_running():
        world.step()
        rate.sleep()

    world.stop()
    simulation_app.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run Isaac Sim")
    parser.add_argument(
        "--robot_model_path",
        type=Path,
        default=Path(__file__).parent / "usda/ar4.usda",
        help="Path to the USD robot model file",
    )
    parser.add_argument(
        "--sim_dt",
        type=float,
        default=1.0 / 30.0,
        help="Simulation timestep in seconds",
    )
    parser.add_argument(
        "--physics_dt",
        type=float,
        default=1.0 / 120.0,
        help="Physics timestep in seconds",
    )
    parser.add_argument(
        "--rendering_dt",
        type=float,
        default=1.0 / 30.0,
        help="Rendering timestep in seconds",
    )
    parser.add_argument(
        "--stage_units_per_meter",
        type=float,
        default=1.0,
        help="Stage units per meter",
    )
    parser.add_argument("--headless", action="store_true", help="Run in headless mode")

    main(parser.parse_args())
