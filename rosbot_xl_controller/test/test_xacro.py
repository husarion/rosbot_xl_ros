# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import itertools

import xacro
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def test_rosbot_description_parsing():
    mecanum_values = ["true", "false"]
    use_sim_values = ["true", "false"]
    simulation_engine_values = ["ignition-gazebo", "webots"]  # 'gazebo-classic'
    lidar_models = [
        "None",
        "slamtec_rplidar_a2",
        "slamtec_rplidar_a3",
        "slamtec_rplidar_s1",
        "slamtec_rplidar_s2",
        "slamtec_rplidar_s3",
        "velodyne_puck",
    ]
    camera_models = [
        "None",
        "intel_realsense_d435",
        "orbbec_astra",
        "stereolabs_zed",
        "stereolabs_zedm",
        "stereolabs_zed2",
        "stereolabs_zed2i",
        "stereolabs_zedx",
        "stereolabs_zedxm",
    ]

    all_combinations = list(
        itertools.product(
            mecanum_values,
            use_sim_values,
            simulation_engine_values,
            lidar_models,
            camera_models,
        )
    )

    for combination in all_combinations:
        (
            mecanum,
            use_sim,
            simulation_engine,
            lidar_model,
            camera_model,
        ) = combination
        mappings = {
            "mecanum": mecanum,
            "use_sim": use_sim,
            "simulation_engine": simulation_engine,
            "lidar_model": lidar_model,
            "camera_model": camera_model,
        }
        xacro_path = PathJoinSubstitution(
            [FindPackageShare("rosbot_xl_description"), "urdf", "rosbot_xl.urdf.xacro"]
        )
        try:
            xacro.process_file(xacro_path, mappings=mappings)
        except xacro.XacroException as e:
            assert False, (
                f"xacro parsing failed: {str(e)} for mecanum: {mecanum}, "
                f"use_sim: {use_sim}, simulation_engine: {simulation_engine}, "
                f"lidar_model: {lidar_model}, camera_model: {camera_model}, "
            )
