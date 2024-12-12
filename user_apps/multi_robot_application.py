# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#
import argparse
import sys

import carb
from omni.isaac.kit import SimulationApp


CONFIG = {"renderer": "RayTracedLighting", "headless": False}

# Example ROS2 bridge sample demonstrating the manual loading of Multiple Robot Navigation scenario
simulation_app = SimulationApp(CONFIG)


import omni
from omni.isaac.core import SimulationContext, World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils import extensions, nucleus, prims, rotations, stage, viewports
from omni.isaac.core.utils.extensions import enable_extension
from pxr import Sdf, Gf, UsdGeom, Usd, UsdSkel, AnimGraphSchema
import carb
import math
import omni.usd
import omni.graph.core as og
import usdrt.Sdf
from omni.kit.viewport.utility import get_active_viewport
import numpy as np
from omni.isaac.core.utils.stage import is_stage_loading, add_reference_to_stage
from omni.isaac.core.tasks import BaseTask
import omni.isaac.core.utils.prims as prims_utils
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.prims import define_prim



class Task1(BaseTask):
    def __init__(self,name):
        super().__init__(name=name, offset=None)

        return

    def set_up_scene(self, scene):
        super().set_up_scene(scene)

        self._cube_1 = scene.add(
            DynamicCuboid(
                prim_path="/new_cube_1",
                name="cube_1",
                position=np.array([-5, -5, 2.0]),
                scale=np.array([1.0, 1.0, 1.0]),
                size=1.0,
                color=np.array([0, 255, 0]),
            )
        )

        # self._camera = scene.add(Camera(
        #     prim_path="/World/camera",
        #     position=np.array([0.0, 0.0, 25.0]),
        #     frequency=20,
        #     resolution=(256, 256),
        #     orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True)))

        # self._camera.initialize()
        # self._camera.add_motion_vectors_to_frame()

        return

    def get_observations(self):
        cube1_position, _ = self._cube_1.get_world_pose()

        observations = {

            self._cube_1.name: {"position": cube1_position}
            }

        return observations


    def pre_step(self, control_index, simulation_time):

        return

class Task2(BaseTask):
    def __init__(self,name):
        super().__init__(name=name, offset=None)

        return

    def set_up_scene(self, scene):
        super().set_up_scene(scene)

        self._cube_2 = scene.add(
            DynamicCuboid(
                prim_path="/new_cube_2",
                name="cube_2",
                position=np.array([-10, -10, 2.0]),
                scale=np.array([1, 1, 1]),
                size=1.0,
                color=np.array([0, 0, 255]),
                linear_velocity=np.array([0, 0, 0.4]),
            )
        )

        return

    def get_observations(self):
        cube2_position, _ = self._cube_2.get_world_pose()

        observations = {
            self._cube_2.name: {"position": cube2_position}
            }

        return observations


class Task3(BaseTask):
    def __init__(self,name):
        super().__init__(name=name, offset=None)

        return

    def set_up_scene(self, scene):
        super().set_up_scene(scene)

        usd_path = "/home/raviteja/code/omniverse/MyRobots/Nova_Carter_ROS/nova_carter_ros.usd"
        prim_path="/World/Nova_Carter_Ros"
        
        print("2....")
        add_reference_to_stage(usd_path=usd_path,prim_path="/World/Nova_Carter_Ros")
        print("3....")

        scene.add(XFormPrim(prim_path=prim_path, name="Nova_Carter", position=np.array([7.5, 3.5, 0.5])))
        print("4....")
        return

    def get_observations(self):

        observations = {
            }

        return observations

class MySimulationApp(object):
    def __init__(self) -> None:

        # enable Anim people extension
        enable_extension("omni.anim.people")
        # enable Anim graph core extension
        enable_extension("omni.anim.graph.core")
        # enable ROS bridge extension
        enable_extension("omni.isaac.ros2_bridge")
        self.my_sim_app = simulation_app
        self.load_environment_usd()
        #self.load_robot_usd()
        self.add_tasks()

        return

    # def load_robot_usd(self):
    #     usd_path = "/home/raviteja/code/omniverse/MyRobots/Nova_Carter_ROS/nova_carter_ros.usd"
    #     usd_prim = add_reference_to_stage(usd_path=usd_path,prim_path="/World/Nova_Carter_Ros")

    #     #stage = omni.usd.get_context().open_stage(usd_path, "/World/Nova_Carter_Ros")


    #     self._my_sim_app.update()
    #     self._my_sim_app.update()

    #     print("Loading Robots...")


    #     while is_stage_loading():
    #         self._my_sim_app.update()
    #     print("Loading Robots Complete")

    def load_environment_usd(self):

        #ENV_USD_PATH = "/Isaac/Samples/PeopleDemo/SimpleEventSimulation/simple_event_simulation.usd"
        environment_usd_path = "/home/raviteja/code/omniverse/MyWorlds/WarehouseWorld/WarehouseWorld.usd"
        #usd_path = "/home/raviteja/code/omniverse/MyWorlds/PeopleHospitalWorld/PeopleHospitalWorld.usd"


        print("0....")
        
        stage = omni.usd.get_context().open_stage(environment_usd_path, None)
        
        print("1....")
        
        # usd_path = "/home/raviteja/code/omniverse/MyRobots/Nova_Carter_ROS/nova_carter_ros.usd"
        # prim_path="/World/Nova_Carter_Ros"
        # add_reference_to_stage(usd_path=usd_path,prim_path="/World/Nova_Carter_Ros")

        # print("2....")

        #self.add_multiple_cameras_to_scene()

        # Wait two frames so that stage starts loading
        self.my_sim_app.update()
        self.my_sim_app.update()

        print("Loading stage...")


        while is_stage_loading():
            self.my_sim_app.update()
        print("Loading Complete")

        return

    def add_multiple_cameras_to_scene(self):
        
        CAMERA_STAGE_PATH = "/World/Camera1"
        ROS_CAMERA_GRAPH_PATH = "/ROS_Camera1"

        cam_loc = Gf.Vec3d(-7, -23, 3.5)
        cam_orientation = (90, 0, 0)
        self.add_camera_to_scene(1, CAMERA_STAGE_PATH, ROS_CAMERA_GRAPH_PATH, cam_loc, cam_orientation)


        # CAMERA_STAGE_PATH = "/World/Camera2"
        # ROS_CAMERA_GRAPH_PATH = "/ROS_Camera2"

        # cam_loc = Gf.Vec3d(-2.5, 31, 5)
        # cam_orientation = (-111, 180, 0)
        # self.add_camera_to_scene(2, CAMERA_STAGE_PATH, ROS_CAMERA_GRAPH_PATH, cam_loc, cam_orientation)

        # CAMERA_STAGE_PATH = "/World/Camera3"
        # ROS_CAMERA_GRAPH_PATH = "/ROS_Camera3"

        # cam_loc = Gf.Vec3d(-3.5, 31, 5)
        # cam_orientation = (-111, 180, 0)
        # self.add_camera_to_scene(3, CAMERA_STAGE_PATH, ROS_CAMERA_GRAPH_PATH, cam_loc, cam_orientation)

        # CAMERA_STAGE_PATH = "/World/Camera4"
        # ROS_CAMERA_GRAPH_PATH = "/ROS_Camera4"

        # cam_loc = Gf.Vec3d(-8, 31, 5)
        # cam_orientation = (-111, 180, 0)
        # self.add_camera_to_scene(4, CAMERA_STAGE_PATH, ROS_CAMERA_GRAPH_PATH, cam_loc, cam_orientation)

        # CAMERA_STAGE_PATH = "/World/Camera5"
        # ROS_CAMERA_GRAPH_PATH = "/ROS_Camera5"

        # cam_loc = Gf.Vec3d(-13, 31, 5)
        # cam_orientation = (-111, 180, 0)
        # self.add_camera_to_scene(5, CAMERA_STAGE_PATH, ROS_CAMERA_GRAPH_PATH, cam_loc, cam_orientation)

        # CAMERA_STAGE_PATH = "/World/Camera6"
        # ROS_CAMERA_GRAPH_PATH = "/ROS_Camera6"

        # cam_loc = Gf.Vec3d(-18, 31, 5)
        # cam_orientation = (-111, 180, 0)
        # self.add_camera_to_scene(6, CAMERA_STAGE_PATH, ROS_CAMERA_GRAPH_PATH, cam_loc, cam_orientation)


        # CAMERA_STAGE_PATH = "/World/Camera7"
        # ROS_CAMERA_GRAPH_PATH = "/ROS_Camera7"

        # cam_loc = Gf.Vec3d(-22.5, 31, 5)
        # cam_orientation = (-111, 180, 0)
        # self.add_camera_to_scene(7, CAMERA_STAGE_PATH, ROS_CAMERA_GRAPH_PATH, cam_loc, cam_orientation)


        # CAMERA_STAGE_PATH = "/World/Camera8"
        # ROS_CAMERA_GRAPH_PATH = "/ROS_Camera8"

        # cam_loc = Gf.Vec3d(6, 0, 5)
        # cam_orientation = (-111, 180, -90)
        # self.add_camera_to_scene(8, CAMERA_STAGE_PATH, ROS_CAMERA_GRAPH_PATH, cam_loc, cam_orientation)


        CAMERA_STAGE_PATH = "/World/Camera9"
        ROS_CAMERA_GRAPH_PATH = "/ROS_Camera9"

        cam_loc = Gf.Vec3d(-25, 0, 5)
        cam_orientation = (-111, 180, 90)
        self.add_camera_to_scene(9, CAMERA_STAGE_PATH, ROS_CAMERA_GRAPH_PATH, cam_loc, cam_orientation)

    def add_camera_to_scene(self, view_port_id, CAMERA_STAGE_PATH, ROS_CAMERA_GRAPH_PATH, cam_loc, cam_orientation, fps=30, focal_length=24, 
        horizontal_aperture=21, vertical_aperture=16, focus_distance=400):

        # Creating a Camera prim
        camera_prim = UsdGeom.Camera(omni.usd.get_context().get_stage().DefinePrim(CAMERA_STAGE_PATH, "Camera"))
        xform_api = UsdGeom.XformCommonAPI(camera_prim)
        xform_api.SetTranslate(cam_loc)
        xform_api.SetRotate(cam_orientation, UsdGeom.XformCommonAPI.RotationOrderXYZ)
        camera_prim.GetHorizontalApertureAttr().Set(horizontal_aperture)
        camera_prim.GetVerticalApertureAttr().Set(vertical_aperture)
        camera_prim.GetProjectionAttr().Set("perspective")
        camera_prim.GetFocalLengthAttr().Set(focal_length)
        camera_prim.GetFocusDistanceAttr().Set(focus_distance)

        self.my_sim_app.update()


        # Creating an on-demand push graph with cameraHelper nodes to generate ROS image publishers
        keys = og.Controller.Keys
        (ros_camera_graph, _, _, _) = og.Controller.edit(
            {
                "graph_path": ROS_CAMERA_GRAPH_PATH,
                "evaluator_name": "push",
                "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND,
            },
            {
                keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnTick"),
                    ("createViewport", "omni.isaac.core_nodes.IsaacCreateViewport"),
                    ("getRenderProduct", "omni.isaac.core_nodes.IsaacGetViewportRenderProduct"),
                    ("setCamera", "omni.isaac.core_nodes.IsaacSetCameraOnRenderProduct"),
                    ("cameraHelperRgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("cameraHelperInfo", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                    ("cameraHelperDepth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
                ],
                keys.CONNECT: [
                    ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                    ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                    ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                    ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                    ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                    ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                    ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                    ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                    ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                    ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                    ("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
                ],
                keys.SET_VALUES: [
                    ("createViewport.inputs:viewportId", view_port_id),
                    ("cameraHelperRgb.inputs:frameId", "sim_camera"),
                    ("cameraHelperRgb.inputs:topicName", CAMERA_STAGE_PATH + "/rgb"),
                    ("cameraHelperRgb.inputs:type", "rgb"),
                    ("cameraHelperInfo.inputs:frameId", "sim_camera"),
                    ("cameraHelperInfo.inputs:topicName", CAMERA_STAGE_PATH + "/camera_info"),
                    ("cameraHelperInfo.inputs:type", "camera_info"),
                    ("cameraHelperDepth.inputs:frameId", "sim_camera"),
                    ("cameraHelperDepth.inputs:topicName", CAMERA_STAGE_PATH + "/depth"),
                    ("cameraHelperDepth.inputs:type", "depth"),
                    ("setCamera.inputs:cameraPrim", [usdrt.Sdf.Path(CAMERA_STAGE_PATH)]),
                ],
            },
        )

        # Run the ROS Camera graph once to generate ROS image publishers in SDGPipeline
        og.Controller.evaluate_sync(ros_camera_graph)

        self.my_sim_app.update()

        # viewport_resolution = (800, 400)
        # window_width = viewport_resolution[0]
        # window_height = viewport_resolution[1]

        # viewport_window = create_viewport_window(
        #         name=window_name,
        #         position_x=0.0,
        #         position_y=0.0,
        #         width=window_width,
        #         height=window_height,
        #         camera_path=camera_prim_path,
        #     )

        # viewport_window.viewport_api.set_active_camera(camera_prim_path)
        # viewport_window.viewport_api.resolution = viewport_resolution

        # viewport_window.viewport_api.set_active_camera(camera_prim_path)
        # viewport_window.viewport_api.resolution = viewport_resolution
        # resolution_before_update = viewport_window.viewport_api.resolution

        # Inside the SDGPipeline graph, Isaac Simulation Gate nodes are added to control the execution rate of each of the ROS image and camera info publishers.
        # By default the step input of each Isaac Simulation Gate node is set to a value of 1 to execute every frame.
        # We can change this value to N for each Isaac Simulation Gate node individually to publish every N number of frames.
        viewport_api = get_active_viewport()

        if viewport_api is not None:
            import omni.syntheticdata._syntheticdata as sd

            # Get name of rendervar for RGB sensor type
            rv_rgb = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)

            # Get path to IsaacSimulationGate node in RGB pipeline
            rgb_camera_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                rv_rgb + "IsaacSimulationGate", viewport_api.get_render_product_path()
            )

            # Get name of rendervar for DistanceToImagePlane sensor type
            rv_depth = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
                sd.SensorType.DistanceToImagePlane.name
            )

            # Get path to IsaacSimulationGate node in Depth pipeline
            depth_camera_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                rv_depth + "IsaacSimulationGate", viewport_api.get_render_product_path()
            )

            # Get path to IsaacSimulationGate node in CameraInfo pipeline
            camera_info_gate_path = omni.syntheticdata.SyntheticData._get_node_path(
                "PostProcessDispatch" + "IsaacSimulationGate", viewport_api.get_render_product_path()
            )

            # Set Rgb execution step to FPS frames
            rgb_step_size = 1

            # Set Depth execution step to 60 frames
            depth_step_size = 60

            # Set Camera info execution step to every frame
            info_step_size = 1

            # Set step input of the Isaac Simulation Gate nodes upstream of ROS publishers to control their execution rate
            og.Controller.attribute(rgb_camera_gate_path + ".inputs:step").set(rgb_step_size)
            og.Controller.attribute(depth_camera_gate_path + ".inputs:step").set(depth_step_size)
            og.Controller.attribute(camera_info_gate_path + ".inputs:step").set(info_step_size)

    def add_tasks(self):

        world_settings = {"physics_dt": 1.0 / 60.0, "stage_units_in_meters": 1.0, "rendering_dt": 1.0 / 60.0}
        self.world = World()
        self._task_list = [Task1(name="Task1"), Task3(name="Task3")]

        for task in self._task_list:
            self.world.add_task(task)

        self.world.add_physics_callback("sim_step", callback_fn=self.physics_step)

        self.world.reset()

        return

    def physics_step(self, step_size):

        current_observations = self.world.get_observations()


        prim_path1 = "/World/male_adult_police_04/male_adult_police_04/ManRoot/male_adult_police_04"
        prim_path2 = "/World/F_Business_02/female_adult_business_02/ManRoot/female_adult_business_02"
        prim_path3 = "/World/F_Medical_01/female_adult_medical_01/ManRoot/female_adult_medical_01"
        prim_path4 = "/World/M_Medical_01/male_adult_medical_01/ManRoot/male_adult_medical_01"
        prim_path5 = "/World/female_adult_police_01_new/female_adult_police_01/ManRoot/female_adult_police_01"
        prim_path_list = [prim_path1, prim_path2, prim_path3, prim_path4, prim_path5]

        curr_time = self.world.current_time

        # for prim_path in prim_path_list:
        #     character = omni.anim.graph.core.get_character(str(prim_path))
        #     pos = carb.Float3(0, 0, 0)
        #     rot = carb.Float4(0, 0, 0, 0)
        #     character.get_world_transform(pos, rot)
        #     print(f"curr_time: {curr_time}, char:{prim_path}, x: {pos[0]}, y: {pos[0]}, z: {pos[2]}")


        return

    def run_app(self):

        #simulation_context = SimulationContext(stage_units_in_meters=1.0)

        #self.my_sim_app.update()

        #self.world.play()

        #self.my_sim_app.update()

        while self.my_sim_app.is_running():
            self.world.step()

            # # runs with a realtime clock
            # self.my_sim_app.update()

        #simulation_context.stop()
        self.my_sim_app.close()

        return

my_sim_app = MySimulationApp()
my_sim_app.run_app()
