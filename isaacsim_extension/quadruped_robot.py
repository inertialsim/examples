# -----------------------------------------------------------------------------
# Copyright (c) 2023-2025, Inertial Simulation LLC.
# This example is licensed under the CC BY-NC-SA 4.0 license.
# https://creativecommons.org/licenses/by-nc-sa/4.0/
# Email: info@inertialsim.com
# -----------------------------------------------------------------------------
import numpy as np

import carb
import omni

from isaacsim.examples.interactive.base_sample import BaseSample
from isaacsim.robot.policy.examples.robots import SpotFlatTerrainPolicy
from isaacsim.gui.components.ui_utils import get_style
from isaacsim.sensors.physics import IMUSensor

from inertialsim.geometry import Vector
from inertialsim.geodesy import Gravity
from inertialsim.sensors.imu import IMU, IMUModel


# An example scenario with an InertialSim IMU built on top of an Isaac Sim IMU
# and mounted on a Spot quadruped robot controlled by the user.
# Inspired by the Quadruped example in Isaac Sim -> Robot Examples -> Policy
class QuadrupedImuExample(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._dt = 1.0 / 500.0
        self._world_settings["stage_units_in_meters"] = 1.0
        self._world_settings["physics_dt"] = self._dt
        self._world_settings["rendering_dt"] = 10 * self._dt
        self._base_command = [0.0, 0.0, 0.0]

        # Keyboard commands to control the robot
        self._input_keyboard_mapping = {
            # Forward
            "NUMPAD_8": [2.0, 0.0, 0.0],
            "UP": [2.0, 0.0, 0.0],
            # Back
            "NUMPAD_2": [-2.0, 0.0, 0.0],
            "DOWN": [-2.0, 0.0, 0.0],
            # Left
            "NUMPAD_6": [0.0, -2.0, 0.0],
            "RIGHT": [0.0, -2.0, 0.0],
            # Right
            "NUMPAD_4": [0.0, 2.0, 0.0],
            "LEFT": [0.0, 2.0, 0.0],
            # Rotate (counter-clockwise)
            "NUMPAD_7": [0.0, 0.0, 2.0],
            "N": [0.0, 0.0, 2.0],
            # Rotate (clockwise)
            "NUMPAD_9": [0.0, 0.0, -2.0],
            "M": [0.0, 0.0, -2.0],
        }

        # Set via callback from the UI
        self.imu_specification = None
        self.isaac_imu_data = []
        self.imu_data = []

    def setup_scene(self) -> None:
        # Add a ground plane with concrete like friction
        self._world.scene.add_default_ground_plane(
            z_position=0,
            name="Ground",
            prim_path="/World/GroundPlane",
            static_friction=0.8,
            dynamic_friction=0.6,
            restitution=0.01,
        )

        # Add the Spot robot
        self.spot = SpotFlatTerrainPolicy(
            prim_path="/World/Spot",
            name="Spot",
            position=np.array([0, 0, 0.8]),
        )

        # Set standard gravity
        self._world.get_physics_context().set_gravity(-Gravity.standard_magnitude())

        # Add an Isaac Sim IMU which simply provides the angular rate and
        # specific force from the physics engine.  No filtering or IMU modeling
        # is applied.
        self.isaac_imu = IMUSensor(
            prim_path="/World/Spot/body/imu", name="Imu", dt=self._dt
        )

        # Add an InertialSim IMU with a max runtime of 1-hour.  The example will
        # error out if left to run longer than that.
        self.imu = IMU(
            IMUModel(),
            self.imu_specification,
            rng=0,
            mode="real-time",
            max_duration=3600.0,
        )

        # Setup callbacks
        timeline = omni.timeline.get_timeline_interface()
        self._event_timer_callback = (
            timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.STOP),
                self._timeline_timer_callback_fn,
            )
        )

        # Add a window to present IMU data
        self.imu_window, self.imu_sliders = self.build_imu_window(
            "IMU Measurements", omni.ui.DockPreference.LEFT_BOTTOM
        )
        self.imu_window.set_visibility_changed_fn(self._on_visibility_changed)

    # Add a window displaying exact angular rate and specific force and
    # InertialSim IMU outputs.
    # Inspired by the Isaac Sim -> Robot Examples -> Sensors -> IMU example
    def build_imu_window(self, label, location):
        axis_labels = ["X: ", "Y: ", "Z: "]
        colors = [
            0xFFBBBBFF,
            0xFFBBFFBB,
            0xBBFFBBBB,
            0xBBAAEEFF,
            0xAABBFFEE,
            0xFFEEAABB,
            0xFFC8D5D0,
            0xFFC89BD0,
            0xFFAF9BA7,
            0xFFA4B99A,
        ]
        style = {
            "background_color": 0xFF888888,
            "color": 0xFF333333,
            "secondary_color": colors[0],
        }

        imu_window = omni.ui.Window(
            title=label,
            width=300,
            height=300,
            visible=True,
            dockPreference=location,
        )

        sliders = []
        with imu_window.frame:
            with omni.ui.VStack(style=get_style(), spacing=3):
                with omni.ui.HStack():
                    omni.ui.Spacer(width=40)
                    omni.ui.Label("Physics", style={"font_size": 16})
                    omni.ui.Label("InertialSim IMU", style={"font_size": 16})
                omni.ui.Label("Gyro measurements (deg/s)")
                for j in range(3):
                    with omni.ui.HStack():
                        omni.ui.Label(axis_labels[j], width=20, tooltip="Angular rate")
                        style["secondary_color"] = colors[3 + j]
                        sliders.append(
                            omni.ui.FloatDrag(
                                min=-100.0,
                                max=100.0,
                                step=0.001,
                                precision=2,
                                style=style,
                            )
                        )
                        omni.ui.Spacer(width=20)
                        sliders.append(
                            omni.ui.FloatDrag(
                                min=-100.0,
                                max=100.0,
                                step=0.001,
                                precision=2,
                                style=style,
                            )
                        )
                        sliders[-1].enabled = False
                        sliders[-2].enabled = False
                        omni.ui.Spacer(width=15)
                omni.ui.Label("Accelerometer measurements (m/s/s)")
                for i in range(3):
                    with omni.ui.HStack():
                        omni.ui.Label(
                            axis_labels[i], width=20, tooltip="Specific force"
                        )
                        style["secondary_color"] = colors[i]
                        sliders.append(
                            omni.ui.FloatDrag(
                                min=-20.0,
                                max=20.0,
                                step=0.001,
                                precision=2,
                                style=style,
                            )
                        )
                        omni.ui.Spacer(width=20)
                        sliders.append(
                            omni.ui.FloatDrag(
                                min=-20.0,
                                max=20.0,
                                step=0.001,
                                precision=2,
                                style=style,
                            )
                        )
                        sliders[-1].enabled = False
                        sliders[-2].enabled = False
                        omni.ui.Spacer(width=15)
        imu_window.visible = True
        return imu_window, sliders

    async def setup_post_load(self) -> None:
        self._appwindow = omni.appwindow.get_default_app_window()
        self._input = carb.input.acquire_input_interface()
        self._keyboard = self._appwindow.get_keyboard()
        self._sub_keyboard = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._sub_keyboard_event
        )
        self._physics_ready = False
        self.get_world().add_physics_callback(
            "physics_step", callback_fn=self.on_physics_step
        )
        await self.get_world().play_async()

    async def setup_post_reset(self) -> None:
        self._physics_ready = False
        await self._world.play_async()

    def on_physics_step(self, step_size) -> None:
        if self._physics_ready:
            self.spot.forward(step_size, self._base_command)

            # Get the Isaac Sim IMU data and display
            data = self.isaac_imu.get_current_frame()
            self.isaac_imu_data.append(data)
            self.imu_sliders[0].model.set_value(float(np.rad2deg(data["ang_vel"][0])))
            self.imu_sliders[2].model.set_value(float(np.rad2deg(data["ang_vel"][1])))
            self.imu_sliders[4].model.set_value(float(np.rad2deg(data["ang_vel"][2])))
            self.imu_sliders[6].model.set_value(float(data["lin_acc"][0]))
            self.imu_sliders[8].model.set_value(float(data["lin_acc"][1]))
            self.imu_sliders[10].model.set_value(float(data["lin_acc"][2]))

            # Simulate the IMU with InertialSim and display
            measurement = self.imu.simulate(
                angular_rate=Vector.from_xyz(np.copy(data["ang_vel"]), data["time"]),
                specific_force=Vector.from_xyz(np.copy(data["lin_acc"]), data["time"]),
            )
            self.imu_data.append(measurement)
            angular_rate = measurement.angular_rate.data[0, :, 0]
            specific_force = measurement.specific_force.data[0, :, 0]
            self.imu_sliders[1].model.set_value(np.rad2deg(angular_rate[0]))
            self.imu_sliders[3].model.set_value(np.rad2deg(angular_rate[1]))
            self.imu_sliders[5].model.set_value(np.rad2deg(angular_rate[2]))
            self.imu_sliders[7].model.set_value(specific_force[0])
            self.imu_sliders[9].model.set_value(specific_force[1])
            self.imu_sliders[11].model.set_value(specific_force[2])
        else:
            self._physics_ready = True
            self.spot.initialize()
            self.spot.post_reset()
            self.spot.robot.set_joints_default_state(self.spot.default_pos)

    def _sub_keyboard_event(self, event, *args, **kwargs) -> bool:
        """Subscriber callback to when kit is updated."""

        # when a key is pressedor released  the command is adjusted w.r.t the key-mapping
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            # on pressing, the command is incremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command += np.array(
                    self._input_keyboard_mapping[event.input.name]
                )

        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            # on release, the command is decremented
            if event.input.name in self._input_keyboard_mapping:
                self._base_command -= np.array(
                    self._input_keyboard_mapping[event.input.name]
                )
        return True

    def _timeline_timer_callback_fn(self, event) -> None:
        if self.spot:
            self._physics_ready = False

    def world_cleanup(self):
        self._event_timer_callback = None
        if self._world.physics_callback_exists("physics_step"):
            self._world.remove_physics_callback("physics_step")

    def _on_visibility_changed(self, visible):
        if not visible:
            self.on_closed()

    def on_closed(self):
        if self.imu_window:
            self.imu_window.destroy()
            self.imu_window = None

    # Update the IMU from a UI callback
    async def change_imu(self, imu):
        self.imu_specification = imu
        self.imu = IMU(
            IMUModel(),
            self.imu_specification,
            rng=0,
            mode="real-time",
            max_duration=3000.0,
        )
