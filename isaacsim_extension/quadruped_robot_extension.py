# -----------------------------------------------------------------------------
# Copyright (c) 2023-2025, Inertial Simulation LLC.
# This example is licensed under the CC BY-NC-SA 4.0 license.
# https://creativecommons.org/licenses/by-nc-sa/4.0/
# Email: info@inertialsim.com
# -----------------------------------------------------------------------------
import os
import asyncio
import pickle

import omni

from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.gui.components.ui_utils import (
    btn_builder,
    get_style,
    dropdown_builder,
)

from isaacsim.examples.interactive.base_sample import BaseSampleUITemplate
from isaacsim.examples.interactive.user_examples import QuadrupedImuExample


from inertialsim.devices import *


class QuadrupedImuExampleExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        # Document the example
        self.example_name = "InertialSim IMU"
        self.category = "Sensors"
        overview = "This Example demonstrates simulation of realistic IMU sensor performance using InertialSim."
        overview += "\nSelect an IMU specification and load the scenario."
        overview += "\nControl the robot manually using the arrow keys (walk) and N/M keys (rotate).  "

        # Create the example UI
        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "InertialSim IMU",
            "overview": overview,
        }
        ui_handle = QuadrupedImuUI(**ui_kwargs)

        # Attach the example class to the UI
        ui_handle.sample = QuadrupedImuExample()
        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=ui_handle.build_window,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )
        return

    def on_shutdown(self):
        get_browser_instance().deregister_example(
            name=self.example_name, category=self.category
        )
        return


class QuadrupedImuUI(BaseSampleUITemplate):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Map of available InertialSim devices from name to class
        self.imu_map = {
            "Advanced Navigation Motus": advanced_navigation_motus,
            "Advanced Navigation Orientus": advanced_navigation_orientus,
            "Analog Devices ADIS16490": analog_devices_adis16490,
            "Analog Devices ADIS16495": analog_devices_adis16495,
            "Analog Devices ADIS16497": analog_devices_adis16497,
            "Bosch BMI088": bosch_bmi088,
            "Bosch BMI270": bosch_bmi270,
            "Honeywell HG1125": honeywell_hg1125,
            "Honeywell HG1700": honeywell_hg1700,
            "Honeywell HG1900": honeywell_hg1900,
            "Honeywell HG1930": honeywell_hg1930,
            "Honeywell HG4930": honeywell_hg4930,
            "LITEF ISA-100": litef_isa100,
            "LITEF LCI-100": litef_lci100,
            "LITEF uIMU": litef_uimu,
            "MicroStrain  3DM-GX5": microstrain_3dmgx5,
            "Northrop Grumman LN-200": northrop_grumman_ln200,
            "Safran STIM300": safran_stim300,
            "Safran STIM318": safran_stim318,
            "Safran STIM320": safran_stim320,
            "VectorNav VN-100": vectornav_vn100,
            "VectorNav VN-110": vectornav_vn110,
            "Xsens MTi-1": xsens_mti1,
            "Xsens MTi-100": xsens_mti100,
            "Xsens MTi-610": xsens_mti610,
        }
        self.selected_imu = "Analog Devices ADIS16490"
        self.loaded = False

    def build_ui(self):
        # Build the example UI
        self.task_ui_elements = {}
        self.build_default_frame()
        with self._controls_frame:
            with omni.ui.VStack(style=get_style(), spacing=5, height=0):
                # Add all InertialSim devices to a dropdown list
                self.task_ui_elements["Select IMU"] = dropdown_builder(
                    "Select IMU",
                    items=[
                        "Advanced Navigation Motus",
                        "Advanced Navigation Orientus",
                        "Analog Devices ADIS16490",
                        "Analog Devices ADIS16495",
                        "Analog Devices ADIS16497",
                        "Bosch BMI088",
                        "Bosch BMI270",
                        "Honeywell HG1125",
                        "Honeywell HG1700",
                        "Honeywell HG1900",
                        "Honeywell HG1930",
                        "Honeywell HG4930",
                        "LITEF ISA-100",
                        "LITEF LCI-100",
                        "LITEF uIMU",
                        "MicroStrain  3DM-GX5",
                        "Northrop Grumman LN-200",
                        "Safran STIM300",
                        "Safran STIM318",
                        "Safran STIM320",
                        "VectorNav VN-100",
                        "VectorNav VN-110",
                        "Xsens MTi-1",
                        "Xsens MTi-100",
                        "Xsens MTi-610",
                    ],
                    on_clicked_fn=self.__on_selected_imu_changed,
                )
                dict = {
                    "label": "Start",
                    "type": "button",
                    "text": "Start",
                    "tooltip": "Load robot and IMU",
                    "on_clicked_fn": self._on_load_world,
                }
                self._buttons["Start"] = btn_builder(**dict)
                self._buttons["Start"].enabled = True
                dict = {
                    "label": "Reset",
                    "type": "button",
                    "text": "Reset",
                    "tooltip": "Reset robot and IMU",
                    "on_clicked_fn": self._on_reset,
                }
                self._buttons["Reset"] = btn_builder(**dict)
                self._buttons["Reset"].enabled = False
                dict = {
                    "label": "Save",
                    "type": "button",
                    "text": "Save",
                    "tooltip": "Save IMU data",
                    "on_clicked_fn": self._on_save,
                }
                self._buttons["Save"] = btn_builder(**dict)
                self._buttons["Save"].enabled = False

        self.build_extra_frames()

    # Save IMU data when button is clicked
    def _on_save(self):
        with open('imu_measurements.pkl', 'wb') as file:
            pickle.dump([self._sample.isaac_imu_data, self._sample.imu_data], file)

    # Return the IMU specification from a list index
    def get_imu(self):
        return self.imu_map[self.selected_imu]

    # Load the world with the default IMU 
    def _on_load_world(self):
        self._sample.imu_specification = self.get_imu()
        self.loaded = True
        super()._on_load_world()

    # Update the world with a newly selected IMU. 
    # WARNING: the simulation will not be stopped or reset and the IMU
    # performance will change immediately.
    def __on_selected_imu_changed(self, selected_index):
        self.selected_imu = selected_index
        if self.loaded:
            asyncio.ensure_future(self._sample.change_imu(self.get_imu()))
