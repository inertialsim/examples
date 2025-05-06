# -----------------------------------------------------------------------------
# Copyright (c) 2023-2025, Inertial Simulation LLC.
# This example is licensed under the CC BY-NC-SA 4.0 license.
# https://creativecommons.org/licenses/by-nc-sa/4.0/
# Email: info@inertialsim.com
# -----------------------------------------------------------------------------
"""Double lane change test at Mcity

Dependencies (tested with pre-built versions):
CARLA (0.9.15) - https://carla.org/
Mcity Digital Twin (1f51d09) - https://github.com/mcity/mcity-digital-twin
Python 3.7 - CARLA 0.9.15 prebuilt only ships with Python 3.7 API package
"""

import csv
import queue

import carla
from agents.navigation.controller import VehiclePIDController


# ISO 3888-1:2018 double lane change geometry.
# Vehicle drives left to right in the scene.
#      ____________________
# ____|                    |_____________
#             ______
# ___________|      |____________________
# | 1 |   2   |  3   |   4  |  5  |  6   |
#
# Dimensions:
# Section 1: length = 15m, width = 1.1*vehicle + 0.25
# Section 2: length = 30m
# Section 3: length = 25m, width = 1.2*vehicle + 0.25
# Section 4: length = 25m
# Section 5: length = 15m, width = 1.3*vehicle + 0.25
# Section 6: length = 15m, width = 1.3*vehicle + 0.25


class DoubleLaneChange:
    """Double lane change test scenario.

    The scenario involves a Lincoln MKZ vehicle at the Mcity test facility. The
    vehicle performs a double lane change test as per ISO 3888-1.

    The CARLA simulator must be running at the default location (localhost,
    port 2000).  The scenario uses synchronous mode for deterministic
    results.
    """

    def __init__(self, dt: float = 0.05):
        """Initialize the scenario.

        Args:
            dt: Simulation time delta (seconds).
        """
        # Scenario settings
        self.dt = dt
        self.lane_width = 3.5
        self.initial_pose = carla.Transform(
            carla.Location(x=152.8, y=71.9, z=271.5),
            carla.Rotation(pitch=1.25, yaw=270.4, roll=0.0),
        )
        self.client = carla.Client("localhost", 2000)
        self.world = self.client.load_world("McityMap_Main")

        # Enables synchronous mode and deterministic results
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = dt
        self.world.apply_settings(settings)
        self.client.reload_world(False)

        # Fixed state abd gab
        self.map = self.world.get_map()
        self.bp_lib = self.world.get_blueprint_library()

        # Initialized in other methods if needed
        self.vehicle = None
        self.controller = None
        self.spectator = None
        self.camera = None
        self.imu = None
        self.csv_file = None
        self.csv_writer = None
        self.image_queue = queue.Queue()
        self.imu_queue = queue.Queue()

    def spawn_vehicle(self):
        """Spawn the primary vehicle"""
        vehicle_bp = self.bp_lib.find("vehicle.lincoln.mkz_2020")
        self.vehicle = self.world.spawn_actor(vehicle_bp, self.initial_pose)
        self.controller = VehiclePIDController(
            self.vehicle,
            args_lateral={"K_P": 1.0, "K_I": 0.01, "K_D": 0.0, "dt": self.dt},
            args_longitudinal={"K_P": 1.0, "K_I": 0.01, "K_D": 0.0, "dt": self.dt},
            max_steering=0.3,
        )
        # Allow the vehicle to settle for 1 second
        for k in range(int(1 / self.dt)):
            self.world.tick()

    def pose_vehicle(self):
        """Pose the vehicle for a hero thumbnail screen capture"""
        hero_location = self.vehicle.get_transform().transform(
            carla.Location(x=90, y=-3)
        )
        hero_pose = carla.Transform(
            hero_location,
            carla.Rotation(pitch=0.0, yaw=67.5, roll=0.0),
        )
        self.vehicle.set_transform(hero_pose)
        spectator_pose = carla.Transform(
            carla.Location(x=158, y=-13.5, z=275.3),
            carla.Rotation(pitch=-3, yaw=-108, roll=0.0),
        )
        self.spectator.set_transform(spectator_pose)
        # Allow the vehicle to settle for 1 second
        for k in range(int(1 / self.dt)):
            self.world.tick()

    def spawn_spectator(self):
        """Spawn the spectator"""
        self.spectator = self.world.get_spectator()
        self.spectator.set_transform(
            carla.Transform(
                carla.Location(x=152, y=-13, z=278),
                carla.Rotation(pitch=-11, yaw=-90, roll=-0.0),
            )
        )

    def set_waypoints(self, debug: bool = False):
        """Create a list of waypoints.

        Waypoints are returned at lane centers relative to the initial pose
        of the vehicle.  CARLA only supports waypoints at lane centers.  The
        initial pose of the vehicle must be aligned with the lanes for the
        scenario.

        Args:
            debug: Draw waypoints for debugging purposes.

        Returns:
            waypoints: List of waypoints.
        """
        locations = waypoint_locations(self.lane_width)
        waypoints = []
        for location in locations:
            world_location = self.initial_pose.transform(location)
            waypoint = self.map.get_waypoint(world_location)
            waypoints.append(waypoint)
            if debug:
                self.world.debug.draw_string(
                    waypoint.transform.location, "WP", life_time=100
                )
        return waypoints

    def spawn_cones(self):
        """Spawn traffic cones.

        Cones are spaced as per the ISO 3888-1 lane layout.
        """
        locations = cone_locations(self.lane_width)
        cone_bp = self.bp_lib.find("static.prop.constructioncone")
        rotation = carla.Rotation(0, 0, 0)
        for location in locations:
            world_location = self.initial_pose.transform(location)
            actor = self.world.spawn_actor(
                cone_bp, carla.Transform(world_location, rotation)
            )
            actor.set_enable_gravity(True)
            actor.set_simulate_physics(True)
        # Allow the cones to settle for 1 second
        for k in range(int(1 / self.dt)):
            self.world.tick()

    def spawn_camera(self):
        """Add a camera to the scene.

        The camera is used to record the scenario and is optional.
        """
        camera_bp = self.bp_lib.find("sensor.camera.rgb")
        camera_bp.set_attribute("image_size_x", str(1920))
        camera_bp.set_attribute("image_size_y", str(1080))
        camera_bp.set_attribute("fov", str(105))
        camera_pose = carla.Transform(
            carla.Location(x=10, y=-3, z=3), carla.Rotation(pitch=-20.0, yaw=160.0)
        )
        self.camera = self.world.spawn_actor(
            camera_bp, camera_pose, attach_to=self.vehicle
        )
        self.camera.listen(self.image_queue.put)

    def spawn_imu(self):
        """Add an IMU to the vehicle."""
        imu_bp = self.bp_lib.find("sensor.other.imu")
        imu_pose = carla.Transform()
        self.imu = self.world.spawn_actor(imu_bp, imu_pose, attach_to=self.vehicle)
        self.csv_file = open("carla_data/imu.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.imu.listen(self.imu_queue.put)

    def spectate(self):
        """Spectate with no driving."""
        while True:
            self.world.tick()

    def drive(self, speed: float = 80.0):
        """Perform the double lane change test.

        Args:
            speed: Vehicle speed in km/h.
        """
        waypoints = self.set_waypoints(debug=False)
        # Drive through each waypoint
        for waypoint in waypoints:
            while (
                self.vehicle.get_location().distance(waypoint.transform.location) > 0.5
            ):
                control = self.controller.run_step(speed, waypoint)
                self.vehicle.apply_control(control)
                self.world.tick()
                if self.camera is not None:
                    image = self.image_queue.get()
                    image.save_to_disk("images/%06d.png" % image.frame)
                if self.imu is not None:
                    imu = self.imu_queue.get()
                    self.csv_writer.writerow(
                        [
                            imu.frame,
                            imu.timestamp,
                            imu.gyroscope.x,
                            imu.gyroscope.y,
                            imu.gyroscope.z,
                            imu.accelerometer.x,
                            imu.accelerometer.y,
                            imu.accelerometer.z,
                        ]
                    )

        # Brake to a stop
        while self.vehicle.get_velocity().length() > 0.05:
            self.vehicle.apply_control(carla.VehicleControl(brake=1.0))
            self.world.tick()
            if self.camera is not None:
                image = self.image_queue.get()
                image.save_to_disk("images/%06d.png" % image.frame)
            if self.imu is not None:
                imu = self.imu_queue.get()
                self.csv_writer.writerow(
                    [
                        imu.frame,
                        imu.timestamp,
                        imu.gyroscope.x,
                        imu.gyroscope.y,
                        imu.gyroscope.z,
                        imu.accelerometer.x,
                        imu.accelerometer.y,
                        imu.accelerometer.z,
                    ]
                )

        # Shutdown the scenario
        if self.camera is not None:
            self.camera.stop()
            self.camera.destroy()
        if self.imu is not None:
            self.imu.destroy()
            self.csv_file.close()
        self.vehicle.destroy()
        return


def waypoint_locations(lane_width: float):
    """List of waypoints relative to the vehicle.

    List of waypoints in forward, right coordinates relative to the initial pose
    of the vehicle.  The vehicle should start aligned with the right lane of a
    two lane road long enough for the runup (100m) and the lane change test
    distance (125m).

    Args:
        lane_width: Width the driving lanes.

    Returns:
        waypoints: List of waypoints relative to vehicle.
    """
    # Runup length to get to constant velocity
    runup = 100.0

    waypoints = []
    # Entrance to Section 2
    waypoints.append(carla.Vector3D(runup + 15.0, 0.0, 0.0))

    # Entrance to Section 3
    waypoints.append(carla.Vector3D(runup + 40.0, -lane_width, 0.0))

    # Exit of Section 3
    waypoints.append(carla.Vector3D(runup + 70.0, -lane_width, 0.0))

    # Entrance to Section 5
    waypoints.append(carla.Vector3D(runup + 90.0, 0.0, 0.0))

    # 10m beyond exit of Section 6
    waypoints.append(carla.Vector3D(runup + 135.0, 0.0, 0.0))

    return waypoints


def cone_locations(lane_width: float):
    """List of cone locations relative to the vehicle.

    List of cone locations in forward, right coordinates relative to the initial pose
    of the vehicle.  The vehicle should start aligned with the right lane of a
    two lane road long enough for the runup (100m) and the lane change test
    distance (125m).

    Args:
        lane_width: Width of the driving lanes.

    Returns:
        waypoints: List of cone locations relative to vehicle.
    """
    # Runup length to get to constant velocity
    runup = 100.0

    # Height to avoid spawn collision with ground
    height = 0.1

    # Padding so cones are not on lane lines
    padded_lane_width = 1.2 * lane_width
    cone_locations = []

    # WARNING: these cone locations are slightly adjusted to fit the Mcity track.

    # Section 1 cones
    # Right
    cone_locations.append(carla.Location(runup, 0.5 * padded_lane_width, height))
    cone_locations.append(carla.Location(runup + 7.5, 0.5 * padded_lane_width, height))
    cone_locations.append(carla.Location(runup + 15.0, 0.5 * padded_lane_width, height))
    # Left
    cone_locations.append(carla.Location(runup, -0.5 * padded_lane_width, height))
    cone_locations.append(carla.Location(runup + 7.5, -0.5 * padded_lane_width, height))
    cone_locations.append(
        carla.Location(runup + 15.0, -0.5 * padded_lane_width, height)
    )

    # Section 2 cones
    # Right
    cone_locations.append(carla.Location(runup + 45, -0.4 * padded_lane_width, height))
    cone_locations.append(
        carla.Location(runup + 45 + 12.5, -0.4 * padded_lane_width, height)
    )
    cone_locations.append(
        carla.Location(runup + 45 + 25.0, -0.4 * padded_lane_width, height)
    )
    # Left
    cone_locations.append(carla.Location(runup + 45, -1.4 * padded_lane_width, height))
    cone_locations.append(
        carla.Location(runup + 45 + 12.5, -1.4 * padded_lane_width, height)
    )
    cone_locations.append(
        carla.Location(runup + 45 + 25.0, -1.4 * padded_lane_width, height)
    )

    # Section 5/6 cones
    # Right
    cone_locations.append(carla.Location(runup + 95, 0.5 * padded_lane_width, height))
    cone_locations.append(
        carla.Location(runup + 95 + 7.5, 0.5 * padded_lane_width, height)
    )
    cone_locations.append(
        carla.Location(runup + 95 + 15.0, 0.5 * padded_lane_width, height)
    )
    cone_locations.append(
        carla.Location(runup + 95 + 22.5, 0.5 * padded_lane_width, height)
    )
    cone_locations.append(
        carla.Location(runup + 95 + 30.0, 0.5 * padded_lane_width, height)
    )
    # Left
    cone_locations.append(carla.Location(runup + 95, -0.5 * padded_lane_width, height))
    cone_locations.append(
        carla.Location(runup + 95 + 7.5, -0.5 * padded_lane_width, height)
    )
    cone_locations.append(
        carla.Location(runup + 95 + 15.0, -0.5 * padded_lane_width, height)
    )
    cone_locations.append(
        carla.Location(runup + 95 + 22.5, -0.5 * padded_lane_width, height)
    )
    cone_locations.append(
        carla.Location(runup + 95 + 30.0, -0.5 * padded_lane_width, height)
    )

    return cone_locations
