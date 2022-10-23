from queue import Queue, Empty
import random
import weakref
import json
import yaml

import carla
import numpy as np
import open3d as o3d


class CarlaSimulator:
    def __init__(
        self,
        fps: float = 10.0,
        points_per_cloud: int = 5000,
    ):
        self.fps = fps
        self.points_per_cloud = points_per_cloud

        self.client = carla.Client(
            "localhost", 2000
        )
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()

        self.original_settings = self.world.get_settings()

        self.sensor_queue = Queue()

    def setup_vehicles(self):
        vehicle_bps = self.world.get_blueprint_library().filter(
            "*vehicle*"
        )
        spawn_points = self.world.get_map().get_spawn_points()

        self.vehicles = []
        for _ in range(10):
            vehicle = self.world.try_spawn_actor(
                random.choice(vehicle_bps), random.choice(spawn_points)
            )
            if vehicle is not None:
                vehicle.set_autopilot(True)
                self.vehicles.append(vehicle)

        # ego_vehicle_transform = carla.Transform(
        #     carla.Location(x=-52.5, y=-15.0, z=0.5),
        #     carla.Rotation(pitch=0, yaw=90, roll=0),
        # )
        # ego_vehicle = self.world.spawn_actor(
        #     random.choice(vehicle_bps), ego_vehicle_transform
        # )
        # self.ego_vehicle = ego_vehicle

    def setup_lidars(self):
        sensor_pose = carla.Transform(
            carla.Location(x=-47.53, y=-10.46, z=5.56),
        )
        waypoint = self.world.get_map().get_waypoint(sensor_pose.location)

        lidar_bp = self.world.get_blueprint_library().find(
            "sensor.lidar.ray_cast"
        )
        # lidar_bp.set_attribute("dropoff_general_rate", "0.35")
        # lidar_bp.set_attribute("dropoff_intensity_limit", "0.8")
        # lidar_bp.set_attribute("dropoff_zero_intensity", "0.4")
        # lidar_bp.set_attribute("points_per_second", str(self.points_per_cloud * self.fps))
        # lidar_bp.set_attribute("rotation_frequency", str(self.fps))
        # lidar_bp.set_attribute("channels", "32.0")
        # lidar_bp.set_attribute("lower_fov", "-30.0")
        # lidar_bp.set_attribute("upper_fov", "10.0")
        # lidar_bp.set_attribute("range", "80.0")
        # lidar_bp.set_attribute("noise_stddev", "0.02")

        lidar_bp.set_attribute("noise_stddev", "0.01")
        lidar_bp.set_attribute("upper_fov", "2.0")
        lidar_bp.set_attribute("lower_fov", "-25.0")
        lidar_bp.set_attribute("channels", "64")
        # lidar_bp.set_attribute("atmosphere_attenuation_rate", str(0.05))
        lidar_bp.set_attribute("range", "100.0")
        lidar_bp.set_attribute("rotation_frequency", "10")
        lidar_bp.set_attribute("points_per_second", "500000")

        loc = carla.Location(
            x=waypoint.transform.location.x,
            y=waypoint.transform.location.y,
            z=waypoint.transform.location.z + 5.5,
        )
        lidar_transform = carla.Transform(
            # carla.Location(x=-64.0, y=7.0, z=3.74),
            loc,
            # sensor_pose.location,
            waypoint.transform.rotation,
        )

        # add sensor
        lidar = self.world.spawn_actor(
            lidar_bp, lidar_transform,
        )

        weak_self = weakref.ref(self)
        lidar.listen(
            lambda sensor_data: self.lidar_sensor_callback(weak_self, sensor_data)
        )
        self.lidar = lidar

    @staticmethod
    def lidar_sensor_callback(weak_self, sensor_data):
        self = weak_self()
        if self is None:
            return

        frame = sensor_data.frame

        if frame % 5 == 0:
            actors = self.world.get_actors()

            vehicle_labels = []
            for vehicle in actors.filter("vehicle.*"):
                pose = vehicle.get_transform()
                bbox = vehicle.bounding_box
                vehicle_labels.append({
                    "location": [
                        pose.location.x,
                        pose.location.y,
                        pose.location.z,
                    ],
                    "rotation": [
                        pose.rotation.pitch,
                        pose.rotation.yaw,
                        pose.rotation.roll,
                    ],
                    "center": [
                        bbox.location.x,
                        bbox.location.y,
                        bbox.location.z,
                    ],
                    "extent": [
                        bbox.extent.x,
                        bbox.extent.y,
                        bbox.extent.z,
                    ],
                })

            lidars = actors.filter("sensor.lidar.ray_cast")
            if lidars is not None:
                lidar = lidars[0]
                pose = lidar.get_transform()
                lidar_pose = [
                    pose.location.x,
                    pose.location.y,
                    pose.location.z,
                    pose.rotation.pitch,
                    pose.rotation.yaw,
                    pose.rotation.roll,
                ]

                points = np.copy(np.frombuffer(sensor_data.raw_data, dtype=np.float32))
                points = np.reshape(points, (-1, 4))

                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(points[:, :3])
                o3d.io.write_point_cloud(
                    f"data/carla_raw/{frame:06d}_lidar0.pcd",
                    pointcloud=pcd,
                    write_ascii=True,
                )

                vehicles_yaml = {
                    i: {
                        "angle": vehicle["rotation"],
                        "center": vehicle["center"],
                        "extent": vehicle["extent"],
                        "location": vehicle["location"],
                    }
                    for i, vehicle in enumerate(vehicle_labels)
                }

                with open("data/carla_raw/{frame:06d}.yaml", "w") as f:
                    yaml.dump(
                        {
                            "RSU": True,
                            "lidar0_pose": lidar_pose,
                            "vehicles": vehicles_yaml,
                        },
                        f,
                        yaml.SafeDumper,
                    )

        self.sensor_queue.put(sensor_data.frame)

    def tick(self):
        self.world.tick()

        snap = self.world.get_snapshot()
        w_frame = snap.frame
        try:
            s_frame = self.sensor_queue.get(True, 1.0)
            print(f"    Frame: {s_frame}")

        except Empty:
            print("    Some of the sensor information is missed")
