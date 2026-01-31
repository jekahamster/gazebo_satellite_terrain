import argparse
import os
import subprocess

import rclpy

from geometry_msgs.msg import Pose
from rclpy.node import Node


class GazeboObjectSpawner(Node):
    """
    Spawns objects in Gazebo Sim using the ros_gz_sim `create` executable (Humble and Jazzy).
    XML/SDF must be passed as a string; use read_model_xml to load from file.

    Example:
        ros2 run gazebo_satellite_terrain spawn_object --name my_model --xml "<sdf>...</sdf>"
    """

    def __init__(
        self,
        node_name: str = "GazeboSpawner",
        world_name: str = "default",
    ) -> None:
        super().__init__(node_name)
        self.node_name = node_name
        self.world_name = world_name

    def read_model_xml(self, urdf_path: str) -> str:
        """Read model XML/SDF from file."""
        with open(urdf_path, "r") as g:
            return g.read()

    def spawn_object_gazebo(
        self, name: str, xml: str, pose: Pose = Pose(), verbose: bool = False
    ) -> bool:
        """
        Spawn an object into Gazebo given xml and a pose via ros_gz_sim create.

        Parameters
        ----------
        name : str
            Name of the object to be spawned.
        xml : str
            XML/SDF description of the object.
        pose : geometry_msgs.msg.Pose
            Position and orientation of object.
        verbose : bool
            Whether to log success.

        Returns
        -------
        bool
            True if spawn succeeded, False otherwise.
        """
        cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-world", self.world_name,
            "-string", xml,
            "-name", name,
            "-x", str(pose.position.x),
            "-y", str(pose.position.y),
            "-z", str(pose.position.z),
            "-R", "0", "-P", "0", "-Y", "0",
        ]
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30,
                env=os.environ,
            )
            if result.returncode == 0:
                if verbose:
                    self.get_logger().info(
                        f"{self.node_name}: Successfully spawned object: '{name}'"
                    )
                return True
            self.get_logger().warn(
                f"{self.node_name}: Spawn failed for '{name}' (exit {result.returncode}): "
                f"{result.stderr or result.stdout or 'no output'}"
            )
            return False
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                f"{self.node_name}: Spawn timed out for '{name}'"
            )
            return False
        except FileNotFoundError:
            self.get_logger().warn(
                f"{self.node_name}: ros2 or ros_gz_sim create not found; is ros_gz_sim installed?"
            )
            return False
        except Exception as e:
            self.get_logger().warn(
                f"{self.node_name}: Spawn failed for '{name}': {e}"
            )
            return False


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="Spawn a model in Gazebo via ros_gz_sim create")
    parser.add_argument("--name", type=str, help="Name of the object in gazebo")
    parser.add_argument("--xml", type=str, help="The full XML/SDF of the object")
    parser.add_argument("--world", type=str, default="default", help="Gazebo world name")
    parser.add_argument("--verbose", type=bool, default=False, help="Log success")
    args, unknown = parser.parse_known_args(args=args)

    spawner = GazeboObjectSpawner(world_name=args.world)
    spawner.spawn_object_gazebo(name=args.name, xml=args.xml, verbose=args.verbose)

    spawner.destroy_node()
    rclpy.shutdown()
