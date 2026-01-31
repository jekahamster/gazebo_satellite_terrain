import os
import shutil
import threading
import requests

import xml.etree.ElementTree as ET
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point as ROSPoint

from gazebo_satellite_terrain.spawn_object import GazeboObjectSpawner
from gazebo_satellite_terrain.objects import Tile


# Subdirectory for temporary tile dirs; easy to clean: rm -rf $MODEL_DIR/spawn_temp
# Layout: spawn_temp/tile_<name>/model.sdf, spawn_temp/tile_<name>/materials/textures/tile_<name>.jpeg
SPAWN_TEMP_DIR_NAME = "spawn_temp"


def _rmtree_safe(path: str) -> None:
    """Remove a directory tree; ignore if already removed or missing."""
    try:
        shutil.rmtree(path, ignore_errors=True)
    except OSError:
        pass


class GazeboSatelliteTileSpawner(GazeboObjectSpawner):
    """
    """

    def __init__(
        self,
        tile_size_meters: float,
        node_name: str = "TileSpawner",
        world_name: str = "default",
    ) -> None:
        super().__init__(node_name=node_name, world_name=world_name)

        model_dir = os.environ.get("MODEL_DIR", "")
        self.model_path = os.path.join(model_dir, "model.sdf")
        self.textures_dir = os.path.join(model_dir, "materials", "textures")
        self.spawn_temp_dir = os.path.join(model_dir, SPAWN_TEMP_DIR_NAME)
        os.makedirs(self.textures_dir, exist_ok=True)
        os.makedirs(self.spawn_temp_dir, exist_ok=True)

        self.set_tile_model_size(new_size=tile_size_meters)


    def spawn_tile(self, tile: Tile) -> None:
        """
        Spawn a tile at its position in Gazebo. Skips spawning if the tile was not downloaded (e.g. Mapbox 404).
        Uses a temporary SDF per tile so the shared model.sdf is never overwritten during spawn (avoids race with Gazebo load).
        """
        if not tile.is_downloaded:
            self.get_logger().warn(
                f"{self.node_name}: Skipping spawn for tile {tile.name} (not downloaded)."
            )
            return

        pose = Pose()
        pose.position = ROSPoint(
            x=float(tile.pos_meters.x),
            y=-float(tile.pos_meters.y),  # TODO: Find out why we need a minus here
            z=0.0,
        )

        model_sdf_path, tile_dir = self._prepare_tile_dir_and_sdf(tile.name)
        try:
            ok = self.spawn_object_gazebo(name=tile.name, pose=pose, file_path=model_sdf_path)
            if ok:
                tile.is_spawned = True
        finally:
            # Gazebo reads the file when it processes the request (may be delayed).
            # Delete the whole tile dir after a delay so it still exists when Gazebo reads it.
            threading.Timer(15.0, _rmtree_safe, [tile_dir]).start()


    def download_tile(self, tile: Tile, verbose: bool = False) -> dict:
        """
        Download a single tile from the mapbox static tile API (See: https://docs.mapbox.com/api/maps/static-tiles/) 
        Set your API-key, username and style in a .env file. Also not this could be used to download 'styled' terrain.
        (See: https://docs.mapbox.com/api/maps/styles/) tiles, but here we use a style that is completely plain satellite images

        TODO: Refactor / generalize to support downloading tiles from other providers
        """
        tile.texture_path = f'{self.textures_dir}/{tile.name}.jpeg'

        if os.path.isfile(tile.texture_path):
            tile.is_downloaded = True
            if verbose:
                self.get_logger().info(f"{self.node_name}: Tile '{tile.name}' has already been downloaded.")
            return {"is_downloaded": True, "already_downloaded": True}

        # Make the GET request
        username = os.environ.get("MAPBOX_USERNAME")
        style_id = os.environ.get("MAPBOX_STYLENAME")
        self.get_logger().info(f"https://api.mapbox.com/styles/v1/{username}/{style_id}/tiles/{tile.size_pixels}/{tile.zoom}/{tile.pos_coords.x}/{tile.pos_coords.y}")
        response = requests.get(
            f"https://api.mapbox.com/styles/v1/{username}/{style_id}/tiles/{tile.size_pixels}/{tile.zoom}/{tile.pos_coords.x}/{tile.pos_coords.y}",
            params = {
                "access_token": os.environ.get("MAPBOX_APIKEY")
            }
        )

        # Handle the response
        if response.status_code == 200:
            tile.is_downloaded = True

            with open(tile.texture_path, "wb") as f:
                f.write(response.content)

            return {"is_downloaded": True, "already_downloaded": False}

        else:
            self.get_logger().warn(
                f"{self.node_name}: Failed to download tile {tile.name} "
                f"({response.status_code}: {response.reason}). Check .env (MAPBOX_APIKEY, MAPBOX_USERNAME, MAPBOX_STYLENAME). Skipping tile."
            )
            return {"is_downloaded": False, "already_downloaded": False}

    def set_tile_model_size(self, new_size: float) -> None:
        """
        The tile size (in meters) is the same for all tiles but will be different depending on the starting latitude, so we set this once when we initilaize the terrain.
        Update the `model.sdf` to set the size of a tile in Gazebo
        """

        parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
        tree = ET.parse(self.model_path, parser=parser)
        
        # Update all the size elements
        for size_element in tree.getroot().findall(".//size"):
            size_element.text = f"{new_size} {new_size}"

        # Save the updated file
        tree.write(self.model_path, encoding='utf-8', xml_declaration=True) 


    def _prepare_tile_dir_and_sdf(self, texture_name: str) -> tuple[str, str]:
        """
        Create spawn_temp/tile_<name>/ with model.sdf and materials/textures/tile_<name>.jpeg.
        Texture is symlinked from the shared textures_dir (no copy). SDF uses relative path for albedo_map.
        Returns (path to model.sdf, path to tile dir for later cleanup).
        """
        tile_dir_name = f"tile_{texture_name}"
        tile_dir = os.path.join(self.spawn_temp_dir, tile_dir_name)
        textures_sub = os.path.join(tile_dir, "materials", "textures")
        os.makedirs(textures_sub, exist_ok=True)

        src_texture = os.path.abspath(os.path.join(self.textures_dir, f"{texture_name}.jpeg"))
        dst_texture = os.path.join(textures_sub, f"tile_{texture_name}.jpeg")
        try:
            os.symlink(src_texture, dst_texture)
        except OSError:
            shutil.copy2(src_texture, dst_texture)

        # Write model.sdf with relative path (forward slash for SDF)
        albedo_path = f"materials/textures/tile_{texture_name}.jpeg"
        parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
        tree = ET.parse(self.model_path, parser=parser)
        for elem in tree.getroot().findall(".//albedo_map"):
            elem.text = albedo_path

        model_sdf_path = os.path.join(tile_dir, "model.sdf")
        tree.write(model_sdf_path, encoding="unicode", xml_declaration=True)
        return model_sdf_path, tile_dir
