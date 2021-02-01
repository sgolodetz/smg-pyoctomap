import numpy as np

from OpenGL.GL import *
from typing import Tuple

from smg.rigging.helpers import CameraPoseConverter
from smg.utility import GeometryUtil

from ..cpp.pyoctomap import OcTree, OcTreeDrawer, Pointcloud, Pose6D


class OctomapUtil:
    """Utility functions related to Octomap."""

    # PUBLIC STATIC METHODS

    @staticmethod
    def draw_octree(tree: OcTree, drawer: OcTreeDrawer) -> None:
        """
        Draw the specified octree.

        :param tree:    The octree.
        :param drawer:  The octree drawer.
        :return:
        """
        OctomapUtil.draw_octree_understandable(tree, drawer)

    @staticmethod
    def draw_octree_octovis(tree: OcTree, drawer: OcTreeDrawer) -> None:
        """
        Draw the specified octree in the style of Octovis.

        .. note::
            Octovis renders transparent cubes, which looks pretty but isn't always easy to understand.

        :param tree:    The octree.
        :param drawer:  The octree drawer.
        :return:
        """
        # Enable blending, lighting and materials.
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        pos: np.ndarray = np.array([0.0, -2.0, -1.0, 0.0])
        glLightfv(GL_LIGHT0, GL_POSITION, pos)

        glEnable(GL_COLOR_MATERIAL)

        # Draw the octree.
        origin_pose: Pose6D = Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        drawer.set_octree(tree, origin_pose)
        drawer.draw()

        # Disable blending, lighting and materials again.
        glDisable(GL_COLOR_MATERIAL)
        glDisable(GL_LIGHTING)
        glDisable(GL_BLEND)

    @staticmethod
    def draw_octree_understandable(tree: OcTree, drawer: OcTreeDrawer, *, render_filled_cubes: bool = True) -> None:
        """
        Draw the specified octree in a way that is easy to understand.

        .. note::
            This method either (i) renders filled cubes with wireframe cubes over the top of them,
            or (ii) just renders the wireframe cubes, but hiding the hidden edges.

        :param tree:                The octree.
        :param drawer:              The octree drawer.
        :param render_filled_cubes: Whether to render the filled cubes.
        """
        # Enable lighting and materials.
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        pos: np.ndarray = np.array([0.0, -2.0, -1.0, 0.0])
        glLightfv(GL_LIGHT0, GL_POSITION, pos)

        glEnable(GL_COLOR_MATERIAL)

        # Draw the octree.
        origin_pose: Pose6D = Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        drawer.set_octree(tree, origin_pose)

        if render_filled_cubes:
            drawer.draw()
        else:
            glColorMask(False, False, False, False)
            drawer.draw()
            glColorMask(True, True, True, True)

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
        glPolygonOffset(-1.0, -1.0)
        glEnable(GL_POLYGON_OFFSET_LINE)
        drawer.draw()
        glDisable(GL_POLYGON_OFFSET_LINE)
        glPolygonOffset(0.0, 0.0)
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)

        # Disable lighting and materials again.
        glDisable(GL_COLOR_MATERIAL)
        glDisable(GL_LIGHTING)

    @staticmethod
    def make_point_cloud(depth_image: np.ndarray, pose: np.ndarray, intrinsics: Tuple[float, float, float, float]) \
            -> Pointcloud:
        """
        Make an Octomap point cloud from a depth image.

        :param depth_image: The depth image (pixels with zero depth are treated as invalid).
        :param pose:        The camera pose (denoting a transformation from camera space to world space).
        :param intrinsics:  The camera intrinsics, as an (fx, fy, cx, cy) tuple.
        :return:            The Octomap point cloud.
        """
        # Back-project the depth image to make a world-space points image.
        ws_points: np.ndarray = GeometryUtil.compute_world_points_image_fast(depth_image, pose, intrinsics)

        # Convert the world-space points image to a flat array containing only the valid points.
        height, width = depth_image.shape
        flat_mask: np.ndarray = np.where(depth_image != 0, 255, 0).reshape(height * width).astype(np.uint8)
        flat_ws_points: np.ndarray = ws_points.reshape((height * width, 3))
        valid_ws_points: np.ndarray = np.compress(flat_mask, flat_ws_points, axis=0)

        # Copy the valid points across to the Octomap point cloud, and return it.
        pcd: Pointcloud = Pointcloud()
        pcd.resize(valid_ws_points.shape[0])
        np.copyto(np.array(pcd, copy=False), valid_ws_points.reshape(-1))
        return pcd

    @staticmethod
    def make_rgbd_point_cloud(rgb_image: np.ndarray, depth_image: np.ndarray, pose: np.ndarray,
                              intrinsics: Tuple[float, float, float, float]) -> Tuple[Pointcloud, np.ndarray]:
        """
        Make a coloured Octomap point cloud from an RGB image and a depth image.

        :param rgb_image:   The RGB image.
        :param depth_image: The depth image (pixels with zero depth are treated as invalid).
        :param pose:        The camera pose (denoting a transformation from camera space to world space).
        :param intrinsics:  The camera intrinsics, as an (fx, fy, cx, cy) tuple.
        :return:            A tuple consisting of an uncoloured Octomap point cloud and a matching array of colours.
        """
        # Back-project the depth image to make a world-space points image.
        ws_points: np.ndarray = GeometryUtil.compute_world_points_image_fast(depth_image, pose, intrinsics)

        # Convert the world-space points image to a flat array containing only the valid points.
        height, width = depth_image.shape
        flat_mask: np.ndarray = np.where(depth_image != 0, 255, 0).reshape(height * width).astype(np.uint8)
        flat_ws_points: np.ndarray = ws_points.reshape((height * width, 3))
        valid_ws_points: np.ndarray = np.compress(flat_mask, flat_ws_points, axis=0)

        # Copy the valid points across to the Octomap point cloud.
        pcd: Pointcloud = Pointcloud()
        pcd.resize(valid_ws_points.shape[0])
        np.copyto(np.array(pcd, copy=False), valid_ws_points.reshape(-1))

        # Convert the RGB image to a flat array containing colours for only the valid points.
        flat_colours: np.ndarray = rgb_image.reshape((height * width, 3))
        valid_flat_colours: np.ndarray = np.compress(flat_mask, flat_colours, axis=0)

        return pcd, valid_flat_colours
