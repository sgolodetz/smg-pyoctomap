import numpy as np

from OpenGL.GL import *
from OpenGL.GLU import *
from typing import Tuple

from smg.pyoctomap import OcTree, OcTreeDrawer, Pointcloud, Pose6D


class OctomapUtil:
    """Utility functions related to Octomap."""

    # PUBLIC STATIC METHODS

    @staticmethod
    def compute_world_points_image(depth_image: np.ndarray, pose: np.ndarray,
                                   intrinsics: Tuple[float, float, float, float],
                                   ws_points: np.ndarray) -> None:
        """
        Compute a world-space points image from a depth image, camera pose, and set of camera intrinsics.

        .. note::
            The origin, i.e. (0, 0, 0), is stored for pixels whose depth is zero.

        :param depth_image:     The depth image (pixels with zero depth are treated as invalid).
        :param pose:            The camera pose (denoting a transformation from camera space to world space).
        :param intrinsics:      The camera intrinsics, as an (fx, fy, cx, cy) tuple.
        :param ws_points:       The output world-space points image.
        """
        # TODO: This should ultimately be moved into GeometryUtil, once I trust it enough.
        height, width = depth_image.shape
        fx, fy, cx, cy = intrinsics
        xl = np.array(range(width))
        yl = np.array(range(height))
        al = np.tile((xl - cx) / fx, height).reshape(height, width) * depth_image
        bl = np.transpose(np.tile((yl - cy) / fy, width).reshape(width, height)) * depth_image
        for i in range(3):
            ws_points[:, :, i] = pose[i, 0] * al + pose[i, 1] * bl + pose[i, 2] * depth_image

    @staticmethod
    def draw_octree(tree: OcTree, pose: np.ndarray, drawer: OcTreeDrawer) -> None:
        """
        Visualise the specified octree from the specified pose.

        :param tree:    The octree.
        :param pose:    The current pose.
        :param drawer:  The octree drawer.
        :return:
        """
        # Set the model-view matrix.
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0)
        glMultMatrixf(np.linalg.inv(pose).flatten(order='F'))

        # Enable blending, lighting and materials.
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        pos: np.ndarray = np.array([0.0, 2.0, -1.0, 0.0])
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
        height, width = depth_image.shape
        ws_points: np.ndarray = np.zeros((height, width, 3), dtype=float)
        OctomapUtil.compute_world_points_image(depth_image, pose, intrinsics, ws_points)

        # Convert the world-space points image to a flat array containing only the valid points.
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
        height, width = depth_image.shape
        ws_points: np.ndarray = np.zeros((height, width, 3), dtype=float)
        OctomapUtil.compute_world_points_image(depth_image, pose, intrinsics, ws_points)

        # Convert the world-space points image to a flat array containing only the valid points.
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

    @staticmethod
    def set_projection_matrix(intrinsics: Tuple[float, float, float, float], width: int, height: int) -> None:
        """
        Set the OpenGL projection matrix based on a set of camera intrinsics.

        :param intrinsics:  The camera intrinsics.
        :param width:       The width of the viewport.
        :param height:      The height of the viewport.
        """
        near_val: float = 0.1
        far_val: float = 1000.0

        # To rederive these equations, use similar triangles. Note that fx = f / sx and fy = f / sy,
        # where sx and sy are the dimensions of a pixel on the image plane.
        fx, fy, cx, cy = intrinsics
        left_val: float = -cx * near_val / fx
        right_val: float = (width - cx) * near_val / fx
        bottom_val: float = -cy * near_val / fy
        top_val: float = (height - cy) * near_val / fy

        glLoadIdentity()
        glFrustum(left_val, right_val, bottom_val, top_val, near_val, far_val)
