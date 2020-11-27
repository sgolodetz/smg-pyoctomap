import numpy as np

from typing import Tuple

from smg.pyoctomap import Pointcloud


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

        # Convert the world-space points image to a 1D array containing only the valid points.
        flat_mask: np.ndarray = np.where(depth_image != 0, 255, 0).reshape(height * width).astype(np.uint8)
        flat_ws_points: np.ndarray = ws_points.reshape((height * width, 3))
        valid_ws_points: np.ndarray = np.compress(flat_mask, flat_ws_points, axis=0)

        # Copy the valid points across to the Octomap point cloud, and return it.
        pcd: Pointcloud = Pointcloud()
        pcd.resize(valid_ws_points.shape[0])
        np.copyto(np.array(pcd, copy=False), valid_ws_points.reshape(-1))
        return pcd
