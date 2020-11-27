import numpy as np
import os

from timeit import default_timer as timer
from typing import Tuple

from smg.pyoctomap import *
from smg.utility import ImageUtil, PoseUtil


def main() -> None:
    intrinsics: Tuple[float, float, float, float] = (532.5694641250893, 531.5410880910171, 320.0, 240.0)

    voxel_size: float = 0.025
    tree: OcTree = OcTree(voxel_size)
    origin: Vector3 = Vector3(0.0, 0.0, 0.0)

    sequence_dir: str = "C:/spaint/build/bin/apps/spaintgui/sequences/Test"
    frame_idx: int = 0

    while True:
        # Try to load the next frame from disk.
        colour_filename: str = os.path.join(sequence_dir, f"frame-{frame_idx:06d}.color.png")
        depth_filename: str = os.path.join(sequence_dir, f"frame-{frame_idx:06d}.depth.png")
        pose_filename: str = os.path.join(sequence_dir, f"frame-{frame_idx:06d}.pose.txt")

        # If the colour image doesn't exist, early out.
        if not os.path.exists(colour_filename):
            break

        # If the colour image exists but the depth image doesn't, skip the frame (it's likely that
        # the user renamed the depth image to prevent this frame being used).
        if not os.path.exists(depth_filename):
            frame_idx += 1
            continue

        print(f"Processing frame {frame_idx}...")

        # Load the depth image and the pose.
        depth_image: np.ndarray = ImageUtil.load_depth_image(depth_filename)
        pose: np.ndarray = PoseUtil.load_pose(pose_filename)

        # Use them to make an Octomap point cloud.
        pcd: Pointcloud = OctomapUtil.make_point_cloud(depth_image, pose, intrinsics)

        # Fuse it into the octree.
        start = timer()
        tree.insert_point_cloud(pcd, origin, discretize=True)
        end = timer()
        print(f"  - Time: {end - start}s")

        # Increment the frame index.
        frame_idx += 1

    # Save the finished octree to disk.
    tree.write_binary("offline_fusion.bt")


if __name__ == "__main__":
    main()
