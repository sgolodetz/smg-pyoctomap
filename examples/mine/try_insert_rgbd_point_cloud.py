import math
import numpy as np

from smg.pyoctomap import *


def main() -> None:
    voxel_size: float = 1.0
    half_voxel_size: float = voxel_size / 2
    tree: ColorOcTree = ColorOcTree(voxel_size)

    # Make the point cloud.
    num_points: int = 8

    pcd: Pointcloud = Pointcloud()
    colours: np.ndarray = np.zeros((num_points, 3), dtype=np.uint8)

    centre: Vector3 = Vector3(half_voxel_size, half_voxel_size, half_voxel_size)
    offset: Vector3 = Vector3(voxel_size * 10, 0.0, 0.0)

    i: int = 0
    for angle in np.linspace(0.0, 2 * math.pi, num_points, endpoint=False):
        angled_offset: Vector3 = offset.copy()
        angled_offset.rotate_ip(0, 0, angle)
        pcd.push_back(centre + angled_offset)
        colours[i, 0] = np.uint8(i * 255.0 / (num_points - 1))
        i += 1

    # Insert the point cloud into the octree.
    origin: Vector3 = centre + Vector3(voxel_size * 5, 0, 0)
    tree.insert_point_cloud(pcd, origin, discretize=True)

    # Colour the relevant octree cells. This isn't very efficient, but I couldn't find a more efficient way
    # to do this without significant changes to the Octomap interface.
    for i in range(num_points):
        n: ColorOcTreeNode = tree.search(pcd[i])
        n.set_color(*colours[i])

    tree.update_inner_occupancy()

    # Save the octree for later viewing in Octovis.
    tree.write("insert_rgbd_point_cloud.ot")


if __name__ == "__main__":
    main()
