import math
import numpy as np

from smg.pyoctomap import *


def main() -> None:
    voxel_size: float = 1.0
    half_voxel_size: float = voxel_size / 2

    tree: ColorOcTree = ColorOcTree(voxel_size)

    num_points: int = 8

    pcd: Pointcloud = Pointcloud()
    colours: np.ndarray = np.zeros((num_points, 3), dtype=float)

    centre: Vector3 = Vector3(half_voxel_size, half_voxel_size, half_voxel_size)
    offset: Vector3 = Vector3(voxel_size * 10, 0.0, 0.0)

    i: int = 0
    for angle in np.linspace(0.0, 2 * math.pi, num_points, endpoint=False):
        angled_offset: Vector3 = offset.copy()
        angled_offset.rotate_ip(0, 0, angle)
        pcd.push_back(centre + angled_offset)
        colours[i, 0] = i * 255.0 / (num_points - 1)
        i += 1

    origin: Vector3 = centre + Vector3(voxel_size * 5, 0, 0)
    tree.insert_point_cloud(pcd, colours, origin, discretize=True)

    for i in range(8):
        n: ColorOcTreeNode = tree.search(pcd[i])
        n.set_color(*colours[i].astype(np.uint8))

    tree.write("insert_point_cloud_colour.ot")


if __name__ == "__main__":
    main()
