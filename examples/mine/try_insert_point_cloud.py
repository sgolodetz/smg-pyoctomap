import math
import numpy as np

from smg.pyoctomap import *


def main() -> None:
    voxel_size: float = 1.0
    half_voxel_size: float = voxel_size / 2

    tree: OcTree = OcTree(voxel_size)
    pcd: Pointcloud = Pointcloud()

    centre: Vector3 = Vector3(half_voxel_size, half_voxel_size, half_voxel_size)
    offset: Vector3 = Vector3(voxel_size * 10, 0.0, 0.0)

    for angle in np.linspace(0.0, 2 * math.pi, 8, endpoint=False):
        angled_offset: Vector3 = offset.copy()
        angled_offset.rotate_ip(0, 0, angle)
        pcd.push_back(centre + angled_offset)

    origin: Vector3 = centre + Vector3(voxel_size * 5, 0, 0)
    tree.insert_point_cloud(pcd, origin)

    tree.write_binary("insert_point_cloud.bt")


if __name__ == "__main__":
    main()
