import numpy as np

from timeit import default_timer as timer
from typing import Tuple

from smg.pyoctomap import *
from smg.utility import ImageUtil


def main() -> None:
    depth_image: np.ndarray = ImageUtil.load_depth_image(
        "C:/spaint/build/bin/apps/spaintgui/sequences/Test/frame-000000.depth.png"
    )
    pose: np.ndarray = np.eye(4)
    intrinsics: Tuple[float, float, float, float] = (532.5694641250893, 531.5410880910171, 320.0, 240.0)
    pcd: Pointcloud = OctomapUtil.make_point_cloud(depth_image, pose, intrinsics)

    voxel_size: float = 0.05
    tree: OcTree = OcTree(voxel_size)
    origin: Vector3 = Vector3(0.0, 0.0, 0.0)

    start = timer()
    tree.insert_point_cloud(pcd, origin)
    end = timer()
    print(end - start)

    tree.write_binary("insert_depth_image.bt")


if __name__ == "__main__":
    main()
