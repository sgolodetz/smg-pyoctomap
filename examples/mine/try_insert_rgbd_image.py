import cv2
import numpy as np

from timeit import default_timer as timer
from typing import Tuple

from smg.pyoctomap import *
from smg.utility import ImageUtil


def main() -> None:
    rgb_image: np.ndarray = ImageUtil.flip_channels(cv2.imread(
        "C:/spaint/build/bin/apps/spaintgui/sequences/Test2/frame-000000.color.png"
    ))
    depth_image: np.ndarray = ImageUtil.load_depth_image(
        "C:/spaint/build/bin/apps/spaintgui/sequences/Test2/frame-000000.depth.png"
    )
    pose: np.ndarray = np.eye(4)
    intrinsics: Tuple[float, float, float, float] = (532.5694641250893, 531.5410880910171, 320.0, 240.0)
    pcd, colours = OctomapUtil.make_rgbd_point_cloud(rgb_image, depth_image, pose, intrinsics)

    voxel_size: float = 0.01
    tree: ColorOcTree = ColorOcTree(voxel_size)
    origin: Vector3 = Vector3(0.0, 0.0, 0.0)

    start = timer()

    tree.insert_point_cloud(pcd, origin, discretize=True)
    points = np.array(pcd, copy=False).reshape(-1, 3)
    for i in range(points.shape[0]):
        n: ColorOcTreeNode = tree.search(Vector3(*points[i]))
        n.set_color(*colours[i])
    tree.update_inner_occupancy()

    end = timer()
    print(end - start)

    tree.write("insert_rgbd_image.ot")


if __name__ == "__main__":
    main()
