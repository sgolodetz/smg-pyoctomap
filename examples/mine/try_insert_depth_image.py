import numpy as np

from smg.pyoctomap import *
from smg.utility import GeometryUtil, ImageUtil


def main() -> None:
    depth_image: np.ndarray = ImageUtil.load_depth_image(
        "C:/spaint/build/bin/apps/spaintgui/sequences/Test/frame-000000.depth.png"
    )

    height, width = depth_image.shape
    fx, fy, cx, cy = (532.5694641250893, 531.5410880910171, 320.0, 240.0)

    depth_mask: np.ndarray = np.where(depth_image != 0, 255, 0).astype(np.uint8)
    pose: np.ndarray = np.eye(4)
    ws_points: np.ndarray = np.zeros((height, width, 3), dtype=float)
    GeometryUtil.compute_world_points_image(depth_image, depth_mask, pose, fx, fy, cx, cy, ws_points)

    pcd: Pointcloud = Pointcloud()

    for y in range(height):
        for x in range(width):
            if depth_mask[y, x] != 0:
                p: np.ndarray = ws_points[y, x]
                pcd.push_back(Vector3(p[0], p[1], p[2]))

    voxel_size: float = 0.005
    tree: OcTree = OcTree(voxel_size)

    origin: Vector3 = Vector3(0.0, 0.0, 0.0)
    tree.insert_point_cloud(pcd, origin)

    tree.write_binary("insert_depth_image.bt")


if __name__ == "__main__":
    main()
