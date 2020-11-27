import numpy as np

from timeit import default_timer as timer

from pyoctomap import *
from smg.utility import GeometryUtil, ImageUtil


def make_pcd_fast(pcd, ws_points):
    np.copyto(np.array(pcd, copy=False), ws_points.reshape(-1))

    print(pcd.size())
    print(pcd[0])
    print(pcd[pcd.size() - 1])


def make_pcd_faster(pcd, ws_points):
    for i in range(ws_points.shape[0]):
        p: np.ndarray = ws_points[i]
        pcd[i] = Vector3(p[0], p[1], p[2])

    print(pcd.size())
    print(pcd[0])
    print(pcd[pcd.size() - 1])


def compute_world_points_image(depth_image: np.ndarray, pose: np.ndarray, fx: float, fy: float, cx: float, cy: float,
                               ws_points: np.ndarray) -> None:
    height, width = depth_image.shape
    xl = np.array(range(width))
    yl = np.array(range(height))
    al = np.tile((xl - cx) / fx, height).reshape(height, width) * depth_image
    bl = np.transpose(np.tile((yl - cy) / fy, width).reshape(width, height)) * depth_image
    for i in range(3):
        ws_points[:, :, i] = pose[i, 0] * al + pose[i, 1] * bl + pose[i, 2] * depth_image


def make_pcd_from_depth_image(depth_image: np.ndarray, fx: float, fy: float, cx: float, cy: float) -> Pointcloud():
    depth_mask: np.ndarray = np.where(depth_image != 0, 255, 0).astype(np.uint8)
    pose: np.ndarray = np.eye(4)
    height, width = depth_image.shape
    ws_points: np.ndarray = np.zeros((height, width, 3), dtype=float)
    # GeometryUtil.compute_world_points_image(depth_image, depth_mask, pose, fx, fy, cx, cy, ws_points)
    compute_world_points_image(depth_image, pose, fx, fy, cx, cy, ws_points)
    print(ws_points[0, 0])

    flat_ws_points: np.ndarray = ws_points.reshape((height * width, 3))
    flat_mask: np.ndarray = depth_mask.reshape(height * width)
    retained_ws_points: np.ndarray = np.compress(flat_mask, flat_ws_points, axis=0)

    pcd: Pointcloud = Pointcloud()
    pcd.resize(retained_ws_points.shape[0])
    np.copyto(np.array(pcd, copy=False), retained_ws_points.reshape(-1))

    return pcd


def make_pcd_slow(pcd, ws_points, mask):
    height, width = mask.shape

    for y in range(height):
        for x in range(width):
            if mask[y, x] != 0:
                p: np.ndarray = ws_points[y, x]
                pcd.push_back(Vector3(p[0], p[1], p[2]))

    print(pcd.size())
    print(pcd[0])
    print(pcd[pcd.size() - 1])


def main() -> None:
    depth_image: np.ndarray = ImageUtil.load_depth_image(
        "C:/spaint/build/bin/apps/spaintgui/sequences/Test/frame-000000.depth.png"
    )

    height, width = depth_image.shape
    fx, fy, cx, cy = (532.5694641250893, 531.5410880910171, 320.0, 240.0)

    # depth_mask: np.ndarray = np.where(depth_image != 0, 255, 0).astype(np.uint8)
    # pose: np.ndarray = np.eye(4)
    # ws_points: np.ndarray = np.zeros((height, width, 3), dtype=float)
    # GeometryUtil.compute_world_points_image(depth_image, depth_mask, pose, fx, fy, cx, cy, ws_points)
    #
    # pcd: Pointcloud = Pointcloud()
    # start = timer()
    # make_pcd_slow(pcd, ws_points, depth_mask)
    # end = timer()
    # print(end - start)
    #
    # flat_ws_points: np.ndarray = ws_points.reshape((height * width, 3))
    # flat_mask: np.ndarray = depth_mask.reshape(height * width)
    # retained_ws_points: np.ndarray = np.compress(flat_mask, flat_ws_points, axis=0)
    #
    # pcd = Pointcloud()
    # start = timer()
    # pcd.resize(retained_ws_points.shape[0])
    # make_pcd_faster(pcd, retained_ws_points)
    # end = timer()
    # print(end - start)
    #
    # pcd = Pointcloud()
    # start = timer()
    # pcd.resize(retained_ws_points.shape[0])
    # make_pcd_fast(pcd, retained_ws_points)
    # end = timer()
    # print(end - start)

    start = timer()
    pcd = make_pcd_from_depth_image(depth_image, fx, fy, cx, cy)
    end = timer()
    print(end - start)

    print(pcd.size())
    print(pcd[0])
    print(pcd[pcd.size() - 1])


if __name__ == "__main__":
    main()
