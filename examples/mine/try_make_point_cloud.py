import numpy as np

from timeit import default_timer as timer
from typing import Tuple

from smg.pyoctomap import *
from smg.utility import ImageUtil


def main() -> None:
    depth_image: np.ndarray = ImageUtil.load_depth_image(
        "C:/spaint/build/bin/apps/spaintgui/sequences/Test/frame-000000.depth.png"
    )

    intrinsics: Tuple[float, float, float, float] = (532.5694641250893, 531.5410880910171, 320.0, 240.0)

    start = timer()
    pcd = OctomapUtil.make_point_cloud(depth_image, intrinsics)
    end = timer()
    print(end - start)

    print(pcd.size())
    print(pcd[0])
    print(pcd[pcd.size() - 1])


if __name__ == "__main__":
    main()
