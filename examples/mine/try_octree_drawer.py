from pyoctomap import *


def main() -> None:
    voxel_size: float = 1.0
    tree: OcTree = OcTree(voxel_size)
    origin: Pose6D = Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    drawer: OcTreeDrawer = OcTreeDrawer()
    drawer.set_octree(tree, origin)
    drawer.draw()


if __name__ == "__main__":
    main()
