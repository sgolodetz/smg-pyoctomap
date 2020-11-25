from smg.pyoctomap import *


def main() -> None:
    voxel_size: float = 1.0
    half_voxel_size: float = voxel_size / 2

    tree: OcTree = OcTree(voxel_size)

    for i in range(100):
        x: float = i * voxel_size + half_voxel_size
        p: Vector3 = Vector3(x, 0, 0)
        tree.update_node(p, i % 2 == 0)
        n: OcTreeNode = tree.search(p)
        if n is not None:
            print(f"Occupancy Probability {p}: {n.get_occupancy()}")

    tree.write_binary("update_node.bt")


if __name__ == "__main__":
    main()
