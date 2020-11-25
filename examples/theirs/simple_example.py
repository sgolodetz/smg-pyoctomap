# This is a Python reimplementation of simple_example from the Octomap code.

from smg.pyoctomap import *


def print_query_info(query: Vector3, node: OcTreeNode):
    if node is not None:
        print(f"occupancy probability at {query}:\t {node.get_occupancy()}")
    else:
        print(f"occupancy probability at {query}:\t is unknown")


def main():
    print()
    print("generating example map")

    tree: OcTree = OcTree(1.0)  # create empty tree with resolution 0.1

    # insert some measurements of occupied cells

    for x in range(-20, 20):
        for y in range(-20, 20):
            for z in range(-20, 20):
                endpoint: Vector3 = Vector3(x * 0.05, y * 0.05, z * 0.05)
                tree.update_node(endpoint, True)  # integrate 'occupied' measurement

    # insert some measurements of free cells

    for x in range(-30, 30):
        for y in range(-30, 30):
            for z in range(-30, 30):
                endpoint: Vector3 = Vector3(x * 0.02 - 1.0, y * 0.02 - 1.0, z * 0.02 - 1.0)
                tree.update_node(endpoint, False)  # integrate 'free' measurement

    print()
    print("performing some queries:")

    query: Vector3 = Vector3(0., 0., 0.)
    result: OcTreeNode = tree.search(query)
    print_query_info(query, result)

    query = Vector3(-1., -1., -1.)
    result = tree.search(query)
    print_query_info(query, result)

    query = Vector3(1., 1., 1.)
    result = tree.search(query)
    print_query_info(query, result)

    print()
    tree.write_binary("simple_tree.bt")
    print("wrote example file simple_tree.bt\n")
    print("now you can use octovis to visualize: octovis simple_tree.bt")
    print("Hint: hit 'F'-key in viewer to see the freespace\n")


if __name__ == "__main__":
    main()
