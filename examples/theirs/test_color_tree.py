# This is a Python reimplementation of test_color_tree from the Octomap code.

from smg.pyoctomap import *


def main() -> None:
    res: float = 0.05  # create empty tree with resolution 0.05 (different from default 0.1 for test)
    tree: ColorOcTree = ColorOcTree(res)
    # insert some measurements of occupied cells
    for x in range(-20, 20):
        for y in range(-20, 20):
            for z in range(-20, 20):
                endpoint: Vector3 = Vector3(x * 0.05 + 0.01, y * 0.05 + 0.01, z * 0.05 + 0.01)
                n: ColorOcTreeNode = tree.update_node(endpoint, True)
                n.set_color(z * 5 + 100, x * 5 + 100, y * 5 + 100)

    # insert some measurements of free cells
    for x in range(-30, 30):
        for y in range(-30, 30):
            for z in range(-30, 30):
                endpoint: Vector3 = Vector3(x * 0.02 + 2.0, y * 0.02 + 2.0, z * 0.02 + 2.0)
                n: ColorOcTreeNode = tree.update_node(endpoint, False)
                n.set_color(255, 255, 0)  # set color to yellow

    # set inner node colors
    tree.update_inner_occupancy()

    # should already be pruned
    #   EXPECT_EQ(tree.size(), tree.calcNumNodes());
    initial_size: int = tree.size()
    #   EXPECT_EQ(initialSize, 1034);
    tree.prune()
    #   EXPECT_EQ(tree.size(), tree.calcNumNodes());
    #   EXPECT_EQ(initialSize, tree.size());

    print()

    print("\nWriting to / from file\n===============================")
    filename: str = "simple_color_tree.ot"
    print(f"Writing color tree to {filename}")
    # write color tree
    #   EXPECT_TRUE(tree.write(filename));
    #
    #
    #   // read tree file
    #   cout << "Reading color tree from "<< filename <<"\n";
    #   AbstractOcTree* read_tree = AbstractOcTree::read(filename);
    #   EXPECT_TRUE(read_tree);
    #   EXPECT_EQ(read_tree->getTreeType().compare(tree.getTreeType()), 0);
    #   EXPECT_FLOAT_EQ(read_tree->getResolution(), tree.getResolution());
    #   EXPECT_EQ(read_tree->size(), tree.size());
    #   ColorOcTree* read_color_tree = dynamic_cast<ColorOcTree*>(read_tree);
    #   EXPECT_TRUE(read_color_tree);
    #
    #   EXPECT_TRUE(tree == *read_color_tree);
    #
    #
    #   {
    #     cout << "Performing some queries:" << endl;
    #     // TODO: some more meaningful tests
    #     point3d query (0., 0., 0.);
    #     ColorOcTreeNode* result = tree.search (query);
    #     ColorOcTreeNode* result2 = read_color_tree->search (query);
    #     std::cout << "READ: ";
    #     print_query_info(query, result);
    #     std::cout << "WRITE: ";
    #     print_query_info(query, result2);
    #     EXPECT_TRUE(result);
    #     EXPECT_TRUE(result2);
    #     EXPECT_EQ(result->getColor(), result2->getColor());
    #     EXPECT_EQ(result->getLogOdds(), result2->getLogOdds());
    #
    #     query = point3d(-1.,-1.,-1.);
    #     result = tree.search (query);
    #     result2 = read_color_tree->search (query);
    #     print_query_info(query, result);
    #     std::cout << "READ: ";
    #     print_query_info(query, result);
    #     std::cout << "WRITE: ";
    #     print_query_info(query, result2);
    #     EXPECT_TRUE(result);
    #     EXPECT_TRUE(result2);
    #     EXPECT_EQ(result->getColor(), result2->getColor());
    #     EXPECT_EQ(result->getLogOdds(), result2->getLogOdds());
    #
    #     query = point3d(1.,1.,1.);
    #     result = tree.search (query);
    #     result2 = read_color_tree->search (query);
    #     print_query_info(query, result);
    #     std::cout << "READ: ";
    #     print_query_info(query, result);
    #     std::cout << "WRITE: ";
    #     print_query_info(query, result2);
    #     EXPECT_FALSE(result);
    #     EXPECT_FALSE(result2);
    #   }
    #
    #   delete read_tree;
    #   read_tree = NULL;
    #
    #   {
    #     std::cout << "\nPruning / expansion\n===============================\n";
    #     EXPECT_EQ(initialSize, tree.size());
    #     EXPECT_EQ(initialSize, tree.calcNumNodes());
    #     std::cout << "Initial size: " << tree.size() << std::endl;
    #
    #     // tree should already be pruned during insertion:
    #     tree.prune();
    #     EXPECT_EQ(initialSize, tree.size());
    #     EXPECT_EQ(initialSize, tree.calcNumNodes());
    #
    #     tree.expand();
    #     std::cout << "Size after expansion: " << tree.size() << std::endl;
    #     EXPECT_EQ(tree.size(), tree.calcNumNodes());
    #
    #     // prune again, should be same as initial size
    #
    #     tree.prune();
    #     EXPECT_EQ(initialSize, tree.size());
    #     EXPECT_EQ(initialSize, tree.calcNumNodes());
    #
    #   }
    #
    #   // delete / create some nodes
    #   {
    #     std::cout << "\nCreating / deleting nodes\n===============================\n";
    #     size_t initialSize = tree.size();
    #     EXPECT_EQ(initialSize, tree.calcNumNodes());
    #     std::cout << "Initial size: " << initialSize << std::endl;
    #
    #     point3d newCoord(-2.0, -2.0, -2.0);
    #     ColorOcTreeNode* newNode = tree.updateNode(newCoord, true);
    #     newNode->setColor(255,0,0);
    #     EXPECT_TRUE(newNode != NULL);
    #
    #     const size_t insertedSize = tree.size();
    #     std::cout << "Size after one insertion: " << insertedSize << std::endl;
    #     EXPECT_EQ(insertedSize, initialSize+6);
    #     EXPECT_EQ(insertedSize, tree.calcNumNodes());
    #
    #     // find parent of newly inserted node:
    #     ColorOcTreeNode* parentNode = tree.search(newCoord, tree.getTreeDepth() -1);
    #     EXPECT_TRUE(parentNode);
    #     EXPECT_TRUE(tree.nodeHasChildren(parentNode));
    #
    #     // only one child exists:
    #     EXPECT_TRUE(tree.nodeChildExists(parentNode, 0));
    #     for (size_t i = 1; i < 8; ++i){
    #       EXPECT_FALSE(tree.nodeChildExists(parentNode, i));
    #     }
    #
    #     tree.deleteNodeChild(parentNode, 0);
    #     EXPECT_EQ(tree.size(), tree.calcNumNodes());
    #     EXPECT_EQ(tree.size(), insertedSize - 1);
    #
    #     tree.prune();
    #     EXPECT_EQ(tree.size(), tree.calcNumNodes());
    #     EXPECT_EQ(tree.size(), insertedSize - 1);
    #
    #     tree.expandNode(parentNode);
    #     EXPECT_EQ(tree.size(), tree.calcNumNodes());
    #     EXPECT_EQ(tree.size(), insertedSize + 7);
    #
    #     EXPECT_TRUE(tree.pruneNode(parentNode));
    #     EXPECT_EQ(tree.size(), tree.calcNumNodes());
    #     EXPECT_EQ(tree.size(), insertedSize - 1);
    #
    #     EXPECT_TRUE(tree.write("simple_color_tree_ed.ot"));
    #
    #   }

    pass


if __name__ == "__main__":
    main()
