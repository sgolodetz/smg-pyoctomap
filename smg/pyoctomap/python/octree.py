import threading

from typing import Optional

from ..cpp.pyoctomap import OcTreeCpp, OcTreeDrawer, OcTreeNode, Pointcloud, Pose6D, Vector3


class OcTree:
    """
    A thread-safe wrapper around a C++ OcTree.

    .. note::
        Basic descriptions of the methods are given here. More detailed ones can be found in the Octomap source code.
    """

    # CONSTRUCTOR

    def __init__(self, resolution: float):
        """
        Construct a thread-safe wrapper around a C++ OcTree.

        :param resolution:  The voxel size of the octree's leaves.
        """
        self.__lock: threading.Lock = threading.Lock()
        self.__octree: OcTreeCpp = OcTreeCpp(resolution)

    # PUBLIC METHODS

    def cast_ray(self, origin: Vector3, direction: Vector3, end: Vector3,
                 ignore_unknown_cells: bool = False, max_range: float = -1.0) -> bool:
        """
        Cast a ray from the specified origin in the specified direction, and return the first non-free cell in 'end'.

        .. note::
            If the origin itself is either occupied or unknown, it will be returned.

        :param origin:                  The origin of the ray.
        :param direction:               The direction of the ray.
        :param end:                     A vector into which to store the (centre of the) first non-free cell on the ray.
        :param ignore_unknown_cells:    Whether to treat unknown cells as free.
        :param max_range:               The maximum range after which the raycast will be aborted (if this is <= 0,
                                        no limit will be imposed).
        :return:                        True, if the ray hits an occupied cell, or False if either (i) the maximum
                                        range or octree bounds are hit, or (ii) an unknown node is hit.
        """
        with self.__lock:
            return self.__octree.cast_ray(origin, direction, end, ignore_unknown_cells, max_range)

    def delete_node(self, position: Vector3, depth: int = 0) -> bool:
        """
        Delete the octree node at the specified depth that contains the specified 3D position (if it exists).

        :param position:    The specified 3D position.
        :param depth:       The specified depth.
        :return:            True, if the node was successfully deleted, or False otherwise.
        """
        with self.__lock:
            return self.__octree.delete_node(position, depth)

    def draw(self, drawer: OcTreeDrawer, origin_pose: Optional[Pose6D] = None) -> None:
        """
        Draw the octree using the specified drawer.

        :param drawer:      The drawer with which to draw the octree.
        :param origin_pose: The global transformation (if any) that should be applied when drawing the octree.
        """
        # Use the identity transformation for the global transformation by default.
        if origin_pose is None:
            origin_pose = Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        with self.__lock:
            drawer.set_octree(self.__octree, origin_pose)
            drawer.draw()

    def get_occupancy_thres(self) -> float:
        """
        Get the probability threshold for a node to be occupied.

        :return:    The probability threshold for a node to be occupied.
        """
        with self.__lock:
            return self.__octree.get_occupancy_thres()

    def get_resolution(self) -> float:
        """
        Get the voxel size of the octree's leaves.

        :return:    The voxel size of the octree's leaves.
        """
        with self.__lock:
            return self.__octree.get_resolution()

    def insert_point_cloud(self, scan: Pointcloud, sensor_origin: Vector3, max_range: float = -1.0,
                           lazy_eval: bool = False, discretize: bool = False) -> None:
        """
        Integrate a point cloud into the octree.

        .. note::
            In practice, setting discretize to True tends to make things a lot faster.

        :param scan:            The point cloud (end points of all rays).
        :param sensor_origin:   The origin of the sensor (start point of each ray).
        :param max_range:       The maximum range over which individual rays will be inserted (-1 = complete rays).
        :param lazy_eval:       Whether to omit the update of inner octree nodes after the insertion. This makes things
                                faster, but at the cost of requiring a later call to update the inner nodes.
        :param discretize:      Whether to discretise the scan into octree key cells prior to integration. This reduces
                                the number of raycasts, resulting in a potential speed-up.
        """
        with self.__lock:
            self.__octree.insert_point_cloud(scan, sensor_origin, max_range, lazy_eval, discretize)

    def insert_ray(self, origin: Vector3, end: Vector3, max_range: float = -1.0, lazy_eval: bool = False) -> bool:
        """
        Insert a single ray into the octree.

        :param origin:      The origin of the ray.
        :param end:         The end point of the ray.
        :param max_range:   The maximum range after which the raycast will be aborted (if this is <= 0, no limit will
                            be imposed).
        :param lazy_eval:   Whether to omit the update of inner octree nodes after the insertion. This makes things
                            faster, but at the cost of requiring a later call to update the inner nodes.
        :return:            True, if the operation succeeded, or False otherwise.
        """
        with self.__lock:
            return self.__octree.insert_ray(origin, end, max_range, lazy_eval)

    def is_in_bounds(self, position: Vector3) -> bool:
        """
        Query whether or not the specified 3D position is within the octree bounds.

        :param position:    The specified 3D position.
        :return:            True, if the point is within the octree bounds, or False otherwise.
        """
        with self.__lock:
            return self.__octree.is_in_bounds(position)

    def is_node_occupied(self, occupancy_node: OcTreeNode) -> bool:
        """
        Query whether or not the specified octree node is occupied.

        :param occupancy_node:  The specified octree node.
        :return:                True, if the specified octree node is occupied, or False otherwise.
        """
        with self.__lock:
            return self.__octree.is_node_occupied(occupancy_node)

    def read_binary(self, filename: str) -> bool:
        """
        Replace the octree with one loaded from a binary file.

        :param filename:    The file from which to read the new octree.
        :return:            True, if the operation succeeds, or False otherwise.
        """
        with self.__lock:
            return self.__octree.read_binary(filename)

    def search(self, position: Vector3, depth: int = 0) -> Optional[OcTreeNode]:
        """
        Search for an octree node at the specified 3D position and depth in the octree.

        :param position:    The specified 3D position.
        :param depth:       The specified depth in the octree.
        :return:            The node, if found, or None otherwise.
        """
        with self.__lock:
            return self.__octree.search(position, depth)

    def set_node_value(self, position: Vector3, log_odds_value: float, lazy_eval: bool = False) -> OcTreeNode:
        """
        Set the log-odds value of the octree node with the specified 3D position to the specified value.

        :param position:        The specified 3D position.
        :param log_odds_value:  The log-odds value to set for the node.
        :param lazy_eval:       Whether to omit the update of inner octree nodes after the update. This makes things
                                faster, but at the cost of requiring a later call to update the inner nodes.
        :return:                The updated node.
        """
        with self.__lock:
            return self.__octree.set_node_value(position, log_odds_value, lazy_eval)

    def set_occupancy_thres(self, prob: float) -> None:
        """
        Set the probability threshold for a node to be occupied.

        :param prob:    The probability threshold for a node to be occupied.
        """
        with self.__lock:
            self.__octree.set_occupancy_thres(prob)

    def update_node(self, position: Vector3, occupied: bool, lazy_eval: bool = False) -> OcTreeNode:
        """
        Integrate an occupancy measurement into the node with the specified 3D position.

        :param position:    The specified 3D position.
        :param occupied:    Whether or not the node was measured as being occupied.
        :param lazy_eval:   Whether to omit the update of inner octree nodes after the update. This makes things
                            faster, but at the cost of requiring a later call to update the inner nodes.
        :return:            The updated node.
        """
        with self.__lock:
            return self.__octree.update_node(position, occupied, lazy_eval)

    def write_binary(self, filename: str) -> bool:
        """
        Write a binary representation of the octree to the specified file.

        :param filename:    The file to which to write the octree.
        :return:            True, if the operation succeeded, or False otherwise.
        """
        with self.__lock:
            return self.__octree.write_binary(filename)
