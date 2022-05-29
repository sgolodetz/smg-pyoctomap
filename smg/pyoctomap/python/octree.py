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

    def delete_node(self, value: Vector3, depth: int = 0) -> bool:
        """
        Delete the octree node at the specified depth that contains the specified 3D point (if it exists).

        :param value:   The specified 3D point.
        :param depth:   The specified depth.
        :return:        True, if the node was successfully deleted, or False otherwise.
        """
        with self.__lock:
            return self.__octree.delete_node(value, depth)

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

    def is_in_bounds(self, value: Vector3) -> bool:
        """
        TODO

        :param value:   TODO
        :return:        TODO
        """
        with self.__lock:
            return self.__octree.is_in_bounds(value)

    def is_node_occupied(self, occupancy_node: OcTreeNode) -> bool:
        """
        TODO

        :param occupancy_node:  TODO
        :return:                TODO
        """
        with self.__lock:
            return self.__octree.is_node_occupied(occupancy_node)

    def read_binary(self, filename: str) -> bool:
        """
        TODO

        :param filename:    TODO
        :return:            TODO
        """
        with self.__lock:
            return self.__octree.read_binary(filename)

    def search(self, value: Vector3, depth: int = 0) -> Optional[OcTreeNode]:
        """
        TODO

        :param value:   TODO
        :param depth:   TODO
        :return:        TODO
        """
        with self.__lock:
            return self.__octree.search(value, depth)

    def set_node_value(self, value: Vector3, log_odds_value: float, lazy_eval: bool = False) -> OcTreeNode:
        """
        TODO

        :param value:           TODO
        :param log_odds_value:  TODO
        :param lazy_eval:       TODO
        :return:                TODO
        """
        with self.__lock:
            return self.__octree.set_node_value(value, log_odds_value, lazy_eval)

    def set_occupancy_thres(self, prob: float) -> None:
        """
        TODO

        :param prob:    TODO
        """
        with self.__lock:
            self.__octree.set_occupancy_thres(prob)

    def update_node(self, value: Vector3, occupied: bool, lazy_eval: bool = False) -> OcTreeNode:
        """
        TODO

        :param value:       TODO
        :param occupied:    TODO
        :param lazy_eval:   TODO
        :return:            TODO
        """
        with self.__lock:
            return self.__octree.update_node(value, occupied, lazy_eval)

    def write_binary(self, filename: str) -> bool:
        """
        TODO

        :param filename:    TODO
        :return:            TODO
        """
        with self.__lock:
            return self.__octree.write_binary(filename)
