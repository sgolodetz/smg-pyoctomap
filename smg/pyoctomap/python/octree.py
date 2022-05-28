import threading

from typing import Optional

from ..cpp.pyoctomap import OcTreeCpp, OcTreeDrawer, OcTreeNode, Pointcloud, Pose6D, Vector3


class OcTree:
    """A thread-safe wrapper around a C++ OcTree."""

    # CONSTRUCTOR

    def __init__(self, resolution: float):
        self.__lock: threading.Lock = threading.Lock()
        self.__octree: OcTreeCpp = OcTreeCpp(resolution)

    # PUBLIC METHODS

    def cast_ray(self, origin: Vector3, direction: Vector3, end: Vector3,
                 ignore_unknown_cells: bool = False, max_range: float = -1.0) -> bool:
        with self.__lock:
            return self.__octree.cast_ray(origin, direction, end, ignore_unknown_cells, max_range)

    def delete_node(self, value: Vector3, depth: int = 0) -> bool:
        with self.__lock:
            return self.__octree.delete_node(value, depth)

    def draw(self, drawer: OcTreeDrawer, origin_pose: Optional[Pose6D] = None) -> None:
        if origin_pose is None:
            origin_pose = Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        with self.__lock:
            drawer.set_octree(self.__octree, origin_pose)
            drawer.draw()

    def get_occupancy_thres(self) -> float:
        with self.__lock:
            return self.__octree.get_occupancy_thres()

    def get_resolution(self) -> float:
        with self.__lock:
            return self.__octree.get_resolution()

    def insert_point_cloud(self, scan: Pointcloud, sensor_origin: Vector3, max_range: float = -1.0,
                           lazy_eval: bool = False, discretize: bool = False) -> None:
        with self.__lock:
            self.__octree.insert_point_cloud(scan, sensor_origin, max_range, lazy_eval, discretize)

    def insert_ray(self, origin: Vector3, end: Vector3, max_range: float = -1.0, lazy_eval: bool = False) -> bool:
        with self.__lock:
            return self.__octree.insert_ray(origin, end, max_range, lazy_eval)

    def is_in_bounds(self, value: Vector3) -> bool:
        with self.__lock:
            return self.__octree.is_in_bounds(value)

    def is_node_occupied(self, occupancy_node: OcTreeNode) -> bool:
        with self.__lock:
            return self.__octree.is_node_occupied(occupancy_node)

    def read_binary(self, filename: str) -> bool:
        with self.__lock:
            return self.__octree.read_binary(filename)

    def search(self, value: Vector3, depth: int = 0) -> Optional[OcTreeNode]:
        with self.__lock:
            return self.__octree.search(value, depth)

    def set_node_value(self, value: Vector3, log_odds_value: float, lazy_eval: bool = False) -> OcTreeNode:
        with self.__lock:
            return self.__octree.set_node_value(value, log_odds_value, lazy_eval)

    def set_occupancy_thres(self, prob: float) -> None:
        with self.__lock:
            self.__octree.set_occupancy_thres(prob)

    def update_node(self, value: Vector3, occupied: bool, lazy_eval: bool = False) -> OcTreeNode:
        with self.__lock:
            return self.__octree.update_node(value, occupied, lazy_eval)

    def write_binary(self, filename: str) -> bool:
        with self.__lock:
            return self.__octree.write_binary(filename)
