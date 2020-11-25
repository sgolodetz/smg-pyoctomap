from typing import Union


# CLASSES

class OcTree:
	def __init__(self, arg: Union[float, str]): ...

	def cast_ray(self, origin: Vector3, direction: Vector3, end: Vector3, ignore_unknown_cells: bool = False, max_range: float = -1.0) -> bool: ...
	def insert_point_cloud(self, scan: Pointcloud, sensor_origin: Vector3, max_range: float = -1.0, lazy_eval: bool = False, discretize: bool = False) -> None: ...
	def insert_ray(self, origin: Vector3, end: Vector3, max_range: float = -1.0, lazy_eval: bool = False) -> bool: ...
	def is_node_occupied(self, occupancy_node: OcTreeNode) -> bool: ...
	def search(self, value: Vector3, depth: int = 0) -> OcTreeNode: ...
	def update_node(self, value: Vector3, occupied: bool, lazy_eval: bool = False) -> OcTreeNode: ...
	def write_binary(self, filename: str) -> bool: ...

class OcTreeNode:
	def get_occupancy(self) -> float: ...

class Pointcloud:
	def __init__(self): ...

	def push_back(self, p: Vector3) -> None: ...

class Vector3:
	x: float
	y: float
	z: float

	def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0): ...

	def __add__(self, rhs: Vector3) -> Vector3: ...
	def __sub__(self, rhs: Vector3) -> Vector3: ...

	def __repr__(self) -> str: ...

	def copy(self) -> Vector3: ...
	def norm(self) -> float: ...
	def rotate_ip(self, roll: float, pitch: float, yaw: float) -> Vector3: ...
