from typing import Optional, Union


# CLASSES

class ColorOcTree:
	def __init__(self, resolution: float): ...

	def insert_point_cloud(self, scan: Pointcloud, sensor_origin: Vector3, max_range: float = -1.0, lazy_eval: bool = False, discretize: bool = False) -> None: ...
	def prune(self) -> None: ...
	def search(self, value: Vector3, depth: int = 0) -> ColorOcTreeNode: ...
	def size(self) -> int: ...
	def update_inner_occupancy(self) -> None: ...
	def update_node(self, value: Vector3, occupied: bool lazy_eval: bool = False) -> ColorOcTreeNode: ...
	def write(self, filename: str) -> bool: ...

class ColorOcTreeNode:
	def set_color(self, r: int, g: int, b: int) -> None: ...

class OcTree:
	def __init__(self, arg: Union[float, str]): ...

	def cast_ray(self, origin: Vector3, direction: Vector3, end: Vector3, ignore_unknown_cells: bool = False, max_range: float = -1.0) -> bool: ...
	def delete_node(self, value: Vector3, depth: int = 0) -> bool: ...
	def get_occupancy_thres(self) -> float: ...
	def get_resolution(self) -> float: ...
	def insert_point_cloud(self, scan: Pointcloud, sensor_origin: Vector3, max_range: float = -1.0, lazy_eval: bool = False, discretize: bool = False) -> None: ...
	def insert_ray(self, origin: Vector3, end: Vector3, max_range: float = -1.0, lazy_eval: bool = False) -> bool: ...
	def is_node_occupied(self, occupancy_node: OcTreeNode) -> bool: ...
	def is_point_in_bounds(self, value: Vector3) -> bool: ...
	def read_binary(self, filename: str) -> bool: ...
	def search(self, value: Vector3, depth: int = 0) -> Optional[OcTreeNode]: ...
	def set_node_value(self, value: Vector3, log_odds_value: float, lazy_eval: bool = False) -> OcTreeNode: ...
	def set_occupancy_thres(self, prob: float) -> None: ...
	def update_node(self, value: Vector3, occupied: bool, lazy_eval: bool = False) -> OcTreeNode: ...
	def write_binary(self, filename: str) -> bool: ...

class OcTreeDrawer:
	def __init__(self): ...

	def draw(self) -> None: ...
	def enable_freespace(self, enabled: bool = True) -> None: ...
	def set_color_mode(self, mode: EColorMode) -> None: ...
	def set_octree(self, octree: OcTree, origin: Pose6D, map_id: int = 0) -> None: ...

class OcTreeNode:
	def get_occupancy(self) -> float: ...

class Pointcloud:
	def __init__(self): ...

	def __getitem__(self, i: int) -> Vector3: ...
	def __setitem__(self, i: int, v: Vector3) -> None: ...

	def push_back(self, p: Vector3) -> None: ...
	def reserve(self, size: int) -> None: ...
	def resize(self, new_size: int) -> None: ...
	def size(self) -> int: ...

class Pose6D:
	def __init__(self, x: float, y: float, z: float, roll: float, pitch: float, yaw: float): ...

class Vector3:
	x: float
	y: float
	z: float

	def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0): ...

	def __add__(self, rhs: Vector3) -> Vector3: ...
	def __mul__(self, rhs: float) -> Vector3: ...
	def __sub__(self, rhs: Vector3) -> Vector3: ...

	def __repr__(self) -> str: ...

	def copy(self) -> Vector3: ...
	def norm(self) -> float: ...
	def rotate_ip(self, roll: float, pitch: float, yaw: float) -> Vector3: ...

# ENUMERATIONS

class EColorMode(int):
	pass

CM_FLAT: EColorMode
CM_PRINTOUT: EColorMode
CM_COLOR_HEIGHT: EColorMode
CM_GRAY_HEIGHT: EColorMode
CM_SEMANTIC: EColorMode
