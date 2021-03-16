import numpy as np

from OpenGL.GL import *
from typing import Tuple

from smg.opengl import OpenGLFrameBuffer, OpenGLMatrixContext, OpenGLUtil
from smg.rigging.helpers import CameraPoseConverter
from smg.utility import GeometryUtil

from ..cpp.pyoctomap import OcTree, OcTreeDrawer, Pose6D


class OctomapPicker:
    """Used to render world-space points images of an octree to allow points in the scene to be picked."""

    # CONSTRUCTOR

    def __init__(self, tree: OcTree, width: int, height: int, intrinsics: Tuple[float, float, float, float]):
        """
        Construct an Octomap picker.

        :param tree:        The octree.
        :param width:       The width of the world-space points images to render.
        :param height:      The height of the world-space points images to render.
        :param intrinsics:  The camera intrinsics.
        """
        self.__drawer: OcTreeDrawer = OcTreeDrawer()
        self.__framebuffer: OpenGLFrameBuffer = OpenGLFrameBuffer(width, height)
        self.__height: int = height
        self.__intrinsics: Tuple[float, float, float, float] = intrinsics
        self.__tree: OcTree = tree
        self.__width: int = width

    # PUBLIC METHODS

    def pick(self, world_from_camera: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Render a world-space points image of the octree from the specified camera pose.

        :param world_from_camera:   The camera pose from which to render.
        :return:                    A tuple consisting of the world-space points image and a mask that indicates
                                    which pixels in the world-space points image are valid.
        """
        with self.__framebuffer:
            # Set the viewport to encompass the whole framebuffer.
            OpenGLUtil.set_viewport((0.0, 0.0), (1.0, 1.0), (self.__width, self.__height))

            # Clear the framebuffer.
            glClearColor(0.0, 0.0, 0.0, 1.0)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

            # Set the projection matrix.
            with OpenGLMatrixContext(GL_PROJECTION, lambda: OpenGLUtil.set_projection_matrix(
                self.__intrinsics, self.__width, self.__height
            )):
                # Set the model-view matrix.
                with OpenGLMatrixContext(GL_MODELVIEW, lambda: OpenGLUtil.load_matrix(
                    CameraPoseConverter.pose_to_modelview(np.linalg.inv(world_from_camera))
                )):
                    # Draw the octree.
                    origin_pose: Pose6D = Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                    self.__drawer.set_octree(self.__tree, origin_pose)
                    self.__drawer.draw()

                    # Get the contents of the depth buffer.
                    depth_image: np.ndarray = OpenGLUtil.read_depth_image(self.__width, self.__height)

                    # Compute the world-space points image.
                    ws_points: np.ndarray = GeometryUtil.compute_world_points_image_fast(
                        depth_image, world_from_camera, self.__intrinsics
                    )

                    # Compute the mask that indicates which world-space points are valid.
                    mask: np.ndarray = np.where(depth_image <= 100.0, 255, 0).astype(np.uint8)

                    # Return a tuple consisting of the masked world-space points image, and the mask itself.
                    return np.where(np.atleast_3d(mask != 0), ws_points, np.zeros(3)), mask
