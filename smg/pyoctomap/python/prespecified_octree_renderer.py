import numpy as np

from OpenGL.GL import *
from typing import Optional, Tuple

from smg.opengl import OpenGLFrameBuffer, OpenGLMatrixContext, OpenGLUtil
from smg.rigging.helpers import CameraPoseConverter

from ..cpp.pyoctomap import OcTree, OcTreeDrawer
from .octomap_util import OctomapUtil


class PrespecifiedOctreeRenderer:
    """TODO"""

    # CONSTRUCTOR

    def __init__(self, tree: OcTree, drawer: OcTreeDrawer):
        """TODO"""
        self.__drawer = drawer     # type: OcTreeDrawer
        self.__framebuffer = None  # type: Optional[OpenGLFrameBuffer]
        self.__tree = tree         # type: OcTree

    # DESTRUCTOR

    def __del__(self):
        """Destroy the renderer."""
        self.terminate()

    # SPECIAL METHODS

    def __enter__(self):
        """No-op (needed to allow the renderer's lifetime to be managed by a with statement)."""
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        """Destroy the renderer at the end of the with statement that's used to manage its lifetime."""
        self.terminate()

    # PUBLIC METHODS

    def render_to_image(self, world_from_camera: np.ndarray, image_size: Tuple[int, int],
                        intrinsics: Tuple[float, float, float, float]) -> np.ndarray:
        """
        Render the pre-specified octree to an image.

        :param world_from_camera:   The pose from which to render the octree.
        :param image_size:          The size of image to render, as a (width, height) tuple.
        :param intrinsics:          The camera intrinsics, as an (fx, fy, cx, cy) tuple.
        :return:                    The rendered image.
        """
        # Make sure the OpenGL frame buffer has been constructed and has the right size.
        width, height = image_size
        if self.__framebuffer is None:
            self.__framebuffer = OpenGLFrameBuffer(width, height)
        elif width != self.__framebuffer.width or height != self.__framebuffer.height:
            self.__framebuffer.terminate()
            self.__framebuffer = OpenGLFrameBuffer(width, height)

        with self.__framebuffer:
            # Set the viewport to encompass the whole frame buffer.
            OpenGLUtil.set_viewport((0.0, 0.0), (1.0, 1.0), (width, height))

            # Clear the background to black.
            glClearColor(1.0, 1.0, 1.0, 1.0)
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

            # Set the projection matrix.
            with OpenGLMatrixContext(GL_PROJECTION, lambda: OpenGLUtil.set_projection_matrix(
                intrinsics, width, height
            )):
                # Set the model-view matrix.
                with OpenGLMatrixContext(GL_MODELVIEW, lambda: OpenGLUtil.load_matrix(
                    CameraPoseConverter.pose_to_modelview(np.linalg.inv(world_from_camera))
                )):
                    # Render the octree.
                    OctomapUtil.draw_octree(self.__tree, self.__drawer)

                    # Read the contents of the frame buffer into an image and return it.
                    return OpenGLUtil.read_bgr_image(width, height)

    def terminate(self) -> None:
        """Destroy the renderer."""
        if self.__framebuffer is not None:
            self.__framebuffer.terminate()
            self.__framebuffer = None
