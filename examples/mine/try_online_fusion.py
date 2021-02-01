import numpy as np
import os
import sys

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

from OpenGL.GL import *
from timeit import default_timer as timer
from typing import Tuple

from smg.opengl import OpenGLMatrixContext, OpenGLUtil
from smg.pyoctomap import *
from smg.rigging.cameras import SimpleCamera
from smg.rigging.controllers import KeyboardCameraController
from smg.rigging.helpers import CameraPoseConverter
from smg.utility import ImageUtil, PoseUtil


def main() -> None:
    # Initialise PyGame and create the window.
    pygame.init()
    window_size: Tuple[int, int] = (640, 480)
    pygame.display.set_mode(window_size, pygame.DOUBLEBUF | pygame.OPENGL)

    # Specify the camera intrinsics.
    intrinsics: Tuple[float, float, float, float] = (532.5694641250893, 531.5410880910171, 320.0, 240.0)

    # Enable the z-buffer.
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LESS)

    # Set up the octree drawer.
    drawer: OcTreeDrawer = OcTreeDrawer()
    drawer.set_color_mode(CM_COLOR_HEIGHT)

    # Create the octree.
    voxel_size: float = 0.05
    tree: OcTree = OcTree(voxel_size)

    # Construct the camera controller.
    camera_controller: KeyboardCameraController = KeyboardCameraController(
        SimpleCamera([0, 0, 0], [0, 0, 1], [0, -1, 0]), canonical_angular_speed=0.05, canonical_linear_speed=0.1
    )

    # noinspection PyUnusedLocal
    pose: np.ndarray = np.eye(4)
    sequence_dir: str = "C:/spaint/build/bin/apps/spaintgui/sequences/Test2"
    frame_idx: int = 0

    while True:
        # Process any PyGame events.
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                tree.write_binary("online_fusion.bt")
                pygame.quit()
                sys.exit(0)

        # Try to load the next frame from disk.
        colour_filename: str = os.path.join(sequence_dir, f"frame-{frame_idx:06d}.color.png")
        depth_filename: str = os.path.join(sequence_dir, f"frame-{frame_idx:06d}.depth.png")
        pose_filename: str = os.path.join(sequence_dir, f"frame-{frame_idx:06d}.pose.txt")

        # If the colour image exists:
        if os.path.exists(colour_filename):
            # If the depth image exists:
            if os.path.exists(depth_filename):
                print(f"Processing frame {frame_idx}...")

                # Load the depth image and the pose.
                depth_image: np.ndarray = ImageUtil.load_depth_image(depth_filename)
                pose = PoseUtil.load_pose(pose_filename)

                # Use them to make an Octomap point cloud.
                pcd: Pointcloud = OctomapUtil.make_point_cloud(depth_image, pose, intrinsics)

                # Fuse the point cloud into the octree.
                start = timer()
                origin: Vector3 = Vector3(0.0, 0.0, 0.0)
                tree.insert_point_cloud(pcd, origin, discretize=True)
                end = timer()
                print(f"  - Time: {end - start}s")
            else:
                # Otherwise, advance the frame index (it's likely that the user renamed the depth image to prevent
                # this frame being used).
                frame_idx += 1

        # Allow the user to control the camera.
        camera_controller.update(pygame.key.get_pressed(), timer() * 1000)

        # Clear the colour and depth buffers.
        glClearColor(1.0, 1.0, 1.0, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Determine the viewing pose.
        # viewing_pose: np.ndarray = np.linalg.inv(pose)
        viewing_pose: np.ndarray = camera_controller.get_pose()

        # Set the projection matrix.
        with OpenGLMatrixContext(GL_PROJECTION, lambda: OpenGLUtil.set_projection_matrix(intrinsics, *window_size)):
            # Set the model-view matrix.
            with OpenGLMatrixContext(GL_MODELVIEW, lambda: OpenGLUtil.load_matrix(
                CameraPoseConverter.pose_to_modelview(viewing_pose)
            )):
                # Draw the octree.
                OctomapUtil.draw_octree(tree, drawer)

        # Swap the front and back buffers.
        pygame.display.flip()

        # Increment the frame index.
        frame_idx += 1


if __name__ == "__main__":
    main()
