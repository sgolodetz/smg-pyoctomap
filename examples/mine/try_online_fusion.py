import numpy as np
import os
import sys

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

from OpenGL.GL import *
from OpenGL.GLU import *
from timeit import default_timer as timer
from typing import Tuple

from smg.pyoctomap import *
from smg.utility import ImageUtil, PoseUtil


def draw_octree(tree: OcTree, pose: np.ndarray, drawer: OcTreeDrawer) -> None:
    """
    Visualise the specified octree from the specified pose.

    :param tree:    The octree.
    :param pose:    The current pose.
    :param drawer:  The octree drawer.
    :return:
    """
    # Clear the buffers.
    glClearColor(1.0, 1.0, 1.0, 1.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    # Set the model-view matrix.
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0)
    glMultMatrixf(np.linalg.inv(pose).flatten(order='F'))

    # Enable blending, lighting and materials.
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    pos: np.ndarray = np.array([0.0, 2.0, -1.0, 0.0])
    glLightfv(GL_LIGHT0, GL_POSITION, pos)

    glEnable(GL_COLOR_MATERIAL)

    # Draw the octree.
    origin_pose: Pose6D = Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    drawer.set_octree(tree, origin_pose)
    drawer.draw()

    # Disable blending, lighting and materials again.
    glDisable(GL_COLOR_MATERIAL)
    glDisable(GL_LIGHTING)
    glDisable(GL_BLEND)


def main() -> None:
    """The main function."""
    # Initialise PyGame and create the window.
    pygame.init()
    window_size: Tuple[int, int] = (640, 480)
    pygame.display.set_mode(window_size, pygame.DOUBLEBUF | pygame.OPENGL)

    # Set the projection matrix.
    glMatrixMode(GL_PROJECTION)
    intrinsics: Tuple[float, float, float, float] = (532.5694641250893, 531.5410880910171, 320.0, 240.0)
    OctomapUtil.set_projection_matrix(intrinsics, *window_size)

    # Enable the z-buffer.
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LESS)

    # Set up the octree drawer.
    drawer: OcTreeDrawer = OcTreeDrawer()
    drawer.set_color_mode(CM_COLOR_HEIGHT)

    # Create the octree.
    voxel_size: float = 0.05
    tree: OcTree = OcTree(voxel_size)

    # Until the sequence is finished:
    sequence_dir: str = "C:/spaint/build/bin/apps/spaintgui/sequences/Test2"
    frame_idx: int = 0
    while True:
        # Process any PyGame events.
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit(0)

        # Try to load the next frame from disk.
        colour_filename: str = os.path.join(sequence_dir, f"frame-{frame_idx:06d}.color.png")
        depth_filename: str = os.path.join(sequence_dir, f"frame-{frame_idx:06d}.depth.png")
        pose_filename: str = os.path.join(sequence_dir, f"frame-{frame_idx:06d}.pose.txt")

        # If the colour image doesn't exist, early out.
        if not os.path.exists(colour_filename):
            break

        # If the colour image exists but the depth image doesn't, skip the frame (it's likely that
        # the user renamed the depth image to prevent this frame being used).
        if not os.path.exists(depth_filename):
            frame_idx += 1
            continue

        print(f"Processing frame {frame_idx}...")

        # Load the depth image and the pose.
        depth_image: np.ndarray = ImageUtil.load_depth_image(depth_filename)
        pose: np.ndarray = PoseUtil.load_pose(pose_filename)

        # Use them to make an Octomap point cloud.
        pcd: Pointcloud = OctomapUtil.make_point_cloud(depth_image, pose, intrinsics)

        # Fuse the point cloud into the octree.
        start = timer()
        origin: Vector3 = Vector3(0.0, 0.0, 0.0)
        tree.insert_point_cloud(pcd, origin, discretize=True)
        end = timer()
        print(f"  - Time: {end - start}s")

        # Draw the octree.
        draw_octree(tree, pose, drawer)

        # Swap the buffers.
        pygame.display.flip()

        # Increment the frame index.
        frame_idx += 1

    # Finally, save the octree to a file for later use.
    tree.write_binary("online_fusion.bt")


if __name__ == "__main__":
    main()
