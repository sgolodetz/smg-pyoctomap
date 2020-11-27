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


def draw_frame(drawer: OcTreeDrawer, pose: np.ndarray) -> None:
    glClearColor(1.0, 1.0, 1.0, 1.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    # print(glGetFloatv(GL_MODELVIEW_MATRIX))
    print(pose)
    # gluLookAt(0.0, -height * 1.5, height, 0.0, 0.0, height / 2, 0.0, 0.0, 1.0)
    # gluLookAt(0.0, -1.0, -2.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0)
    gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0)
    glMultMatrixf(np.linalg.inv(pose).flatten(order='F'))
    # glLoadMatrixf(np.linalg.inv(pose))
    print(np.transpose(glGetFloatv(GL_MODELVIEW_MATRIX)))

    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    # pos: np.ndarray = np.array([-1.0, 1.0, 1.0, 0.0])
    pos: np.ndarray = np.array([0.0, 2.0, -1.0, 0.0])
    glLightfv(GL_LIGHT0, GL_POSITION, pos)

    glEnable(GL_COLOR_MATERIAL)

    drawer.draw()

    glDisable(GL_COLOR_MATERIAL)
    glDisable(GL_LIGHTING)
    glDisable(GL_BLEND)

    pygame.display.flip()


def main() -> None:
    pygame.init()
    window_size: Tuple[int, int] = (640, 480)
    pygame.display.set_mode(window_size, pygame.DOUBLEBUF | pygame.OPENGL)

    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LESS)

    intrinsics: Tuple[float, float, float, float] = (532.5694641250893, 531.5410880910171, 320.0, 240.0)

    glMatrixMode(GL_PROJECTION)
    OctomapUtil.set_projection_matrix(intrinsics, window_size[0], window_size[1])

    drawer: OcTreeDrawer = OcTreeDrawer()
    # drawer.enable_freespace()
    drawer.set_color_mode(CM_COLOR_HEIGHT)

    voxel_size: float = 0.05
    tree: OcTree = OcTree(voxel_size)
    origin: Vector3 = Vector3(0.0, 0.0, 0.0)

    sequence_dir: str = "C:/spaint/build/bin/apps/spaintgui/sequences/Test2"
    frame_idx: int = 0

    while True:  # frame_idx < 100:
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

        # Fuse it into the octree.
        start = timer()
        tree.insert_point_cloud(pcd, origin, discretize=True)
        end = timer()
        print(f"  - Time: {end - start}s")

        # Increment the frame index.
        frame_idx += 1

        # Visualise the octree.
        origin_pose: Pose6D = Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        drawer.set_octree(tree, origin_pose)
        draw_frame(drawer, pose)
        pygame.time.wait(1)

    tree.write_binary("online_fusion.bt")


if __name__ == "__main__":
    main()
