import math
import numpy as np
import os
import sys

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

from OpenGL.GL import *
from OpenGL.GLU import *
from typing import Tuple

from pyoctomap import *


def draw_frame(drawer: OcTreeDrawer) -> None:
    glClearColor(1.0, 1.0, 1.0, 1.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    glTranslatef(0.0, 0.0, -50.0)

    drawer.draw()

    pygame.display.flip()


def main() -> None:
    pygame.init()
    window_size: Tuple[int, int] = (640, 480)
    pygame.display.set_mode(window_size, pygame.DOUBLEBUF | pygame.OPENGL)

    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, (window_size[0] / window_size[1]), 0.1, 100.0)

    voxel_size: float = 1.0
    half_voxel_size: float = voxel_size / 2.0

    tree: OcTree = OcTree(voxel_size)

    origin: Vector3 = Vector3(half_voxel_size, half_voxel_size, half_voxel_size)
    offset: Vector3 = Vector3(voxel_size * 10, 0.0, 0.0)

    drawer: OcTreeDrawer = OcTreeDrawer()
    drawer.enable_freespace()

    origin_pose: Pose6D = Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    angles: np.ndarray = np.linspace(0.0, 2 * math.pi, 256, endpoint=False)
    i: int = 0

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit(0)

        if i < len(angles):
            angled_offset: Vector3 = offset.copy()
            angled_offset.rotate_ip(0, 0, angles[i])
            tree.insert_ray(origin, origin + angled_offset * (1 + i / (len(angles) - 1)))
            drawer.set_octree(tree, origin_pose)
            i += 1

        draw_frame(drawer)
        pygame.time.wait(2048 // len(angles))


if __name__ == "__main__":
    main()
