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


def draw_frame(drawer: OcTreeDrawer, height: float) -> None:
    glClearColor(1.0, 1.0, 1.0, 1.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    gluLookAt(0.0, -height * 2, height / 2, 0.0, 0.0, height / 2, 0.0, 0.0, 1.0)

    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    pos: np.ndarray = np.array([-1.0, 1.0, 1.0, 0.0])
    glLightfv(GL_LIGHT0, GL_POSITION, pos)

    glEnable(GL_COLOR_MATERIAL)

    drawer.draw()

    glDisable(GL_LIGHTING)
    glDisable(GL_BLEND)

    pygame.display.flip()


def main() -> None:
    pygame.init()
    window_size: Tuple[int, int] = (640, 480)
    pygame.display.set_mode(window_size, pygame.DOUBLEBUF | pygame.OPENGL)

    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, (window_size[0] / window_size[1]), 0.1, 1000.0)

    voxel_size: float = 1.0
    half_voxel_size: float = voxel_size / 2.0

    tree: OcTree = OcTree(voxel_size)

    origin: Vector3 = Vector3(half_voxel_size, half_voxel_size, half_voxel_size)
    offset: Vector3 = Vector3(voxel_size * 20, 0.0, 0.0)

    drawer: OcTreeDrawer = OcTreeDrawer()
    drawer.enable_freespace()

    origin_pose: Pose6D = Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    angles: np.ndarray = np.linspace(0.0, 10 * math.pi, 1024, endpoint=False)
    height: float = 100.0
    i: int = 0

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit(0)

        if i < len(angles):
            horizontal_offset: Vector3 = offset.copy()
            horizontal_offset.rotate_ip(0, 0, angles[i])
            vertical_offset: Vector3 = Vector3(0, 0, i * height / len(angles))
            tree.insert_ray(origin, origin + horizontal_offset + vertical_offset)
            tree.insert_ray(origin, origin + horizontal_offset * 0.5 + vertical_offset)
            drawer.set_octree(tree, origin_pose)
            i += 1
        else:
            pygame.time.wait(2048)
            break

        draw_frame(drawer, height)
        pygame.time.wait(1)

    tree.write_binary("octree_drawer.bt")


if __name__ == "__main__":
    main()
