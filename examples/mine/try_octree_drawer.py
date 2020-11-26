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

cubeVertices = ((1,1,1),(1,1,-1),(1,-1,-1),(1,-1,1),(-1,1,1),(-1,-1,-1),(-1,-1,1),(-1,1,-1))
cubeEdges = ((0,1),(0,3),(0,4),(1,2),(1,7),(2,5),(2,3),(3,6),(4,6),(4,7),(5,6),(5,7))
cubeQuads = ((0,3,6,4),(2,5,6,3),(1,2,5,7),(1,0,4,7),(7,4,6,5),(2,3,0,1))


def draw_frame(drawer: OcTreeDrawer) -> None:
    glClearColor(1.0, 1.0, 1.0, 1.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    glTranslatef(0.0, 0.0, -50.0)

    # glColor3f(1.0, 0.0, 0.0)
    # glBegin(GL_LINES)
    # glVertex3f(0.0, 0.0, 0.0)
    # glVertex3f(1000.0, 0.0, 0.0)
    # glEnd()

    # glBegin(GL_LINES)
    # for cubeEdge in cubeEdges:
    #     for cubeVertex in cubeEdge:
    #         glVertex3fv(cubeVertices[cubeVertex])
    # glEnd()

    drawer.draw()

    pygame.display.flip()


def main() -> None:
    pygame.init()
    window_size: Tuple[int, int] = (640, 480)
    pygame.display.set_mode(window_size, pygame.DOUBLEBUF | pygame.OPENGL)

    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, (window_size[0] / window_size[1]), 0.1, 50.0)

    ptr: np.ndarray = np.transpose(glGetFloatv(GL_PROJECTION_MATRIX))
    print(ptr)

    voxel_size: float = 1.0
    half_voxel_size: float = voxel_size / 2.0

    tree: OcTree = OcTree(voxel_size)

    origin: Vector3 = Vector3(half_voxel_size, half_voxel_size, half_voxel_size)
    offset: Vector3 = Vector3(voxel_size * 10, 0.0, 0.0)

    for angle in np.linspace(0.0, 2 * math.pi, 8, endpoint=False):
        angled_offset: Vector3 = offset.copy()
        angled_offset.rotate_ip(0, 0, angle)
        tree.insert_ray(origin, origin + angled_offset)

    origin: Pose6D = Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    drawer: OcTreeDrawer = OcTreeDrawer()
    drawer.set_octree(tree, origin)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit(0)

        draw_frame(drawer)
        pygame.time.wait(10)


if __name__ == "__main__":
    main()
