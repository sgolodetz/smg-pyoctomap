import math
import numpy as np
import os
import sys

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

from OpenGL.GL import *
from OpenGL.GLU import *
from time import perf_counter as timer
from typing import Tuple

from smg.pyoctomap import *
from smg.rigging import SimpleCamera


def draw_frame(camera: SimpleCamera, drawer: OcTreeDrawer) -> None:
    glClearColor(1.0, 1.0, 1.0, 1.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    # gluLookAt(0.0, -height * 1.5, height, 0.0, 0.0, height / 2, 0.0, 0.0, 1.0)
    gluLookAt(*camera.p(), *(camera.p() + camera.n()), *camera.v())

    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

    glEnable(GL_LIGHTING)
    glEnable(GL_LIGHT0)
    pos: np.ndarray = np.array([-1.0, 1.0, 1.0, 0.0])
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

    glMatrixMode(GL_PROJECTION)
    gluPerspective(45, (window_size[0] / window_size[1]), 0.1, 1000.0)

    voxel_size: float = 1.0
    half_voxel_size: float = voxel_size / 2.0

    tree: OcTree = OcTree(voxel_size)

    origin: Vector3 = Vector3(half_voxel_size, half_voxel_size, half_voxel_size)
    offset: Vector3 = Vector3(voxel_size * 20, 0.0, 0.0)

    drawer: OcTreeDrawer = OcTreeDrawer()
    drawer.enable_freespace()
    drawer.set_color_mode(CM_COLOR_HEIGHT)

    origin_pose: Pose6D = Pose6D(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    angles: np.ndarray = np.linspace(0.0, 10 * math.pi, 1024, endpoint=False)
    height: float = 100.0
    i: int = 0

    camera: SimpleCamera = SimpleCamera([0, -height * 1.5, height / 2], [0, 1, 0], [0, 0, 1])

    prev_time_ms: float = timer() * 1000

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                tree.write_binary("octree_drawer.bt")
                pygame.quit()
                sys.exit(0)

        cur_time_ms: float = timer() * 1000
        canonical_frame_time_ms: int = 16
        scaling_factor: float = (cur_time_ms - prev_time_ms) / canonical_frame_time_ms
        prev_time_ms = cur_time_ms

        pressed_keys = pygame.key.get_pressed()
        speed: float = 1.0 * scaling_factor
        angular_speed: float = 0.03 * scaling_factor
        up: np.ndarray = np.array([0, 0, 1])
        if pressed_keys[pygame.K_w]:
            camera.move_n(speed)
        if pressed_keys[pygame.K_s]:
            camera.move_n(-speed)
        if pressed_keys[pygame.K_d]:
            camera.move_u(-speed)
        if pressed_keys[pygame.K_a]:
            camera.move_u(speed)
        if pressed_keys[pygame.K_q]:
            camera.move(up, speed)
        if pressed_keys[pygame.K_e]:
            camera.move(up, -speed)
        if pressed_keys[pygame.K_RIGHT]:
            camera.rotate(up, -angular_speed)
        if pressed_keys[pygame.K_LEFT]:
            camera.rotate(up, angular_speed)
        if pressed_keys[pygame.K_UP]:
            camera.rotate(camera.u(), angular_speed)
        if pressed_keys[pygame.K_DOWN]:
            camera.rotate(camera.u(), -angular_speed)

        if i < len(angles):
            horizontal_offset: Vector3 = offset.copy()
            horizontal_offset.rotate_ip(0, 0, angles[i])
            vertical_offset: Vector3 = Vector3(0, 0, i * height / len(angles))
            tree.insert_ray(origin, origin + horizontal_offset + vertical_offset)
            tree.insert_ray(origin, origin + horizontal_offset * 0.5 + vertical_offset)
            drawer.set_octree(tree, origin_pose)
            i += 1

        draw_frame(camera, drawer)
        pygame.time.wait(1)


if __name__ == "__main__":
    main()
