import cv2
import numpy as np
import os

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
import pygame

from OpenGL.GL import *
from timeit import default_timer as timer
from typing import Optional, Tuple

from smg.opengl import OpenGLUtil
from smg.openni import OpenNICamera
from smg.pyoctomap import *
from smg.pyorbslam2 import RGBDTracker
from smg.rigging.cameras import SimpleCamera
from smg.rigging.controllers import KeyboardCameraController


def main() -> None:
    # Initialise PyGame and create the window.
    pygame.init()
    window_size: Tuple[int, int] = (640, 480)
    pygame.display.set_mode(window_size, pygame.DOUBLEBUF | pygame.OPENGL)

    # Set the projection matrix.
    glMatrixMode(GL_PROJECTION)
    intrinsics: Tuple[float, float, float, float] = (532.5694641250893, 531.5410880910171, 320.0, 240.0)
    OpenGLUtil.set_projection_matrix(intrinsics, *window_size)

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

    with OpenNICamera(mirror_images=True) as camera:
        with RGBDTracker(
            settings_file=f"settings-kinect.yaml", use_viewer=False,
            voc_file="C:/orbslam2/Vocabulary/ORBvoc.txt", wait_till_ready=False
        ) as tracker:
            while True:
                # Process any PyGame events.
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        tree.write_binary("openni_fusion.bt")
                        pygame.quit()
                        # noinspection PyProtectedMember
                        os._exit(0)

                # Get an RGB-D image from the camera.
                colour_image, depth_image = camera.get_images()
                cv2.imshow("Image", colour_image)
                c: int = cv2.waitKey(1)
                if c == ord('q'):
                    break

                # Try to estimate the camera pose. If the tracker's not ready, or pose estimation fails, continue.
                if not tracker.is_ready():
                    continue
                pose: Optional[np.ndarray] = tracker.estimate_pose(colour_image, depth_image)
                if pose is None:
                    continue

                pose = np.linalg.inv(pose)

                # Use the depth and the pose to make an Octomap point cloud.
                pcd: Pointcloud = OctomapUtil.make_point_cloud(depth_image, pose, intrinsics)

                # Fuse the point cloud into the octree.
                start = timer()
                origin: Vector3 = Vector3(0.0, 0.0, 0.0)
                tree.insert_point_cloud(pcd, origin, discretize=True)
                end = timer()
                print(f"  - Time: {end - start}s")

                # Allow the user to control the camera.
                camera_controller.update(pygame.key.get_pressed(), timer() * 1000)

                # Clear the colour and depth buffers.
                glClearColor(1.0, 1.0, 1.0, 1.0)
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

                # Draw the octree.
                OctomapUtil.draw_octree(tree, np.linalg.inv(pose), drawer)
                # OctomapUtil.draw_octree(tree, camera_controller.get_pose(), drawer)

                # Swap the front and back buffers.
                pygame.display.flip()


if __name__ == "__main__":
    main()
