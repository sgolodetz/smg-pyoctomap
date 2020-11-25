# This is a Python reimplementation of test_raycasting from the Octomap code.

import math
import numpy as np
import sys
import types

from inspect import currentframe, getframeinfo, Traceback

from smg.pyoctomap import *


def expect_eq(a, b, frame: types.FrameType) -> None:
    if a != b:
        frameinfo: Traceback = getframeinfo(frame)
        print(f"test failed: {a}!={b} in {frameinfo.filename}, line {frameinfo.lineno}")
        sys.exit(1)


def expect_false(args, frame: types.FrameType) -> None:
    if args:
        frameinfo: Traceback = getframeinfo(frame)
        print(f"test failed (EXPECT_FALSE) in {frameinfo.filename}, line {frameinfo.lineno}")
        sys.exit(1)


def expect_float_eq(a, b, frame: types.FrameType) -> None:
    if not math.fabs(a - b) <= 1e-5:
        frameinfo: Traceback = getframeinfo(frame)
        print(f"test failed: {a} != {b} in {frameinfo.filename}, line {frameinfo.lineno}")
        sys.exit(1)


def expect_near(a, b, prec, frame: types.FrameType) -> None:
    if not math.fabs(a - b) <= prec:
        frameinfo: Traceback = getframeinfo(frame)
        print(f"test failed: |{a} - {b}| > {prec} in {frameinfo.filename}, line {frameinfo.lineno}")
        sys.exit(1)


def expect_true(args, frame: types.FrameType) -> None:
    if not args:
        frameinfo: Traceback = getframeinfo(frame)
        print(f"test failed (EXPECT_TRUE) in {frameinfo.filename}, line {frameinfo.lineno}")
        sys.exit(1)


def main() -> None:
    tree: OcTree = OcTree(0.05)

    origin: Vector3 = Vector3(0.01, 0.01, 0.02)
    point_on_surface: Vector3 = Vector3(2.01, 0.01, 0.01)

    print(f"Generating sphere at {origin}...")

    sphere_beams: int = 500
    angle: float = 2.0 * math.pi / sphere_beams
    p: Pointcloud = Pointcloud()
    for i in range(sphere_beams):
        for j in range(sphere_beams):
            p.push_back(origin + point_on_surface)
            point_on_surface.rotate_ip(0, 0, angle)
        point_on_surface.rotate_ip(0, angle, 0)
    tree.insert_point_cloud(p, origin)

    print("Writing to sphere.bt...")
    expect_true(tree.write_binary("sphere.bt"), currentframe())

    ###

    print("Casting rays in sphere ...")

    sampled_surface: OcTree = OcTree(0.05)

    direction: Vector3 = Vector3(1.0, 0.0, 0.0)
    obstacle: Vector3 = Vector3(0.0, 0.0, 0.0)

    hit: int = 0
    miss: int = 0
    unknown: int = 0
    mean_dist: float = 0.0

    for i in range(sphere_beams):
        for j in range(sphere_beams):
            if not tree.cast_ray(origin, direction, obstacle, False, 3.0):
                # hit unknown
                if not tree.search(obstacle):
                    unknown += 1
                else:
                    miss += 1
            else:
                hit += 1
                mean_dist += (obstacle - origin).norm()
                sampled_surface.update_node(obstacle, True)

            direction.rotate_ip(0, 0, angle)

        direction.rotate_ip(0, angle, 0)

    print("Writing sampled_surface.bt")
    expect_true(sampled_surface.write_binary("sampled_surface.bt"), currentframe())

    mean_dist /= float(hit)
    print(f" hits / misses / unknown: {hit} / {miss} / {unknown}")
    print(f" mean obstacle dist: {mean_dist}")
    expect_near(mean_dist, 2.0, 0.05, currentframe())
    expect_eq(hit, sphere_beams * sphere_beams, currentframe())
    expect_eq(miss, 0, currentframe())
    expect_eq(unknown, 0, currentframe())

    ###

    print("generating single rays...")
    single_beams: OcTree = OcTree(0.03333)
    num_beams: int = 17
    beam_length: float = 10.0
    single_origin: Vector3 = Vector3(1.0, 0.45, 0.45)
    # single_origin_top: Vector3 = Vector3(1.0, 0.45, 1.0)
    single_endpoint: Vector3 = Vector3(beam_length, 0.0, 0.0)

    for i in range(num_beams):
        for j in range(num_beams):
            if not single_beams.insert_ray(single_origin, single_origin + single_endpoint):
                print(f"ERROR while inserting ray from {single_origin} to {single_endpoint}")
            single_endpoint.rotate_ip(0, 0, np.deg2rad(360.0 / num_beams))
        single_endpoint.rotate_ip(0, np.deg2rad(360.0 / num_beams), 0)

    print("done.")
    print("writing to beams.bt...")
    expect_true(single_beams.write_binary("beams.bt"), currentframe())

    # more tests from unit_tests.cpp:
    res: float = 0.1
    res_2: float = res / 2.0
    cube_tree: OcTree = OcTree(res)
    # fill a cube with "free", end is "occupied":
    for x in np.linspace(-0.95, 0.95, 20):
        for y in np.linspace(-0.95, 0.95, 20):
            for z in np.linspace(-0.95, 0.95, 20):
                if x < 0.9:
                    expect_true(cube_tree.update_node(Vector3(x, y, z), False), currentframe())
                else:
                    expect_true(cube_tree.update_node(Vector3(x, y, z), True), currentframe())

    # fill some "floor":
    expect_true(cube_tree.update_node(Vector3(res_2, res_2, -res_2), True), currentframe())
    expect_true(cube_tree.update_node(Vector3(3 * res_2, res_2, -res_2), True), currentframe())
    expect_true(cube_tree.update_node(Vector3(-res_2, res_2, -res_2), True), currentframe())
    expect_true(cube_tree.update_node(Vector3(-3 * res_2, res_2, -res_2), True), currentframe())

    cube_tree.write_binary("raycasting_cube.bt")
    origin = Vector3(0.0, 0.0, 0.0)
    end: Vector3 = Vector3()
    # hit the corner:
    direction: Vector3 = Vector3(0.95, 0.95, 0.95)
    expect_true(cube_tree.cast_ray(origin, direction, end, False), currentframe())
    expect_true(cube_tree.is_node_occupied(cube_tree.search(end)), currentframe())
    print(f"Hit occupied voxel: {end}")
    direction = Vector3(1.0, 0.0, 0.0)
    expect_true(cube_tree.cast_ray(origin, direction, end, False), currentframe())
    expect_true(cube_tree.is_node_occupied(cube_tree.search(end)), currentframe())
    print(f"Hit occupied voxel: {end}")
    expect_near(1.0, (origin - end).norm(), res_2, currentframe())

    # hit bottom:
    origin = Vector3(res_2, res_2, 0.5)
    direction = Vector3(0.0, 0.0, -1.0)
    expect_true(cube_tree.cast_ray(origin, direction, end, False), currentframe())
    expect_true(cube_tree.is_node_occupied(cube_tree.search(end)), currentframe())
    print(f"Hit voxel: {end}")
    expect_float_eq(origin.x, end.x, currentframe())
    expect_float_eq(origin.y, end.y, currentframe())
    expect_float_eq(-res_2, end.z, currentframe())

    # hit boundary of unknown:
    origin = Vector3(0.0, 0.0, 0.0)
    direction = Vector3(0.0, 1.0, 0.0)
    expect_false(cube_tree.cast_ray(origin, direction, end, False), currentframe())
    expect_false(cube_tree.search(end), currentframe())
    print(f"Boundary unknown hit: {end}")

    # hit boundary of octree:
    expect_false(cube_tree.cast_ray(origin, direction, end, True), currentframe())
    expect_false(cube_tree.search(end), currentframe())
    expect_float_eq(end.x, res_2, currentframe())
    expect_float_eq(end.y, 32768 * res - res_2, currentframe())
    expect_float_eq(end.z, res_2, currentframe())
    direction = Vector3(-1.0, 0.0, 0.0)
    expect_false(cube_tree.cast_ray(origin, direction, end, True), currentframe())
    expect_false(cube_tree.search(end), currentframe())
    expect_float_eq(end.x, -32767 * res - res_2, currentframe())
    expect_float_eq(end.y, res_2, currentframe())
    expect_float_eq(end.z, res_2, currentframe())

    # test maxrange:
    expect_false(cube_tree.cast_ray(origin, direction, end, True, 0.9), currentframe())
    print(f"Max range endpoint: {end}")
    end_pt: OcTreeNode = cube_tree.search(end)
    expect_true(end_pt, currentframe())
    expect_false(cube_tree.is_node_occupied(end_pt), currentframe())
    dist: float = (origin - end).norm()
    expect_near(0.9, dist, res, currentframe())

    print("Test successful")


if __name__ == "__main__":
    main()
