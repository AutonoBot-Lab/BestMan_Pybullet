# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : utils.py
# @Time           : 2024-10-12 15:52:13
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : Adopted from https://github.com/StanfordVL/iGibson/blob/master/igibson/external/pybullet_tools/utils.py
"""


from itertools import product

import fcl
import pybullet as p
import trimesh

from ..utils import *


def create_fcl_bvh_from_bullet(robot_id, link_index):
    collision_shape_data = p.getCollisionShapeData(robot_id, link_index)[0]
    mesh_file_path = collision_shape_data[4].decode("utf-8")
    mesh = trimesh.load(mesh_file_path)
    bvh_model = fcl.BVHModel()
    vertices = mesh.vertices
    faces = mesh.faces
    bvh_model.beginModel(len(vertices), len(faces))
    bvh_model.addSubModel(vertices, faces)
    bvh_model.endModel()
    return fcl.CollisionObject(bvh_model)


def check_body_collision(body1, body2, links1=None, links2=None):
    """
    Checks collision between any link pair from two different bodies.
    Args:
        body1: ID of the first body.
        links1: List of link indices for the first body.
        body2: ID of the second body.
        links2: List of link indices for the second body (default is all links).
    Returns:
        True if any link from body1 collides with any link from body2, otherwise False.
    """
    if links1 is None:
        links1 = get_all_links(body1)
    if links2 is None:
        links2 = get_all_links(body2)
    return any(
        check_link_collision(body1, body2, link1, link2)
        for link1, link2 in product(links1, links2)
        if not (body1 == body2 and link1 == link2)
    )
