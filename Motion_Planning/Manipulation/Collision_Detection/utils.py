# !/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
# @FileName       : utils.py
# @Time           : 2024-10-12 15:52:13
# @Author         : yk
# @Email          : yangkui1127@gmail.com
# @Description:   : Adopted from https://github.com/StanfordVL/iGibson/blob/master/igibson/external/pybullet_tools/utils.py
"""

from collections import namedtuple
from itertools import combinations, product

import pybullet as p

BASE_LINK = -1
MAX_DISTANCE = 0


def get_moving_links(body, joints):
    """
    Gets all links that are affected by the specified joints.
    Args:
        body: The body ID.
        joints: List of joint indices.
    Returns:
        A list of link indices that are affected by the specified joints.
    """
    moving_links = set()
    for joint in joints:
        link = child_link_from_joint(joint)
        if link not in moving_links:
            moving_links.update(get_link_subtree(body, link))
    return list(moving_links)


def get_moving_pairs(body, moving_joints):
    """
    Gets pairs of moving links that may collide with each other.
    Args:
        body: The body ID.
        moving_joints: List of joint indices.
    Yields:
        Tuples of link indices representing pairs of links that may collide.
    """
    moving_links = get_moving_links(body, moving_joints)
    for link1, link2 in combinations(moving_links, 2):
        ancestors1 = set(get_joint_ancestors(body, link1)) & set(moving_joints)
        ancestors2 = set(get_joint_ancestors(body, link2)) & set(moving_joints)
        if ancestors1 != ancestors2:
            yield link1, link2


def get_arm_link_pairs(body, joints, disabled_collisions=set(), only_moving=True):
    """
    Gets pairs of links that may collide for self-collision detection.
    Args:
        body: The body ID.
        joints: List of joint indices of the body.
        disabled_collisions: Set of disabled link pairs.
        only_moving: Boolean indicating whether to only consider moving links.
    Returns:
        A list of link pairs that should be checked for collisions.
    """
    moving_links = get_moving_links(body, joints)
    fixed_links = list(set(get_joints(body)) - set(moving_links))
    check_link_pairs = list(product(moving_links, fixed_links))
    if only_moving:
        check_link_pairs.extend(get_moving_pairs(body, joints))
    else:
        check_link_pairs.extend(combinations(moving_links, 2))
    check_link_pairs = list(
        filter(lambda pair: not are_links_adjacent(body, *pair), check_link_pairs)
    )
    check_link_pairs = list(
        filter(
            lambda pair: (pair not in disabled_collisions)
            and (pair[::-1] not in disabled_collisions),
            check_link_pairs,
        )
    )
    return check_link_pairs


#####################################

# Named tuple for joint information
JointInfo = namedtuple(
    "JointInfo",
    [
        "jointIndex",
        "jointName",
        "jointType",
        "qIndex",
        "uIndex",
        "flags",
        "jointDamping",
        "jointFriction",
        "jointLowerLimit",
        "jointUpperLimit",
        "jointMaxForce",
        "jointMaxVelocity",
        "linkName",
        "jointAxis",
        "parentFramePos",
        "parentFrameOrn",
        "parentIndex",
    ],
)


def get_joint_info(body, joint):
    return JointInfo(*p.getJointInfo(body, joint))


def child_link_from_joint(joint):
    return joint  # link


def get_num_joints(body):
    return p.getNumJoints(body)


def get_joints(body):
    return list(range(get_num_joints(body)))


get_links = get_joints


def get_all_links(body):
    return [BASE_LINK] + list(get_joints(body))


def get_link_parent(body, link):
    if link == BASE_LINK:
        return None
    return get_joint_info(body, link).parentIndex


def get_all_link_parents(body):
    return {link: get_link_parent(body, link) for link in get_links(body)}


def get_all_link_children(body):
    children = {}
    for child, parent in get_all_link_parents(body).items():
        if parent not in children:
            children[parent] = []
        children[parent].append(child)
    return children


def get_link_children(body, link):
    children = get_all_link_children(body)
    return children.get(link, [])


def get_link_ancestors(body, link):
    parent = get_link_parent(body, link)
    if parent is None:
        return []
    return get_link_ancestors(body, parent) + [parent]


def get_joint_ancestors(body, joint):
    link = child_link_from_joint(joint)
    return get_link_ancestors(body, link) + [link]


def get_link_descendants(body, link, test=lambda l: True):
    descendants = []
    for child in get_link_children(body, link):
        if test(child):
            descendants.append(child)
            descendants.extend(get_link_descendants(body, child, test=test))
    return descendants


def get_link_subtree(body, link, **kwargs):
    return [link] + get_link_descendants(body, link, **kwargs)


def are_links_adjacent(body, link1, link2):
    return (get_link_parent(body, link1) == link2) or (
        get_link_parent(body, link2) == link1
    )
