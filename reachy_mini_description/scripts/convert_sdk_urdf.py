#!/usr/bin/env python3
# Copyright 2026 SocialMinds Lab, IIIA-CSIC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Generate urdf/reachy_mini.urdf and meshes/ from the Pollen SDK's robot model.

The SDK's URDF (reachy_mini/descriptions/reachy_mini/urdf/robot.urdf, exported
from Onshape) models the Stewart platform as a tree that reaches the head
through one leg and closes the other five loops with dummy frames. That is what
a kinematics solver wants, not what robot_state_publisher can use: the passive
joint angles are never measured. This script rewrites it as:

  base_link (new root)
    body_foot_3dprint ... body_yaw ... body_down_3dprint  (revolute, from joint_states)
       stewart_1..6 -> horns                              (revolute, from joint_states)
    head                                                  (TF from the driver's FK)
       xl_330 (head platform: shells, camera, antennas)   (fixed / revolute from joint_states)
    rod_1..6                                              (TF from the driver)

These "floating" links are attached with `continuous` joints (head_pose,
rod_i_pose) that never appear in joint_states: robot_state_publisher only
publishes a moving joint once it has received its value, so it leaves them
alone, and the driver broadcasts them on /tf (base_link -> head from forward
kinematics, base_link -> rod_i from the horn tip and the platform anchor).
A real `floating` joint would not work: kdl_parser turns it into a fixed joint
and robot_state_publisher would publish a static identity transform.
Rod frames: origin at the horn tip, x axis towards the platform anchor, mesh
centred at x = rod_length / 2.

Links and meshes keep their SDK names, but only the visible parts are kept
(HIDDEN_MESHES) and the head is recoloured (COLORS). RViz (at least up to
rviz_default_plugins 15.2) paints every visual of a link with the material of
its first visual, so each link keeps one colour and the visuals of the other
colours move to fixed children `<link>_color<n>` (identity transform).

Usage:  convert_sdk_urdf.py <sdk urdf dir> <package dir>
"""

import os
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation as R

ROD_LENGTH = 0.085
PACKAGE = 'reachy_mini_description'

# Visuals dropped by mesh-file prefix: parts hidden inside the shells (the servos'
# cases, horns, screws, connectors and LED caps, the antenna holders, the camera
# board and its bracket) or only seen through the eye lenses, which are made
# opaque below. Checked by rendering the model from many viewpoints: they are
# never or barely visible (the body yaw servo shows a little from right above).
HIDDEN_MESHES = ('dc15_a01_', 'phs_1_7x20_5_dc10', 'bts2_m2_6x8', 'b3b_eh',
                 'antenna_holder_', 'antenna_interface_3dprint', 'arducam',
                 'pp01102_arducam_carter', 'm12_fisheye_lens')
# Colour overrides by mesh-file prefix: white head, black eyes
COLORS = {
    'neck_reference_3dprint': '1 1 1 1',
    'glasses_dolder_3dprint': '0.05 0.05 0.05 1',
    'lens_cap_d': '0.05 0.05 0.05 1',
    'big_lens_d40': '0.05 0.05 0.05 1',
    'small_lens_d30': '0.05 0.05 0.05 1',
}


def origin_to_matrix(o):
    """URDF <origin xyz rpy> -> 4x4."""
    xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
    rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
    m = np.eye(4)
    m[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
    m[:3, 3] = xyz
    return m


def matrix_to_origin(m):
    """4x4 -> <origin> element."""
    o = ET.Element('origin')
    o.set('xyz', ' '.join(f'{v:.6g}' for v in m[:3, 3]))
    o.set('rpy', ' '.join(f'{v:.6g}' for v in R.from_matrix(m[:3, :3]).as_euler('xyz')))
    return o


def main(sdk_urdf_dir, package_dir):
    """Convert."""
    root = ET.parse(os.path.join(sdk_urdf_dir, 'robot.urdf')).getroot()
    root.set('name', 'reachy_mini')
    links = {link.get('name'): link for link in root.findall('link')}
    joints = {j.get('name'): j for j in root.findall('joint')}

    # 1. drop the passive chains, the rods and the loop-closing frames
    drop_links = [n for n in links if n.startswith(('passive_', 'stewart_link_rod', 'closing_'))]
    drop_links.append('dummy_torso_yaw')
    for n in drop_links:
        root.remove(links.pop(n))
    for n, j in list(joints.items()):
        if j.find('parent').get('link') in drop_links or j.find('child').get('link') in drop_links:
            root.remove(joints.pop(n))

    # 2. the head: `head` was a fixed child of the platform link xl_330; make it the
    #    floating root of the head subtree and re-attach xl_330 under it (inverse origin)
    head_joint = joints.pop('head_frame')
    root.remove(head_joint)
    t_xl_head = origin_to_matrix(head_joint.find('origin'))
    j = ET.SubElement(root, 'joint', name='head_pose', type='continuous')
    ET.SubElement(j, 'parent', link='base_link')
    ET.SubElement(j, 'child', link='head')
    ET.SubElement(j, 'axis', xyz='0 0 1')
    j = ET.SubElement(root, 'joint', name='head_platform', type='fixed')
    ET.SubElement(j, 'parent', link='head')
    ET.SubElement(j, 'child', link='xl_330')
    j.append(matrix_to_origin(np.linalg.inv(t_xl_head)))

    # 3. new root base_link, identity to the SDK root; body yaw joint renamed to match joint_states
    base = ET.Element('link', name='base_link')
    root.insert(0, base)
    j = ET.SubElement(root, 'joint', name='base_to_foot', type='fixed')
    ET.SubElement(j, 'parent', link='base_link')
    ET.SubElement(j, 'child', link='body_foot_3dprint')
    joints['yaw_body'].set('name', 'body_yaw')

    # 4. rods: six identical floating links, mesh centred along +x
    for i in range(1, 7):
        link = ET.SubElement(root, 'link', name=f'rod_{i}')
        v = ET.SubElement(link, 'visual')
        ET.SubElement(v, 'origin', xyz=f'{ROD_LENGTH / 2:.4f} 0 0', rpy='0 0 0')
        g = ET.SubElement(v, 'geometry')
        ET.SubElement(g, 'mesh', filename=f'package://{PACKAGE}/meshes/stewart_link_rod.stl')
        mat = ET.SubElement(v, 'material', name='rod_grey')
        ET.SubElement(mat, 'color', rgba='0.3 0.3 0.3 1')
        j = ET.SubElement(root, 'joint', name=f'rod_{i}_pose', type='continuous')
        ET.SubElement(j, 'parent', link='base_link')
        ET.SubElement(j, 'child', link=f'rod_{i}')
        ET.SubElement(j, 'axis', xyz='0 0 1')

    # 5. drop the collision geometry (not needed for visualisation, doubles the size)
    #    and the hidden visuals, recolour the head
    for link in root.findall('link'):
        for c in link.findall('collision'):
            link.remove(c)
        for v in link.findall('visual'):
            mesh = v.find('geometry/mesh')
            name = os.path.basename(mesh.get('filename')) if mesh is not None else ''
            if name.startswith(HIDDEN_MESHES):
                link.remove(v)
                continue
            for prefix, rgba in COLORS.items():
                if name.startswith(prefix):
                    v.find('material/color').set('rgba', rgba)

    # 6. one colour per link: RViz uses the first visual's material for the whole link
    for link in root.findall('link'):
        by_color = {}
        for v in link.findall('visual'):
            by_color.setdefault(v.find('material/color').get('rgba'), []).append(v)
        for n, visuals in enumerate(list(by_color.values())[1:], start=1):
            name = f'{link.get("name")}_color{n}'
            child = ET.SubElement(root, 'link', name=name)
            for v in visuals:
                link.remove(v)
                child.append(v)
            j = ET.SubElement(root, 'joint', name=f'{name}_joint', type='fixed')
            ET.SubElement(j, 'parent', link=link.get('name'))
            ET.SubElement(j, 'child', link=name)

    # 7. meshes: only the referenced ones, with package:// paths
    os.makedirs(os.path.join(package_dir, 'meshes'), exist_ok=True)
    copied = set()
    for m in root.iter('mesh'):
        name = os.path.basename(m.get('filename'))
        m.set('filename', f'package://{PACKAGE}/meshes/{name}')
        if name not in copied:
            shutil.copy(os.path.join(sdk_urdf_dir, 'assets', name),
                        os.path.join(package_dir, 'meshes', name))
            copied.add(name)
    for name in os.listdir(os.path.join(package_dir, 'meshes')):
        if name.endswith('.stl') and name not in copied:
            os.remove(os.path.join(package_dir, 'meshes', name))

    ET.indent(root, space='  ')
    out = os.path.join(package_dir, 'urdf', 'reachy_mini.urdf')
    with open(out, 'w', encoding='utf-8') as f:
        f.write('<?xml version="1.0"?>\n')
        f.write('<!-- Generated by scripts/convert_sdk_urdf.py from the Pollen Robotics SDK model '
                '(Apache-2.0). Do not edit by hand. -->\n')
        f.write(ET.tostring(root, encoding='unicode'))
    print(f'{out}: {len(root.findall("link"))} links, {len(root.findall("joint"))} joints, '
          f'{len(copied)} meshes')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
