"""URDForge Forward Kinematics (FK) & Interactive Motion Engine.

Evaluates the robot's kinematic tree in real-time, computing SE(3) world transformations
for every link given active joint positions. Powers the 3D viewport FK debugging sliders.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np

from urdforge.core.math_types import Transform3D
from urdforge.core.robot_model import JointModel, JointType, RobotModel


@dataclass
class FKResult:
    """Result of forward kinematics evaluation."""

    link_poses_world: Dict[str, Transform3D]  # link_name -> Transform3D in world frame
    joint_poses_world: Dict[str, Transform3D]  # joint_name -> Transform3D of joint frame in world
    limit_violations: Dict[str, str] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "link_poses": {k: v.to_dict() for k, v in self.link_poses_world.items()},
            "joint_poses": {k: v.to_dict() for k, v in self.joint_poses_world.items()},
            "limit_violations": self.limit_violations,
        }


class ForwardKinematicsEngine:
    """Computes recursive forward kinematics on the robot model."""

    def __init__(self, robot: RobotModel) -> None:
        self.robot = robot

    def compute_joint_motion_transform(self, joint: JointModel, value: float) -> Transform3D:
        """Compute the relative transformation T_motion(q) across the joint."""
        if joint.joint_type == JointType.FIXED:
            return Transform3D.identity()

        ax = np.asarray(joint.axis, dtype=np.float64)
        ax = ax / np.linalg.norm(ax)

        if joint.joint_type in (JointType.REVOLUTE, JointType.CONTINUOUS):
            # Rotation around axis by angle 'value' (radians)
            theta = float(value)
            # Rodrigues rotation matrix
            vx = np.array(
                [
                    [0.0, -ax[2], ax[1]],
                    [ax[2], 0.0, -ax[0]],
                    [-ax[1], ax[0], 0.0],
                ],
                dtype=np.float64,
            )
            rot = np.eye(3, dtype=np.float64) + math.sin(theta) * vx + (1.0 - math.cos(theta)) * np.matmul(vx, vx)
            mat = np.eye(4, dtype=np.float64)
            mat[:3, :3] = rot
            return Transform3D(matrix=mat)

        elif joint.joint_type == JointType.PRISMATIC:
            # Translation along axis by displacement 'value' (meters)
            disp = ax * float(value)
            mat = np.eye(4, dtype=np.float64)
            mat[:3, 3] = disp
            return Transform3D(matrix=mat)

        return Transform3D.identity()

    def evaluate(self, joint_values: Optional[Dict[str, float]] = None) -> FKResult:
        """Evaluate forward kinematics for all links and joints.

        Args:
            joint_values: Dict mapping joint_name to position (radians or meters).
                          If omitted, uses joint.current_value.
        """
        vals = joint_values if joint_values is not None else {}
        link_poses: Dict[str, Transform3D] = {}
        joint_poses: Dict[str, Transform3D] = {}
        limit_violations: Dict[str, str] = {}

        # 1. Base link is at world origin
        root = self.robot.root_link
        link_poses[root] = Transform3D.identity()

        # 2. Build adjacency for breadth-first or depth-first recursion
        children_map: Dict[str, List[JointModel]] = {}
        for j in self.robot.joints.values():
            children_map.setdefault(j.parent_link, []).append(j)

        # 3. Recursive traversal
        visited_links: Set[str] = {root}
        queue: List[str] = [root]

        while queue:
            curr_link = queue.pop(0)
            curr_pose = link_poses[curr_link]

            for joint in children_map.get(curr_link, []):
                q = float(vals.get(joint.name, joint.current_value))

                # Check limits
                if joint.joint_type == JointType.REVOLUTE:
                    if q < joint.limits.lower - 1e-4 or q > joint.limits.upper + 1e-4:
                        limit_violations[joint.name] = (
                            f"Value {q:.3f} rad exceeds limits [{joint.limits.lower:.3f}, {joint.limits.upper:.3f}]"
                        )
                elif joint.joint_type == JointType.PRISMATIC:
                    if q < joint.limits.lower - 1e-7 or q > joint.limits.upper + 1e-7:
                        limit_violations[joint.name] = (
                            f"Value {q:.7g} m exceeds limits [{joint.limits.lower:.7g}, {joint.limits.upper:.7g}]"
                        )

                # Joint origin pose in world
                joint_frame_world = curr_pose @ joint.origin
                joint_poses[joint.name] = joint_frame_world

                # Motion transform
                t_motion = self.compute_joint_motion_transform(joint, q)

                # Child link pose in world: T_world_parent * T_origin * T_motion
                child_pose = joint_frame_world @ t_motion
                child_link = joint.child_link
                link_poses[child_link] = child_pose

                if child_link not in visited_links:
                    visited_links.add(child_link)
                    queue.append(child_link)

        return FKResult(
            link_poses_world=link_poses,
            joint_poses_world=joint_poses,
            limit_violations=limit_violations,
        )
