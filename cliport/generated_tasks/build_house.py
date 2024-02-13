import numpy as np
import os
import pybullet as p
import random
from cliport.tasks import primitives
from cliport.tasks.grippers import Spatula
from cliport.tasks.task import Task
from cliport.utils import utils
import numpy as np
from cliport.tasks.task import Task
from cliport.utils import utils


class BuildHouse(Task):
    """Construct a simple house structure by stacking modifiable blocks."""

    def __init__(self):
        super().__init__()
        self.max_steps = 10
        self.lang_template = "build a house with {base} as the base, {walls} as the walls, {roof} as the roof, and {chimney} as the chimney"
        self.task_completed_desc = "done building the house."
        self.additional_reset()

    def reset(self, env):
        super().reset(env)

        # Define the colors and sizes for each part of the house
        base_color = utils.COLORS['green']
        wall_color = utils.COLORS['red']
        roof_color = utils.COLORS['blue']
        chimney_color = utils.COLORS['yellow']

        base_size = (0.15, 0.05, 0.02)  # Large green blocks
        wall_size = (0.1, 0.05, 0.02)   # Medium red blocks
        roof_size = (0.15, 0.1, 0.02)   # Large blue block
        chimney_size = (0.05, 0.05, 0.05)  # Small yellow block

        # Add base blocks
        base_blocks = []
        for _ in range(3):
            pose = self.get_random_pose(env, base_size)
            urdf = 'stacking/block.urdf'
            base_id = env.add_object(urdf, pose, color=base_color)
            base_blocks.append(base_id)

        # Add wall blocks
        wall_blocks = []
        for _ in range(2):
            pose = self.get_random_pose(env, wall_size)
            urdf = 'stacking/block.urdf'
            wall_id = env.add_object(urdf, pose, color=wall_color)
            wall_blocks.append(wall_id)

        # Add roof block
        roof_pose = self.get_random_pose(env, roof_size)
        roof_urdf = 'stacking/block.urdf'
        roof_id = env.add_object(roof_urdf, roof_pose, color=roof_color)

        # Add chimney block
        chimney_pose = self.get_random_pose(env, chimney_size)
        chimney_urdf = 'stacking/block.urdf'
        chimney_id = env.add_object(chimney_urdf, chimney_pose, color=chimney_color)

        # Define target poses for each part of the house
        base_target_poses = [self.get_random_pose(env, base_size) for _ in base_blocks]
        wall_target_poses = [self.get_random_pose(env, wall_size) for _ in wall_blocks]
        roof_target_pose = self.get_random_pose(env, roof_size)
        chimney_target_pose = self.get_random_pose(env, chimney_size)

        # Add goals for each part of the house
        # Base layer goal
        self.add_goal(objs=base_blocks, matches=np.ones((3, 3)), targ_poses=base_target_poses, replace=False,
                      rotations=True, metric='pose', params=None, step_max_reward=1 / 4,
                      language_goal=self.lang_template.format(base="three large green blocks", walls="", roof="", chimney=""))

        # Wall layer goal
        self.add_goal(objs=wall_blocks, matches=np.ones((2, 2)), targ_poses=wall_target_poses, replace=False,
                      rotations=True, metric='pose', params=None, step_max_reward=1 / 4,
                      language_goal=self.lang_template.format(base="", walls="two medium red blocks on top of the green blocks", roof="", chimney=""))

        # Roof layer goal
        self.add_goal(objs=[roof_id], matches=np.ones((1, 1)), targ_poses=[roof_target_pose], replace=False,
                      rotations=True, metric='pose', params=None, step_max_reward=1 / 4,
                      language_goal=self.lang_template.format(base="", walls="", roof="a large blue block spanning the red blocks as the roof", chimney=""))

        # Chimney goal
        self.add_goal(objs=[chimney_id], matches=np.ones((1, 1)), targ_poses=[chimney_target_pose], replace=False,
                      rotations=True, metric='pose', params=None, step_max_reward=1 / 4,
                      language_goal=self.lang_template.format(base="", walls="", roof="", chimney="a small yellow block on top as a chimney"))