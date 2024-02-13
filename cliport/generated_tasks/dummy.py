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

class DummyTask(Task):
    """
    This is a dummy task that demonstrates the creation of a simple task
    where the robot needs to place a colored block into a designated zone.
    """

    def __init__(self):
        super().__init__()
        self.max_steps = 5
        self.lang_template = "place the {color} block into the designated zone"
        self.task_completed_desc = "done placing the block."
        self.additional_reset()

    def reset(self, env):
        super().reset(env)

        # Define the color and size of the block.
        block_color_name = 'red'  # The color of the block to be placed.
        block_color = utils.COLORS[block_color_name]  # The RGB values for the color.
        block_size = (0.05, 0.05, 0.05)  # The size of the block (x, y, z dimensions).

        # Define the size and pose of the designated zone.
        zone_size = (0.15, 0.15, 0)  # The size of the zone (x, y, z dimensions).
        zone_pose = self.get_random_pose(env, zone_size)  # Random pose for the zone.

        # Add the designated zone to the environment.
        env.add_object('zone/zone.urdf', zone_pose, 'fixed')

        # Add the block to the environment.
        block_urdf = 'stacking/block.urdf'
        block_pose = self.get_random_pose(env, block_size)
        block_id = env.add_object(block_urdf, block_pose, color=block_color)

        # Define the language goal for the task.
        language_goal = self.lang_template.format(color=block_color_name)

        # Add the goal for the task.
        # The goal is to place the block into the zone, so we set up a match matrix
        # that indicates a single block (1) needs to match with a single zone (1).
        self.add_goal(objs=[block_id], matches=np.ones((1, 1)), targ_poses=[zone_pose], replace=False,
                      rotations=True, metric='zone', params=[(zone_pose, zone_size)], step_max_reward=1,
                      language_goal=language_goal)

        # The number of language goals matches the number of motion goals.
        # In this case, there is one motion goal (placing the block into the zone)
        # and one corresponding language goal (the instruction to place the block).