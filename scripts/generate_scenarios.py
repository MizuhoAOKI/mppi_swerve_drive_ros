#! /usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
import time
import yaml
import rospkg
import subprocess
import argparse
import datetime
import numpy as np

# settings
INITIAL_X_POS = 0.0
INITIAL_Y_POS = 0.0
MIN_GOAL_DISTANCE =  6.0 # [m]
MAX_GOAL_DISTANCE = 16.0 # [m]
YAW_CHANGE_THRESHOLD = np.pi / 8.0 # [rad]

# check function
def is_goal_valid(goal_x: float, goal_y: float, prev_goal_x: float, prev_goal_y: float, preprev_goal_x: float, preprev_goal_y: float) -> bool:
    """
    Check whether the given goal position is valid based on spatial and directional constraints.

    A goal is considered valid if:
    - It is not too close from the previous goal (>= MIN_GOAL_DISTANCE).
    - It is not too far from the previous goal (<= MAX_GOAL_DISTANCE).
    - The direction change from two previous goals to this one is not too small (yaw change >= YAW_CHANGE_THRESHOLD).

    Args:
        goal_x (float): x-coordinate of the candidate goal.
        goal_y (float): y-coordinate of the candidate goal.
        prev_goal_x (float): x-coordinate of the previous goal.
        prev_goal_y (float): y-coordinate of the previous goal.
        preprev_goal_x (float): x-coordinate of the goal before the previous one.
        preprev_goal_y (float): y-coordinate of the goal before the previous one.

    Returns:
        bool: True if the goal is valid, False otherwise.
    """
    # reject if the goal is too close to the previous goal
    if np.sqrt((goal_x - prev_goal_x)**2 + (goal_y - prev_goal_y)**2) < MIN_GOAL_DISTANCE:
        return False

    # reject if the goal is too close to the previous goal
    if np.sqrt((goal_x - prev_goal_x)**2 + (goal_y - prev_goal_y)**2) > MAX_GOAL_DISTANCE:
        return False

    # check if the vehicle yaw direction changes larger than 90 degrees
    if not (preprev_goal_x == prev_goal_x and preprev_goal_y == prev_goal_y):

        # calculate yaw direction difference
        yaw_diff = np.fmod(np.abs(np.arctan2(goal_y - prev_goal_y, goal_x - prev_goal_x) - np.arctan2(prev_goal_y - preprev_goal_y, prev_goal_x - preprev_goal_x)), (2.0 * np.pi))
        yaw_diff = np.min([yaw_diff, 2.0*np.pi - yaw_diff])

        # reject if the yaw direction change is too small
        if yaw_diff < YAW_CHANGE_THRESHOLD:
            return False

    # all checks passed
    return True

# main function
def generate_scenarios(out_dir: str, num_goals: int, num_episodes: int) -> None:
    """
    Generate navigation evaluation scenarios and save them as YAML files.

    For each episode, a sequence of goal positions are generated on a fixed 2D grid.
    Each goal is validated to reject overly straight or monotonous trajectories.
    Valid goal sequences are saved as separate YAML files per episode.

    Args:
        out_dir (str): Directory to save the generated scenario YAML files.
        num_goals (int): Number of goal points to include in each episode.
        num_episodes (int): Number of scenario episodes to generate.

    Returns:
        None
    """

    # make output directory
    if not os.path.exists(out_dir):
        os.makedirs(out_dir)

    # list of x, y candidate points where there is sufficient space for the vehicle to stop
    x_grid_points = np.array([-7.5, -5.0, -2.5, 0.0, 2.5, 5.0, 7.5])
    y_grid_points = np.array([-7.5, -5.0, -2.5, 0.0, 2.5, 5.0, 7.5])

    # declare dictionary
    goals = np.zeros((num_goals, 2))

    # generate yaml files
    for e in range(num_episodes):

        # start new episode
        print("\n", "#"*10, "\n")

        # declare dictionary
        yaml_dict = {
            'goals': []
        }

        # add goal points
        for g in range(num_goals):

            # sample a valid goal point
            while True:

                # sample a goal point randomly
                goals[g, 0] = np.random.choice(x_grid_points)
                goals[g, 1] = np.random.choice(y_grid_points)

                # get previous goal points
                prev_goal_x    = goals[g-1, 0] if g > 0 else INITIAL_X_POS
                prev_goal_y    = goals[g-1, 1] if g > 0 else INITIAL_Y_POS
                preprev_goal_x = goals[g-2, 0] if g > 1 else INITIAL_X_POS
                preprev_goal_y = goals[g-2, 1] if g > 1 else INITIAL_Y_POS

                # check if the goal is valid, otherwise resample
                if is_goal_valid(goals[g, 0], goals[g, 1], prev_goal_x, prev_goal_y, preprev_goal_x, preprev_goal_y):
                    break

            # add goal point to the dictionary
            yaml_dict['goals'].append({
                'goal_x': float(goals[g, 0]),
                'goal_y': float(goals[g, 1]),
            })

            # print the goal point
            print(f"[Episode {e}], add goal point {g} : ({goals[g, 0]: 4.1f}, {goals[g, 1]: 4.1f})")

        # write yaml
        yaml_file = os.path.join(out_dir, f'episode_{e:04}.yaml')
        with open(yaml_file, 'w') as f:
            yaml.dump(yaml_dict, f)

    # announce the location of the generated yaml files
    print("\n", "#"*10, "\n")
    print(f"Generated {num_episodes} episodes of scenarios in {out_dir}")
    print("\n", "#"*10, "\n")


if __name__ == '__main__':
    """
    Script entry point: Parses command-line arguments and invokes scenario generation.

    Usage example:
        cd mppi_swerve_drive_ros/scripts
        python3 generate_scenarios.py --out_dir ../data/scenarios --num_goals 10 --num_episodes 100
    """
    # parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--out_dir', type=str, default='./scenarios', help='output directory of the generated scenarios')
    parser.add_argument('--num_goals', type=int, default=1, help='number of goals')
    parser.add_argument('--num_episodes', type=int, default=1, help='number of episodes')
    args = parser.parse_args()

    # generate scenarios
    generate_scenarios(
        out_dir = args.out_dir,
        num_goals = args.num_goals,
        num_episodes = args.num_episodes,
    )
