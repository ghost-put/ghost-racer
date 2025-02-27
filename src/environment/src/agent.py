#!/usr/bin/env python

from env.GazeboEnv import GazeboEnv
from train import train
import rospy

"""
    Remember to install missing dependencies with "rosdep install environment"
"""


def main():
    env = GazeboEnv()
    for _ in range(100):
        observation, reward, done, info = env.step(6)
    print(observation)


if __name__ == "__main__":
    try:
        # main()
        train()
    except rospy.ROSInterruptException:
        print("Agent has been closed.")
