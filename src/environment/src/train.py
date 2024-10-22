"""
Code taken from: https://github.com/keras-rl/keras-rl/blob/master/examples/dqn_cartpole.py (with some modifications)
This code should only be used as an example. The agent doesn't learn anything meaningfull.
"""

from env.GazeboEnv import GazeboEnv

import numpy as np
from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.optimizers import Adam

from rl.agents.dqn import DQNAgent
from rl.policy import BoltzmannQPolicy
from rl.memory import SequentialMemory


def train():
    np.random.seed(123)
    env = GazeboEnv()
    env.seed(123)
    nb_actions = env.action_space.n
    model = Sequential()

    model.add(Flatten(input_shape=(1,) + env.observation_space.shape))
    model.add(Dense(16))
    model.add(Activation('relu'))
    model.add(Dense(16))
    model.add(Activation('relu'))
    model.add(Dense(16))
    model.add(Activation('relu'))
    model.add(Dense(nb_actions))
    model.add(Activation('linear'))
    print(model.summary())

    memory = SequentialMemory(limit=50000, window_length=1)
    policy = BoltzmannQPolicy()
    dqn = DQNAgent(model=model, nb_actions=nb_actions, memory=memory, nb_steps_warmup=10,
                   target_model_update=1e-2, policy=policy)
    dqn.compile(Adam(lr=1e-3), metrics=['mae'])

    dqn.fit(env, nb_steps=50000, visualize=True, verbose=2)

    #dqn.save_weights('dqn_{}_weights.h5f'.format(ENV_NAME), overwrite=True)

    dqn.test(env, nb_episodes=5, visualize=False)
