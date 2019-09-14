from PIL import Image
from geometry_msgs.msg import Twist
from gym import spaces
from scipy.misc import toimage

import numpy as np
import matplotlib.pyplot as plt

from env.Env import Env
from env.GazeboMixin import GazeboMixin
from env.State import State


class GazeboEnv(Env, GazeboMixin):
    # TODO It should not be hardcoded
    BOARD_WIDTH = 6.95
    BOARD_HEIGHT = 16.7

    LEFT_CAMERA_TOPIC = '/conde_camera_tracking_left/image_raw'
    CENTER_CAMERA_TOPIC = '/conde_camera_signalling_panel/image_raw'
    RIGHT_CAMERA_TOPIC = '/conde_camera_tracking_right/image_raw'

    LINEAR_ACTION = 0.3
    TWIST_ACTION = 0.25

    BOARD_IMAGE_PATH = 'src/environment/src/env/data/board.jpeg'

    def __init__(self):
        super(GazeboEnv, self).__init__()
        metadata = {'render.modes': ['human', 'rgb_array']}
        self.action_space = spaces.Discrete(9)
        self.board = self._load_board()

    def step(self, action):
        """

        :param action: int
        :return: observation: State, reward: float, done: bool, info: dict
        """
        info = dict()

        message = self._get_message_from_action(action)

        self._unpause_gazebo()
        self._publish_gazebo(message)
        self._ros_sleep()
        self._pause_gazebo()

        observation = self._get_observation()
        reward = self._calculate_reward()

        done = False  # TODO?

        return observation, reward, done, info

    def reset(self):
        self._reset_gazebo()
        self._unpause_gazebo()
        self._get_observation()
        # TODO set state, and reward here
        self._pause_gazebo()

    def render(self, mode='human'):
        if mode == 'rgb_array':
            return np.array(self.observation)
        elif mode == 'human':
            # TODO some window?
            return None
        else:
            super(GazeboEnv, self).render(mode=mode)  # just raise an exception

    def _get_observation(self):
        """

        :return: State
        """
        left_image = self._get_image_data_from_topic(self.LEFT_CAMERA_TOPIC)
        center_image = self._get_image_data_from_topic(self.CENTER_CAMERA_TOPIC)
        right_image = self._get_image_data_from_topic(self.RIGHT_CAMERA_TOPIC)

        state = State(left_image, center_image, right_image)
        return state

    def _calculate_reward(self):
        car_x, car_y = self._get_car_position()
        relative_car_x = int((self.BOARD_HEIGHT / 2 - car_y) * 100)
        relative_car_y = int((car_x + self.BOARD_WIDTH / 2) * 100)

        self._print_car_position_on_board(relative_car_x, relative_car_y)
        # TODO find distance from car to nearest center of road
        return 0

    def _get_message_from_action(self, action):
        """

        :param action: int
        :return: Twist
        """
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        if action == 0:  # F
            twist.linear.x = self.LINEAR_ACTION
        elif action == 1:  # L
            twist.angular.z = -self.TWIST_ACTION
        elif action == 2:  # B
            twist.linear.x = -self.LINEAR_ACTION
        elif action == 3:  # R
            twist.angular.z = self.TWIST_ACTION
        elif action == 4:  # FL
            twist.linear.x = self.LINEAR_ACTION
            twist.angular.z = -self.TWIST_ACTION
        elif action == 5:  # FR
            twist.linear.x = self.LINEAR_ACTION
            twist.angular.z = self.TWIST_ACTION
        elif action == 6:  # BL
            twist.linear.x = -self.LINEAR_ACTION
            twist.angular.z = -self.TWIST_ACTION
        elif action == 7:  # BR
            twist.linear.x = -self.LINEAR_ACTION
            twist.angular.z = self.TWIST_ACTION
        else:
            raise ValueError

        return twist

    def _get_car_position(self):
        model_state = self._get_model_states()
        position = model_state.pose.position
        return position.x, position.y

    def _load_board(self):
        """
        This function loads image, resize to BOARD_WIDTH x BOARD_HEIGHT
        and after that apply threshold, scale values to 0, 255 and return
        board image as numpy array with uint8 0,255 values
        :return:
        """
        board_image = Image.open(self.BOARD_IMAGE_PATH).convert('L')
        width, height = int(self.BOARD_WIDTH * 100), int(self.BOARD_HEIGHT * 100)
        board_image = board_image.resize((width, height), Image.ANTIALIAS)
        board_image = board_image.point(lambda p: p > 50)
        image_array = np.array(board_image) * 255
        return image_array

    def _print_car_position_on_board(self, car_x, car_y):
        distance = 15
        img = self.board
        img[car_x - distance:car_x + distance, car_y - distance:car_y + distance] = 255
        toimage(img).show()
