import numpy as np
from math import sqrt


def _get_angle_between_vector_and_x_axis(vec):
    angle = np.arctan2(vec[1], vec[0])
    if angle < 0:
        angle = 2 * np.pi + angle
    return angle


def normalized(vec):
    return vec / np.linalg.norm(vec)


def get_straight_from_points(first_point, second_point):
    """
    :param first_point: (x, y)
    :param second_point:  (x, y)
    :return: a, b straight's coefficients
    """
    # Ax + By + C = 0
    delta_y, delta_x = second_point[1] - first_point[1], second_point[0] - first_point[0]
    if np.isclose(delta_x, 0.0):
        return 1, 0, -first_point[1]

    if np.isclose(delta_y, 0.0):
        return 0, 1, -first_point[0]

    a = np.array([[first_point[0], 1], [second_point[0], 1]])
    b = np.array([first_point[1], second_point[1]])
    straight = np.linalg.solve(a, b)

    # ax + by + c = 0 -> b=-1, ax + c = y
    return straight[0], -1, straight[1]


def point_to_straight_distance(line, point):
    """
    :param line: a tuple of 3 points (a, b, c)
    :param point: a tuple of 2 points (x, y)
    :return:
        distance from a line to the point
    """
    a, b, c = line
    x, y = point
    denominator = sqrt(a ** 2 + b ** 2)
    if np.isclose(denominator, 0.0):
        return 0.0
    return np.abs(a * x + b * y + c) / denominator


def vector_on_straight(straight):
    """
    :param straight1 (a, b, c)
    :return: numpy.array representing vector on given straight
    """
    #the idea is to take two arbitrary points on the straight and construct the vector connecting these two points

    if np.isclose(straight[0], 0.0):
        return np.array([1.0, 0.0])
    if np.isclose(straight[1], 0.0):
        return np.array([0.0, 1.0])
    #substitute x = 0
    p1 = np.array([0, -(0 * straight[0] + straight[2]) / straight[1]])
    #substitute x = 1
    p2 = np.array([1, -(1 * straight[0] + straight[2]) / straight[1]])
    return p2 - p1

def angle_between_two_straight(straight1, straight2):
    """
    :param straight1: (a, b, c)
    :param straight2: (a, b, c)
    :return: angle between two straight lines in degrees
    """
    v1 = vector_on_straight(straight1)
    v2 = vector_on_straight(straight2)
    return np.rad2deg(angle_between_vectors(v1, v2))

def angle_between_vectors(vec1, vec2):
    return np.arccos(np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)))


def get_two_straight_lines_intersection(straight1, straight2):
    """
    :param straight1: (a, b, c)
    :param straight2: (a, b, c)
    :return: point (x, y) that represents an intersection between straight1 and straight2
    """
    a = np.array([[straight1[0], straight1[1]], [straight2[0], straight2[1]]])
    b = np.array([-straight1[2], -straight2[2]])

    point = np.linalg.solve(a, b)

    return point  # x, y
