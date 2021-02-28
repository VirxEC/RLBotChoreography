from __future__ import annotations

import math

import numpy as np
from rlbot.agents.base_agent import SimpleControllerState
from rlbot.utils.structures.game_data_struct import (PlayerInfo, Rotator,
                                                     Vector3)
from util.agent import Vector


class Drone:
    def __init__(self, index: int, team: int):
        self.index: int = index
        self.team: int = team
        self.pos: np.ndarray = np.zeros(3)
        self.rot: np.ndarray = np.zeros(3)
        self.vel: np.ndarray = np.zeros(3)
        self.boost: float = 0.0
        self.time: float = 0.0
        self.orient_m: np.ndarray = np.identity(3)
        self.ctrl: SimpleControllerState = SimpleControllerState()
        self.on_ground = False

        self._vec = Vector  # ignore this property
        self.location = self._vec()
        self.orientation = Matrix3()
        self.velocity = self._vec()
        self._local_velocity = self._vec()
        self.angular_velocity = self._vec()
        self.demolished = False
        self.airborne = False
        self.supersonic = False
        self.jumped = False
        self.doublejumped = False

    def update(self, game_car: PlayerInfo, time: float):
        self.pos = a3v(game_car.physics.location)
        self.rot = a3r(game_car.physics.rotation)
        self.vel = a3v(game_car.physics.velocity)
        self.boost = game_car.boost
        self.orient_m = orient_matrix(self.rot)
        self.time = time
        self.on_ground = game_car.has_wheel_contact

        car_phy = game_car.physics
        self.location = self._vec.from_vector(car_phy.location)
        self.velocity = self._vec.from_vector(car_phy.velocity)
        self.orientation = Matrix3.from_rotator(car_phy.rotation)
        self._local_velocity = self.local(self.velocity)
        self.angular_velocity = self.orientation.dot((car_phy.angular_velocity.x, car_phy.angular_velocity.y, car_phy.angular_velocity.z))
        self.demolished = game_car.is_demolished
        self.airborne = not game_car.has_wheel_contact
        self.supersonic = game_car.is_super_sonic
        self.jumped = game_car.jumped
        self.doublejumped = game_car.double_jumped

    def local(self, value):
        # Generic localization
        return self.orientation.dot(value)

    def local_velocity(self, velocity=None):
        # Returns the velocity of an item relative to the car
        # x is the velocity forwards (+) or backwards (-)
        # y is the velocity to the left (+) or right (-)
        # z if the velocity upwards (+) or downwards (-)
        if velocity is None:
            return self._local_velocity

        return self.local(velocity)

    def local_location(self, location):
        # Returns the location of an item relative to the car
        # x is how far the location is forwards (+) or backwards (-)
        # y is how far the location is to the left (+) or right (-)
        # z is how far the location is upwards (+) or downwards (-)
        return self.local(location - self.location)

    @property
    def forward(self):
        # A vector pointing forwards relative to the cars orientation. Its magnitude == 1
        return self.orientation.forward

    @property
    def right(self):
        # A vector pointing left relative to the cars orientation. Its magnitude == 1
        return self.orientation.right

    @property
    def up(self):
        # A vector pointing up relative to the cars orientation. Its magnitude == 1
        return self.orientation.up

    def reset_ctrl(self):
        self.ctrl = SimpleControllerState()


def seek_pos(drone, position, max_speed=1410):
    """
    Tries to intelligently drive so it stops at a given position.
    """

    # Get speed.
    speed = np.linalg.norm(drone.vel)

    # Simplified speed controller.
    if speed < max_speed:
        drone.ctrl.throttle = 1.0
    else:
        drone.ctrl.throttle = 0.0

    def special_sauce(x, a):
        """Modified sigmoid to smooth out steering."""
        # Graph: https://www.geogebra.org/m/udfp2zcy
        return 2 / (1 + np.exp(a * x)) - 1

    # Calculates the 2D angle to the position. Positive is clockwise.
    local_target = local(drone.orient_m, drone.pos, position)
    angle = np.arctan2(local_target[1], local_target[0])

    # Calculates steer.
    drone.ctrl.steer = special_sauce(angle, -5)


def slow_to_pos(drone, position):
    # Calculate distance and velocity.
    distance = np.linalg.norm(position - drone.pos)
    velocity = np.linalg.norm(drone.vel)
    # Calculates the target position in local coordinates.
    local_target = local(drone.orient_m, drone.pos, position)
    # Finds 2D angle to target. Positive is clockwise.
    angle = np.arctan2(local_target[1], local_target[0])

    def special_sauce(x, a):
        """Modified sigmoid to smooth out steering."""
        # Graph: https://www.geogebra.org/m/udfp2zcy
        return 2 / (1 + np.exp(a * x)) - 1

    # Calculates steer.
    drone.ctrl.steer = special_sauce(angle, -5)

    # Throttle controller.
    if abs(angle) > 2:
        # If I'm facing the wrong way, do a little drift.
        drone.ctrl.throttle = 1.0
        drone.ctrl.handbrake = True
    elif distance > 100:
        # A simple PD controller to stop at target.
        drone.ctrl.throttle = cap(0.3 * distance - 0.2 * velocity, -1.0, 1.0)
        if distance > 1000:
            drone.ctrl.boost = True


def slow_to_pos2(drone, position):
    """
    Tries to intelligently drive so it stops at a given position.
    """
    TURN_DIS = 800 # Slows down for turns when farther than this.
    TURN_SLOW = 300 # Maximum speed slowdown for turning.
    STOP_DIS = 40 # Stops if closer than this.

    def special_sauce(x, a):
        """Modified sigmoid function."""
        return 2 / (1 + np.exp(a * x)) - 1

    # Get distance and speed.
    distance = np.linalg.norm(position - drone.pos)
    speed = np.linalg.norm(drone.vel)

    # Calculates the 2D angle to the position. Positive is clockwise.
    local_target = local(drone.orient_m, drone.pos, position)
    angle = np.arctan2(local_target[1], local_target[0])

    # Calculates steer
    drone.ctrl.steer = special_sauce(angle, -5)

    # Manages desired speed so that cars slow down when close and when turning.
    desired_speed = distance/2
    if distance > TURN_DIS:
        desired_speed -= TURN_SLOW * special_sauce(angle, -2)
    desired_speed = cap(desired_speed, 0.0, 2300.0)

    # Simplified speed controller.
    if speed < desired_speed and distance > STOP_DIS:
        drone.ctrl.throttle = 1.0
    else:
        drone.ctrl.throttle = 0.0


def turn_to_pos(drone, position, game_time):
    # Wiggle rate per second.
    RATE = 0.2

    # Calculates the target position in local coordinates.
    local_target = local(drone.orient_m, drone.pos, position)
    # Finds 2D angle to target. Positive is clockwise.
    angle = np.arctan2(local_target[1], local_target[0])

    # Toggles forward.
    drone.forward = round(game_time / RATE) % 2

    # Wiggles forward and back switching the steer to rotate on the spot.
    if drone.forward:
        drone.ctrl.throttle = 0.5
        drone.ctrl.steer = cap(angle, -1, 1)
    else:
        drone.ctrl.throttle = -0.5
        drone.ctrl.steer = cap(-angle, -1, 1)


def fast_to_pos(drone, position):
    # Calculates the target position in local coordinates.
    local_target = local(drone.orient_m, drone.pos, position)
    # Finds 2D angle to target. Positive is clockwise.
    angle = np.arctan2(local_target[1], local_target[0])

    def special_sauce(x, a):
        """Modified sigmoid to smooth out steering."""
        # Graph: https://www.geogebra.org/m/udfp2zcy
        return 2 / (1 + np.exp(a * x)) - 1

    # Control towards hit position. Fully boosting.
    drone.ctrl.steer = special_sauce(angle, -5)
    drone.ctrl.throttle = 1.0
    drone.ctrl.boost = True


def local(A: np.ndarray, p0: np.ndarray, p1: np.ndarray) -> np.ndarray:
    """Transforms world coordinates into local coordinates.

    Arguments:
        A {np.ndarray} -- The local orientation matrix.
        p0 {np.ndarray} -- World x, y, and z coordinates of the start point for the vector.
        p1 {np.ndarray} -- World x, y, and z coordinates of the end point for the vector.

    Returns:
        np.ndarray -- Local x, y, and z coordinates.
    """
    return np.dot(A.T, p1 - p0)


def cap(value: float, minimum: float, maximum: float) -> float:
    """Caps the value at given minimum and maximum.

    Arguments:
        value {float} -- The value being capped.
        minimum {float} -- Smallest value.
        maximum {float} -- Largest value.

    Returns:
        float -- The capped value or the original value if within range.
    """
    if value > maximum:
        return maximum
    elif value < minimum:
        return minimum
    else:
        return value


def a3l(l: list) -> np.ndarray:
    """Converts list to numpy array.

    Arguments:
        L {list} -- The list to convert containing 3 elements.

    Returns:
        np.array -- Numpy array with the same contents as the list.
    """
    return np.array([l[0], l[1], l[2]])


def a3r(r: Rotator) -> np.ndarray:
    """Converts rotator to numpy array.

    Arguments:
        R {Rotator} -- Rotator class containing pitch, yaw, and roll.

    Returns:
        np.ndarray -- Numpy array with the same contents as the rotator.
    """
    return np.array([r.pitch, r.yaw, r.roll])


def a3v(v: Vector3) -> np.ndarray:
    """Converts vector3 to numpy array.

    Arguments:
        V {Vector3} -- Vector3 class containing x, y, and z.

    Returns:
        np.ndarray -- Numpy array with the same contents as the vector3.
    """
    return np.array([v.x, v.y, v.z])


def normalize(V : np.ndarray) -> np.ndarray:
    """normalizes a vector.

    Arguments:
        V {np.ndarray} -- Vector.

    Returns:
        np.ndarray -- normalized vector.
    """
    magnitude = np.linalg.norm(V)
    if magnitude != 0.0:
        return V / magnitude

    return V


class Matrix3:
    # The Matrix3's sole purpose is to convert roll, pitch, and yaw data from the gametickpacket into an orientation matrix
    # An orientation matrix contains 3 Vector's
    # Matrix3[0] is the "forward" direction of a given car
    # Matrix3[1] is the "left" direction of a given car
    # Matrix3[2] is the "up" direction of a given car
    def __init__(self, pitch=0, yaw=0, roll=0):
        CP = math.cos(pitch)
        SP = math.sin(pitch)
        CY = math.cos(yaw)
        SY = math.sin(yaw)
        CR = math.cos(roll)
        SR = math.sin(roll)
        # List of 3 vectors, each descriping the direction of an axis: Forward, Left, and Up
        self.data = (
            Vector(CP*CY, CP*SY, SP),
            Vector(CY*SP*SR-CR*SY, SY*SP*SR+CR*CY, -CP*SR),
            Vector(-CR*CY*SP-SR*SY, -CR*SY*SP+SR*CY, CP*CR)
        )
        self.forward, self.right, self.up = self.data

    def __getitem__(self, key):
        return self.data[key]

    def __str__(self):
        return f"[{self.forward}\n {self.right}\n {self.up}]"

    @staticmethod
    def from_rotator(rotator) -> Matrix3:
        return Matrix3(rotator.pitch, rotator.yaw, rotator.roll)

    def dot(self, vector):
        return Vector(self.forward.dot(vector), self.right.dot(vector), self.up.dot(vector))

    def det(self):
        return self[0][0] * self[1][1] * self[2][2] + self[0][1] * self[1][2] * self[2][0] + \
               self[0][2] * self[1][0] * self[2][1] - self[0][0] * self[1][2] * self[2][1] - \
               self[0][1] * self[1][0] * self[2][2] - self[0][2] * self[1][1] * self[2][0]


def orient_matrix(R: np.ndarray) -> np.ndarray:
    """Converts from Euler angles to an orientation matrix.

    Arguments:
        R {np.ndarray} -- Pitch, yaw, and roll.

    Returns:
        np.ndarray -- Orientation matrix of shape (3, 3).
    """
    # Credits to chip https://samuelpmish.github.io/notes/RocketLeague/aerial_control/
    pitch: float = R[0]
    yaw: float = R[1]
    roll: float = R[2]

    CR: float = np.cos(roll)
    SR: float = np.sin(roll)
    CP: float = np.cos(pitch)
    SP: float = np.sin(pitch)
    CY: float = np.cos(yaw)
    SY: float = np.sin(yaw)

    A = np.zeros((3, 3))

    # front direction
    A[0, 0] = CP * CY
    A[1, 0] = CP * SY
    A[2, 0] = SP

    # right direction (should be left but for some reason it is weird)
    A[0, 1] = CY * SP * SR - CR * SY
    A[1, 1] = SY * SP * SR + CR * CY
    A[2, 1] = -CP * SR

    # up direction
    A[0, 2] = -CR * CY * SP - SR * SY
    A[1, 2] = -CR * SY * SP + SR * CY
    A[2, 2] = CP * CR

    return A
