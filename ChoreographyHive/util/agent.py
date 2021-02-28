from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np


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


# Vector supports 1D, 2D and 3D Vectors, as well as calculations between them
# Arithmetic with 1D and 2D lists/tuples aren't supported - just set the remaining values to 0 manually
# With this new setup, Vector is much faster because it's just a wrapper for numpy
class Vector:
    def __init__(self, x: float = 0, y: float = 0, z: float = 0):
        # this is a private property - this is so all other things treat this class like a list, and so should you!
        self._np = np.array([x, y, z])

    def __getitem__(self, index):
        return self._np[index].item()

    def __setitem__(self, index, value):
        self._np[index] = value

    @property
    def x(self):
        return self._np[0].item()

    @x.setter
    def x(self, value):
        self._np[0] = value

    @property
    def y(self):
        return self._np[1].item()

    @y.setter
    def y(self, value):
        self._np[1] = value

    @property
    def z(self):
        return self._np[2].item()

    @z.setter
    def z(self, value):
        self._np[2] = value

    # self == value
    def __eq__(self, value):
        if isinstance(value, float) or isinstance(value, int):
            return self.magnitude() == value

        if hasattr(value, "_np"):
            value = value._np
        return (self._np == value).all()

    # len(self)
    def __len__(self):
        return 3  # this is a 3 dimensional vector, so we return 3

    # str(self)
    def __str__(self):
        # Vector's can be printed to console
        return f"[{self.x} {self.y} {self.z}]"

    # repr(self)
    def __repr__(self):
        return f"Vector(x={self.x}, y={self.y}, z={self.z})"

    # -self
    def __neg__(self):
        return Vector(*(self._np * -1))

    # self + value
    def __add__(self, value):
        if hasattr(value, "_np"):
            value = value._np
        return Vector(*(self._np+value))
    __radd__ = __add__

    # self - value
    def __sub__(self, value):
        if hasattr(value, "_np"):
            value = value._np
        return Vector(*(self._np-value))

    def __rsub__(self, value):
        return -self + value

    # self * value
    def __mul__(self, value):
        if hasattr(value, "_np"):
            value = value._np
        return Vector(*(self._np*value))
    __rmul__ = __mul__

    # self / value
    def __truediv__(self, value):
        if hasattr(value, "_np"):
            value = value._np
        return Vector(*(self._np/value))

    def __rtruediv__(self, value):
        return self * (1 / value)

    # round(self)
    def __round__(self, decimals=0) -> Vector:
        # Rounds all of the values
        return Vector(*np.around(self._np, decimals=decimals))

    @staticmethod
    def from_vector(vec) -> Vector:
        return Vector(vec.x, vec.y, vec.z)

    def magnitude(self) -> float:
        # Returns the length of the vector
        return np.linalg.norm(self._np).item()

    def dot(self, value: Vector) -> float:
        # Returns the dot product of two vectors
        if hasattr(value, "_np"):
            value = value._np
        return np.dot(self._np, value).item()

    def cross(self, value: Vector) -> Vector:
        # Returns the cross product of two vectors
        if hasattr(value, "_np"):
            value = value._np
        return Vector(*np.cross(self._np, value))

    def copy(self) -> Vector:
        # Returns a copy of the vector
        return Vector(*self._np)

    def normalize(self, return_magnitude=False) -> List[Vector, float] or Vector:
        # normalize() returns a Vector that shares the same direction but has a length of 1
        # normalize(True) can also be used if you'd like the length of this Vector (used for optimization)
        magnitude = self.magnitude()
        if magnitude != 0:
            norm_vec = Vector(*(self._np / magnitude))
            if return_magnitude:
                return norm_vec, magnitude
            return norm_vec
        if return_magnitude:
            return Vector(), 0
        return Vector()

    def flatten(self) -> Vector:
        # Sets Z (Vector[2]) to 0, making the Vector 2D
        return Vector(self._np[0], self._np[1])

    def angle2D(self, value: Vector) -> float:
        # Returns the 2D angle between this Vector and another Vector in radians
        return self.flatten().angle(value.flatten())

    def angle(self, value: Vector) -> float:
        # Returns the angle between this Vector and another Vector in radians
        return math.acos(max(min(np.dot(self.normalize()._np, value.normalize()._np).item(), 1), -1))

    def rotate(self, angle: float) -> Vector:
        # Rotates this Vector by the given angle in radians
        # Note that this is only 2D, in the x and y axis
        return Vector((math.cos(angle)*self.x) - (math.sin(angle)*self.y), (math.sin(angle)*self.x) + (math.cos(angle)*self.y), self.z)

    def clamp2D(self, start: Vector, end: Vector) -> Vector:
        # Similar to integer clamping, Vector's clamp2D() forces the Vector's direction between a start and end Vector
        # Such that Start < Vector < End in terms of clockwise rotation
        # Note that this is only 2D, in the x and y axis
        s = self.normalize()._np
        right = np.dot(s, np.cross(end._np, (0, 0, -1))) < 0
        left = np.dot(s, np.cross(start._np, (0, 0, -1))) > 0
        if (right and left) if np.dot(end._np, np.cross(start._np, (0, 0, -1))) > 0 else (right or left):
            return self
        if np.dot(start._np, s) < np.dot(end._np, s):
            return end
        return start

    def clamp(self, start: Vector, end: Vector) -> Vector:
        # This extends clamp2D so it also clamps the vector's z
        s = self.clamp2D(start, end)
        start_z = min(start.z, end.z)
        end_z = max(start.z, end.z)

        if s.z < start_z:
            s.z = start_z
        elif s.z > end_z:
            s.z = end_z

        return s

    def dist(self, value: Vector) -> float:
        # Distance between 2 vectors
        if hasattr(value, "_np"):
            value = value._np
        return np.linalg.norm(self._np - value).item()

    def flat_dist(self, value: Vector) -> float:
        # Distance between 2 vectors on a 2D plane
        return value.flatten().dist(self.flatten())

    def cap(self, low: float, high: float) -> Vector:
        # Caps all values in a Vector between 'low' and 'high'
        return Vector(*(max(min(item, high), low) for item in self._np))

    def midpoint(self, value: Vector) -> Vector:
        # Midpoint of the 2 vectors
        if hasattr(value, "_np"):
            value = value._np
        return Vector(*((self._np + value) / 2))

    def scale(self, value: float) -> Vector:
        # Returns a vector that has the same direction but with a value as the magnitude
        return self.normalize() * value
