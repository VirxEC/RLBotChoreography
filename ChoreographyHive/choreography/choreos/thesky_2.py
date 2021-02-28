import math
from typing import List

import numpy as np
from choreography.choreography import Choreography
from choreography.drone import Drone, slow_to_pos
from choreography.group_step import (BlindBehaviorStep, DroneListStep,
                                     PerDroneStep, StepResult)
from rlbot.agents.base_agent import SimpleControllerState
from rlbot.utils.game_state_util import (BallState, CarState, GameInfoState,
                                         GameState, Physics, Rotator, Vector3)
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.structures.game_interface import GameInterface
from util.vec import Vec3
from util.agent import Vector


radius = 3000
radius2 = 2048
radian_offset = 0
gravity = Vector(z=-650)
max_speed = 2300
throttle_accel = 66 + (2/3)
brake_accel = Vector(x=-3500)
boost_per_second = 33 + (1/3)
jump_max_duration = 0.2
jump_speed = 291 + (2/3)
jump_acc = 1458 + (1/3)
boost_accel = 991 + (2/3)
delta_time = 1/60


class TheSky2(Choreography):
    """
    For seconds 12-36 of The Sky
    """

    def __init__(self, game_interface: GameInterface):
        super().__init__()
        self.game_interface = game_interface

    def generate_sequence(self, drones: List[Drone]):
        self.sequence.clear()

        pause_time = 1.5
        self.drone_aerials = []

        self.sequence.append(DroneListStep(self.setup))
        self.sequence.append(DroneListStep(self.circular_procession))
        self.sequence.append(DroneListStep(self.setup_circle_align))
        self.sequence.append(DroneListStep(self.circle_align))
        self.sequence.append(DroneListStep(self.act_2_start))
        # self.sequence.append(DroneListStep(self.end_choreo))

    @staticmethod
    def get_num_bots() -> int:
        return 66

    def spin_around_rising_ball(self, packet, drones, start_time) -> StepResult:
        return StepResult(finished=True)

    def end_choreo(self, packet, drones, start_time) -> StepResult:
        self.game_interface.set_game_state(GameState(ball=BallState(Physics(location=Vector3(0, 5300, 400)))))
        return StepResult(finished=True)

    def setup(self, packet, drones, start_time) -> StepResult:
        car_states = {}
        radian_spacing = 2 * math.pi / len(drones)

        for index, drone in enumerate(drones):
            progress = index * radian_spacing + radian_offset
            target = Vec3(radius * math.sin(progress), radius * math.cos(progress), 0)

            car_states[drone.index] = CarState(
                Physics(location=Vector3(target.x, target.y, 50),
                        velocity=Vector3(0, 0, 0),
                        rotation=Rotator(0, -progress, 0)))

        self.game_interface.set_game_state(GameState(
            cars=car_states,
            ball=BallState(physics=Physics(
                location=Vector3(0, 0, 93),
                velocity=Vector3(0, 0, 0),
                angular_velocity=Vector3(0, 0, 0)
            ))
        ))

        return StepResult(finished=True)

    def circular_procession(self, packet: GameTickPacket, drones, start_time) -> StepResult:
        """
        Makes all cars drive in a slowly shrinking circle.
        https://gfycat.com/yearlygreathermitcrab
        """
        elapsed = packet.game_info.seconds_elapsed - start_time
        inactive_drones = elapsed / 0.44 - 5
        radian_spacing = 2 * math.pi / max(len(drones) - inactive_drones, 16)
        adjusted_radius = radius - elapsed * 75
        
        for i, drone in enumerate(drones):
            if i >= inactive_drones:
                progress = i * radian_spacing + elapsed * .25
                target = [adjusted_radius * math.sin(progress), adjusted_radius * math.cos(progress), 0]
                slow_to_pos(drone, target)
            else:
                if len(self.drone_aerials) == i:
                    progress = i * radian_spacing + (elapsed + 2) * .25
                    target = Vector(adjusted_radius * math.sin(progress), adjusted_radius * math.cos(progress), 200 + i * 10)
                    self.drone_aerials.append(Aerial(target, False))

                self.drone_aerials[i].target.z += 0.1
                self.drone_aerials[i].run(drone, packet.game_info.seconds_elapsed)
        return StepResult(finished=inactive_drones > len(drones) + 4)

    def setup_circle_align(self, packet: GameTickPacket, drones, start_time) -> StepResult:
        radian_spacing = 2 * math.pi / len(drones)
        elapsed = packet.game_info.seconds_elapsed - start_time

        for i, drone in enumerate(drones):
            progress = i * radian_spacing
            self.drone_aerials[i].target = Vector(radius * math.sin(progress), radius * math.cos(progress), 1000)
            self.drone_aerials[i].run(drone, packet.game_info.seconds_elapsed)

        return StepResult(finished=True)

    def circle_align(self, packet: GameTickPacket, drones, start_time) -> StepResult:
        for i, drone in enumerate(drones):
            self.drone_aerials[i].run(drone, packet.game_info.seconds_elapsed)

        return StepResult(finished=packet.game_info.seconds_elapsed - start_time > 10)

    def act_2_start(self, packet: GameTickPacket, drones, start_time) -> StepResult:
        radian_spacing = 2 * math.pi / 22
        radian_spacing_v = 2 * math.pi / 11
        elapsed = packet.game_info.seconds_elapsed - start_time

        for i, drone in enumerate(drones):
            # 0 & 1: center circle
            # 2, 3, 4, & 5: side circles
            group = i % 6
            
            if group == 0 or group == 1:
                progress = (i // 6 * 2 + group) * radian_spacing + elapsed * .25
                self.drone_aerials[i].target = Vector(radius2 * math.sin(progress), radius2 * math.cos(progress), 1000)
            
            self.drone_aerials[i].run(drone, packet.game_info.seconds_elapsed)

        return StepResult()



def cap(x, low, high):
    # caps/clamps a number between a low and high value
    return low if x < low else (high if x > high else x)


def sign(x):
    # returns the sign of a number, -1, 0, +1
    if x < 0:
        return -1

    if x > 0:
        return 1

    return 0


def defaultPD(me, local_target, upside_down=False, up=None):
    # points the car towards a given local target.
    # Direction can be changed to allow the car to steer towards a target while driving backwards

    if up is None:
        up = me.local(Vector(z=-1 if upside_down else 1))  # where "up" is in local coordinates
    target_angles = (
        math.atan2(local_target.z, local_target.x),  # angle required to pitch towards target
        math.atan2(local_target.y, local_target.x),  # angle required to yaw towards target
        math.atan2(up.y, up.z)  # angle required to roll upright
    )
    # Once we have the angles we need to rotate, we feed them into PD loops to determing the controller inputs
    me.ctrl.steer = steerPD(target_angles[1], 0)
    me.ctrl.pitch = steerPD(target_angles[0], me.angular_velocity.y/4)
    me.ctrl.yaw = steerPD(target_angles[1], -me.angular_velocity.z/4)
    me.ctrl.roll = steerPD(target_angles[2], me.angular_velocity.x/4)
    # Returns the angles, which can be useful for other purposes
    return target_angles


def steerPD(angle, rate):
    # A Proportional-Derivative control loop used for defaultPD
    return cap(((35*(angle+rate))**3)/10, -1, 1)


# A combination of Blind and Deaf's hover code and VirxERLU's car control
class Aerial:
    def __init__(self, target, fast_aerial=True):
        self.fast_aerial = fast_aerial
        self.target = target

        self.jump_type_fast = None
        self.jumping = False
        self.dodging = False
        self.jump_time = -1
        self.counter = 0

    def run(self, me, time):
        me.reset_ctrl()

        if self.jumping or (self.jump_time == -1 and me.on_ground):
            if self.jump_time == -1:
                self.jump_type_fast = self.fast_aerial
                self.jumping = True
                self.jump_time = time
                self.counter = 0

            jump_elapsed = time - self.jump_time

            if self.jump_type_fast:
                if jump_elapsed <= jump_max_duration:
                    me.ctrl.jump = True
                else:
                    self.counter += 1

                if self.counter == 3:
                    me.ctrl.jump = True
                    self.dodging = True
                elif self.counter == 4:
                    self.dodging = self.jumping = False
                    self.jump_time = -1
            elif jump_elapsed <= jump_max_duration:
                me.ctrl.jump = True
            else:
                self.jumping = False
                self.jump_time = -1

        delta_x = self.target - me.location
        if delta_x.magnitude() > boost_accel:
            delta_x *= boost_accel / delta_x.magnitude()

        delta_xy = Vector(delta_x.x - me.velocity.x, delta_x.y - me.velocity.y, 1000 if not self.jumping else 0)
        direction = delta_xy.normalize()

        if self.counter in {0, 4}: defaultPD(me, me.local(delta_xy), up=sign(math.sin(time)) * (Vector() - me.location).flatten().normalize())

        # only boost/throttle if we're facing the right direction
        if abs(me.forward.angle(delta_xy)) < 0.5 and not self.jumping:
            me.ctrl.throttle = 1
            # tap boost to keep height
            if (delta_x.z - me.velocity.z * 0.5) > 0:
                me.ctrl.boost = True

            # if the target is relatively far, hold boost even when we're higher than the target to keep moving
            if delta_x.z < 0 and me.forward.z < 0.5:
                me.ctrl.boost = True
