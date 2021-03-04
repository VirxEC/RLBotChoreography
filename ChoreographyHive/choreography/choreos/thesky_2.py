import math
import random
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
from util.agent import Vector, ball_object
from util.vec import Vec3

radius = 3000
radius2 = 2048
radius3 = 700
radian_offset = 0
demo_cooldown = (41-13)/60

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
    For seconds 0:12-1:56 of The Sky
    """

    def __init__(self, game_interface: GameInterface):
        super().__init__()
        self.game_interface = game_interface
        self.ball = ball_object()
        
        self.attacked_center = False

    def generate_sequence(self, drones: List[Drone]):
        self.sequence.clear()

        pause_time = 1.5
        self.drone_aerials = []
        self.last_demo_time = -1
        self.alive_drones = list(range(60))

        self.sequence.append(DroneListStep(self.setup))
        self.sequence.append(DroneListStep(self.circular_procession))
        self.sequence.append(DroneListStep(self.setup_circle_align))
        self.sequence.append(DroneListStep(self.circle_align))
        self.sequence.append(DroneListStep(self.act_2))
        self.sequence.append(DroneListStep(self.act_2_end))
        self.sequence.append(DroneListStep(self.end_choreo))

    @staticmethod
    def get_num_bots() -> int:
        return 64

    def spin_around_rising_ball(self, packet, drones, start_time) -> StepResult:
        return StepResult(finished=True)

    def end_choreo(self, packet, drones, start_time) -> StepResult:
        self.game_interface.set_game_state(GameState(ball=BallState(Physics(location=Vector3(0, 5300, 400)))))
        return StepResult(finished=True)

    def setup(self, packet, drones, start_time) -> StepResult:
        self.game_interface.set_game_state(GameState(game_info=GameInfoState(game_speed=0.25)))

        car_states = {}
        radian_spacing = 2 * math.pi / 60

        for index, drone in enumerate(drones):
            if 61 <= index <= 64:
                car_states[drone.index] = CarState(
                    Physics(location=Vector3(3520, 5100, 0),
                            velocity=Vector3(0, 0, 0)))
                continue

            if index == 60:
                car_states[drone.index] = CarState(
                    Physics(location=Vector3(0, 0, 20),
                            velocity=Vector3(0, 0, 0),
                            rotation=Rotator(0, 0, 0)))
                continue

            progress = index * radian_spacing
            target = Vec3(radius * math.sin(progress), radius * math.cos(progress), 0)

            car_states[drone.index] = CarState(
                Physics(location=Vector3(target.x, target.y, 20),
                        velocity=Vector3(0, 0, 0),
                        rotation=Rotator(0, -progress, 0)))

        self.game_interface.set_game_state(GameState(
            cars=car_states,
            ball=BallState(physics=Physics(
                location=Vector3(0, 0, 155),
                velocity=Vector3(0, 0, 0),
                angular_velocity=Vector3(0, 0, 0)
            ))
        ))

        return StepResult(finished=True)

    def circular_procession(self, packet: GameTickPacket, drones, start_time) -> StepResult:
        self.ball.update(packet)

        elapsed = packet.game_info.seconds_elapsed - start_time
        inactive_drones = max((elapsed - 4) / 0.48, 0)
        radian_spacing = 2 * math.pi / max(60 - inactive_drones, 16)
        adjusted_radius = radius - elapsed * 75
        
        for i, drone in enumerate(drones):
            if i >= inactive_drones:
                if i < 60:
                    progress = i * radian_spacing + elapsed * .25
                    target = [adjusted_radius * math.sin(progress), adjusted_radius * math.cos(progress), 0]
                    slow_to_pos(drone, target)
                continue

            if len(self.drone_aerials) == i:
                progress = i * radian_spacing + (elapsed + 2) * .25
                target = Vector(adjusted_radius * math.sin(progress), adjusted_radius * math.cos(progress), 200 + i * 10)
                self.drone_aerials.append(Hover(target, i != 60))

            self.drone_aerials[i].target.z += 0.1
            self.drone_aerials[i].run(drone, packet.game_info.seconds_elapsed)
            
            if i == 60:
                break
        return StepResult(finished=inactive_drones > 61)

    def setup_circle_align(self, packet: GameTickPacket, drones, start_time) -> StepResult:
        self.ball.update(packet)
        self.game_interface.set_game_state(GameState(ball=BallState(physics=Physics(location=Vector3(drones[60].location.x, drones[60].location.y), velocity=Vector3(0, 0), angular_velocity=Vector3(*drones[60].raw_angular_velocity)))))

        radian_spacing = 2 * math.pi / 20
        radian_spacing_v = 2 * math.pi / 10

        for i, drone in enumerate(drones):
            if i == 60:
                self.drone_aerials[i].target = Vector(0, 0, 1000)
            else:
                # 0 & 1: center circle
                # 2, 3, 4, & 5: side circles
                group = i % 6
                
                if group < 2:
                    progress = (i // 6 * 2 + group) * radian_spacing
                    self.drone_aerials[i].target = Vector(radius2 * math.sin(progress), radius2 * math.cos(progress), 1000)
                elif group < 4:
                    progress = (i // 6 * 2 + (group - 2)) * radian_spacing
                    Q = radius3 * math.sin(progress)
                    adjusted_radius = radius2 + Q
                    self.drone_aerials[i].target = Vector(adjusted_radius * math.sin(progress), adjusted_radius * math.cos(progress), 1000 + Q)
                else:
                    progress = (i // 6 * 2 + (group - 4)) * radian_spacing
                    Q = radius3 * math.sin(progress)
                    adjusted_radius = radius2 - Q
                    self.drone_aerials[i].target = Vector(adjusted_radius * math.sin(progress), adjusted_radius * math.cos(progress), 1000 - Q)
            self.drone_aerials[i].run(drone, packet.game_info.seconds_elapsed)

            if i == 60:
                break

        return StepResult(finished=True)

    def circle_align(self, packet: GameTickPacket, drones, start_time) -> StepResult:
        self.ball.update(packet)
        self.game_interface.set_game_state(GameState(ball=BallState(physics=Physics(location=Vector3(drones[60].location.x, drones[60].location.y), velocity=Vector3(0, 0), angular_velocity=Vector3(*drones[60].raw_angular_velocity)))))

        for i, drone in enumerate(drones):
            self.drone_aerials[i].run(drone, packet.game_info.seconds_elapsed)
            if i == 60:
                break

        return StepResult(finished=packet.game_info.seconds_elapsed - start_time > 14)

    def get_random_demo_target(self):
        target = random.choice(self.alive_drones)
        self.alive_drones.remove(target)
        return target

    def act_2(self, packet: GameTickPacket, drones, start_time) -> StepResult:
        self.ball.update(packet)

        if self.odd_tick % 2 == 0:
            self.game_interface.set_game_state(GameState(ball=BallState(physics=Physics(location=Vector3(drones[60].location.x, drones[60].location.y), velocity=Vector3(0, 0), angular_velocity=Vector3(*drones[60].raw_angular_velocity)))))

        radian_spacing = 2 * math.pi / 20
        elapsed = packet.game_info.seconds_elapsed - start_time
        hover_height = 1022 - max(0, (elapsed - 60) * 100)

        # elapsed @ 16 seconds (1:06): foreshadow attack
        # elapsed @ 31 seconds (1:21): start attack
        # elapsed @ 60 seconds (1:50): attack center air dribbler then stop

        for i, drone in enumerate(drones):
            if i < 60:
                if drone.demolished:
                    continue
                elif i not in self.alive_drones:
                    self.alive_drones.append(i)
                # 0 & 1: center circle
                # 2, 3, 4, & 5: side circles
                group = i % 6
                
                if group < 2:
                    progress = (i // 6 * 2 + group) * radian_spacing + elapsed * .3
                    self.drone_aerials[i].target = Vector(radius2 * math.sin(progress), radius2 * math.cos(progress), hover_height)
                elif group < 4:
                    progress = (i // 6 * 2 + (group - 2)) * radian_spacing + elapsed * .3
                    Q = radius3 * math.sin(progress)
                    adjusted_radius = radius2 + Q
                    self.drone_aerials[i].target = Vector(adjusted_radius * math.sin(progress), adjusted_radius * math.cos(progress), hover_height + Q)
                else:
                    progress = (i // 6 * 2 + (group - 4)) * radian_spacing + elapsed * .3
                    Q = radius3 * math.sin(progress)
                    adjusted_radius = radius2 - Q
                    self.drone_aerials[i].target = Vector(adjusted_radius * math.sin(progress), adjusted_radius * math.cos(progress), hover_height - Q)
            
            self.drone_aerials[i].run(drone, packet.game_info.seconds_elapsed)

            if i == 60:
                break

        if elapsed >= 31:
            if elapsed - self.last_demo_time >= demo_cooldown:
                car_states = {}

                for i in (61, 62):  # (61, 62, 63)
                    target = drones[self.get_random_demo_target()] if elapsed < 60 else drones[60]
                    car_states[i] = CarState(physics=Physics(
                        location=Vector3( target.location.x - 100, target.location.y, target.location.z),
                        velocity=Vector3(2300, 0, 0),
                        rotation=Vector3(0, 0, 0)
                    ))

                if elapsed >= 60:
                    self.attacked_center = True

                self.game_interface.set_game_state(GameState(
                    cars=car_states
                ))

                self.last_demo_time = elapsed


        return StepResult(finished=elapsed > 63)

    def act_2_end(self, packet: GameTickPacket, drones, start_time) -> StepResult:
        return StepResult(finished=packet.game_info.seconds_elapsed - start_time > 10)


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


# A combination of Blind and Deaf's hover code and VirxERLU's car control + jump code
class Hover:
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

        delta_xy = Vector(delta_x.x - me.velocity.x, delta_x.y - me.velocity.y, 1000 if (not self.jumping or not self.jump_type_fast) else 0)
        direction = delta_xy.normalize()

        if self.counter in {0, 4}: defaultPD(me, me.local(delta_xy), up=sign(math.sin(time)) * (-1 if me.index == 60 else 1) * (Vector() - me.location).flatten().normalize())

        me.ctrl.throttle = 1
        # only boost/throttle if we're facing the right direction
        if abs(me.forward.angle(delta_xy)) < 1 and (not self.jumping or not self.jump_type_fast):
            # tap boost to keep height
            if delta_x.z - me.velocity.z * 0.5 > 0:
                me.ctrl.boost = True

            # if the target is relatively far, hold boost even when we're higher than the target to keep moving
            if delta_x.z < 0 and me.forward.z < 0.5:
                me.ctrl.boost = True
