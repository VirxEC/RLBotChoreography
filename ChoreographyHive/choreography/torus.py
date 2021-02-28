import math
from dataclasses import dataclass
from typing import List, Tuple

from RLUtilities.GameInfo import GameInfo
from RLUtilities.Maneuvers import vec3, Aerial
from rlbot.agents.base_agent import SimpleControllerState
from rlbot.messages.flat.GameTickPacket import GameTickPacket
from rlbot.utils.game_state_util import GameState, CarState, Physics, Vector3, Rotator, BallState
from rlbot.utils.structures.game_interface import GameInterface

from choreography.choreography import Choreography
from choreography.common.preparation import LetAllCarsSpawn, HideBall
from choreography.drone import Drone, slow_to_pos
from choreography.group_step import BlindBehaviorStep, DroneListStep, StepResult, GroupStep, SubGroupChoreography, \
    SubGroupOrchestrator
from util.vec import Vec3


@dataclass
class Breadcrumb:
    position: Vec3
    velocity: Vec3
    time_index: int
    has_wheel_contact: bool


def get_time_index(packet):
    seconds_elapsed = packet.game_info.seconds_elapsed
    return int(seconds_elapsed * 4)  # Every quarter second


def stagger(n):
    return (n % 2) * 2 - 1


class Piecewise:
    def __init__(self, points: List[Tuple[float, float]]):
        self.points = points

    def lerp(self, x: float):
        if self.points[0][0] > x:
            return self.points[0][1]
        if self.points[-1][0] < x:
            return self.points[-1][1]

        for i in range(0, len(self.points) - 1):
            a = self.points[i]
            b = self.points[i + 1]
            if x > b[0]:
                continue
            span = b[0] - a[0]
            to_x = x - a[0]
            progress = to_x / span
            vertical_span = b[1] - a[1]
            return a[1] + vertical_span * progress


STANDARD_RADIUS = 4000
RADIUS_SPEED = Piecewise([(200, 2.0), (500, 1.2), (1400, 0.7), (2500, 0.5), (4000, 0.5)])

GROUND_PROCESSION_RATE = 0.5
TORUS_RATE = 0.1

def circular_procession(packet, drones, start_time) -> StepResult:
    """
    Makes all cars drive in a slowly shrinking circle.
    https://gfycat.com/yearlygreathermitcrab
    """
    radian_spacing = 2 * math.pi / len(drones)
    elapsed = packet.game_info.seconds_elapsed - start_time
    radius = 600 + elapsed * 25
    for i, drone in enumerate(drones):
        progress = i * radian_spacing + elapsed * GROUND_PROCESSION_RATE
        target = [radius * math.sin(progress), radius * math.cos(progress), 0]
        slow_to_pos(drone, target)
        drone.ctrl.boost = False
        drone.ctrl.throttle = min(drone.ctrl.throttle, 0.3)
    return StepResult(finished=elapsed * GROUND_PROCESSION_RATE > 10 * math.pi)


def arrange_in_ground_circle(drones: List[Drone], game_interface: GameInterface, radius: float, radian_offset: float):
    if len(drones) == 0:
        return

    car_states = {}
    radian_spacing = 2 * math.pi / len(drones)

    for index, drone in enumerate(drones):
        progress = index * radian_spacing + radian_offset
        target = Vec3(radius * math.sin(progress), radius * math.cos(progress), 1000)

        car_states[drone.index] = CarState(
            Physics(location=Vector3(target.x, target.y, 50),
                    velocity=Vector3(0, 0, 0),
                    rotation=Rotator(0, -progress, 0)))
    game_interface.set_game_state(GameState(cars=car_states))


class TorusSubChoreography(SubGroupChoreography):
    def __init__(self, game_interface: GameInterface, game_info: GameInfo, time_offset: float, drones: List[Drone],
                 start_time: float):
        super().__init__(drones, start_time)
        self.game_interface = game_interface
        self.renderer = self.game_interface.renderer
        self.game_info = game_info
        self.time_offset = time_offset
        self.aerials: List[Aerial] = []
        self.angular_progress: List[float] = []
        self.previous_seconds_elapsed = 0

    def generate_sequence(self, drones: List[Drone]):
        self.aerials = []
        self.angular_progress = []

        if len(drones) == 0:
            return

        # self.sequence.append(DroneListStep(self.arrange_in_ground_circle, self.time_offset))
        self.sequence.append(DroneListStep(circular_procession, self.time_offset))
        self.sequence.append(DroneListStep(self.torus_flight_pattern))

    def arrange_in_ground_circle(self, packet, drones, start_time) -> StepResult:
        elapsed = packet.game_info.seconds_elapsed - start_time
        arrange_in_ground_circle(drones, self.game_interface, 3000, elapsed * GROUND_PROCESSION_RATE)
        return StepResult(finished=True)

    def torus_flight_pattern(self, packet, drones: List[Drone], start_time) -> StepResult:

        # self.renderer.begin_rendering(drones[0].index)
        radian_spacing = 2 * math.pi / len(drones)

        if len(self.aerials) == 0:
            for index, drone in enumerate(drones):
                self.aerials.append(Aerial(self.game_info.cars[drone.index], vec3(0, 0, 0), 0))
                self.angular_progress.append(index * radian_spacing)

        elapsed = packet.game_info.seconds_elapsed - start_time
        # radius = 4000 - elapsed * 100
        if self.previous_seconds_elapsed == 0:
            time_delta = 0
        else:
            time_delta = packet.game_info.seconds_elapsed - self.previous_seconds_elapsed

        radius = 900 * (1 + math.cos(elapsed * TORUS_RATE)) + 300
        height = 450 * (1 + math.sin(elapsed * TORUS_RATE)) + 800

        # self.renderer.draw_string_2d(10, 10, 2, 2, f"r {radius}", self.renderer.white())
        # self.renderer.draw_string_2d(10, 30, 2, 2, f"z {drones[0].pos[2]}", self.renderer.white())

        for index, drone in enumerate(drones):
            if "sniped" in drone.attributes:
                continue

            # This function was fit from the following data points, where I experimentally found deltas which
            # worked well with sampled radius values.
            # {2500, 0.7}, {1000, 0.9}, {500, 1.2}, {200, 2.0}
            # Originally was 2476 / (radius + 1038)
            # angular_delta = time_delta * (800 / radius + 0.2)
            angular_delta = time_delta * RADIUS_SPEED.lerp(radius)
            self.angular_progress[index] += angular_delta
            aerial = self.aerials[index]
            progress = self.angular_progress[index]
            target = Vec3(radius * math.sin(progress), radius * math.cos(progress), height)
            to_target = target - Vec3(drone.pos[0], drone.pos[1], drone.pos[2])
            # self.renderer.draw_line_3d(drone.pos, target, self.renderer.yellow())
            aerial.target = vec3(target.x, target.y, target.z)
            aerial.t_arrival = drone.time + to_target.length() / 2000 + 0.3
            aerial.step(0.008)
            drone.ctrl.boost = aerial.controls.boost
            drone.ctrl.pitch = aerial.controls.pitch
            drone.ctrl.yaw = aerial.controls.yaw
            drone.ctrl.roll = aerial.controls.roll
            drone.ctrl.jump = aerial.controls.jump

        self.previous_seconds_elapsed = packet.game_info.seconds_elapsed
        # self.renderer.end_rendering()

        return StepResult(finished=elapsed > 160)


class TorusChoreography(Choreography):

    def __init__(self, game_interface: GameInterface):
        super().__init__()
        self.game_interface = game_interface

        self.game_info = GameInfo(0, 0)
        self.renderer = self.game_interface.renderer
        self.drones_per_ring = 8

    def pre_step(self, packet: GameTickPacket, drones: List[Drone]):
        self.game_info.read_packet(packet)

    @staticmethod
    def get_num_bots():
        return 48

    def generate_sequence(self, drones):
        self.sequence.clear()

        if len(drones) == 0:
            return

        pause_time = 0.2

        self.sequence.append(HideBall())
        self.sequence.append(LetAllCarsSpawn(self.get_num_bots()))
        self.sequence.append(BlindBehaviorStep(SimpleControllerState(), pause_time))

        num_rings = len(drones) // self.drones_per_ring
        torus_period = 2 * math.pi / TORUS_RATE
        self.sequence.append(SubGroupOrchestrator(group_list=[
            TorusSubChoreography(self.game_interface, self.game_info, -i * torus_period / num_rings,
                                 drones[i * self.drones_per_ring:(i + 1) * self.drones_per_ring], 0)
            for i in range(0, num_rings)
        ]))