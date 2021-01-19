from typing import List

import numpy as np
from choreography.choreography import Choreography
from choreography.drone import Drone, slow_to_pos
from choreography.group_step import (BlindBehaviorStep, DroneListStep,
                                     PerDroneStep, StepResult)
from rlbot.agents.base_agent import SimpleControllerState
from rlbot.utils.game_state_util import (BallState, CarState, GameState,
                                         Physics, Rotator, Vector3)
from rlbot.utils.structures.game_interface import GameInterface


class TheSky(Choreography):
    """
    A simple choreography where two squares of bots cross. Requires 64 bots.
    """

    def __init__(self, game_interface: GameInterface):
        super().__init__()
        self.game_interface = game_interface

    def generate_sequence(self, drones: List[Drone]):
        self.sequence.clear()

        pause_time = 1.5

        self.sequence.append(DroneListStep(self.hide_ball))
        self.sequence.append(DroneListStep(self.make_square))
        self.sequence.append(BlindBehaviorStep(SimpleControllerState(), pause_time))

    @staticmethod
    def get_num_bots() -> int:
        return 64

    def line_up(self, packet, drones, start_time) -> StepResult:
        """
        Puts all the cars in a tidy line, very close together.
        """
        start_x = -2000
        y_increment = 100
        start_y = -len(drones) * y_increment / 2
        start_z = 40
        car_states = {}
        for drone in drones:
            car_states[drone.index] = CarState(
                Physics(location=Vector3(start_x, start_y + drone.index * y_increment, start_z),
                        velocity=Vector3(0, 0, 0),
                        rotation=Rotator(0, 0, 0)))
        self.game_interface.set_game_state(GameState(cars=car_states))
        return StepResult(finished=True)


    def make_square(self, packet, drones, start_time) -> StepResult:
        """
        Gathers the bots in a 4 by 16 rectangle.
        """

        spacing = 250
        y_offset = 2550
        x_offset = 3 * spacing / 2

        car_states = {}
        for i, drone in drones:
            car_states[drone.index] = CarState(
                Physics(location=Vector3(x_offset - spacing*(i % 4), -y_offset - spacing*(i // 16), 20),
                        velocity=Vector3(0, 0, 0),
                        rotation=Rotator(0, 0, 0)))

        self.game_interface.set_game_state(GameState(cars=car_states))
        return StepResult(finished=True)

    def wave_jump(self, packet, drone, start_time) -> StepResult:
        """
        Makes all cars jump in sequence, "doing the wave" if they happen to be lined up.
        https://gfycat.com/remorsefulsillyichthyosaurs
        """
        elapsed = packet.game_info.seconds_elapsed - start_time
        jump_start = drone.index / 4 * 0.06
        jump_end = jump_start + .5
        drone.ctrl = SimpleControllerState(jump=jump_start < elapsed < jump_end)
        wheel_contact = packet.game_cars[drone.index].has_wheel_contact
        return StepResult(finished=elapsed > jump_end and wheel_contact)

    def hide_ball(self, packet, drones, start_time) -> StepResult:
        """
        Places the ball above the roof of the arena to keep it out of the way.
        """
        self.game_interface.set_game_state(GameState(ball=BallState(physics=Physics(
            location=Vector3(30000, 30000, 30000),
            velocity=Vector3(0, 0, 0),
            angular_velocity=Vector3(0, 0, 0)))))
        return StepResult(finished=True)
