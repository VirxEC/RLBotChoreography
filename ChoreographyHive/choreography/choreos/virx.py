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


class CrossingSquares(Choreography):
    """
    A simple choreography where two squares of bots cross. Requires 32 bots.
    """

    def __init__(self, game_interface: GameInterface):
        super().__init__()
        self.game_interface = game_interface

    def generate_sequence(self, drones: List[Drone]):
        self.sequence.clear()

        self.sequence.append(BlindBehaviorStep(SimpleControllerState(), 1))
        self.sequence.append(DroneListStep(self.hide_ball))

    @staticmethod
    def get_num_bots() -> int:
        return 32

    def hide_ball(self, packet, drones, start_time) -> StepResult:
        """
        Places the ball above the roof of the arena to keep it out of the way.
        """
        self.game_interface.set_game_state(GameState(ball=BallState(physics=Physics(
            location=Vector3(30000, 30000, 30000),
            velocity=Vector3(0, 0, 0),
            angular_velocity=Vector3(0, 0, 0)))))
        return StepResult(finished=True)
