from typing import List

import numpy as np
from choreography.choreography import Choreography
from choreography.drone import Drone, slow_to_pos
from choreography.group_step import (BlindBehaviorStep, DroneListStep,
                                     PerDroneStep, StepResult)
from rlbot.agents.base_agent import SimpleControllerState
from rlbot.utils.game_state_util import (BallState, CarState, GameInfoState,
                                         GameState, Physics, Rotator, Vector3)
from rlbot.utils.structures.game_interface import GameInterface


class TheSky(Choreography):
    """
    For seconds 0-14 of The Sky
    """

    def __init__(self, game_interface: GameInterface):
        super().__init__()
        self.game_interface = game_interface

    def generate_sequence(self, drones: List[Drone]):
        self.sequence.clear()

        pause_time = 1.5

        self.sequence.append(DroneListStep(self.setup))
        self.sequence.append(DroneListStep(self.make_square))
        self.sequence.append(BlindBehaviorStep(SimpleControllerState(), pause_time))
        self.sequence.append(PerDroneStep(self.wave_jump_front_to_back, 3))
        self.sequence.append(BlindBehaviorStep(SimpleControllerState(), pause_time))
        self.sequence.append(DroneListStep(self.end_choreo))

    @staticmethod
    def get_num_bots() -> int:
        return 64

    def end_choreo(self, packet, drones, start_time) -> StepResult:
        self.game_interface.set_game_state(GameState(ball=BallState(Physics(location=Vector3(0, 5300, 400)))))
        return StepResult(finished=True)

    def interweave(self, packet, drones, start_time) -> StepResult:
        """
        Make bots jump alternating such that they jump over each other.
        """
        elapsed = packet.game_info.seconds_elapsed - start_time
        start = 0
        hold = 0.135
        buffer = 0.53

        car_states = {}
        for drone in drones:
            # Speed controller
            vel = np.linalg.norm(drone.vel*np.array([1,1,0]))
            drone.ctrl.throttle = 0 if vel > 850 else 1

            # jump controller
            if (drone.index % 2 == 0):
                if start < elapsed < start+hold:
                    drone.ctrl.jump = True
                elif start+2*buffer < elapsed < start+2*buffer+hold:
                    drone.ctrl.jump = True
                elif start+4*buffer < elapsed < start+4*buffer+hold:
                    drone.ctrl.jump = True
                elif start+6*buffer < elapsed < start+6*buffer+hold:
                    drone.ctrl.jump = True
            else:
                if start+buffer < elapsed < start+buffer+hold:
                    drone.ctrl.jump = True
                elif start+3*buffer < elapsed < start+3*buffer+hold:
                    drone.ctrl.jump = True
                elif start+5*buffer < elapsed < start+5*buffer+hold:
                    drone.ctrl.jump = True

        self.game_interface.set_game_state(GameState(cars=car_states))
        return StepResult(finished=elapsed > start+8*buffer)

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


    def make_squares(self, packet, drones, start_time) -> StepResult:
        """
        Separates all the bots into two squares, facing each other.
        """
        self.squareA = drones[:32]
        self.squareB = drones[32:]

        spacing = 250
        y_offset = 3500
        x_offset = 7 * spacing / 2

        car_states = {}
        for i, drone in enumerate(self.squareA):
            car_states[drone.index] = CarState(
                Physics(location=Vector3(x_offset - spacing*(i % 8), -y_offset - spacing*(i // 8), 20),
                        velocity=Vector3(0, 0, 0),
                        rotation=Rotator(0, np.pi/2, 0)))

        for i, drone in enumerate(self.squareB):
            car_states[drone.index] = CarState(
                Physics(location=Vector3(-x_offset + spacing*(i % 8), y_offset + spacing*(i // 8), 20),
                        velocity=Vector3(0, 0, 0),
                        rotation=Rotator(0, -np.pi/2, 0)))

        self.game_interface.set_game_state(GameState(cars=car_states))
        return StepResult(finished=True)


    def delayed_start(self, packet, drones, start_time) -> StepResult:
        """
        Spreads bots out by delaying the start of each row.
        """
        elapsed = packet.game_info.seconds_elapsed - start_time

        for drone in drones:
            throttle_start = (drone.index % 32 // 8) * 0.85
            drone.ctrl = SimpleControllerState()

            if throttle_start < elapsed:
                # Speed controller.
                vel = np.linalg.norm(drone.vel*np.array([1,1,0]))
                drone.ctrl.throttle = 0 if vel > 800 else 0.7

        return StepResult(finished=elapsed > 4.5)


    def make_square(self, packet, drones, start_time) -> StepResult:
        """
        Gathers the bots in a 16 by 4 rectangle.
        """

        x_spacing = 250
        x_offset = 3 * x_spacing
        y_spacing = 140
        y_offset = 3 * y_spacing / 2

        car_states = {}
        for i, drone in enumerate(drones):
            car_states[drone.index] = CarState(
                Physics(location=Vector3(x_offset - x_spacing*(i % 16), y_offset - y_spacing*(i // 16), 20),
                        velocity=Vector3(0, 0, 0),
                        rotation=Rotator(0, 0, 0)))

        self.game_interface.set_game_state(GameState(cars=car_states))
        return StepResult(finished=True)

    def wave_jump_left(self, packet, drone, start_time) -> StepResult:
        """
        Makes all cars jump in sequence, "doing the wave" if they happen to be lined up.
        https://gfycat.com/remorsefulsillyichthyosaurs
        """
        elapsed = packet.game_info.seconds_elapsed - start_time
        jump_start = drone.index // 4 * 0.03
        jump_end = jump_start + 0.05
        drone.ctrl = SimpleControllerState(jump=jump_start < elapsed < jump_end)
        wheel_contact = packet.game_cars[drone.index].has_wheel_contact
        return StepResult(finished=elapsed > jump_end and wheel_contact)

    def wave_jump_right(self, packet, drone, start_time) -> StepResult:
        """
        Makes all cars jump in sequence, "doing the wave" if they happen to be lined up.
        https://gfycat.com/remorsefulsillyichthyosaurs
        """
        elapsed = packet.game_info.seconds_elapsed - start_time
        jump_start = (self.get_num_bots() - drone.index) // 4 * 0.03
        jump_end = jump_start + 0.05
        drone.ctrl = SimpleControllerState(jump=jump_start < elapsed < jump_end)
        wheel_contact = packet.game_cars[drone.index].has_wheel_contact
        return StepResult(finished=elapsed > jump_end and wheel_contact)

    def wave_jump_front_to_back(self, packet, drone, start_time) -> StepResult:
        """
        Makes all cars jump in sequence, "doing the wave" if they happen to be lined up.
        https://gfycat.com/remorsefulsillyichthyosaurs
        """
        elapsed = packet.game_info.seconds_elapsed - start_time
        jump_start = drone.index % 16 * 0.03
        jump_end = jump_start + 0.2
        drone.ctrl = SimpleControllerState(jump=jump_start < elapsed < jump_end)
        wheel_contact = packet.game_cars[drone.index].has_wheel_contact
        return StepResult(finished=elapsed > jump_end and wheel_contact)

    def setup(self, packet, drones, start_time) -> StepResult:
        """
        Places the ball above the roof of the arena to keep it out of the way.
        """
        self.game_interface.set_game_state(GameState(game_info=GameInfoState(world_gravity_z=-1300), ball=BallState(physics=Physics(
            location=Vector3(0, 0, 3000),
            velocity=Vector3(0, 0, 0),
            angular_velocity=Vector3(0, 0, 0)))))
        return StepResult(finished=True)
