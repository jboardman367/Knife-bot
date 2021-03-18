from rlbot.agents.base_agent import BaseAgent, SimpleControllerState
from rlbot.messages.flat.QuickChatSelection import QuickChatSelection
from rlbot.utils.structures.game_data_struct import GameTickPacket
from rlbot.utils.game_state_util import GameState, BallState, CarState, Physics, Vector3, Rotator, GameInfoState, BoostState

from util.ball_prediction_analysis import find_slice_at_time
from util.boost_pad_tracker import BoostPadTracker
from util.sequence import Sequence, ControlStep
from util.vec import Vec3, Line, Arc
from util.package import Package
from util.object_models import Car, Ball
from random import randint
import math

#temporary
from util.action import *


class MyBot(BaseAgent):

    def __init__(self, name, team, index):
        super().__init__(name, team, index)
        self.active_sequence: Sequence = None
        self.boost_pad_tracker = BoostPadTracker()
        self.ball = None
        self.me = None
        self.allies = []
        self.foes = []
        self.ball_prediction = None
        self.current_intent = None

        #temporary
        self.test_line = None

    def initialize_agent(self):
        # Set up information about the boost pads now that the game is active and the info is available
        self.boost_pad_tracker.initialize_boosts(self.get_field_info())

    def get_output(self, packet: GameTickPacket) -> SimpleControllerState:
        """
        This function will be called by the framework many times per second. This is where you can
        see the motion of the ball, etc. and return controls to drive your car.
        """

        # Keep our boost pad info updated with which pads are currently active
        self.boost_pad_tracker.update_boost_status(packet)

        #update cars and ball
        self.update_objects(packet)

        # Create package of all useful info to be handed around as one object
        package = Package(packet, self.me, self.ball, self.ball_prediction, self.allies, self.foes, self.renderer)

        return self.blocking_debug(package)

        #check current intent
        if self.current_intent is not None:
            if self.current_intent.check(package):
                return self.current_intent.get_controls(package) #An existing and valid intent returns controls

        #At this point, self.current_intent is None or invalid, so find a new one
        #This will be a decision consisting of 2 layers: car-ball-goal, other players
        self.find_intent(package)

        #return the first controls of the new intent
        if self.current_intent is not None:
            return self.current_intent.get_controls(package)

        #Make sure that the method always returns
        return SimpleControllerState()



    def update_objects(self, packet):
        # ball
        if self.ball is None:
            self.ball = Ball(packet)
        else:
            self.ball.update(packet)

        self.ball_prediction = self.get_ball_prediction_struct()
        # draw 3 sec of path
        self.renderer.draw_polyline_3d(
            [Vec3(ball_slice.physics.location) for ball_slice in self.ball_prediction.slices[:180:5]],
            self.renderer.yellow())

        # self
        if self.me is None:
            self.me = Car(self.index, packet)
        elif self.me.index != self.index:
            self.me = Car(self.index, packet)
        else:
            self.me.update(packet)

        # check if number of players has changed, and reset allies and foes if it has
        if len(self.allies) + len(self.foes) + 1 != len(packet.game_cars):
            self.allies, self.foes = [], []

        # allies
        if len(self.allies) == 0:
            for index in range(packet.num_cars):
                if packet.game_cars[index].team == self.me.team and index != self.me.index:
                    self.allies.append(Car(index, packet))
        else:
            for car in self.allies:
                car.update(packet)

        # foes
        if len(self.foes) == 0:
            for index in range(packet.num_cars):
                if packet.game_cars[index].team != self.me.team:
                    self.foes.append(Car(index, packet))
        else:
            for car in self.foes:
                car.update(packet)

    def blocking_debug(self, package) -> SimpleControllerState:
        if self.test_line is None or package.packet.game_info.seconds_elapsed%3<=0.01:
            self.test_line = Line(
                Vec3(randint(-500,500),-4000,100),
                Vec3(randint(-20,20),50,0)
            )
            self.set_game_state(
                GameState(
                    cars = {
                        0:CarState(
                            physics = Physics(
                                location = Vector3(
                                    x=0,
                                    y=-4000,
                                    z=17
                                ),
                                rotation = Rotator(
                                    0,
                                    math.pi/2,
                                    0
                                ),
                                velocity = Vector3(
                                    x=0,
                                    y=1400,
                                    z=0
                                ),
                                angular_velocity = Vector3(
                                    x=0,
                                    y=0,
                                    z=0
                                )
                            )
                        )
                    },
                    ball = BallState(
                        Physics(
                            velocity = Vector3(
                                x=0,
                                y=0,
                                z=0
                            ),
                            location = Vector3(
                                x=3000,
                                y=-5000,
                                z=98
                            ),
                            angular_velocity = Vector3(
                                x=0,
                                y=0,
                                z=0
                            )
                        )
                    )
                )
            )
        #self.test_line = Line(
        #    package.ball.location,
        #    package.ball.location - package.me.location
        #)
        #positive side is orange
        self.test_line.render(package.renderer)
        return an_actual_pd(package, self.test_line, package.me, target_vel=1400)

    def find_intent(self, package):
        # gather variables to use in the decisions

        # go through the tree
        pass