import numpy as np

import config
from entities.depot import Depot


class Environment:
    """The environment is an entity that represents the area of interest on which events are generated.
    WARNING this corresponds to an old view we had, according to which the events are generated on the map at
    random and then maybe felt from the drones. Now events are generated on the drones that they feel with
    a certain probability."""

    def __init__(self, width, height):
        self.depot = None
        self.drones = None
        self.width = width
        self.height = height

        self.event_generator = EventGenerator(height, width)
        self.active_events = []

    def add_drones(self, drones: list):
        """add a list of drones in the env"""
        self.drones = drones

    def add_depot(self, depot: Depot):
        """add depot in the env"""
        self.depot = depot


class EventGenerator:

    def __init__(self, height, width):
        """uniform event generator"""
        self.height = height
        self.width = width
        self.rnd_env = np.random.RandomState(config.seed)

    def uniform_event_generator(self):
        """generates an event in the map"""
        x = self.rnd_env.randint(0, self.height)
        y = self.rnd_env.randint(0, self.width)
        return x, y

    def poisson_event_generator(self):
        """generates an event in the map"""
        pass
