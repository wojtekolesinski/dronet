from enum import Enum

from routing_algorithms.aodv import AODVRouting
from routing_algorithms.georouting import GeoRouting
from routing_algorithms.olsr import OLSRRouting
from routing_algorithms.q_learning_routing import QLearningRouting
from routing_algorithms.random_routing import RandomRouting


class RoutingAlgorithm(Enum):
    GEO = GeoRouting
    RND = RandomRouting
    QL = QLearningRouting
    OLSR = OLSRRouting
    AODV = AODVRouting

    @staticmethod
    def keylist():
        return list(map(lambda c: c.name, RoutingAlgorithm))


class ChannelError(Enum):
    UNIFORM = 1
    GAUSSIAN = 2
    NO_ERROR = 3

    @staticmethod
    def keylist():
        return list(map(lambda c: c.name, ChannelError))
