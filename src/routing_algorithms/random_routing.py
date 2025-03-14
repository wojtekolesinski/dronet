from numpy.random import RandomState

import config
from routing_algorithms.base import BaseRouting


class RandomRouting(BaseRouting):

    def __init__(self, drone):
        BaseRouting.__init__(self, drone)
        self.random = RandomState(config.seed)

    def relay_selection(self, packet):
        """
        This function returns a random relay for packets.

        @param opt_neighbors: a list of tuples (hello_packet, drone)
        @return: a random drone as relay
        """
        return self.random.choice(list(self.neighbours.values()))
