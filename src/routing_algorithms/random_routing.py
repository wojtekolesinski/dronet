from numpy.random import RandomState

import config
from routing_algorithms.BASE_routing import BASE_routing


class RandomRouting(BASE_routing):

    def __init__(self, drone):
        BASE_routing.__init__(self, drone)
        self.random = RandomState(config.seed)

    def relay_selection(self, packet):
        """
        This function returns a random relay for packets.

        @param opt_neighbors: a list of tuples (hello_packet, drone)
        @return: a random drone as relay
        """
        return self.filter_neighbours_for_packet(packet).pop()
