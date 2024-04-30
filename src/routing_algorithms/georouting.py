from entities.packets import DataPacket, HelloPacket, Packet
from routing_algorithms.BASE_routing import BASE_routing, NeighbourNode
from utilities.utilities import euclidean_distance


class GeoRouting(BASE_routing):

    def __init__(self, drone):
        BASE_routing.__init__(self, drone)

    def relay_selection(self, packet: DataPacket) -> NeighbourNode | None:
        """
        This function returns a relay for packets according to geographic routing.

        @return: The best drone to use as relay or None if no relay is selected
        """

        cur_pos = self.drone.coords
        depot_pos = self.drone.depot.coords
        # my_distance_to_depot = util.euclidean_distance(cur_pos, depot_pos)

        best_distance = euclidean_distance(cur_pos, depot_pos)
        best_drone = None
        neighbours = self.filter_neighbours_for_packet(packet)
        for neighbour in neighbours.values():
            neighbor_pos = neighbour.coords
            neighbor_distance_to_depot = euclidean_distance(neighbor_pos, depot_pos)
            if neighbor_distance_to_depot < best_distance:
                best_drone = neighbour
                best_distance = neighbor_distance_to_depot

        return best_drone
