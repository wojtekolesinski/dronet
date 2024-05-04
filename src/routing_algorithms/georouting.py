import config
from entities.packets import DataPacket, HelloPacket, Packet
from routing_algorithms.base import BaseRouting, NeighbourNode
from utilities.types import Point
from utilities.utilities import euclidean_distance


class GeoRouting(BaseRouting):

    def __init__(self, drone):
        BaseRouting.__init__(self, drone)

    def get_position_estimate(
        self, start: Point, target: Point, speed: int, time_diff: int
    ) -> Point:
        def f(x: float) -> float:
            a = (target[1] - start[1]) / (target[0] - start[0])
            b = start[1] - a * start[0]
            return a * x + b

        distance = euclidean_distance(start, target)
        time_to_travel_distance = distance / speed
        if time_diff >= time_to_travel_distance:
            return target

        time_passed_fraction = time_diff / time_to_travel_distance
        assert time_passed_fraction < 1

        current_x = start[0] + (target[0] - start[0]) * time_passed_fraction
        return current_x, f(current_x)

    def relay_selection(self, packet: Packet) -> NeighbourNode | None:
        """
        This function returns a relay for packets according to geographic routing.

        @return: The best drone to use as relay or None if no relay is selected
        """

        cur_pos = self.drone.coords
        if packet.dst == config.DEPOT_ADDRESS:
            dst_pos = config.depot_coordinates
        elif dst := self.neighbours.get(packet.dst, None):
            dst_pos = self.get_position_estimate(
                dst.coords, dst.next_target, dst.speed, self.drone.time - dst.timestamp
            )
        else:
            dst_pos = packet.event_ref.coords

        # my_distance_to_depot = util.euclidean_distance(cur_pos, depot_pos)

        best_distance = euclidean_distance(cur_pos, dst_pos)
        best_drone = None
        neighbours = self.filter_neighbours_for_packet(packet)
        for neighbour in neighbours.values():
            neighbor_pos = neighbour.coords
            neighbor_distance_to_depot = euclidean_distance(neighbor_pos, dst_pos)
            if neighbor_distance_to_depot < best_distance:
                best_drone = neighbour
                best_distance = neighbor_distance_to_depot

        return best_drone
