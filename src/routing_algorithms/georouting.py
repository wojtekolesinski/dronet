import config
from entities.packets import Packet
from routing_algorithms.base import BaseRouting
from utilities.types import NetAddr, Point
from utilities.utilities import euclidean_distance


class GeoRouting(BaseRouting):

    def __init__(self, drone):
        BaseRouting.__init__(self, drone)

    def get_position_estimate(
        self, start: Point, target: Point, speed: int, time_diff: int
    ) -> Point:
        def f(x: float) -> float:
            try:
                a = (target[1] - start[1]) / (target[0] - start[0])
            except:
                a = target[1] - start[1]
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

    def relay_selection(self, packet: Packet) -> NetAddr | None:
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

        best_distance = euclidean_distance(cur_pos, dst_pos)
        best_drone = None
        neighbours = self.neighbours
        for neighbour in neighbours.values():
            neighbor_pos = neighbour.coords
            neighbor_distance_to_depot = euclidean_distance(neighbor_pos, dst_pos)
            if neighbor_distance_to_depot < best_distance:
                best_drone = neighbour
                best_distance = neighbor_distance_to_depot

        if best_drone is None:
            return
        return best_drone.address
