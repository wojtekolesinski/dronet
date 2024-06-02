import config
from entities.communicating_entity import CommunicatingEntity
from entities.depot import Depot
from entities.event import Event
from entities.packets import Packet
from simulation.metrics import Metrics
from simulation.net import MediumDispatcher
from utilities import utilities
from utilities.types import NetAddr, Path, Point


class Drone(CommunicatingEntity):
    depot: Depot
    path: Path

    def __init__(
        self,
        identifier: int,
        address: NetAddr,
        network: MediumDispatcher,
        path: Path,
        depot: Depot,
    ):
        super().__init__(
            identifier,
            path[0],
            address,
            network,
            config.drone_sensing_range,
            config.drone_communication_range,
            config.drone_max_buffer_size,
            config.drone_speed,
        )
        self.depot = depot
        self.path = path
        self.residual_energy = config.drone_max_energy

        self.current_waypoint = 0

    def consume_packet(self, packet: Packet):
        self.router.process(packet)

    def update_packets(self):
        """
        Removes the expired packets from the buffer
        """
        to_drop = []
        for pck in self.buffer:
            if not pck.is_expired(self.time):
                continue

            to_drop.append(pck.identifier)

        self.remove_packets(to_drop)

    def feel_event(self, cur_step: int):
        """
        feel a new event, and adds the packet relative to it, in its buffer.
            if the drones is doing movement the packet is not added in the buffer
        """
        ev = Event(self.coords, cur_step)  # the event
        packet = self.router.make_data_packet(ev, cur_step)
        self.buffer.append(packet)
        print(f"NEW PACKET WITH ID {packet.identifier}")
        Metrics.instance().all_data_packets_in_simulation += 1

    def routing(self):
        """do the routing"""
        packets = self.router.routing_control(self.time)
        self.output_buffer.extend(packets)

        routed_packets = []
        if self.router.has_neighbours():
            for packet in self.all_packets():
                if p := self.router.route_packet(packet):
                    routed_packets.append(p)
        self.output_buffer.extend(routed_packets)

    def move(self, time: int):
        """Move the drone to the next point
        time -> time_step_duration (how much time between two simulation frame)
        """
        if self.current_waypoint >= len(self.path) - 1:
            self.current_waypoint = -1

        p0 = self.coords
        p1 = self.path[self.current_waypoint + 1]

        all_distance = utilities.euclidean_distance(p0, p1)
        distance = time * self.speed
        if all_distance == 0 or distance == 0:
            self.__update_position(p1)
            return

        t = distance / all_distance
        if t >= 1:
            self.__update_position(p1)
        elif t <= 0:
            print("Error move drone, ratio < 0")
            exit(1)
        else:
            self.coords = (((1 - t) * p0[0] + t * p1[0]), ((1 - t) * p0[1] + t * p1[1]))

    def next_target(self) -> Point:
        # reached the end of the path, start back to 0
        if self.current_waypoint >= len(self.path) - 1:
            return self.path[0]

        return self.path[self.current_waypoint + 1]

    def next_move_to_mission_point(self):
        """get the next future position of the drones, according the mission"""
        current_waypoint = self.current_waypoint
        if current_waypoint >= len(self.path) - 1:
            current_waypoint = -1

        p0 = self.coords
        p1 = self.path[current_waypoint + 1]
        all_distance = utilities.euclidean_distance(p0, p1)
        distance = config.time_step_duration * self.speed
        if all_distance == 0 or distance == 0:
            return self.path[current_waypoint]

        t = distance / all_distance
        if t >= 1:
            return self.path[current_waypoint]
        elif t <= 0:
            print("Error move drone, ratio < 0")
            exit(1)
        else:
            return ((1 - t) * p0[0] + t * p1[0]), ((1 - t) * p0[1] + t * p1[1])

    def __update_position(self, p1: Point):
        self.current_waypoint += 1
        self.coords = self.path[self.current_waypoint]

    def __repr__(self):
        return "Drone " + str(self.identifier)

    def __hash__(self):
        return hash(self.identifier)
