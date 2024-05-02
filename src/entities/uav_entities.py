import numpy as np

import config
from entities.communicating_entity import CommunicatingEntity
from entities.depot import Depot
from entities.event import Event
from entities.packets import ACKPacket, DataPacket, HelloPacket, Packet
from simulation.metrics import Metrics
from simulation.net import MediumDispatcher
from utilities import utilities
from utilities.types import NetAddr, Path, Point


class Drone(CommunicatingEntity):

    last_mission_coords: Point

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

        # TODO: cleanup the parameters
        self.depot = depot
        self.path: Path = path
        self.residual_energy = config.drone_max_energy

        # dynamic parameters
        # used later to check if there is an event that is about to expire
        self.current_waypoint = 0

        # setup drone routing algorithm
        self.router = config.routing_algorithm.value(self)

    def consume_packet(self, packet: Packet):
        # TODO: the drone should not acknowledge all of the incoming data packets,
        # but it should relay these packets, if it is not the destination
        if isinstance(packet, HelloPacket):
            self.router.handle_hello(packet)
            return

        if packet.dst != self.address:
            self.retransmission_buffer.add(packet)
            # if len(self.retransmission_buffer) > 1:
            # print(self.retransmission_buffer)
            return

        elif isinstance(packet, DataPacket):
            self.acknowledge_packet(packet)
            self.buffer.append(packet)

        elif isinstance(packet, ACKPacket):
            self.remove_packets([packet.acked_packet_id])
            if self.buffer_length() == 0:
                self.router.retransmission_count = 0

    def update_packets(self):
        """
        Removes the expired packets from the buffer

        @param cur_step: Integer representing the current time step
        @return:
        """
        to_drop = []
        for pck in self.buffer:
            if not pck.is_expired(self.time):
                continue

            to_drop.append(pck.identifier)
            if config.routing_algorithm.name not in "GEO" "RND" "GEOS":
                feedback = -1
                current_drone = self

                # for drone in self.simulator.drones:
                #     drone.routing_algorithm.feedback(
                #         current_drone,
                #         pck.event_ref.identifier,
                #         config.event_duration,
                #         feedback,
                #     )
        self.remove_packets(to_drop)

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

    def feel_event(self, cur_step: int):
        """
        feel a new event, and adds the packet relative to it, in its buffer.
            if the drones is doing movement the packet is not added in the buffer
        """
        ev = Event(self.coords, cur_step)  # the event
        pk = DataPacket(self.address, self.depot.address, cur_step, ev)
        self.buffer.append(pk)
        print(f"NEW PACKET WITH ID {pk.identifier}")
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
        """Move the drone to the next point if self.move_to_depot is false, else it moves towards the depot.

        time -> time_step_duration (how much time between two simulation frame)
        """
        self.__move_to_mission(time)

        # metrics: number of time steps on mission, incremented each time drone is doing sensing mission
        Metrics.instance().time_on_mission += 1

    def next_target(self) -> Point:
        # reached the end of the path, start back to 0
        if self.current_waypoint >= len(self.path) - 1:
            return self.path[0]

        return self.path[self.current_waypoint + 1]

    def __move_to_mission(self, time: int):
        """When invoked the drone moves on the map. TODO: Add comments and clean.
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

    def __update_position(self, p1: Point):
        self.current_waypoint += 1
        self.coords = self.path[self.current_waypoint]

    def __repr__(self):
        return "Drone " + str(self.identifier)

    def __hash__(self):
        return hash(self.identifier)
