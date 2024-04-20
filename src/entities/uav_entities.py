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
        simulator,
    ):
        super().__init__(
            identifier,
            path[0],
            address,
            network,
            config.drone_sensing_range,
            config.drone_communication_range,
            config.drone_max_buffer_size,
        )

        # TODO: cleanup the parameters
        self.simulator = simulator
        self.depot = depot
        self.path: Path = path
        self.speed = config.drone_speed
        self.residual_energy = config.drone_max_energy
        # if i'm coming back to my applicative mission
        self.come_back_to_mission = False
        self.last_move_routing = False  # if in the last step i was moving to depot

        # dynamic parameters
        # used later to check if there is an event that is about to expire
        self.tightest_event_deadline = None
        self.current_waypoint = 0
        self.move_to_depot = False

        # setup drone routing algorithm
        self.routing_algorithm = config.routing_algorithm.value(self, self.simulator)

    def consume_packet(self, packet: Packet):
        if isinstance(packet, HelloPacket):
            self.routing_algorithm.handle_hello(packet)

        elif isinstance(packet, DataPacket):
            self.acknowledge_packet(packet)
            self.buffer.append(packet)

        elif isinstance(packet, ACKPacket):
            self.remove_packets([packet.acked_packet_id])
            if self.buffer_length() == 0:
                self.routing_algorithm.current_n_transmission = 0
                self.move_to_depot = False

    def update_packets(self, cur_step):
        """
        Removes the expired packets from the buffer

        @param cur_step: Integer representing the current time step
        @return:
        """
        to_remove_packets = 0
        tmp_buffer = []
        self.tightest_event_deadline = np.nan

        for pck in self.buffer:
            if not pck.is_expired(cur_step):
                tmp_buffer.append(pck)  # append again only if it is not expired
                self.tightest_event_deadline = np.nanmin(
                    [self.tightest_event_deadline, pck.event_ref.deadline]
                )
                continue

            to_remove_packets += 1

            if config.routing_algorithm.name not in "GEO" "RND" "GEOS":

                feedback = -1
                current_drone = self

                for drone in self.simulator.drones:
                    drone.routing_algorithm.feedback(
                        current_drone,
                        pck.event_ref.identifier,
                        config.event_duration,
                        feedback,
                    )
        self.buffer = tmp_buffer

        if self.buffer_length() == 0:
            self.move_to_depot = False

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
        if not self.move_to_depot and not self.come_back_to_mission:
            # TODO: unify buffer use
            # proposed solution:
            # buffer contains packets, that need to be handled, or routed
            # output buffer contains packets that are supposed to be sent at the end of this second
            self.buffer.append(pk)
            Metrics.instance().all_data_packets_in_simulation += 1
        else:  # store the events that are missing due to movement routing
            Metrics.instance().events_not_listened.add(ev)


            if not self.is_known_packet(packet):
                self.buffer.append(packet)

    def routing(self, cur_step):
        """do the routing"""
        self.routing_algorithm.routing(cur_step)

    def move(self, time):
        """Move the drone to the next point if self.move_routing is false, else it moves towards the depot.

        time -> time_step_duration (how much time between two simulation frame)
        """
        if self.move_to_depot or self.come_back_to_mission:
            # metrics: number of time steps on active routing (movement) a counter that is incremented each time
            # drone is moving to the depot for active routing, i.e., move_routing = True
            # or the drone is coming back to its mission
            Metrics.instance().time_on_active_routing += 1

        if self.move_to_depot:
            if (
                not self.last_move_routing
            ):  # this is the first time that we are doing move-routing
                self.last_mission_coords = self.coords

            self.__move_to_depot(time)
        else:
            if self.last_move_routing:  # I'm coming back to the mission
                self.come_back_to_mission = True

            self.__move_to_mission(time)

            # metrics: number of time steps on mission, incremented each time drone is doing sensing mission
            Metrics.instance().time_on_mission += 1

        # set the last move routing
        self.last_move_routing = self.move_to_depot

    def next_target(self) -> Point:
        if self.move_to_depot:
            return self.depot.coords

        if self.come_back_to_mission:
            return self.last_mission_coords

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
        if self.come_back_to_mission:  # after move
            p1 = self.last_mission_coords
        else:
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
        if self.come_back_to_mission:
            self.come_back_to_mission = False
            self.coords = p1
        else:
            self.current_waypoint += 1
            self.coords = self.path[self.current_waypoint]

    def __move_to_depot(self, time: int):
        """When invoked the drone moves to the depot. TODO: Add comments and clean.
        time -> time_step_duration (how much time between two simulation frame)
        """
        p0 = self.coords
        p1 = self.depot.coords

        all_distance = utilities.euclidean_distance(p0, p1)
        distance = time * self.speed
        if all_distance == 0:
            self.move_to_depot = False
            return

        t = distance / all_distance

        if t >= 1:
            self.coords = p1  # with the next step you would surpass the target
        elif t <= 0:
            print("Error routing move drone, ratio < 0")
            exit(1)
        else:
            self.coords = (((1 - t) * p0[0] + t * p1[0]), ((1 - t) * p0[1] + t * p1[1]))

    def __repr__(self):
        return "Drone " + str(self.identifier)

    def __hash__(self):
        return hash(self.identifier)
