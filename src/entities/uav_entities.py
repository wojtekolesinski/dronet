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

        self.simulator = simulator
        self.depot = depot
        self.path = path
        self.speed = config.drone_speed
        self.residual_energy = config.drone_max_energy
        self.come_back_to_mission = (
            False  # if i'm coming back to my applicative mission
        )
        self.last_move_routing = False  # if in the last step i was moving to depot

        # dynamic parameters
        self.tightest_event_deadline = (
            None  # used later to check if there is an event that is about to expire
        )
        self.current_waypoint = 0
        self.distance_from_depot = 0
        self.move_routing = False  # if true, it moves to the depot

        # setup drone routing algorithm
        self.routing_algorithm = config.routing_algorithm.value(self, self.simulator)

        # last mission coord to restore the mission after movement
        self.last_mission_coords = None

    def update_packets(self, cur_step):
        """
        Removes the expired packets from the buffer

        @param cur_step: Integer representing the current time step
        @return:
        """
        to_remove_packets = 0
        tmp_buffer = []
        self.tightest_event_deadline = np.nan

        for pck in self.__buffer:
            if not pck.is_expired(cur_step):
                tmp_buffer.append(pck)  # append again only if it is not expired
                self.tightest_event_deadline = np.nanmin(
                    [self.tightest_event_deadline, pck.event_ref.deadline]
                )

            else:

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
        self.__buffer = tmp_buffer

        if self.buffer_length() == 0:
            self.move_routing = False

    def packet_is_expiring(self, cur_step):
        """return true if exist a packet that is expiring and must be returned to the depot as soon as possible
        -> start to move manually to the depot.

        This method is optional, there is flag src.utilities.config.ROUTING_IF_EXPIRING
        """
        time_to_depot = self.distance_from_depot / self.speed
        event_time_to_dead = (
            self.tightest_event_deadline - cur_step
        ) * config.time_step_duration
        return (
            event_time_to_dead - 5 < time_to_depot <= event_time_to_dead
        )  # 5 seconds of tolerance

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

    def feel_event(self, cur_step):
        """
        feel a new event, and adds the packet relative to it, in its buffer.
            if the drones is doing movement the packet is not added in the buffer
        """

        ev = Event(self.coords, cur_step)  # the event
        pk = DataPacket(cur_step, ev, self)
        if not self.move_routing and not self.come_back_to_mission:
            self.__buffer.append(pk)
            Metrics.instance().all_data_packets_in_simulation += 1
        else:  # store the events that are missing due to movement routing
            Metrics.instance().events_not_listened.add(ev)

    def accept_packets(self, packets):
        """Self drone adds packets of another drone, when it feels it passing by."""

        for packet in packets:
            # add if not notified yet, else don't, proprietary drone will delete all packets, but it is ok
            # because they have already been notified by someone already

            if not self.is_known_packet(packet):
                self.__buffer.append(packet)

    def routing(self, drones, depot, cur_step):
        """do the routing"""
        self.distance_from_depot = utilities.euclidean_distance(
            self.depot.coords, self.coords
        )
        self.routing_algorithm.routing(depot, drones, cur_step)

    def move(self, time):
        """Move the drone to the next point if self.move_routing is false, else it moves towards the depot.

        time -> time_step_duration (how much time between two simulation frame)
        """
        if self.move_routing or self.come_back_to_mission:
            # metrics: number of time steps on active routing (movement) a counter that is incremented each time
            # drone is moving to the depot for active routing, i.e., move_routing = True
            # or the drone is coming back to its mission
            Metrics.instance().time_on_active_routing += 1

        if self.move_routing:
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
        self.last_move_routing = self.move_routing

    def is_full(self):
        return self.buffer_length() == self.buffer_max_size

    def is_known_packet(self, packet: DataPacket):
        """Returns True if drone has already a similar packet (i.e., referred to the same event)."""
        for pk in self.__buffer:
            if pk.event_ref == packet.event_ref:
                return True
        return False

    def remove_packets(self, packets):
        """Removes the packets from the buffer."""
        for packet in packets:
            if packet in self.__buffer:
                self.__buffer.remove(packet)
                if config.DEBUG:
                    print(
                        "ROUTING del: drone: "
                        + str(self.identifier)
                        + " - removed a packet id: "
                        + str(packet.identifier)
                    )

    def next_target(self) -> Point:
        if self.move_routing:
            return self.depot.coords
        elif self.come_back_to_mission:
            return self.last_mission_coords
        else:
            if (
                self.current_waypoint >= len(self.path) - 1
            ):  # reached the end of the path, start back to 0
                return self.path[0]
            else:
                return self.path[self.current_waypoint + 1]

    def __move_to_mission(self, time):
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

    def __update_position(self, p1):
        if self.come_back_to_mission:
            self.come_back_to_mission = False
            self.coords = p1
        else:
            self.current_waypoint += 1
            self.coords = self.path[self.current_waypoint]

    def __move_to_depot(self, time):
        """When invoked the drone moves to the depot. TODO: Add comments and clean.
        time -> time_step_duration (how much time between two simulation frame)
        """
        p0 = self.coords
        p1 = self.depot.coords

        all_distance = utilities.euclidean_distance(p0, p1)
        distance = time * self.speed
        if all_distance == 0:
            self.move_routing = False
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
