import abc
import dataclasses

import config
from entities.communicating_entity import CommunicatingEntity
from entities.packets import HelloPacket, Packet
from simulation.metrics import Metrics
from utilities.types import NetAddr, Point


@dataclasses.dataclass(frozen=True)
class NeighbourNode:
    address: NetAddr
    timestamp: int
    coords: Point
    next_target: Point
    speed: int

    def __eq__(self, value: object, /) -> bool:
        if not isinstance(value, NeighbourNode):
            return False
        return self.address == value.address

    @staticmethod
    def from_hello_packet(packet: HelloPacket):
        return NeighbourNode(
            packet.src,
            packet.timestamp,
            packet.cur_pos,
            packet.next_target,
            packet.speed,
        )


class BASE_routing(metaclass=abc.ABCMeta):

    def __init__(self, drone: CommunicatingEntity):
        """The drone that is doing routing and simulator object."""
        self.drone = drone
        self.retransmission_count = 0
        # TODO: this should be a dict, as currently updating the set is broken
        self.neighbours: dict[NetAddr, NeighbourNode] = dict()

    @abc.abstractmethod
    def relay_selection(self, packet: Packet) -> NeighbourNode:
        pass

    def filter_neighbours_for_packet(
        self, packet: Packet
    ) -> dict[NetAddr, NeighbourNode]:
        # filter out the original sender of the packet, and the last relay node
        # neighbours = set(
        #     [
        #         n
        #         for n in self.neighbours
        #         if n.address not in (packet.src, packet.src_relay)
        #     ]
        # )
        return self.neighbours

    def handle_hello(self, packet: HelloPacket):
        self.neighbours[packet.src] = NeighbourNode.from_hello_packet(packet)

    def drone_identification(self, cur_step: int) -> HelloPacket | None:
        """handle drone hello messages to identify neighbors"""
        if cur_step % config.HELLO_DELAY != 0:  # still not time to communicate
            return

        return HelloPacket(
            self.drone.address,
            config.BROADCAST_ADDRESS,
            cur_step,
            self.drone.coords,
            self.drone.speed,
            self.drone.next_target(),
        )

    def update_neighbours(self, cur_step: int):
        """delete neighbour nodes that didn't send HelloPackets in a while"""
        to_delete = []
        for neighbour in self.neighbours.values():
            if neighbour.timestamp + config.OLD_HELLO_PACKET < cur_step:
                to_delete.append(neighbour.address)
        for n in to_delete:
            del self.neighbours[n]
            print(f"deleting neighbour: {n}")

    def routing_control(self, cur_step: int) -> list[Packet]:
        self.update_neighbours(cur_step)
        packets = []
        if hello_packet := self.drone_identification(cur_step):
            packets.append(hello_packet)
        return packets

    def route_packet(self, packet: Packet) -> Packet | None:
        Metrics.instance().mean_numbers_of_possible_relays.append(len(self.neighbours))
        best_neighbor = self.relay_selection(packet)
        if best_neighbor is None:
            # print("Cannot find a neighbour")
            return

        # TODO: handle ttl
        packet.dst_relay = best_neighbor.address
        packet.src_relay = self.drone.address
        self.retransmission_count += 1
        return packet

    def has_neighbours(self) -> bool:
        return len(self.neighbours) > 0
