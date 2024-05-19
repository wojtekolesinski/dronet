import abc

import config
from entities.base import Entity
from entities.packets import ACKPacket, Packet
from simulation.net import MediumDispatcher
from utilities.types import NetAddr, Point


class CommunicatingEntity(Entity):
    address: NetAddr
    network: MediumDispatcher
    sensing_range: int
    communication_range: int
    buffer: list[Packet]
    output_buffer: list[Packet]

    def __init__(
        self,
        identifier: int,
        coords: Point,
        address: NetAddr,
        network: MediumDispatcher,
        sensing_range: int,
        communication_range: int,
        buffer_size: int,
        speed: int,
    ):
        super().__init__(identifier, coords)
        self.address = address
        self.network = network
        self.sensing_range = sensing_range
        self.communication_range = communication_range
        self.buffer_size = buffer_size
        self.buffer: list[Packet] = []
        self.output_buffer: list[Packet] = []
        self.retransmission_buffer: set[Packet] = set()
        self.time = 0
        self.speed = speed
        self.router = config.routing_algorithm.value(self)

    def listen(self):
        packets = self.network.listen(
            self.address, self.coords, self.communication_range
        )
        for packet in packets:
            self.consume_packet(packet)

    @abc.abstractmethod
    def consume_packet(self, packet: Packet):
        pass

    @abc.abstractmethod
    def next_target(self) -> Point:
        pass

    @abc.abstractmethod
    def routing(self):
        pass

    def acknowledge_packet(self, packet: Packet):
        ack_packet = self.router.make_ack_packet(packet)
        self.output_buffer.append(ack_packet)

    def set_time(self, timestamp: int):
        self.time = timestamp

    def empty_buffer(self):
        """empty output buffer and remove packets from retransmission_buffer, that were routed successfully"""
        for packet in self.output_buffer:
            if packet in self.retransmission_buffer:
                self.retransmission_buffer.remove(packet)
        self.output_buffer = []

    def all_packets(self):
        """return packets, that should be routed in this moment.
        This includes:
        - packets from buffer, that were generated in this moment.
        - packets from buffer, that should be retransmitted
        - all packets from retransmission_buffer"""
        packets = []
        for packet in self.retransmission_buffer:
            if packet.dst == config.BROADCAST_ADDRESS:
                self.output_buffer.append(packet)
            else:
                packets.append(packet)
        for packet in self.buffer:
            if (self.time - packet.timestamp) % config.retransmission_delay == 0:
                packets.append(packet)

        return packets

    def buffer_length(self):
        return len(self.buffer)

    def is_full(self):
        return self.buffer_length() == self.buffer_size

    def send_packets(self):
        for packet in self.output_buffer:
            packet.src_relay = self.address
            self.network.send(packet, self.coords, self.communication_range)
        self.empty_buffer()

    def remove_packets(self, packet_ids: list[int]):
        """Removes the packets from the buffer."""
        to_remove = []
        for packet in self.buffer:
            if packet.identifier in packet_ids:
                to_remove.append(packet)
                if config.DEBUG:
                    print(
                        "ROUTING del: drone: "
                        + str(self.identifier)
                        + " - removed a packet id: "
                        + str(packet.identifier)
                    )
        for packet in to_remove:
            self.buffer.remove(packet)
