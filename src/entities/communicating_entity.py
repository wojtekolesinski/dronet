import abc
from collections import defaultdict

import config
from entities.base import Entity
from entities.packets import ACKPacket, DataPacket, Packet
from simulation.net import MediumDispatcher
from utilities.types import NetAddr, Point


class CommunicatingEntity(Entity):
    address: NetAddr
    network: MediumDispatcher
    sensing_range: int
    communication_range: int
    buffer_size: int
    buffer: list[Packet]
    output_buffer: list[Packet]
    retransmission_buffer: set[Packet]
    time: int
    speed: int

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

    def routing(self):
        """do the routing"""
        packets = self.router.routing_control(self.time)
        self.output_buffer.extend(packets)

        routed_packets = []
        if self.router.has_neighbours():
            for packet in self.all_packets():
                if p := self.router.route_packet(packet):
                    routed_packets.append(p)
                elif isinstance(packet, DataPacket):
                    # if we cannot route the data packet, let's try to do it next time
                    # packet.timestamp += 1
                    pass
        self.output_buffer.extend(routed_packets)

    def acknowledge_packet(self, packet: Packet):
        ack_packet = self.router.make_ack_packet(packet)
        # print("MADE ACK PACKET", ack_packet)
        self.buffer.append(ack_packet)

    def set_time(self, timestamp: int):
        self.time = timestamp

    def empty_buffer(self):
        """empty output buffer and remove packets that were routed successfully from retransmission_buffer"""
        # for packet in self.output_buffer:
        #     if packet in self.retransmission_buffer:
        #         self.retransmission_buffer.remove(packet)
        self.retransmission_buffer.clear()
        self.output_buffer.clear()

    def all_packets(self):
        """return packets, that should be routed at this moment.
        This includes:
        - packets from buffer, that were generated at this moment.
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

    def send_packets(self, depot=False):
        seen = set()
        dups = 0
        for packet in self.output_buffer:
            if packet.identifier in seen:
                dups += 1
                continue
            seen.add(packet.identifier)
            packet.src_relay = self.address
            # if depot:
            #     print("sending from depot: " + str(packet))
            self.network.send(packet, self.coords, self.communication_range)
        self.empty_buffer()

    def remove_packets(self, packet_ids: list[int]):
        """Removes the packets from the buffer."""
        if not packet_ids:
            return
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
            # print("Dropping packet", packet)
            self.buffer.remove(packet)
