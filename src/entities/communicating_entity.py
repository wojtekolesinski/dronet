import abc

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
    ):
        super().__init__(identifier, coords)
        self.address = address
        self.network = network
        self.sensing_range = sensing_range
        self.communication_range = communication_range
        self.buffer_size = buffer_size
        self.buffer: list[Packet] = []
        self.output_buffer: list[Packet] = []
        self.no_transmission = False
        self.time = 0

    def listen(self):
        packets = self.network.listen(
            self.address, self.coords, self.communication_range
        )
        for packet in packets:
            self.consume_packet(packet)

    @abc.abstractmethod
    def consume_packet(self, packet: Packet):
        pass

    def acknowledge_packet(self, packet: Packet):
        self.output_buffer.append(
            ACKPacket(self.address, packet.src_relay, self.time, packet.identifier)
        )

    def empty_buffer(self):
        self.output_buffer = []

    def all_packets(self):
        return self.output_buffer

    def buffer_length(self):
        return len(self.output_buffer)

    def is_full(self):
        return self.buffer_length() == self.buffer_size

    def send_packets(self):
        for packet in self.output_buffer:
            self.network.send(packet, self.coords, self.communication_range)

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
