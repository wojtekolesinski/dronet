import config
from entities.base import Entity
from entities.event import Event
from simulation.metrics import Metrics
from utilities.types import NetAddr, Point


class Packet(Entity):
    """A packet is an object created out of an event monitored on the aoi."""

    src: NetAddr
    dst: NetAddr
    src_relay: NetAddr
    dst_relay: NetAddr
    timestamp: int
    ttl: int
    hop_count: int
    event_ref: Event | None = None

    def __init__(
        self,
        source: NetAddr,
        destination: NetAddr,
        timestamp: int,
        event_ref: Event | None = None,
    ):
        """the event associated to the packet, time step in which the packet was created
        as for now, every packet is an event."""

        event_ref_crafted = (
            event_ref if event_ref is not None else Event((-1, -1), -1)
        )  # default event if packet is not associated to the event

        # id(self) is the id of this instance (unique for every new created packet),
        # the coordinates are those of the event
        super().__init__(id(self), event_ref_crafted.coords)

        self.src = source
        self.src_relay = source
        self.dst = destination
        self.dst_relay = destination
        self.timestamp = timestamp
        self.event_ref = event_ref_crafted
        self.ttl = config.packets_max_ttl
        self.hop_count = 0

        if event_ref is not None:
            self.add = Metrics.instance().drones_packets.add(self)

    def age_of_packet(self, cur_step: int):
        return cur_step - self.timestamp

    def to_json(self):
        """return the json repr of the obj"""

        return {
            "coord": self.coords,
            "i_gen": self.timestamp,
            "i_dead": self.event_ref.deadline,
            "id": self.identifier,
            "TTL": self.ttl,
            "id_event": self.event_ref.identifier,
        }

    def is_expired(self, cur_step):
        """a packet expires if the deadline of the event expires, or the maximum TTL is reached"""
        return cur_step > self.event_ref.deadline

    def __repr__(self):
        packet_type = str(self.__class__).split(".")[-1].split("'")[0]
        return (
            packet_type
            + "id:"
            + str(self.identifier)
            + " event id: "
            + str(self.event_ref.identifier)
            + " c:"
            + str(self.coords)
        )


class DataPacket(Packet):
    """Basically a Packet"""

    event_ref: Event

    def __init__(
        self,
        source: NetAddr,
        destination: NetAddr,
        timestamp: int,
        event_ref: Event,
    ):
        super().__init__(source, destination, timestamp, event_ref)


class ACKPacket(Packet):
    def __init__(
        self,
        source: NetAddr,
        destination: NetAddr,
        timestamp: int,
        acked_packet_id: int,
        event_ref: Event | None = None,
    ):
        super().__init__(source, destination, timestamp, event_ref)
        self.acked_packet_id = (
            acked_packet_id  # packet that the drone who creates it wants to ACK
        )


class HelloPacket(Packet):
    """The hello message is responsible to give info about neighborhood"""

    def __init__(
        self,
        source: NetAddr,
        destination: NetAddr,
        timestamp: int,
        cur_pos: Point,
        speed: int,
        next_target: Point,
        event_ref: Event | None = None,
    ):
        super().__init__(source, destination, timestamp, event_ref)
        self.cur_pos = cur_pos
        self.speed = speed
        self.next_target = next_target
