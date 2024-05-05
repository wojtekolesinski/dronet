from dataclasses import dataclass
from typing import Literal

import config
from entities.event import Event
from entities.packets.base import DataPacket, HelloPacket, Packet
from utilities.types import NetAddr, Point

LinkType = Literal["UNSPEC_LINK", "ASYM_LINK", "SYM_LINK", "LOST_LINK"]
NeighbourType = Literal["SYM_NEIGH", "MPR_NEIGH", "NOT_NEIGH"]


@dataclass
class LinkCode:
    link_type: LinkType
    neighbour_type: NeighbourType


class OLSRPacket:
    message_type: Literal["hello", "tc", "other"]
    sequence_number: int

    @property
    def vtime(self):
        return config.vtime

    def __init__(self, sequence_number: int) -> None:
        self.sequence_number = sequence_number


class OLSRHelloPacket(HelloPacket, OLSRPacket):
    willingness: int
    htime: int
    links: dict[LinkCode, list[NetAddr]]

    def __init__(
        self,
        source: NetAddr,
        destination: NetAddr,
        timestamp: int,
        cur_pos: Point,
        speed: int,
        next_target: Point,
        sequence_number: int,
        event_ref: Event | None = None,
    ):
        HelloPacket.__init__(
            self, source, destination, timestamp, cur_pos, speed, next_target, event_ref
        )
        OLSRPacket.__init__(self, sequence_number)
        self.message_type = "hello"


class OLSRTopologyControlPacket(Packet, OLSRPacket):
    ansn: int
    advertised_neigbours: list[NetAddr]

    def __init__(
        self,
        source: NetAddr,
        destination: NetAddr,
        timestamp: int,
        sequence_number: int,
        ansn: int,
        advertised_neigbours: list[NetAddr],
        event_ref: Event | None = None,
    ):
        Packet.__init__(self, source, destination, timestamp, event_ref)
        OLSRPacket.__init__(self, sequence_number)
        self.message_type = "tc"
        self.ttl = 255
        self.ansn = ansn
        self.advertised_neigbours = advertised_neigbours


class OLSRDataPacket(DataPacket, OLSRPacket):
    def __init__(
        self,
        source: NetAddr,
        destination: NetAddr,
        timestamp: int,
        sequence_number: int,
        event_ref: Event,
    ):
        DataPacket.__init__(self, source, destination, timestamp, event_ref)
        OLSRPacket.__init__(self, sequence_number)
