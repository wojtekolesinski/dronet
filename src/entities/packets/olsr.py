from dataclasses import dataclass
from typing import Literal

import config
from entities.event import Event
from entities.packets.base import HelloPacket
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

    def __init__(self) -> None:
        pass


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
        event_ref: Event | None = None,
    ):
        super().__init__(
            source, destination, timestamp, cur_pos, speed, next_target, event_ref
        )
