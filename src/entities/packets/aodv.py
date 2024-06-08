from entities.packets import Packet
from src.entities.event import Event
from src.utilities.types import NetAddr


class AODVPacket:
    TYPE: int


class RReqPacket(Packet, AODVPacket):
    TYPE = 1
    rreq_id: int
    hop_count: int
    dst_addr: NetAddr
    org_seq: int
    dst_seq: int | None

    def __init__(
        self,
        source: NetAddr,
        destination: NetAddr,
        timestamp: int,
        rreq_id: int,
        hop_count: int,
        dst_addr: NetAddr,
        org_seq: int,
        dst_seq: int | None = None,
    ):
        super().__init__(source, destination, timestamp, None)
        self.rreq_id = rreq_id
        self.hop_count = hop_count
        self.dst_addr = dst_addr
        self.dst_seq = dst_seq
        self.org_seq = org_seq


class RRepPacket(Packet, AODVPacket):
    TYPE = 2
    hop_count: int
    lifetime: int
    dst_addr: NetAddr
    dst_seq: int
    org_addr: NetAddr

    def __init__(
        self,
        source: NetAddr,
        destination: NetAddr,
        timestamp: int,
        hop_count: int,
        lifetime: int,
        dst_addr: NetAddr,
        dst_seq: int,
        org_addr: NetAddr,
    ):

        super().__init__(source, destination, timestamp, None)
        self.hop_count = hop_count
        self.lifetime = lifetime
        self.dst_addr = dst_addr
        self.dst_seq = dst_seq
        self.org_addr = org_addr


class RErrPacket(Packet, AODVPacket):
    TYPE = 3
    destinations: list[tuple[NetAddr, int]]

    def __init__(
        self,
        source: NetAddr,
        destination: NetAddr,
        timestamp: int,
        destinations: list[tuple[NetAddr, int]],
    ):
        super().__init__(source, destination, timestamp, None)
        self.destinations = destinations


class RRepAckPacket(Packet, AODVPacket):
    TYPE = 4
