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
    dst_seq: int

    def __init__(
        self,
        source: NetAddr,
        destination: NetAddr,
        timestamp: int,
        hop_count: int,
        lifetime: int,
        dst_seq: int,
    ):

        super().__init__(source, destination, timestamp, None)
        self.hop_count = hop_count
        self.lifetime = lifetime
        self.dst_seq = dst_seq


class RErrPacket(Packet, AODVPacket):
    TYPE = 3


class RRepAckPacket(Packet, AODVPacket):
    TYPE = 4
