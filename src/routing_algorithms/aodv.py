import itertools
from dataclasses import dataclass
from typing import Dict

import config
from entities.packets import aodv
from entities.packets.base import HelloPacket, Packet
from routing_algorithms.base import BaseRouting
from src.entities.communicating_entity import CommunicatingEntity
from utilities.types import NetAddr

# aodv specific constants
NET_DIAMETER = config.n_drones // 3
NODE_TRAVERSAL_TIME = 1
NET_TRAVERSAL_TIME = 2 * NODE_TRAVERSAL_TIME * NET_DIAMETER
PATH_DISCOVERY_TIME = 2 * NET_TRAVERSAL_TIME
ACTIVE_ROUTE_TIMEOUT = config.OLD_HELLO_PACKET
MY_ROUTE_TIMEOUT = 2 * ACTIVE_ROUTE_TIMEOUT
TTL_START = 1
TTL_INCREMENT = 2
TTL_THRESHOLD = 7
RING_TRAVERSAL_TIME = 2 * NODE_TRAVERSAL_TIME * 1


@dataclass
class RoutingTableEntry:
    dest: NetAddr
    seq_number: int
    seq_number_valid: bool
    is_valid: bool
    hop_count: int
    next_hop: NetAddr
    lifetime: int
    precursors: list = []
    is_repairable: bool = False
    is_being_repaired: bool = False


@dataclass
class RReqInfo:
    time: int
    ttl: int
    retry_count: int
    backoff: int


class AODVRouting(BaseRouting):
    routing_table: Dict[NetAddr, RoutingTableEntry]
    rreq_buffer: Dict[NetAddr, RReqInfo]
    sequence_number: int

    def __init__(self, drone: CommunicatingEntity):
        super().__init__(drone)

        self.routing_table = dict()
        self.rreq_buffer = dict()
        self.sequence_number = 0

        self.rreq_id = itertools.count(0, 1)

    def process(self, packet: Packet):
        if isinstance(packet, aodv.AODVPacket):
            self._process_control(packet)

        return super().process(packet)

    def _process_control(self, packet: aodv.AODVPacket):
        if isinstance(packet, aodv.RReqPacket):
            self._process_rreq(packet)

    def _update_route(
        self,
        addr: NetAddr,
        seq: int = -1,
        hop_count: int = 0,
        next_hop: NetAddr = -1,
        lifetime: int = ACTIVE_ROUTE_TIMEOUT,
    ):
        route_entry = self.routing_table.get(addr)
        if route_entry is None:
            route_entry = RoutingTableEntry(
                dest=addr,
                seq_number=seq,
                seq_number_valid=(seq > 0),
                is_valid=(seq > 0),
                hop_count=hop_count,
                next_hop=next_hop,
                lifetime=lifetime,
            )

        # TODO: update route if:
        #       - seq is higher than the existing one
        #       - seq num is equal, but the hop_count is smaller
        #       - seq is unknown

    def _process_rreq(self, packet: aodv.RReqPacket):
        # TODO: create or update the existing route for last hop
        last_hop = packet.src_relay

        # TODO: create or update a reverse route for the destination

        # TODO: if possible generate a RRep
        #       discard the RReq
        #       unicast the RRep to the destination too
        dst_addr = packet.dst_addr
        route = self.routing_table.get(dst_addr)
        if dst_addr == self.drone.address or (route is not None and route.is_valid):
            self._generate_rrep(packet)
            return

        # TODO: else:
        #           decrease ttl/hop limit
        #           increase hop count
        #           broadcast the RReq

    def _generate_rrep(self, packet: aodv.RReqPacket):
        dst_addr = packet.dst_addr
        if dst_addr == self.drone.address:
            if packet.dst_seq is not None:
                assert packet.dst_seq <= self.sequence_number + 1
                if packet.dst_seq == self.sequence_number + 1:
                    self.sequence_number += 1

            hop_count = 0
            dst_seq = self.sequence_number
            lifetime = MY_ROUTE_TIMEOUT
        else:
            route = self.routing_table[dst_addr]
            hop_count = route.hop_count
            dst_seq = route.seq_number
            lifetime = route.lifetime - self.drone.time
            # TODO: update precursors (6.6.2)

        rrep = aodv.RRepPacket(
            source=self.drone.address,
            destination=packet.src,
            timestamp=self.drone.time,
            hop_count=hop_count,
            lifetime=lifetime,
            dst_seq=dst_seq,
        )

    def _request_route(self, dest: NetAddr):
        # TODO: route requests registry
        #        - keep track of requests, current ttl and backoff time
        rreq_info = self.rreq_buffer.get(dest)
        if rreq_info is None:
            # TODO: increment sequence number (6.1)
            ...
            # rreq_info = RReqInfo(
            #     self.drone.time,
            #     1,
            #     0,
            # )

        packet = aodv.RReqPacket
        pass

    def relay_selection(self, packet: Packet) -> NetAddr | None:
        route_info = self.routing_table.get(packet.dst)
        if route_info is not None and route_info.lifetime > self.drone.time:
            # TODO: prolong lifetime for source, destination and next hop
            return route_info.next_hop

        self._request_route(packet.dst)
        return None

    def drone_identification(self, cur_step: int) -> HelloPacket | None:
        return super().drone_identification(cur_step)

    def routing_control(self, cur_step: int) -> list[Packet]:
        return super().routing_control(cur_step)

    def has_neighbours(self) -> bool:
        return super().has_neighbours()

    def should_forward(self, packet: Packet) -> bool:
        return super().should_forward(packet)
