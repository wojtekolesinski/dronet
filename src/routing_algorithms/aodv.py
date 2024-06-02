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
ALLOWED_HELLO_LOSS = 2


@dataclass
class RoutingTableEntry:
    dest: NetAddr
    seq_number: int
    seq_number_valid: bool
    is_valid: bool
    hop_count: int
    next_hop: NetAddr
    expiry_time: int
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
    pending_rreq_buffer: Dict[NetAddr, RReqInfo]
    received_rreqs = Dict[tuple[NetAddr, int], int]
    sequence_number: int

    def __init__(self, drone: CommunicatingEntity):
        super().__init__(drone)

        self.routing_table = dict()
        self.pending_rreq_buffer = dict()
        self.received_rreqs = dict()
        self.sequence_number = 0

        self.rreq_id = itertools.count(0, 1)

    def process(self, packet: Packet):
        if isinstance(packet, aodv.AODVPacket):
            self._process_control(packet)

        return super().process(packet)

    def _process_control(self, packet: aodv.AODVPacket):
        if isinstance(packet, aodv.RReqPacket):
            return self._process_rreq(packet)

        if isinstance(packet, aodv.RRepPacket):
            return self._process_rrep(packet)

    def _update_route(
        self,
        addr: NetAddr,
        seq: int = -1,
        hop_count: int = 0,
        next_hop: NetAddr = -1,
        lifetime: int = ACTIVE_ROUTE_TIMEOUT,
    ):
        route = self.routing_table.get(addr)
        if route is None:
            route = RoutingTableEntry(
                dest=addr,
                seq_number=seq,
                seq_number_valid=(seq > 0),
                is_valid=True,
                hop_count=hop_count,
                next_hop=next_hop,
                expiry_time=self.drone.time + lifetime,
            )
            self.routing_table[addr] = route
        else:
            should_update = (
                not route.seq_number_valid
                or (seq > route.seq_number)
                or (seq == route.seq_number and not route.is_valid)
                or (seq == route.seq_number and hop_count + 1 < route.hop_count)
            )
            if should_update:
                route.is_valid = True
                route.seq_number_valid = seq != -1
                route.next_hop = next_hop
                route.hop_count = hop_count
                route.expiry_time = max(route.expiry_time, self.drone.time + lifetime)
                route.seq_number = seq

    def _process_rreq(self, packet: aodv.RReqPacket):
        last_hop = packet.src_relay
        self._update_route(last_hop, hop_count=1, next_hop=last_hop)

        rreq_key: tuple[NetAddr, int] = (packet.src, packet.rreq_id)
        if (timestamp := self.received_rreqs.get(rreq_key)) is not None:
            if timestamp + PATH_DISCOVERY_TIME > self.drone.time:
                return
        self.received_rreqs[rreq_key] = self.drone.time

        self._update_route(
            addr=packet.src,
            seq=packet.org_seq,
            hop_count=packet.hop_count,
            next_hop=packet.src_relay,
            lifetime=2 * NET_TRAVERSAL_TIME
            - 2 * packet.hop_count * NODE_TRAVERSAL_TIME,
        )

        dst_addr = packet.dst_addr
        route = self.routing_table.get(dst_addr)
        if dst_addr == self.drone.address or (route is not None and route.is_valid):
            self._generate_rrep(packet)
            return

        if packet.ttl > 1:
            packet.ttl -= 1
            packet.hop_count += 1
            packet.dst_seq = max(
                packet.dst_seq if packet.dst_seq else 0,
                route.seq_number if route else 0,
            )
            self.drone.output_buffer.append(packet)

    def _process_rrep(self, packet: aodv.RRepPacket):
        packet.hop_count += 1
        route = self.routing_table.get(packet.dst_addr)
        if route is None:
            route = RoutingTableEntry(
                dest=packet.dst_addr,
                seq_number=packet.dst_seq,
                is_valid=True,
                seq_number_valid=True,
                hop_count=packet.hop_count,
                next_hop=packet.src_relay,
                expiry_time=self.drone.time + packet.lifetime,
            )
        else:
            should_update = (
                not route.seq_number_valid
                or (packet.dst_seq > route.seq_number)
                or (packet.dst_seq == route.seq_number and not route.is_valid)
                or (
                    packet.dst_seq == route.seq_number
                    and packet.hop_count < route.hop_count
                )
            )
            if should_update:
                route.is_valid = True
                route.seq_number_valid = True
                route.next_hop = packet.src_relay
                route.hop_count = packet.hop_count
                route.expiry_time = self.drone.time + packet.lifetime
                route.seq_number = packet.dst_seq

        if packet.org_addr == self.drone.address:
            return

        if packet.ttl <= 1:
            return

        packet.ttl -= 1
        route_to_org = self.routing_table[packet.org_addr]
        route.precursors.append(route_to_org.next_hop)
        self.routing_table[route.next_hop].precursors.append(route_to_org.next_hop)
        packet.dst_relay = route_to_org.next_hop
        self.drone.output_buffer.append(packet)

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
            lifetime = route.expiry_time - self.drone.time
            # update precursors
            self.routing_table[packet.src].precursors.append(route.next_hop)

        rrep = aodv.RRepPacket(
            source=self.drone.address,
            destination=packet.src,
            timestamp=self.drone.time,
            hop_count=hop_count,
            lifetime=lifetime,
            dst_addr=dst_addr,
            dst_seq=dst_seq,
            org_addr=packet.src,
        )

        self.drone.buffer.append(rrep)

        # gratuitous RREP
        org_route = self.routing_table[packet.src]
        grat_rrep = aodv.RRepPacket(
            source=self.drone.address,
            destination=dst_addr,
            timestamp=self.drone.time,
            hop_count=org_route.hop_count,
            lifetime=org_route.expiry_time - self.drone.time,
            dst_addr=packet.src,
            dst_seq=packet.org_seq,
            org_addr=dst_addr,
        )
        self.drone.buffer.append(grat_rrep)

    def _request_route(self, dest: NetAddr):
        # TODO: route requests registry
        #        - keep track of requests, current ttl and backoff time
        rreq_info = self.pending_rreq_buffer.get(dest)
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
        if route_info is not None and route_info.expiry_time > self.drone.time:
            # TODO: prolong lifetime for source, destination and next hop
            return route_info.next_hop

        self._request_route(packet.dst)
        return None

    def drone_identification(self, cur_step: int) -> HelloPacket | None:
        if cur_step % config.HELLO_DELAY != 0:  # still not time to communicate
            return

        packet = aodv.RRepPacket(
            source=self.drone.address,
            destination=config.BROADCAST_ADDRESS,
            timestamp=self.drone.time,
            hop_count=0,
            lifetime=ALLOWED_HELLO_LOSS * config.HELLO_DELAY,
            dst_addr=self.drone.address,
            dst_seq=self.sequence_number,
            org_addr=-1,
        )
        packet.ttl = 1
        return packet  # type: ignore

    def routing_control(self, cur_step: int) -> list[Packet]:
        self._clean()
        return super().routing_control(cur_step)

    def _clean(self):
        # TODO: mark expired routes as invalid
        # TODO: send RErr for each expired route
        # TODO: check buffered packets for valid routes
        # TODO: cleanup received_rreqs
        pass

    def has_neighbours(self) -> bool:
        return True

    def should_forward(self, packet: Packet) -> bool:
        return super().should_forward(packet)
