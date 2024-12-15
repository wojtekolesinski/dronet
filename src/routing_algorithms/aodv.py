import itertools
import logging
import sys
from dataclasses import dataclass, field

import config
from entities.communicating_entity import CommunicatingEntity
from entities.packets import aodv
from entities.packets.base import HelloPacket, Packet
from routing_algorithms.base import BaseRouting
from utilities.types import NetAddr

logger = logging.getLogger(__name__)

# aodv specific constants
ACTIVE_ROUTE_TIMEOUT = config.OLD_HELLO_PACKET
ALLOWED_HELLO_LOSS = 2
DELETE_PERIOD = ALLOWED_HELLO_LOSS * config.HELLO_DELAY
MY_ROUTE_TIMEOUT = 2 * ACTIVE_ROUTE_TIMEOUT
NET_DIAMETER = config.n_drones // 3
NODE_TRAVERSAL_TIME = 1
NET_TRAVERSAL_TIME = 2 * NODE_TRAVERSAL_TIME * NET_DIAMETER
PATH_DISCOVERY_TIME = 2 * NET_TRAVERSAL_TIME
RING_TRAVERSAL_TIME = 2 * NODE_TRAVERSAL_TIME * 1
RREQ_RETRIES = 2
TTL_INCREMENT = 2
TTL_START = 1
TTL_THRESHOLD = 7


@dataclass
class RoutingTableEntry:
    dest: NetAddr
    seq_number: int
    seq_number_valid: bool
    is_valid: bool
    hop_count: int
    next_hop: NetAddr
    expiry_time: int
    precursors: set[NetAddr] = field(default_factory=set)
    is_repairable: bool = False
    is_being_repaired: bool = False


@dataclass
class RReqInfo:
    time: int
    ttl: int
    retry_count: int  # retries after max ttl has been reached
    backoff: int


class AODVRouting(BaseRouting):
    routing_table: dict[NetAddr, RoutingTableEntry]
    pending_rreq_buffer: dict[NetAddr, RReqInfo]
    received_rreqs: dict[tuple[NetAddr, int], int]
    sequence_number: int

    def __init__(self, drone: CommunicatingEntity):
        super().__init__(drone)

        self.routing_table = dict()
        self.pending_rreq_buffer = dict()
        self.received_rreqs = dict()
        self.sequence_number = 0

        self.rreq_id = itertools.count(0, 1)

    def log_size(self):
        if config.DEBUG:
            print(
                f"{sys.getsizeof(self.routing_table)}, "
                f"{sys.getsizeof(self.pending_rreq_buffer)}, "
                f"{sys.getsizeof(self.received_rreqs)}"
            )

    def process(self, packet: Packet):
        if isinstance(packet, aodv.AODVPacket):
            return self._process_control(packet)

        return super().process(packet)

    def _process_control(self, packet: aodv.AODVPacket):
        if isinstance(packet, aodv.RReqPacket):
            return self._process_rreq(packet)

        if isinstance(packet, aodv.RRepPacket):
            return self._process_rrep(packet)

        if isinstance(packet, aodv.RErrPacket):
            return self._process_rerr(packet)

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
            # logger.debug(f"new route with lifetime: {lifetime}")
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

        rreq_key = (packet.src, packet.rreq_id)
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
        # TODO: delete this
        if self.drone.address in (2, 3) and dst_addr == 4:
            print(f"{self.drone.address=} GOT RREQ {packet=}, {route=}")

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
        if self.drone.address == 2 and packet.src == 3 and packet.dst_addr == 1:
            print(f"GOT RREP FOR DEPOT {packet=}")
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
            self.routing_table[packet.dst_addr] = route
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

        # Hello packet
        # if packet.org_addr == -1:
        #     self.routing_table[route.dest] = route
        #     return

        if packet.ttl < 1:
            return

        packet.ttl -= 1
        try:
            route_to_org = self.routing_table[packet.org_addr]
            route.precursors.add(route_to_org.next_hop)
            self.routing_table[route.next_hop].precursors.add(route_to_org.next_hop)
        except KeyError:
            print("keyerror")
        packet.dst_relay = route_to_org.next_hop
        if packet.ttl > 0:
            self.drone.output_buffer.append(packet)

    def _process_rerr(self, packet: aodv.RErrPacket):
        unreachable: list[tuple[NetAddr, int | None]] = []
        for addr, seq in packet.destinations:
            route = self.routing_table.get(addr)
            if route is not None and route.next_hop == packet.src:
                unreachable.append((addr, seq))

        if unreachable:
            self._generate_rerr(unreachable)

    def _generate_rerr(self, destinations: list[tuple[NetAddr, int | None]]):
        new_destinations = []
        for dest, seq in destinations:
            route = self.routing_table.get(dest)
            if not route:
                route = RoutingTableEntry(
                    dest=dest,
                    seq_number=-1,
                    seq_number_valid=False,
                    is_valid=False,
                    hop_count=-1,
                    next_hop=-1,
                    expiry_time=-1,
                )
                self.routing_table[dest] = route

            if seq:
                route.seq_number = seq
            else:
                if route.seq_number_valid:
                    route.seq_number += 1

            route.is_valid = False
            route.expiry_time = self.drone.time + DELETE_PERIOD
            new_destinations.append((dest, route.seq_number))

        packet = aodv.RErrPacket(
            source=self.drone.address,
            destination=config.BROADCAST_ADDRESS,
            timestamp=self.drone.time,
            destinations=new_destinations,
        )
        self.drone.output_buffer.append(packet)

    def _generate_rrep(self, packet: aodv.RReqPacket):
        dst_addr = packet.dst_addr
        if dst_addr == self.drone.address:
            if packet.dst_seq is not None:
                # TODO: this breaks
                # assert packet.dst_seq <= self.sequence_number + 1
                self.sequence_number = max(packet.dst_seq, self.sequence_number)

            hop_count = 0
            dst_seq = self.sequence_number
            lifetime = MY_ROUTE_TIMEOUT
        else:
            route = self.routing_table[dst_addr]
            hop_count = route.hop_count
            dst_seq = route.seq_number
            lifetime = route.expiry_time - self.drone.time
            # update precursors
            self.routing_table[packet.src].precursors.add(route.next_hop)

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

        if packet.src_relay != rrep.dst:
            rrep.dst_relay = packet.src_relay
        if self.drone.address == 3 and rrep.dst_addr == 4:
            if config.DEBUG:
                print(f"{self.drone.address=} SENDING RREP {rrep=}")
        self.drone.output_buffer.append(rrep)

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
        self.drone.output_buffer.append(grat_rrep)

    def _request_route(self, dest: NetAddr):
        rreq_info = self.pending_rreq_buffer.get(dest)

        if rreq_info is not None:  # route already requested
            # backoff not complete
            if rreq_info.time + rreq_info.backoff >= self.drone.time:
                return

            if rreq_info.ttl < TTL_THRESHOLD:
                rreq_info.time = self.drone.time
                rreq_info.backoff *= 2
            elif rreq_info.retry_count < RREQ_RETRIES:
                rreq_info.retry_count += 1
            else:
                self._route_timeout(dest)
                return
        else:
            rreq_info = RReqInfo(
                time=self.drone.time,
                ttl=TTL_START,
                retry_count=0,
                backoff=NET_TRAVERSAL_TIME,
            )

        self.sequence_number += 1
        packet = aodv.RReqPacket(
            source=self.drone.address,
            destination=config.BROADCAST_ADDRESS,
            timestamp=self.drone.time,
            rreq_id=next(self.rreq_id),
            hop_count=0,
            dst_addr=dest,
            org_seq=self.sequence_number,
        )
        self.drone.output_buffer.append(packet)

    def relay_selection(self, packet: Packet) -> NetAddr | None:
        route_info = self.routing_table.get(packet.dst)
        if route_info is not None and route_info.is_valid:
            routes_to_extend = [
                route_info,  # destination
                self.routing_table.get(route_info.hop_count),  # next hop
                self.routing_table.get(packet.src),  # source
            ]
            for r in routes_to_extend:
                if r is not None:
                    r.expiry_time = max(
                        r.expiry_time, self.drone.time + ACTIVE_ROUTE_TIMEOUT
                    )

            return route_info.next_hop

        if packet.src == self.drone.address:
            self._request_route(packet.dst)
        else:
            self._generate_rerr([(packet.dst, None)])
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
            org_addr=self.drone.address,
        )
        packet.ttl = 1
        return packet  # type: ignore

    def routing_control(self, cur_step: int) -> list[Packet]:
        self._clean()
        return super().routing_control(cur_step)

    def _clean(self):
        # do not clean too often
        if (self.drone.time + 2) % 10 != 0:
            return
        # find newly expired routes and delete old ones
        newly_broken = []
        to_delete = []
        for addr, route in self.routing_table.items():
            if self.drone.time > route.expiry_time:
                if route.is_valid:
                    newly_broken.append(addr)
                else:
                    to_delete.append(addr)

                route.is_valid = False
        for addr in to_delete:
            del self.routing_table[addr]

        unreachable = set()
        while newly_broken:
            current = newly_broken.pop()
            unreachable.add(current)
            for addr, route in self.routing_table.items():
                if route.next_hop == current and route.next_hop not in unreachable:
                    newly_broken.append(addr)

        destinations: list[tuple[NetAddr, int | None]] = [
            (addr, None) for addr in unreachable
        ]
        if destinations:
            self._generate_rerr(destinations)

        to_delete = []
        for key, timestamp in self.received_rreqs.items():
            if timestamp + PATH_DISCOVERY_TIME < self.drone.time:
                to_delete.append(key)

        for key in to_delete:
            del self.received_rreqs[key]

    def _route_timeout(self, destination: NetAddr):
        to_drop = []
        logger.debug(f"Route timeout for destination {destination}")
        for packet in self.drone.buffer:
            if packet.dst == destination:
                to_drop.append(packet.identifier)
        logger.debug(f"Dropping {len(to_drop)} packets")

        self.drone.buffer = [
            p for p in self.drone.buffer if p.identifier not in to_drop
        ]

    def has_neighbours(self) -> bool:
        return True
