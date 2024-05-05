import itertools
from collections import defaultdict
from dataclasses import dataclass
from typing import Literal

import config
from entities.communicating_entity import CommunicatingEntity
from entities.packets.base import DataPacket, HelloPacket, Packet
from entities.packets.olsr import (LinkCode, LinkType, NeighbourType,
                                   OLSRDataPacket, OLSRHelloPacket, OLSRPacket,
                                   OLSRTopologyControlPacket)
from routing_algorithms.base import BaseRouting
from src.entities.event import Event
from utilities.types import NetAddr


@dataclass
class LinkTuple:
    address: NetAddr
    sym_time: int
    asym_time: int
    time: int


@dataclass
class NeigbourTuple:
    address: NetAddr
    status: Literal["sym", "not_sym"]
    willingness: int


@dataclass
class TwoHopNeigbourTuple:
    address: NetAddr
    two_hop_address: NetAddr
    time: int


@dataclass
class MprSelectorTuple:
    address: NetAddr
    time: int


@dataclass
class TopologyTuple:
    address: NetAddr
    last_address: NetAddr
    seq: int
    time: int


@dataclass
class RouteTuple:
    destination_address: NetAddr
    next_address: NetAddr
    dist: int


@dataclass
class DuplicateTuple:
    address: NetAddr
    seq: int
    retransmitted: bool
    time: int


def get_link_type(link: LinkTuple, cur_time: int) -> LinkType:
    if link.sym_time >= cur_time:
        return "SYM_LINK"
    if link.asym_time >= cur_time:
        return "ASYM_LINK"
    return "LOST_LINK"


class OLSRRouting(BaseRouting):
    links: dict[NetAddr, LinkTuple]
    neighbours: dict[NetAddr, NeigbourTuple]
    two_hop_neighbours: dict[tuple[NetAddr, NetAddr], TwoHopNeigbourTuple]
    mprs: set[NetAddr]
    mpr_selectors: dict[NetAddr, MprSelectorTuple]
    topology_info: set[TopologyTuple]
    routing_table: set[RouteTuple]
    duplicate_set: dict[tuple[NetAddr, int], DuplicateTuple]
    willingness: int

    def __init__(self, drone: CommunicatingEntity):
        super().__init__(drone)
        self.links = dict()
        self.neighbours = dict()
        self.two_hop_neighbours = dict()
        self.mprs = set()
        self.mpr_selectors = dict()
        self.topology_info = set()
        self.duplicate_set = dict()
        self.willingness = 3
        self.ansn = itertools.count(0, 1)
        self.sequence_number = itertools.count(0, 1)

    def process(self, packet: Packet):
        assert isinstance(packet, OLSRPacket)
        if packet.ttl <= 0:
            return

        if (packet.src, packet.sequence_number) in self.duplicate_set:
            return

        self.duplicate_set[(packet.src, packet.sequence_number)] = DuplicateTuple(
            address=packet.src,
            seq=packet.sequence_number,
            retransmitted=False,
            time=self.drone.time,
        )

        if isinstance(packet, OLSRTopologyControlPacket):
            self.process_tc(packet)
        super().process(packet)

    def should_forward(self, packet: Packet) -> bool:
        assert isinstance(packet, OLSRPacket)
        if (
            packet.src,
            packet.sequence_number,
        ) in self.duplicate_set and self.duplicate_set[
            (packet.src, packet.sequence_number)
        ].retransmitted:
            return False

        if packet.src_relay not in self.neighbours:
            return False

        if self.neighbours[packet.src_relay].status != "sym":
            return False

        retransmit = packet.src_relay in self.mpr_selectors and packet.ttl > 1

        dt = self.duplicate_set.get(
            (packet.src, packet.sequence_number),
            DuplicateTuple(
                address=packet.src,
                seq=packet.sequence_number,
                retransmitted=retransmit,
                time=self.drone.time + config.duplicate_hold_time,
            ),
        )
        self.duplicate_set[(packet.src, packet.sequence_number)] = dt
        packet.ttl -= 1
        packet.hop_count += 1
        return True

    def get_neighbour_type(self, address: NetAddr) -> NeighbourType:
        if address in self.mprs:
            return "MPR_NEIGH"

        neighbours_for_link = list(
            filter(lambda n: n.address == address, self.neighbours.values())
        )
        if len(neighbours_for_link) == 0:
            raise Exception("Cannot determine neighbour type")

        neighbour = neighbours_for_link[0]
        if neighbour.status == "sym":
            return "SYM_NEIGH"
        return "NOT_NEIGH"

    def drone_identification(self, cur_step: int) -> HelloPacket | None:
        if super().drone_identification(cur_step) is None:
            return
        links_for_hello: dict[LinkCode, list[NetAddr]] = defaultdict(list)
        for link in self.links.values():
            code = LinkCode(
                get_link_type(link, cur_step), self.get_neighbour_type(link.address)
            )
            links_for_hello[code].append(link.address)

        covered_addresses = list(itertools.chain(links_for_hello.values()))
        for neigh in self.neighbours.values():
            if neigh.address not in covered_addresses:
                code = LinkCode("UNSPEC_LINK", self.get_neighbour_type(neigh.address))
                links_for_hello[code].append(neigh.address)

        olsr_packet = OLSRHelloPacket(
            self.drone.address,
            config.BROADCAST_ADDRESS,
            cur_step,
            self.drone.coords,
            self.drone.speed,
            self.drone.next_target(),
            next(self.sequence_number),
        )
        olsr_packet.willingness = self.willingness
        olsr_packet.htime = config.HELLO_DELAY
        olsr_packet.links = links_for_hello

        return olsr_packet

    def process_hello(self, packet: HelloPacket):
        assert isinstance(packet, OLSRHelloPacket)

        validity_time = self.drone.time + packet.vtime

        # update link set
        link = self.links.get(
            packet.src,
            LinkTuple(
                address=packet.src,
                sym_time=self.drone.time - 1,
                asym_time=-1,
                time=validity_time,
            ),
        )
        link.asym_time = validity_time
        for link_code, addresses in packet.links.items():
            if self.drone.address in addresses:
                if link_code.link_type == "LOST_LINK":
                    link.sym_time = self.drone.time - 1
                elif link_code.link_type in ("ASYM_LINK", "SYM_LINK"):
                    link.sym_time = validity_time
                    link.time = link.sym_time + config.OLD_HELLO_PACKET
                break
        link.time = max(link.time, link.asym_time)
        self.links[link.address] = link

        # update neigbour set
        neighbour = self.neighbours.get(
            packet.src,
            NeigbourTuple(
                address=packet.src,
                status="not_sym",
                willingness=packet.willingness,
            ),
        )
        if link.sym_time >= self.drone.time:
            neighbour.status = "sym"
        self.neighbours[packet.src] = neighbour

        # update 2-hop neighbours
        for link_code, addresses in packet.links.items():
            if link_code.neighbour_type in ("SYM_NEIGH", "MPR_NEIGH"):
                for address in addresses:
                    if address == self.drone.address:
                        continue

                    neighbour = self.two_hop_neighbours.get(
                        (packet.src, address),
                        TwoHopNeigbourTuple(packet.src, address, validity_time),
                    )
                    self.two_hop_neighbours[(packet.src, address)] = neighbour
            elif link_code.neighbour_type == "NOT_NEIGH":
                for address in addresses:
                    del self.two_hop_neighbours[(packet.src, address)]

        # update mpr selectors
        for link_code, addresses in packet.links.items():
            if self.drone.address in addresses:
                if link_code.neighbour_type == "MPR_NEIGH":
                    selector = self.mpr_selectors.get(
                        packet.src, MprSelectorTuple(packet.src, validity_time)
                    )
                    selector.time = validity_time
                    self.mpr_selectors[packet.src] = selector
                break

    def process_tc(self, packet: OLSRTopologyControlPacket):
        if packet.src_relay not in self.neighbours:
            return

        for tt in self.topology_info:
            if tt.last_address == packet.src and tt.seq > packet.ansn:
                return

        temp_topology_dict = dict()
        for tt in self.topology_info:
            if tt.last_address == packet.src and tt.seq < packet.ansn:
                continue
            temp_topology_dict[(tt.last_address, tt.address)] = tt

        for neigbhour in packet.advertised_neigbours:
            tt = temp_topology_dict.get(
                (packet.src, neigbhour),
                TopologyTuple(
                    address=neigbhour,
                    last_address=packet.src,
                    seq=packet.ansn,
                    time=self.drone.time + packet.vtime,
                ),
            )
            temp_topology_dict[(tt.last_address, tt.address)] = tt

        self.topology_info = set(temp_topology_dict.values())

    def update_routing_table(self):
        self.routing_table = set()

        for n in self.neighbours.values():
            if n.status == "sym":
                self.routing_table.add(
                    RouteTuple(
                        destination_address=n.address, next_address=n.address, dist=1
                    )
                )

        for n in self.two_hop_neighbours.values():
            self.routing_table.add(
                RouteTuple(
                    destination_address=n.two_hop_address,
                    next_address=n.address,
                    dist=2,
                )
            )

        dist = 2
        visited = {r.destination_address: r.dist for r in self.routing_table}
        new_neigbour = True
        while new_neigbour:
            new_neigbour = False
            for tt in self.topology_info:
                if tt.address in visited or tt.last_address not in visited:
                    continue

                if visited[tt.last_address] != dist:
                    continue

                new_neigbour = True
                self.routing_table.add(
                    RouteTuple(
                        destination_address=tt.address,
                        next_address=tt.last_address,
                        dist=dist + 1,
                    )
                )
                visited[tt.address] = dist + 1

    def update_mprs(self):
        # TODO: implement the full selection heuristic
        self.mprs = set()
        for n in self.neighbours.values():
            if self.get_neighbour_type(n.address) == "SYM_NEIGH":
                self.mprs.add(n.address)

    def update_neighbours(self, cur_step: int):

        to_delete: list[NetAddr] = list()
        for address, link in self.links.items():
            if link.time < cur_step:
                to_delete.append(address)

        for addr in to_delete:
            del self.links[addr]
            del self.neighbours[addr]
            for key in self.two_hop_neighbours:
                if key[0] == addr:
                    del self.two_hop_neighbours[key]

        to_delete = list()
        for address, selector in self.mpr_selectors.items():
            if selector.time < cur_step:
                to_delete.append(address)

        for address in to_delete:
            del self.mpr_selectors[address]

        self.update_mprs()
        self.update_routing_table()

    def routing_control(self, cur_step: int) -> list[Packet]:
        packets = super().routing_control(cur_step)
        tc_packet = OLSRTopologyControlPacket(
            self.drone.address,
            config.BROADCAST_ADDRESS,
            cur_step,
            next(self.sequence_number),
            next(self.ansn),
            list(self.neighbours.keys()),
        )
        packets.append(tc_packet)
        return packets

    def relay_selection(self, packet: Packet) -> NetAddr | None:
        lookup: dict[NetAddr, list[RouteTuple]] = defaultdict(list)
        for rt in self.routing_table:
            lookup[rt.destination_address].append(rt)

        if packet.dst not in lookup:
            return None

        next_hop = lookup[packet.dst][0]
        while next_hop.dist > 1:
            next_hop = lookup[next_hop.next_address][0]

        return next_hop.destination_address

    def make_data_packet(self, event: Event, cur_step: int) -> DataPacket:
        return OLSRDataPacket(
            self.drone.address,
            config.DEPOT_ADDRESS,
            cur_step,
            next(self.sequence_number),
            event,
        )
