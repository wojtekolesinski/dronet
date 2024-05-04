import itertools
from collections import defaultdict
from dataclasses import dataclass
from typing import Literal

import config
from entities.communicating_entity import CommunicatingEntity
from entities.packets.base import HelloPacket
from entities.packets.olsr import (LinkCode, LinkType, NeighbourType,
                                   OLSRHelloPacket)
from routing_algorithms.base import BaseRouting
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
    topology_info: dict[NetAddr, TopologyTuple]
    willingness: int

    def __init__(self, drone: CommunicatingEntity):
        super().__init__(drone)
        self.links = dict()
        self.neighbours = dict()
        self.two_hop_neighbours = dict()
        self.mprs = set()
        self.mpr_selectors = dict()
        self.topology_info = dict()
        self.willingness = 3

    def get_neighbour_type(self, address: NetAddr) -> NeighbourType:
        if address in self.mprs:
            return "MPR_NEIGH"

        neighbours_for_link = list(
            filter(lambda n: n.address == address, self.neighbours)
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
        )
        olsr_packet.willingness = self.willingness
        olsr_packet.htime = config.HELLO_DELAY
        olsr_packet.links = links_for_hello

        return olsr_packet

    def handle_hello(self, packet: HelloPacket):
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
                    selector = self.mpr_selectors.get(packet.src, MprSelectorTuple(packet.src, validity_time))
                    selector.time = validity_time
                    self.mpr_selectors[packet.src] = selector
                break


    def update_mprs(self):
        # TODO: implement the full selection heuristic
        self.mprs = set()
        for n in self.neighbours.values()
            if self.get_neighbour_type(n.address) == "SYM_NEIGH":
                self.mprs.add(n.address)

    def update_neighbours(self, cur_step: int):
        return super().update_neighbours(cur_step)
        # TODO: delete links after timeout
        # delete neigbour coressponding to the link
        # update neigbourhood according to 8.5
        # update mprs the same way
