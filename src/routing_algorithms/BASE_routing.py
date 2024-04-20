import abc

import config
from entities.packets import ACKPacket, DataPacket, HelloPacket, Packet
from entities.uav_entities import Drone
from simulation.metrics import Metrics
from simulation.net import MediumDispatcher
from utilities import utilities as util


class BASE_routing(metaclass=abc.ABCMeta):

    def __init__(self, drone: Drone, simulator):
        """The drone that is doing routing and simulator object."""

        self.drone = drone
        self.current_n_transmission = 0
        self.hello_messages = {}  # { drone_id : most recent hello packet}
        self.network_disp: MediumDispatcher = simulator.network_dispatcher
        self.simulator = simulator

    @abc.abstractmethod
    def relay_selection(
        self, geo_neighbors: list[HelloPacket], packet: Packet
    ) -> HelloPacket:
        pass

    def handle_hello(self, packet: HelloPacket):
        src_id = packet.src
        self.hello_messages[src_id] = packet

    def drone_identification(self, cur_step: int):
        """handle drone hello messages to identify neighbors"""
        if cur_step % config.HELLO_DELAY != 0:  # still not time to communicate
            return

        my_hello = HelloPacket(
            self.drone.address,
            config.BROADCAST_ADDRESS,
            cur_step,
            self.drone.coords,
            self.drone.speed,
            self.drone.next_target(),
        )

        self.network_disp.send(
            my_hello, self.drone.coords, self.drone.communication_range
        )

    def routing(self, cur_step: int):
        # set up this routing pass
        self.drone_identification(cur_step)

        self.send_packets(cur_step)


    def send_packets(self, cur_step):
        """procedure 3 -> choice next hop and try to send it the data packet"""

        if (
            util.euclidean_distance(self.simulator.depot.coords, self.drone.coords)
            <= self.simulator.depot_com_range
        ):
            # add error in case
            self.transfer_to_depot()
            self.drone.move_to_depot = False
            self.current_n_transmission = 0
            return

        if cur_step % self.simulator.drone_retransmission_delta == 0:

            # TODO: the packets needs to have a relay address and a final address,
            # if the final address doesn't match the current one, the packet needs to be relayed
            opt_neighbors = []
            for hello_packet in self.hello_messages.values():
                # check if packet is too old
                if hello_packet.timestamp < cur_step - config.OLD_HELLO_PACKET:
                    continue
                opt_neighbors.append(hello_packet)

            if len(opt_neighbors) == 0:
                return

            # send packets
            for pkd in self.drone.all_packets():
                Metrics.instance().mean_numbers_of_possible_relays.append(
                    len(opt_neighbors)
                )
                # compute score
                best_neighbor = self.relay_selection(opt_neighbors, pkd)
                if best_neighbor is not None:
                    pkd.dst_relay = best_neighbor.src
                    self.network_disp.send(
                        pkd, self.drone.coords, self.drone.communication_range
                    )

                self.current_n_transmission += 1

    def transfer_to_depot(self):
        """self.drone is close enough to depot and offloads its buffer to it, restarting the monitoring
        mission from where it left it
        """
        for packet in self.drone.output_buffer:
            self.network_disp.send(
                packet, self.drone.coords, self.drone.communication_range
            )
        self.drone.empty_buffer()
        self.drone.move_to_depot = False
