import abc

from scipy.stats import norm

import config
from entities.packets import ACKPacket, DataPacket, HelloPacket, Packet
from simulation.metrics import Metrics
from src.entities.uav_entities import Drone
from utilities import utilities as util


class BASE_routing(metaclass=abc.ABCMeta):

    def __init__(self, drone: Drone, simulator):
        """The drone that is doing routing and simulator object."""

        self.drone = drone
        self.current_n_transmission = 0
        self.hello_messages = {}  # { drone_id : most recent hello packet}
        self.network_disp = simulator.network_dispatcher
        self.simulator = simulator
        self.no_transmission = False

    @abc.abstractmethod
    def relay_selection(self, geo_neighbors, packet):
        pass

    def routing_close(self):
        self.no_transmission = False

    def drone_reception(self, src_drone, packet: Packet, current_ts):
        """handle reception an ACKs for a packets"""
        if isinstance(packet, HelloPacket):
            src_id = packet.src_drone.identifier
            self.hello_messages[src_id] = packet  # add packet to our dictionary

        elif isinstance(packet, DataPacket):
            self.no_transmission = True
            self.drone.accept_packets([packet])
            # build ack for the reception
            ack_packet = ACKPacket(self.drone, src_drone, packet, current_ts)
            self.unicast_message(ack_packet, self.drone, src_drone, current_ts)

        elif isinstance(packet, ACKPacket):
            self.drone.remove_packets([packet.acked_packet])
            # packet.acked_packet.optional_data
            # print(self.is_packet_received_drone_reward, "ACK", self.drone.identifier)
            if self.drone.buffer_length() == 0:
                self.current_n_transmission = 0
                self.drone.move_routing = False

    def drone_identification(self, drones, cur_step: int):
        """handle drone hello messages to identify neighbors"""
        # if self.drone in drones: drones.remove(self.drone)  # do not send hello to yourself
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

        self.network_disp.send(my_hello)
        self.broadcast_message(my_hello, self.drone, drones, cur_step)

    def routing(self, depot, drones, cur_step):
        # set up this routing pass
        self.drone_identification(drones, cur_step)

        self.send_packets(cur_step)

        # close this routing pass
        self.routing_close()

    def send_packets(self, cur_step):
        """procedure 3 -> choice next hop and try to send it the data packet"""

        # FLOW 0
        if self.no_transmission or self.drone.buffer_length() == 0:
            return

        # FLOW 1
        if (
            util.euclidean_distance(self.simulator.depot.coords, self.drone.coords)
            <= self.simulator.depot_com_range
        ):
            # add error in case
            self.transfer_to_depot(self.drone.depot, cur_step)

            self.drone.move_routing = False
            self.current_n_transmission = 0
            return

        if cur_step % self.simulator.drone_retransmission_delta == 0:

            opt_neighbors = []
            for hpk_id in self.hello_messages:
                hpk: HelloPacket = self.hello_messages[hpk_id]

                # check if packet is too old
                if hpk.time_step_creation < cur_step - config.OLD_HELLO_PACKET:
                    continue

                opt_neighbors.append((hpk, hpk.src_drone))

            if len(opt_neighbors) == 0:
                return

            # send packets
            for pkd in self.drone.all_packets():

                Metrics.instance().mean_numbers_of_possible_relays.append(
                    len(opt_neighbors)
                )

                best_neighbor = self.relay_selection(
                    opt_neighbors, pkd
                )  # compute score

                if best_neighbor is not None:

                    self.unicast_message(pkd, self.drone, best_neighbor, cur_step)

                self.current_n_transmission += 1

    def geo_neighborhood(self, drones, no_error=False):
        """
        @param drones:
        @param no_error:
        @return: A list all the Drones that are in self.drone neighbourhood (no matter the distance to depot),
            in all direction in its transmission range, paired with their distance from self.drone
        """

        closest_drones = (
            []
        )  # list of this drone's neighbours and their distance from self.drone: (drone, distance)

        for other_drone in drones:

            if self.drone.identifier != other_drone.identifier:  # not the same drone
                drones_distance = util.euclidean_distance(
                    self.drone.coords, other_drone.coords
                )  # distance between two drones

                if drones_distance <= min(
                    self.drone.communication_range, other_drone.communication_range
                ):  # one feels the other & vv

                    # CHANNEL UNPREDICTABILITY
                    if self.channel_success(drones_distance, no_error=no_error):
                        closest_drones.append((other_drone, drones_distance))

        return closest_drones

    def broadcast_message(self, packet, src_drone, dst_drones, curr_step):
        """send a message to my neigh drones"""
        for d_drone in dst_drones:
            self.unicast_message(packet, src_drone, d_drone, curr_step)

    def unicast_message(self, packet, src_drone, dst_drone, curr_step):
        """send a message to my neigh drones"""
        # Broadcast using Network dispatcher
        self.simulator.network_dispatcher.send_packet_to_medium(
            packet, src_drone, dst_drone, curr_step + config.LIL_DELTA
        )

    def transfer_to_depot(self, depot, cur_step):
        """self.drone is close enough to depot and offloads its buffer to it, restarting the monitoring
        mission from where it left it
        """
        depot.transfer_notified_packets(self.drone, cur_step)
        self.drone.empty_buffer()
        self.drone.move_routing = False

    # --- PRIVATE ---
