import numpy as np

import config
import utilities.utilities as util
from entities.packets import DataPacket, Packet
from simulation.metrics import Metrics
from utilities.types import NetAddr, Point


class MediumDispatcher:

    def __init__(self):
        self.packets: list[tuple[Packet, Point, int]] = []
        self.metric_class = Metrics.instance()
        self.random = np.random.RandomState(config.seed)
        if config.communication_error_type == config.ChannelError.GAUSSIAN:
            self.buckets_probability = self.__init_guassian()

    def send_packet_to_medium(
        self, packet: Packet, src_drone, dst_drone, to_send_ts: int
    ):
        if isinstance(packet, DataPacket):
            # self.metric_class.all_data_packets_in_simulation += 1
            pass
        else:
            self.metric_class.all_control_packets_in_simulation += 1

        # self.packets.append((packet, src_drone, dst_drone, to_send_ts))

    def send(self, packet: Packet, pos: Point, communication_range: int):
        self.packets.append((packet, pos, communication_range))

    def channel_success(self, drones_distance, no_error=False) -> bool:
        """
        Precondition: two drones are close enough to communicate. Return true if the communication
        goes through, false otherwise.
        """

        if no_error:
            return True

        if config.communication_error_type == config.ChannelError.NO_ERROR:
            return True

        if config.communication_error_type == config.ChannelError.UNIFORM:
            return self.random.rand() <= config.communication_success_prob

        if config.communication_error_type == config.ChannelError.GAUSSIAN:
            return self.random.rand() <= self.gaussian_success_handler(drones_distance)

        raise ValueError("unsupported communication_error_type")

    def listen(
        self, address: NetAddr, pos: Point, communication_range: int
    ) -> list[Packet]:
        packets_to_send = list()

        for packet, packet_pos, comm_range in self.packets:
            if (
                packet.destination != address
                and packet.destination != config.BROADCAST_ADDRESS
            ):
                continue

            distance = util.euclidean_distance(pos, packet_pos)
            if distance > min(communication_range, comm_range):
                continue

            if not self.channel_success(distance, no_error=True):
                continue

            packets_to_send.append(packet)
        return packets_to_send

    def run_medium(self, current_ts: int):
        to_drop_indices = []
        original_self_packets = self.packets[:]
        self.packets = []

        for i in range(len(original_self_packets)):
            packet, src_drone, dst_drone, to_send_ts = original_self_packets[i]

            # not time to send this packet
            if to_send_ts != current_ts:
                continue

            to_drop_indices.append(i)

            # cannot send packets to self
            if src_drone.identifier == dst_drone.identifier:
                continue

            drones_distance = util.euclidean_distance(
                src_drone.coords, dst_drone.coords
            )

            # check if drones are in range
            if drones_distance > min(
                src_drone.communication_range, dst_drone.communication_range
            ):
                continue

            if dst_drone.routing_algorithm.channel_success(
                drones_distance, no_error=True
            ):
                # reception of a packet
                dst_drone.routing_algorithm.drone_reception(
                    src_drone, packet, current_ts
                )

        original_self_packets = [
            original_self_packets[i]
            for i in range(len(original_self_packets))
            if i not in to_drop_indices
        ]
        self.packets = original_self_packets + self.packets

    def gaussian_success_handler(self, drones_distance):
        """get the probability of the drone bucket"""
        bucket_id = int(drones_distance / self.radius_corona) * self.radius_corona
        return self.buckets_probability[bucket_id] * config.GUASSIAN_SCALE

    def __init_guassian(self, mu=0, sigma_wrt_range=1.15, bucket_width_wrt_range=0.5):

        # bucket width is 0.5 times the communication radius by default
        self.radius_corona = int(
            config.drone_communication_range * bucket_width_wrt_range
        )

        # sigma is 1.15 times the communication radius by default
        sigma = config.drone_communication_range * sigma_wrt_range

        max_prob = norm.cdf(mu + self.radius_corona, loc=mu, scale=sigma) - norm.cdf(
            0, loc=mu, scale=sigma
        )

        # maps a bucket starter to its probability of gaussian success
        buckets_probability = {}
        for bk in range(0, config.drone_communication_range, self.radius_corona):
            prob_leq = norm.cdf(bk, loc=mu, scale=sigma)
            prob_leq_plus = norm.cdf(bk + self.radius_corona, loc=mu, scale=sigma)
            prob = (prob_leq_plus - prob_leq) / max_prob
            buckets_probability[bk] = prob

        return buckets_probability
