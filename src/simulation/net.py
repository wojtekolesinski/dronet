import utilities.utilities as util
from entities.packets import DataPacket
from simulation.metrics import Metrics


class MediumDispatcher:

    def __init__(self):
        self.packets = []
        self.metric_class = Metrics.instance()

    def send_packet_to_medium(self, packet, src_drone, dst_drone, to_send_ts):

        if isinstance(packet, DataPacket):
            # self.metric_class.all_data_packets_in_simulation += 1
            pass
        else:
            self.metric_class.all_control_packets_in_simulation += 1

        self.packets.append((packet, src_drone, dst_drone, to_send_ts))

    def run_medium(self, current_ts):
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
