import config
from entities.communicating_entity import CommunicatingEntity
from simulation.metrics import Metrics
from simulation.net import MediumDispatcher
from utilities.types import NetAddr, Point


class Depot(CommunicatingEntity):
    """The depot is an Entity."""

    def __init__(
        self, address: NetAddr, coords: Point, network: MediumDispatcher, simulator
    ):
        super().__init__(
            id(self),
            coords,
            address,
            network,
            -1,
            config.depot_communication_range,
            10_000,
        )

        self.simulator = simulator

    def transfer_notified_packets(self, current_drone, cur_step):
        """function called when a drone wants to offload packets to the depot"""

        packets_to_offload = current_drone.all_packets()
        self.buffer += packets_to_offload

        for pck in packets_to_offload:

            if config.routing_algorithm.name not in "GEO" "RND" "GEOS":

                feedback = 1
                delivery_delay = cur_step - pck.event_ref.current_time

                for drone in self.simulator.drones:
                    drone.routing_algorithm.feedback(
                        current_drone,
                        pck.event_ref.identifier,
                        delivery_delay,
                        feedback,
                    )
            # print(f"DEPOT -> Drone {current_drone.identifier} packet: {pck.event_ref} total packets in sim: {len(Metrics.instance().drones_packets_to_depot)}")

            # add metrics: all the packets notified to the depot
            Metrics.instance().drones_packets_to_depot.add((pck, cur_step))
            Metrics.instance().drones_packets_to_depot_list.append((pck, cur_step))
            pck.time_delivery = cur_step
