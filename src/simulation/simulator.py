import logging
import math
import time
from collections import defaultdict
from datetime import datetime
from typing import Any

import numpy as np
import pygame
from tqdm import tqdm

import config
from drawing import pp_draw
from entities.depot import Depot
from entities.drone import Drone
from entities.environment import Environment
from simulation.metrics import Metrics
from simulation.net import MediumDispatcher
from utilities import utilities
from utilities.types import Point

"""
This file contains the Simulation class. It allows to explicit all the relevant parameters of the simulation,
as default all the parameters are set to be those in the config file. For extensive experimental campains, 
you can initialize the Simulator with non default values. 
"""

logger = logging.getLogger(__name__)


class Simulator:

    def __init__(
        self,
        len_simulation: int = config.len_simulation,
        time_step_duration: float = config.time_step_duration,
        seed: int = config.seed,
        n_drones: int = config.n_drones,
        env_width: int = config.env_width,
        env_height: int = config.env_height,
        drone_speed: int = config.drone_speed,
        drone_max_buffer_size: int = config.drone_max_buffer_size,
        drone_max_energy: int = config.drone_max_energy,
        retransmission_delay: int = config.retransmission_delay,
        drone_communication_success_prob: float = config.communication_success_prob,
        depot_communication_range: int = config.depot_communication_range,
        depot_coordinates: Point = config.depot_coordinates,
        event_duration: int = config.event_duration,
        event_generation_prob: float = config.event_generation_prob,
        event_generation_delay: int = config.event_generation_delay,
        packets_max_ttl: int = config.packets_max_ttl,
        show_plot: bool = config.show_plot,
        routing_algorithm=config.routing_algorithm,
        communication_error_type=config.communication_error_type,
        prob_size_cell_r=config.CELL_PROB_SIZE_R,
    ):
        self.drone_speed = drone_speed
        self.drone_max_buffer_size = drone_max_buffer_size
        self.drone_max_energy = drone_max_energy
        self.drone_retransmission_delta = retransmission_delay
        self.drone_communication_success = drone_communication_success_prob
        self.n_drones = n_drones
        self.env_width = env_width
        self.env_height = env_height
        self.depot_com_range = depot_communication_range
        self.depot_coordinates = depot_coordinates
        self.len_simulation = len_simulation
        self.time_step_duration = time_step_duration
        self.seed = seed
        self.event_duration = event_duration
        self.event_max_retrasmission = math.ceil(
            event_duration / retransmission_delay
        )  # 600 esempio
        self.event_generation_prob = event_generation_prob
        self.event_generation_delay = event_generation_delay
        self.packets_max_ttl = packets_max_ttl
        self.show_plot = show_plot
        self.routing_algorithm = routing_algorithm
        self.communication_error_type = communication_error_type

        # --------------- cell for drones -------------
        self.prob_size_cell_r = prob_size_cell_r
        self.prob_size_cell = int(
            config.drone_communication_range * self.prob_size_cell_r
        )
        self.cell_prob_map: dict[Any, list[int]] = defaultdict(lambda: [0, 0, 0])

        self.sim_save_file = config.SAVE_PLOT_DIR + self.__sim_name()
        self.path_to_depot = None

        # Setup vari
        # for stats
        self.metrics = Metrics.instance()

        # setup network
        self.__setup_net_dispatcher()

        # Setup the simulation
        self.__set_simulation()
        self.__set_metrics()

        self.simulation_name = (
            "out__"
            + str(self.seed)
            + "_"
            + str(self.n_drones)
            + "_"
            + str(self.routing_algorithm)
        )
        self.simulation_test_dir = self.simulation_name + "/"

        self.start = time.time()
        self.event_generator = utilities.EventGenerator(
            self.seed, self.event_generation_delay
        )

    def __setup_net_dispatcher(self):
        self.network_dispatcher = MediumDispatcher()

    def __set_metrics(self):
        """the method sets up all the parameters in the metrics class"""
        self.metrics.info_mission(
            {
                "len_simulation": self.len_simulation,
                "time_step_duration": self.time_step_duration,
                "seed": self.seed,
                "n_drones": self.n_drones,
                "env_width": self.env_width,
                "env_height": self.env_height,
                "drone_com_range": config.drone_communication_range,
                "drone_sen_range": config.drone_sensing_range,
                "drone_speed": self.drone_speed,
                "drone_max_buffer_size": self.drone_max_buffer_size,
                "drone_max_energy": self.drone_max_energy,
                "drone_retransmission_delta": self.drone_retransmission_delta,
                "drone_communication_success": self.drone_communication_success,
                "depot_com_range": self.depot_com_range,
                "depot_coordinates": self.depot_coordinates,
                "event_duration": self.event_duration,
                "packets_max_ttl": self.packets_max_ttl,
                "show_plot": self.show_plot,
                "routing_algorithm": str(self.routing_algorithm),
                "communication_error_type": str(self.communication_error_type),
            }
        )

    def pause_sim(self):
        def _space_pressed():
            events = pygame.event.get()
            for ev in events:
                if ev.type == pygame.KEYDOWN and ev.key == pygame.K_SPACE:
                    return True
            return False

        if _space_pressed():
            while not _space_pressed():
                time.sleep(0.1)

    def __set_random_generators(self):
        if self.seed is not None:
            self.rnd_network = np.random.RandomState(self.seed)
            self.rnd_event = np.random.RandomState(self.seed)

    def __set_simulation(self):
        """the method creates all the uav entities"""

        self.__set_random_generators()

        self.path_manager = utilities.PathManager(
            config.PATH_FROM_JSON, config.JSONS_PATH_PREFIX, self.seed
        )
        print(f"{config.JSONS_PATH_PREFIX=}")
        self.environment = Environment(self.env_width, self.env_height)

        self.depot = Depot(
            config.DEPOT_ADDRESS,
            config.depot_coordinates,
            self.network_dispatcher,
            self,
        )

        self.drones: list[Drone] = []

        # drone 0 is the first
        for i in range(self.n_drones):
            self.drones.append(
                Drone(
                    i,
                    config.DEPOT_ADDRESS + 1 + i,
                    self.network_dispatcher,
                    self.path_manager.path(i),
                    self.depot,
                )
            )

        self.environment.add_drones(self.drones)
        self.environment.add_depot(self.depot)

        # Set the maximum distance between the drones and the depot
        self.max_dist_drone_depot = utilities.euclidean_distance(
            self.depot.coords, (self.env_width, self.env_height)
        )

        if self.show_plot or config.SAVE_PLOT:
            self.draw_manager = pp_draw.PathPlanningDrawer(
                self.environment.width,
                self.environment.height,
                self.cell_prob_map,
                self.prob_size_cell,
                borders=True,
            )

    def __sim_name(self):
        """
        return the identification name for
        the current simulation. It is useful to print
        the simulation progress
        """
        return "sim_seed" + str(self.seed) + "drones" + str(self.n_drones) + "_step"

    def __plot(self, cur_step):
        """plot the simulation"""
        if cur_step % config.SKIP_SIM_STEP != 0:
            return

        # delay draw
        if config.WAIT_SIM_STEP > 0:
            time.sleep(config.WAIT_SIM_STEP)

        # drones plot
        for drone in self.drones:
            self.draw_manager.draw_drone(drone, cur_step)

        # depot plot
        self.draw_manager.draw_depot(self.depot)

        # events
        for event in self.environment.active_events:
            self.draw_manager.draw_event(event)

        # Draw simulation info
        self.draw_manager.draw_simulation_info(
            cur_step=cur_step, max_steps=self.len_simulation
        )

        # rendering
        self.draw_manager.update(
            show=self.show_plot,
            save=config.SAVE_PLOT,
            filename=self.sim_save_file + str(cur_step) + ".png",
        )

    def increase_meetings_probs(self, drones, cur_step):
        """Increases the probabilities of meeting someone."""
        cells = set()
        for drone in drones:
            coords = drone.coords
            cell_index = utilities.TraversedCells.coord_to_cell(
                size_cell=self.prob_size_cell,
                width_area=self.env_width,
                x_pos=coords[0],  # e.g. 1500
                y_pos=coords[1],
            )  # e.g. 500
            cells.add(int(cell_index[0]))

        for cell, cell_center in utilities.TraversedCells.all_centers(
            self.env_width, self.env_height, self.prob_size_cell
        ):

            index_cell = int(cell[0])
            old_vals = self.cell_prob_map[index_cell]

            if index_cell in cells:
                old_vals[0] += 1

            old_vals[1] = cur_step + 1
            old_vals[2] = old_vals[0] / max(1, old_vals[1])
            self.cell_prob_map[index_cell] = old_vals

    def handle_events_generation(self, cur_step: int):
        index = self.event_generator.handle_events_generation(
            cur_step, len(self.drones)
        )
        if index is not None:
            drone = self.drones[index]
            drone.feel_event(cur_step)

    def apply_for_each_drone(self, method, *args):
        for drone in self.drones:
            method(drone, *args)

    def apply_for_all_entities(self, method, *args):
        for entity in [self.depot, *self.drones]:
            method(entity, *args)

    def run(self):
        """
        Simulator main function
        @return: None
        """

        start = datetime.now()
        # logger.setLevel(logging.INFO)

        def log_elapsed(_name):
            nonlocal start
            end = datetime.now()
            # logger.debug(f"Time elapsed for {_name}: {end - start}s")

        for cur_step in tqdm(range(self.len_simulation), disable=True):
            # for cur_step in tqdm(range(self.len_simulation)):
            self.handle_events_generation(cur_step)

            self.apply_for_each_drone(Drone.set_time, cur_step)
            self.depot.time = cur_step

            # logger.info(
            #     f"Network dispatcher packets: {self.network_dispatcher.packets.__len__()}"
            # )
            self.apply_for_each_drone(Drone.listen)
            self.depot.listen()
            self.network_dispatcher.packets = []

            self.apply_for_each_drone(Drone.update_packets)

            self.apply_for_each_drone(Drone.routing)
            self.depot.routing()

            self.apply_for_each_drone(Drone.send_packets)
            self.depot.send_packets()
            self.depot.buffer = []

            self.apply_for_each_drone(Drone.move, self.time_step_duration)

            # self.apply_for_each_drone(Drone.log_self)

            # for drone in self.drones:
            #     # 1. update expired packets on drone buffers
            #     # 2. try routing packets vs other drones or depot
            #     # 3. actually move the drone towards next waypoint or depot

            # in case we need probability map
            if config.ENABLE_PROBABILITIES:
                self.increase_meetings_probs(self.drones, cur_step)

            if self.show_plot or config.SAVE_PLOT:
                self.pause_sim()
                self.__plot(cur_step)

            # log step time
            if cur_step % 500 == 0:
                log_elapsed(f"step {cur_step}")
            start = datetime.now()

        if config.DEBUG:
            print(
                "End of simulation, sim time: "
                + str((cur_step + 1) * self.time_step_duration)
                + " sec, #iteration: "
                + str(cur_step + 1)
            )

    def close(self):
        """do some stuff at the end of simulation"""
        print("Closing simulation")

        self.print_metrics(plot_id="final")
        self.save_metrics(config.ROOT_EVALUATION_DATA + self.simulation_name)
        Metrics._instance = None

    def print_metrics(self, plot_id="final"):
        """add signature"""
        self.metrics.print_overall_stats()

    def save_metrics(self, filename_path, save_pickle=False):
        """add signature"""
        self.metrics.save_as_json(filename_path + ".json")
        if save_pickle:
            self.metrics.save(filename_path + ".pickle")
