import itertools
import logging

import tqdm

from simulation.simulator import Simulator

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


def run_experiments():
    import config
    from enums import RoutingAlgorithm

    algorithms = [
        # RoutingAlgorithm.OLSR,
        RoutingAlgorithm.AODV,
        # RoutingAlgorithm.GEO,
        # RoutingAlgorithm.RND,
    ]
    # algorithms = [RoutingAlgorithm.GEO]
    n_drones = [5, 10, 15, 20, 30, 40]
    seeds = [12, 23, 34, 45, 56]

    for algorithm, drones, seed in tqdm.tqdm(
        itertools.product(algorithms, n_drones, seeds)
    ):
        config.routing_algorithm = algorithm
        sim = Simulator(seed=seed, n_drones=drones, routing_algorithm=algorithm)
        logger.info("Running the simulation")
        sim.run()  # run the simulation
        sim.close()


def main():
    """the place where to run simulations and experiments."""
    logger.info("Initiating the simulator")
    # empty constructor means that all the parameters of the simulation are taken from utilities.config.py
    sim = Simulator()
    logger.info("Running the simulation")
    sim.run()  # run the simulation
    sim.close()


if __name__ == "__main__":
    # main()
    run_experiments()
