import logging

from simulation.simulator import Simulator

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


def main():
    """the place where to run simulations and experiments."""
    import os

    # os.chdir("..")

    logger.info("Initiating the simulator")
    sim = (
        Simulator()
    )  # empty constructor means that all the parameters of the simulation are taken from utilities.config.py
    logger.info("Running the simulation")
    sim.run()  # run the simulation
    sim.close()


if __name__ == "__main__":
    main()
