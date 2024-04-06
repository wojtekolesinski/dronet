from simulation.simulator import Simulator

# os.environ["SDL_VIDEODRIVER"] = "dummy"
def main():
    """ the place where to run simulations and experiments. """

    sim = Simulator()   # empty constructor means that all the parameters of the simulation are taken from src.utilities.config.py
    sim.run()            # run the simulation
    sim.close()


if __name__ == "__main__":
    main()
