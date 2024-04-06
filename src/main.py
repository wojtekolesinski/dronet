from simulation.simulator import Simulator


# os.environ["SDL_VIDEODRIVER"] = "dummy"
def main():
    """the place where to run simulations and experiments."""

    import os

    print(os.getcwd())
    os.chdir("..")
    print(os.getcwd())
    sim = (
        Simulator()
    )  # empty constructor means that all the parameters of the simulation are taken from utilities.config.py
    sim.run()  # run the simulation
    sim.close()


if __name__ == "__main__":
    main()
