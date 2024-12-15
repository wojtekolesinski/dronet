from routing_algorithms.base import NeighbourNode
from utilities.types import NetAddr, Point

"""
This file contains all the constants and parameters of the simulator.
It comes handy when you want to make one shot simulations, making parameters and constants vary in every
simulation. For an extensive experimental campaign read the header at src.simulator.

Attributes that one needs tweak often are tagged with # ***
"""

# ----------------------------------------------------------------------------------
#
#                  ██████  ██████  ███    ██ ███████ ██  ██████
#                 ██      ██    ██ ████   ██ ██      ██ ██
#                 ██      ██    ██ ██ ██  ██ █████   ██ ██   ███
#                 ██      ██    ██ ██  ██ ██ ██      ██ ██    ██
#                  ██████  ██████  ██   ████ ██      ██  ██████
#
# ----------------------------------------------------------------------------------

# ----------------------- PATH DRONES -----------------------------------------#
CIRCLE_PATH = False  # bool: whether to use cirlce paths around the depot
DEMO_PATH = False  # bool: whether to use handcrafted tours or not
# to set up handcrafted torus see utilities.utilities
PATH_FROM_JSON = (
    True  # bool: whether to use the path (for drones) store in the JSONS_PATH_PREFIX,
)
# otherwise path are generated online
JSONS_PATH_PREFIX = (
    "data/tours/RANDOM_missions01.json"  # str: the path to the drones tours,
    # "data/tours/test_routing_no_movement.json"  # str: the path to the drones tours,
)
# the {} should be used to specify the seed -> es. data/tours/RANDOM_missions1.json for seed 1.
RANDOM_STEPS = [
    250,
    500,
    700,
    900,
    1100,
    1400,
]  # the step after each new random directions is taken, in case of dynamic generation
RANDOM_START_POINT = (
    True  # bool whether the drones start the mission at random positions
)

# ------------------------------- CONSTANTS ------------------------------- #

DEBUG = False  # bool: whether to print debug strings or not.
EXPERIMENTS_DIR = (
    "data/evaluation_tests/"  # output data : the results of the simulation
)

# drawing
show_plot = False  # bool: whether to plot or not the simulation.
WAIT_SIM_STEP = (
    0  # .1     # float: seconds, pauses the rendering for 'DELAY_PLOT' seconds.
)
SKIP_SIM_STEP = (
    10  # int: steps, plot the simulation every 'RENDERING_STEP' steps. At least 1.
)
DRAW_SIZE = 700  # int: size of the drawing window.
IS_SHOW_NEXT_TARGET_VEC = (
    True  # bool : whether show the direction and next target of the drone
)

SAVE_PLOT = False  # bool: whether to save the plots of the simulation or not.
SAVE_PLOT_DIR = "data/plots/"


# add constants here...

# ----------------------------- SIMULATION PARAMS. ---------------------------- #
len_simulation = 10000  # int: steps of simulation. # ***
time_step_duration = 0.150  # float: seconds duration of a step in seconds.
seed = 10  # int: seed of this simulation.

n_drones = 3  # int: number of drones. # ***
# n_drones = 50  # int: number of drones. # ***
env_width = 1500  # int: meters, width of environment.
env_height = 1500  # int: height of environment.

# events
event_duration = 1500  # SIM_DURATION  # int: steps, number of time steps that an event lasts  -> to seconds = step * step_duration.
event_generation_delay = 20  # int: steps, a new packet is felt (generated on the drone) every 'D_FEEL_EVENT' steps. # ***
event_generation_prob = 0.8  # float: probability that the drones feels the event generated on the drone. # ***

""" e.g. given D_FEEL_EVENT = 500, P_FEEL_EVENT = .5, every 500 steps with probability .5 the drone will feel an event."""

# drones
drone_communication_range = 400  # float: meters, communication range of the drones.
drone_sensing_range = 0  # int: meters, the sensing range of the drones.
drone_speed = 8  # int: m/s, drone speed.
drone_max_buffer_size = 10000  # int: max number of packets in the buffer of a drone.
drone_max_energy = 1000000  # int: max energy of a drone.

# depot
depot_communication_range = 400  # int: meters, communication range of the depot.
depot_coordinates = (750, 0)  # (int, int): coordinates of the depot.


# ------------------------------- ROUTING PARAMS. ------------------------------- #


communication_success_prob: float = (
    1  # float: probability to have success in a communication.
)
GUASSIAN_SCALE = 0.6  # float [0,1]: scale the error probability of the guassian -> success * GUASSIAN_SCALER
packets_max_ttl = (
    64  # float: threshold in the maximum number of hops. Causes loss of packets.
)
retransmission_delay = 30  # int: how many time steps to wait before transmit again (for k retransmissions). # ---  #delta_k

# ------------------------------------------- ROUTING MISC --------------------------------- #
HELLO_DELAY = (
    30  # int : how many time steps wait before transmit again an hello message
)
RECEPTION_GRANTED = (
    0.95  # float : the min amount of success to evalute a neigh as relay
)
LIL_DELTA = 1  # INT:  > 0
OLD_HELLO_PACKET = 50

ROOT_EVALUATION_DATA = "data/evaluation_tests/"

NN_MODEL_PATH = "data/nnmodels/"
BROADCAST_ADDRESS: NetAddr = 255
DEPOT_ADDRESS: NetAddr = 1

# --------------- new cell probabilities -------------- #
CELL_PROB_SIZE_R = 1.875  # the percentage of cell size with respect to drone com range
ENABLE_PROBABILITIES = False


DEPOT_NODE = NeighbourNode(
    DEPOT_ADDRESS,
    len_simulation,
    depot_coordinates,
    Point(),
    0,
)


# olsr
vtime: int = 45
duplicate_hold_time: int = 60


from enums import ChannelError, RoutingAlgorithm

routing_algorithm = RoutingAlgorithm.AODV
communication_error_type = ChannelError.GAUSSIAN
