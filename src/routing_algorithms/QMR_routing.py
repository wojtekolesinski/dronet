from src.routing_algorithms.BASE_routing import BASE_routing
from src.utilities import utilities as util
import numpy as np
import sys

class QFanet(BASE_routing):

    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone=drone, simulator=simulator)
        self.sigma = 0.3
        self.taken_actions = {}  # id event : (old_state, old_action)
        self.lr = 0.2 # to tune
        self.look_back = 10 # to tune
        self.weights = np.asarray([i for i in range(self.look_back)], dtype=np.float64) # not sure about this. The paper does not say how to calculate this
        self.weights /= np.sum(self.weights, dtype=np.float64)
        self.qtable = np.zeros(shape=(simulator.n_drones))
        self.rewards_history = np.zeros(shape=(simulator.n_drones, self.look_back))
        # self.weights_history = np.zeros(shape=(simulator.n_drones, self.look_back))


    def feedback(self, drone, id_event, delay, outcome, reward):
        """
        Feedback returned when the packet arrives at the depot or
        Expire. This function have to be implemented in RL-based protocols ONLY
        @param drone: The drone that holds the packet
        @param id_event: The Event id
        @param delay: packet delay
        @param outcome: -1 or 1 (read below)
        @return:
        """

        # Be aware, due to network errors we can give the same event to multiple drones and receive multiple
        # feedback for the same packet!!

        if id_event in self.taken_actions:

            # Drone id and Taken actions
            print(f"\nIdentifier: {self.drone.identifier}, Taken Actions: {self.taken_actions}, Time Step: {self.simulator.cur_step}")

            # take the old action

            state, action = self.taken_actions[id_event]

            # remove the entry, the action has received the feedback
            del self.taken_actions[id_event]

            # reward or update using the old state and the selected action at that time
            
            self.qtable[action] = (1 - self.lr) * (np.matmul(self.weights.T, self.rewards_history[action, :])) + self.lr * reward

            # Save the last reward

            raw_slice = self.rewards_history[action, :]
            print("Slice:", raw_slice)
            np.roll(raw_slice, 1)
            raw_slice[0, 1] = reward
            self.rewards_history[action, :] = raw_slice



    def relay_selection(self, opt_neighbors: list, packet):
        """
        This function returns the best relay to send packets.
        @param packet:
        @param opt_neighbors: a list of tuple (hello_packet, source_drone)
        @return: The best drone to use as relay
        """
        # get velocity constraints: 
        #     the constraint necessary to obtain the minimum delay between hops.
        #     pag. 8 of q-fanet
        depot_position = np.asarray(self.simulator.depot.coords)
        wave_speed = 299337984 # m/s
        # for each neighbor I divide the difference in distance to the depot by the transmission time

        

        nieghbor_speed = [(np.linalg.norm(np.asarray(self.drone.cur_pos, depot_position)) - np.linalg.norm(np.asarray(neighbor_drone.cur_pos), depot_position)) / ( () / wave_speed) for _, neighbor_drone in opt_neighbors]

        return None  # here you should return a drone object!