from src.routing_algorithms.BASE_routing import BASE_routing
from src.utilities import utilities as util
import numpy as np
import sys
import math
from src.utilities import utilities

class QMR(BASE_routing):

    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone=drone, simulator=simulator)
        self.sigma = 0.1
        self.taken_actions = {}  # id event : (old_state, old_action)
        self.lr = 0.6 # to tune
        self.look_back = 10 # to tune
        self.weights = np.asarray([i for i in range(self.look_back)], dtype=np.float64) # not sure about this. The paper does not say how to calculate this
        self.weights /= np.sum(self.weights, dtype=np.float64)
        self.qtable = np.zeros(shape=(simulator.n_drones)) + 0.5
        self.rewards_history = np.zeros(shape=(simulator.n_drones, self.look_back))


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

            # take the old action
            state, action = self.taken_actions[id_event]
            # remove the entry, the action has received the feedback
            del self.taken_actions[id_event]
            # reward or update using the old state and the selected action at that time
            weights_reward_mult = np.matmul(self.weights.T, self.rewards_history[action, :].flatten())
            print("weights_reward_mult", weights_reward_mult)
            self.qtable[action] = (1 - self.lr) * (weights_reward_mult) + self.lr * reward

            # Save the last reward
            raw_slice = self.rewards_history[action, :]
            raw_slice = np.roll(raw_slice, 1)
            raw_slice[0] = reward
            self.rewards_history[action, :] = raw_slice
            print(f"\nIdentifier: {self.drone.identifier}, Taken Actions: {self.taken_actions}, Time Step: {self.simulator.cur_step}" "\nDrone:", self.drone.identifier,"\nSlice:", raw_slice, "\nQtable:", self.qtable)


    def get_delay(self, d_1, d_2, packet):
        drone_position = d_1.coords
        neighbor_position = d_2.coords
        wave_speed = 299337984 # m/s
        # for each neighbor I divide the difference in distance to the depot by the transmission time
        packet_size =  sys.getsizeof(packet)
        distance = utilities.euclidean_distance(drone_position, neighbor_position)
        
        # consider transmission and propagation delay
        transmission_delay = packet_size / d_1.transmission_rate
        propagation_delay = distance / wave_speed

        return transmission_delay + propagation_delay



    def relay_selection(self, opt_neighbors: list, packet):
        """
        This function returns the best relay to send packets.
        @param packet:
        @param opt_neighbors: a list of tuple (hello_packet, source_drone)
        @return: The best drone to use as relay
        """

        depot_position = self.simulator.depot.coords
        drone_position = self.drone.coords
        
        distances = np.asarray([utilities.euclidean_distance(drone_position, depot_position) - utilities.euclidean_distance(neighbor.coords, depot_position) for _, neighbor in opt_neighbors])  # check whether the current drone is the closest to the depot
        candidate_neighbors = np.asarray(opt_neighbors)[np.where(distances < 0)[0]]
        if not candidate_neighbors.empty():
            # No candidate neighbor: N

            delays = np.array([self.get_delay(self.drone, neighbor) for _, neighbor in opt_neighbors])

            t1 = self.simulator.cur_step
            t2 = t1 + 
            
        
            #select the neighbor with the largest k-weigthed Q-value
            for neighbor in candidate_neighbors:
                d_iD = utilities.euclidean_distance(drone_position, depot_position)
                deadline_i = 1 # to be defined
                V_i = d_iD / deadline_i
                angle_j = math.atan2(neighbor.coords,neighbor.nex)
                predicted_x = neighbor.coords[0] + neighbor.speed * math.cos(angle_j) * (t3-t1)
                d_ij = math.sqrt((math.pow2()))
            d_ij = 
            M_ij = 0 if 

            # energy consumption
            E_i = self.drone.residual_energy / self.drone.initial_energy
        
        else:
            # No candidate neighbor: Y

        
