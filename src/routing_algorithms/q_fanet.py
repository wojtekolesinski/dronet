from src.routing_algorithms.BASE_routing import BASE_routing
from src.utilities import utilities as util
import numpy as np
import sys
from src.utilities import utilities

class QFanet(BASE_routing):

    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone=drone, simulator=simulator)
        self.sigma = 0.1
        self.taken_actions = {}  # id event : (old_state, old_action)
        self.lr = 0.6 # to tune
        self.look_back = 10 # to tune
        self.weights = np.asarray([1 for i in range(self.look_back)], dtype=np.float64) # not sure about this. The paper does not say how to calculate this
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
            self.qtable[action] = (1 - self.lr) * (np.matmul(self.weights.T, self.rewards_history[action, :].flatten())) + self.lr * reward

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
        # get velocity constraints: 
        #     the constraint necessary to obtain the minimum delay between hops.
        #     pag. 8 of q-fanet
        depot_position = self.simulator.depot.coords
        drone_position = self.drone.coords
        
        distances = np.asarray([utilities.euclidean_distance(drone_position, depot_position) - utilities.euclidean_distance(neighbor.coords, depot_position) for _, neighbor in opt_neighbors])  # check whether the current drone is the closest to the depot
        if np.all(distances < 0):
            # QMR submodule
            # I am a minimum
            delays = [self.get_delay(self.drone, neighbor, packet) for _, neighbor in opt_neighbors]

            neighbor_speed = np.asarray([(utilities.euclidean_distance(drone_position, depot_position) - utilities.euclidean_distance(neighbor_drone.coords, depot_position)) / (delays[idx]) for idx, (_, neighbor_drone) in enumerate(opt_neighbors)])

            positive_speed_idx = np.where(neighbor_speed > 0)[0]
            print("neighbor speed:", neighbor_speed, "positive speed:", positive_speed_idx)
            if len(positive_speed_idx) <= 0:
                # penalty mechanism: bad reward
                for drone in self.simulator.drones:
                        delivery_delay = self.simulator.cur_step - packet.event_ref.current_time
                        drone.routing_algorithm.feedback(self.drone,
                                                        packet.event_ref.identifier,
                                                        delivery_delay,
                                                        -1,
                                                        -100)
            else: 
                # give to faster
                delivery_delay = self.simulator.cur_step - packet.event_ref.current_time                
                for drone in self.simulator.drones:
                    drone.routing_algorithm.feedback(self.drone,
                                                            packet.event_ref.identifier,
                                                            delivery_delay,
                                                            1,
                                                            50)
                action = np.argmax(neighbor_speed)
                self.taken_actions[packet.event_ref.identifier] = (None, opt_neighbors[action][1].identifier)
                return opt_neighbors[action][1]
        else:
            # q-learning submodule
            if self.simulator.rnd_routing.rand() <= self.sigma:
                # random choice
                action = self.simulator.rnd_routing.randint(0, len(opt_neighbors))
                self.taken_actions[packet.event_ref.identifier] = (None, opt_neighbors[action][1].identifier)
                return opt_neighbors[action][1]
            else:
                # choose the maximum based on the q_table
                # Todo: give a feedback to the previous drone and take the best
                delivery_delay = self.simulator.cur_step - packet.event_ref.current_time
                for drone in self.simulator.drones:
                    drone.routing_algorithm.feedback(self.drone,
                                                            packet.event_ref.identifier,
                                                            delivery_delay,
                                                            1,
                                                            50)
                neighbors_idx = [d.identifier for _, d in opt_neighbors]
                q_values = self.qtable[neighbors_idx]
                action = np.argmax(q_values)
                self.taken_actions[packet.event_ref.identifier] = (None, neighbors_idx[action])
                return opt_neighbors[action][1]
        # calculate mean of 
        
