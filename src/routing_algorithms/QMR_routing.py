from src.routing_algorithms.BASE_routing import BASE_routing
from src.utilities import utilities as util
import numpy as np
import sys
import math
from src.utilities import utilities
import src.utilities.config as config


class QMR(BASE_routing):

    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone=drone, simulator=simulator)
        self.sigma = 0.1
        self.taken_actions = {}  # id event : (old_state, old_action)
        self.lr = 0.6 # to tune
        self.look_back = 10 # to tune
        self.weights = np.asarray([i for i in range(self.look_back)], dtype=np.float64) # not sure about this. The paper does not say how to calculate this
        self.weights /= np.sum(self.weights, dtype=np.float64)
        self.qtable = np.zeros(shape=(simulator.n_drones, simulator.n_drones)) + 0.5
        self.rewards_history = np.zeros(shape=(simulator.n_drones, self.look_back))
        self.delay_history = np.zeros(shape=(simulator.drones, self.look_back))     # history delay used for normalized one-hop delay
        self.one_hop_del_weight = 0.6


    def feedback(self, drone, id_event, delay, outcome, reward, E_j, hop_delay):
        """
        Feedback returned when the packet arrives at the depot or
        Expire. This function have to be implemented in RL-based protocols ONLY
        @param drone: The drone that holds the packet
        @param id_event: The Event id
        @param delay: packet delay
        @param outcome: -1 or 1 (read below)
        @return:
        """

        # Read this

        # During the feedback function, we have to calculate the adaptive learning rate and the adaptive discount factor.
        # In order to do this, we need:
        # - energy of the that is sending the feedback drone (j). This must be passed as parameter of the feedback function
        # - the hop_delay between drone i and drone j. We can calculate this measure if we know the link
        # - the normalized onehop delay for the link. We can easily calculate it if we know the link (aka the action).
        # - the learning rate: we need the one hop delay calculated before
        # - the discount factor: we need the list of neighbors at time t-1 and the current neighbors.

        if id_event in self.taken_actions:
            
            state, action, old_neighbors = self.taken_actions[id_event]
            del self.taken_actions[id_event]

            reward = self.get_reward(outcome, hop_delay, drone, E_j)

            # calculate adaptive learning rate
            normalized_one_hop_delay = self.get_normalized_one_hop_delay(action, hop_delay, roll=False)
            adaptive_lr = np.asarray([1 - np.exp(d) if v!= 0 else 0.3   for d, v in normalized_one_hop_delay])

            # calculate adaptive discount factor
            cur_step = self.simulator.cur_step
            cur_neighbors = [self.hello_messages[hpk_id].src_drone.identifier for hpk_id in self.hello_messages if self.hello_messages[hpk_id].time_step_creation < cur_step - config.OLD_HELLO_PACKET]
            union_card = len(list(set(old_neighbors) | set(cur_neighbors)))
            intes_card = len(list(set(old_neighbors) & set(cur_neighbors)))
            disc_fact = (union_card - intes_card) / union_card


            # remove the entry, the action has received the feedback
            # reward or update using the old state and the selected action at that time
            weights_reward_mult = np.matmul(self.weights.T, self.rewards_history[action, :].flatten())
            print("weights_reward_mult", weights_reward_mult)
            self.qtable[state, action] = (1 - self.lr) * (weights_reward_mult) + self.lr * reward

            # Save the last reward
            raw_slice = self.rewards_history[action, :]
            raw_slice = np.roll(raw_slice, 1)
            raw_slice[0] = reward
            self.rewards_history[action, :] = raw_slice
            print(f"\nIdentifier: {self.drone.identifier}, Taken Actions: {self.taken_actions}, Time Step: {self.simulator.cur_step}" "\nDrone:", self.drone.identifier,"\nSlice:", raw_slice, "\nQtable:", self.qtable)

    def get_reward(self, o, delay, drone, E_j):
        if o > 0:
            return 1
        elif o < 0:
            return 0
        else:
            return self.one_hop_del_weight * np.exp(delay) + (1 - self.one_hop_del_weight) * E_j


    def get_delay(self, d_1, d_2, packet):
        # this is not the same delay as the paper because we don't have access to the queue delay.
        drone_position = d_1.coords
        neighbor_position = d_2.coords
        wave_speed = 299337984 # m/s
        # for each neighbor I divide the difference in distance to the depot by the transmission time
        packet_size =  sys.getsizeof(packet) * 8
        distance = utilities.euclidean_distance(drone_position, neighbor_position)
        
        # consider transmission and propagation delay
        transmission_delay = packet_size / d_1.transmission_rate
        propagation_delay = distance / wave_speed

        print("delay:", transmission_delay + propagation_delay)

        return transmission_delay + propagation_delay

    def get_normalized_one_hop_delay(self, hop, delay, roll = True):
        # get the one hop delay based on the history.
        history = self.delay_history[hop]
        var = np.var(history)
        one_hop_delay = (delay - np.mean(history)) / var
        if roll:
            history = np.roll(history, 1)
            history[0] = one_hop_delay
            self.delay_history[hop, :] = history
        return one_hop_delay, var




    def relay_selection(self, opt_neighbors: list, packet):
        """
        This function returns the best relay to send packets.
        @param packet:
        @param opt_neighbors: a list of tuple (hello_packet, source_drone)
        @return: The best drone to use as relay
        """

        action = None

        depot_position = self.simulator.depot.coords
        drone_position = self.drone.coords
        
        distances = np.asarray([utilities.euclidean_distance(drone_position, depot_position) - utilities.euclidean_distance(neighbor.coords, depot_position) for _, neighbor in opt_neighbors])  # check whether the current drone is the closest to the depot
        candidate_neighbors = np.asarray(opt_neighbors)[np.where(distances < 0)[0]]
        if not candidate_neighbors.empty():
            # No candidate neighbor: N

            delays = np.array([self.get_delay(self.drone, neighbor) for _, neighbor in opt_neighbors])

            t1 = self.simulator.cur_step
            t2 = t1 + 5
            t3 = t2 + 3 # hardcoded values. If each step is about 0.02 sec, then we are considernig t1 + 0.1 and t1 + 0.16 seconds after the current time. 
                        # Each drone wil travel about 2.28m each timestep if he is travelling at 114 m/s (speed of a military drone)
                        # This should be an acceptable value as the delay is about 0.0019 and the time taken by a ping is then 0.0038 which is 1/5.26 of a step            
        
            #select the neighbor with the largest k-weigthed Q-value
            for neighbor in candidate_neighbors:
                t1 = self.simulator.cur_step 
                #in reality this is the time when the drone is added to the neighbor table
                # t2 is the current moment where the drone is selecting the neighbor
                # t3 should be t2 + delay
                t3 = t1 + self.get_delay(self.drone, neighbor, packet)
                d_iD = utilities.euclidean_distance(drone_position, depot_position)
                deadline_i = 1 # to be defined
                V_i = d_iD / deadline_i
                angle_j = math.atan2(neighbor.coords,neighbor.next_target.coords)
                predicted_x = neighbor.coords[0] + neighbor.speed * math.cos(angle_j) * (t3-t1)
                predicted_y = neighbor.coords[1] + neighbor.speed * math.sin(angle_j) * (t3-t1)
                estimated_future_position = (predicted_x, predicted_y)
                d_ij = math.sqrt(math.pow(predicted_x - drone_position[0],2)+math.pow(predicted_y - drone_position[1],2))
                # propraogation range of the node   
                v_ij = d_iD - utilities.euclidean_distance(d_iD, estimated_future_position)
                R = 180 # to be defined
                M_ij = 1 - d_ij / R if d_ij <= R else 0
                LQ_ij = 1 # to be defined
                k = M_ij * LQ_ij
            # max(k * self.qtable for _, neighbor in candidate_neighbors if n)

            # energy consumption
            E_i = self.drone.residual_energy / self.drone.initial_energy

        
        else:
            # No candidate neighbor: Y
            pass
            
        if action is not None:
            self.taken_actions[packet.event_ref.identifier] = (self.drone.identifier, action, [drone.identifier for hp, drone in opt_neighbors])   # save the taken action and the list of neighbors at this moment. 
                                                                                                                        # When the reward comes, I can use this to calculate the adaptive discount factor.

        

        
