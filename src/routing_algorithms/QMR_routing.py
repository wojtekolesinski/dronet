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
        self.qtable = np.zeros(shape=(simulator.n_drones)) + 0.5
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

            reward = self.get_reward(outcome, hop_delay, E_j)    # formula 3


            # calculate normalized one hop delay   formula 4
            normalized_one_hop_delay, variance = self.get_normalized_one_hop_delay(action, hop_delay, roll=False)

            # calculate adaptive learning rate   formula 5
            adaptive_lr =1 - np.exp(normalized_one_hop_delay) if variance!= 0 else 0.3

            # calculate adaptive discount factor    formula 6
            cur_step = self.simulator.cur_step
            cur_neighbors = [self.hello_messages[hpk_id].src_drone.identifier for hpk_id in self.hello_messages if self.hello_messages[hpk_id].time_step_creation < cur_step - config.OLD_HELLO_PACKET]
            union_card = len(list(set(old_neighbors) | set(cur_neighbors)))
            intes_card = len(list(set(old_neighbors) & set(cur_neighbors)))
            disc_fact = (union_card - intes_card) / union_card

            # update the q_table    formula 1
            self.qtable[action] = self.qtable[action] + adaptive_lr * (reward + disc_fact * np.max(drone.qtable[action]) - self.qtable[action])

    def get_reward(self, o, delay, E_j):
        if o > 0:
            return 1
        elif o < 0:
            return 0
        else:
            return self.one_hop_del_weight * np.exp(delay) + (1 - self.one_hop_del_weight) * E_j


    def get_delay(self, hello_packet, packet):
        # this is not the same delay as the paper because we don't have access to the queue delay.
        drone_position = self.drone.coords
        neighbor_position = hello_packet.cur_pos
        wave_speed = 299337984 # m/s
        # for each neighbor I divide the difference in distance to the depot by the transmission time
        packet_size =  sys.getsizeof(packet) * 8
        distance = utilities.euclidean_distance(drone_position, neighbor_position)
        
        # consider transmission and propagation delay
        transmission_delay = packet_size / self.drone.transmission_rate
        propagation_delay = distance / wave_speed

        print("delay:", transmission_delay + propagation_delay)

        return transmission_delay + propagation_delay

    def get_normalized_one_hop_delay(self, hop, delay, roll = True):
        if roll:
            history = np.roll(history, 1)
            history[0] = delay
            self.delay_history[hop, :] = history
        # get the one hop delay based on the history.
        history = self.delay_history[hop]
        var = np.var(history)
        one_hop_delay = (delay - np.mean(history)) / var
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
        hello_packets = candidate_neighbors[:,0]
        
        if not candidate_neighbors.empty():
            # No candidate neighbor: N

            delays = np.array([self.get_delay(hp, packet) for hp in hello_packets])

            t1 = np.asarray([hp.time_step_creation * config.TS_DURATION for hp in hello_packets])   # in s
            t2 = self.simulator.cur_step * config.TS_DURATION      # in s
            #formula 15  time when packet will arrive at node j
            t3 = np.asarray([t2 + d for d in delays]) # hardcoded values. If each step is about 0.02 sec, then we are considernig t1 + 0.1 and t1 + 0.16 seconds after the current time.
                        # Each drone wil travel about 2.28m each timestep if he is travelling at 114 m/s (speed of a military drone)
                        # This should be an acceptable value as the delay is about 0.0019 and the time taken by a ping is then 0.0038 which is 1/5.26 of a step
                        # in s
            d_iD = utilities.euclidean_distance(drone_position, depot_position)
            
            # formula 11 (we made it different) 
            TTL = config.PACKETS_MAX_TTL - packet.TTL 
            deadlines = np.asarray([TTL * config.TS_DURATION - d for d in delays])

            # formula 12 (Requested Velocity to transmit the data packet)
            V = d_iD / deadlines  # m/s
            
            # Angles
            angles = np.asarray([np.arctan2(hp.next_target.coords, hp.cur_pos) for hp in hello_packets])

            # formula 13 & 14 predicted positions of neighbor
            estimated_position = np.asarray([
                np.asarray(
                    [hp.cur_pos[0] + hp.speed * math.cos(angles[idx]) * (t3[idx]-t1[idx]),       # x
                    hp.cur_pos[1] + hp.speed * math.sin(angles[idx]) * (t3[idx]-t1[idx])])       # y   
                                                                                        for idx, hp in enumerate(hello_packets)])   # for each hp I calculate x and y

            # formula 20
            dist = lambda x: utilities.euclidean_distance(drone_position, x)
            distances = dist(estimated_position)

            # formula 18 Link Quality
            LQ = 1 

            # formula 19 The coefficient of neighbor relationship
            M = np.asarray([1 - distance / config.COMMUNICATION_RANGE_DRONE if distance <= config.COMMUNICATION_RANGE_DRONE else 0 for distance in distances])

            # formula 16
            dist = lambda x: utilities.euclidean_distance(depot_position, x)
            neighbor_dist_from_depot = dist(estimated_position)
        
            actual_velocities = (d_iD - neighbor_dist_from_depot) / delays

            # formula 17 weights
            k = M * LQ

            # get valid elements

            valid_neighbors = np.where((actual_velocities - V) > 0)[0]
            valid_q_values = self.qtable[valid_neighbors]
            valid_k = k[valid_neighbors]

            # formula 21 choose the best drone
            selected_valid_drone = np.argmax(valid_k * valid_q_values)
            action = valid_neighbors[selected_valid_drone]
            
            # send feedback to previous drone
            E_i = self.drone.residual_energy / self.drone.initial_energy
            delivery_delay = self.simulator.cur_step - packet.time_step_creation
            for drone in self.simulator.drones:
                delay = self.get_delay(self.drone, drone)
                drone.routing_algorithm.feedback(self.drone,
                                                            packet.event_ref.identifier,
                                                            delivery_delay,
                                                            0,
                                                            None,
                                                            E_i,
                                                            delay)         # self, drone, id_event, delay, outcome, reward, E_i, hop_delay

        
        else:
            # No candidate neighbor: Y
            pass
            
        if action is not None:
            self.taken_actions[packet.event_ref.identifier] = (self.drone.identifier, action, [drone.identifier for hp, drone in opt_neighbors])   # save the taken action and the list of neighbors at this moment. 
                                                                                                                        # When the reward comes, I can use this to calculate the adaptive discount factor.

        

        
