from src.routing_algorithms.BASE_routing import BASE_routing
from src.utilities import utilities as util
import numpy as np
import sys
import math
from src.utilities import utilities
import src.utilities.config as config


class QMR(BASE_routing):

    def __init__(self, drone, simulator):
        """
        Init function.
        """
        BASE_routing.__init__(self, drone=drone, simulator=simulator)
        self.taken_actions = {}  # id event : (old_state, old_action)
        self.random = np.random.RandomState(self.simulator.seed)
        self.look_back = 10 # to tune
        self.qtable = np.zeros(shape=(self.simulator.n_drones)) + 0.5
        self.rewards_history = np.zeros(shape=(self.simulator.n_drones, self.look_back))
        self.delay_history = [[0] for _ in range(self.simulator.n_drones)]# np.zeros(shape=(self.simulator.n_drones, self.look_back))    # history delay used for normalized one-hop delay
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

        # During the feedback function, we have to calculate the adaptive learning rate and the adaptive discount factor.
        # In order to do this, we need:
        # - energy of the that is sending the feedback drone (j). This must be passed as parameter of the feedback function
        # - the hop_delay between drone i and drone j. We can calculate this measure if we know the link
        # - the normalized one_hop delay for the link. We can easily calculate it if we know the link (aka the action).
        # - the list of neighbors at time t-1 and the current neighbors.
        #
        # Once we have these measures, we can calculate the learning rate and the discount factor:
        # - the learning rate: { (1 - exp(- normalized_one_hop_delay)       if variance is 0
        #                      { 0.3                                        otherwise
        # - the discount factor: (union(past_neighbors, current_neighbors) - intersection(past_neighbors, current_neighbors))
        #                        --------------------------------------------------------------------------------------------
        #                                                  union(past_neighbors, current_neighbors)
        #
        #
        #
        #
        #

        if id_event in self.taken_actions:
            
            # get action
            state, action, old_neighbors = self.taken_actions[id_event]
            del self.taken_actions[id_event]

            # Calculate reward
            reward = self.get_reward(outcome, hop_delay, E_j)    # formula 3

            # calculate normalized one hop delay   formula 4
            if hop_delay is not None:
                # If the feedback comes from another drone, he will send the delay between himself and this drone
                normalized_one_hop_delay, variance = self.get_normalized_one_hop_delay(action, hop_delay)
                self.add_delay_to_history(drone.identifier, hop_delay)
            else:
                # If the feedback comes from the environment (a packet expired or is arrived to the depot), 
                # we can't calculate an actual delay, so we use the past delays.
                normalized_one_hop_delay, variance = self.get_normalized_one_hop_delay(action, self.delay_history[drone.identifier][-1])
            
            
            # calculate adaptive learning rate   formula 5
            adaptive_lr = 0.3 if variance == 0 else 1 - np.exp(-normalized_one_hop_delay)

            # calculate adaptive discount factor    formula 6
            cur_step = self.simulator.cur_step
            cur_neighbors = [self.hello_messages[hpk_id].src_drone.identifier for hpk_id in self.hello_messages if self.hello_messages[hpk_id].time_step_creation < cur_step - config.OLD_HELLO_PACKET]
            union_card = len(list(set(old_neighbors) | set(cur_neighbors)))
            intes_card = len(list(set(old_neighbors) & set(cur_neighbors)))
            disc_fact = (union_card - intes_card) / union_card

            # update the q_table    formula 1
            self.qtable[action] = self.qtable[action] + adaptive_lr * (reward + disc_fact * np.max(drone.routing_algorithm.qtable) - self.qtable[action])
            # print(self.qtable)
            

    def get_reward(self, o, delay, E_j):
        """
        Reward function for qmr taken by formula 3.

        QMR returns maximum reward (1) if teh feedback came from a drone that has just delivered a packet to the depot.
        Otherwise, it returns minimum reward (-1) if the next drone (the one who is sending the feedback) is a local minimum.
        The last option is that the next drone succesfully passed the packet to another drone. In that case we use the formula:
            hop_delay_weight * exp(-hop_delay_ij) + (1 - hop_delay_weight) * Energy_Rateo
        """
        rew = 0
        if o > 0:
            rew = 1
        elif o < 0:
            rew = -1
        else:
            rew = self.one_hop_del_weight * np.exp(-delay) + (1 - self.one_hop_del_weight) * E_j
            # print(self.one_hop_del_weight, "*", np.exp(-delay)," + ",  1, "-", self.one_hop_del_weight, "*", E_j, " = ", rew)
        return rew


    def get_delay(self, hello_packet, packet):
        # this is not the same delay as the paper because we don't have access to the queue delay.
        drone_position = self.drone.coords
        neighbor_position = hello_packet.cur_pos
        # We know that waves may be a lil bit faster that that, but we don't care cuz it works better like dis
        wave_speed = 300000 # m/s
        # for each neighbor I divide the difference in distance to the depot by the transmission time
        packet_size =  packet.get_packet_size() * 8  #  size of packet in bits sys.getsizeof(packet) * 8  
        distance = utilities.euclidean_distance(drone_position, neighbor_position)      #  distance
        
        # calculate, sum and return transmission and propagation delay
        transmission_delay = packet_size / self.drone.transmission_rate
        propagation_delay = distance / wave_speed

        return transmission_delay + propagation_delay


    def get_normalized_one_hop_delay(self, hop, delay):
        """
        Get the normalized one hop delay and variance based on history.
        """
        # get the one hop delay based on the history.
        history = np.asarray(self.delay_history[hop] + [delay])

        var = np.std(history)
        mean = np.mean(history)
        one_hop_delay = np.abs((delay - mean) / var)
        return one_hop_delay, var
    

    def add_delay_to_history(self, drone_id, delay):
        """
        Add a delay to the history.
        """
        self.delay_history[drone_id].append(delay)
        # history = np.roll(history, 1)
        # history[0] = delay
        # self.delay_history[drone_id, :] = history


    def relay_selection(self, opt_neighbors: list, packet):
        """
        This function returns the best relay to send packets.
        @param packet:
        @param opt_neighbors: a list of tuple (hello_packet, source_drone)
        @return: The best drone to use as relay
        """

        if len(opt_neighbors) <= 0:
            return

        # Final action
        action = None
        # outcome to send as feedback to the previous link
        outcome = None
        # delay
        delay = None

        # get position of depot and drone
        depot_position = self.simulator.depot.coords
        drone_position = self.drone.coords
        
        # get th elist of hello packets
        hello_packets = np.asarray([hp for hp, _ in opt_neighbors])
        neighbors_ids = np.asarray([d.identifier for _, d in opt_neighbors])
        
        # 
        delays = np.array([self.get_delay(hp, packet) for hp in hello_packets])
        drone2delay = {d.identifier: self.get_delay(hp, packet) for hp, d in opt_neighbors}
        
        t1 = np.asarray([hp.time_step_creation * config.TS_DURATION for hp in hello_packets])   # in s
        t2 = self.simulator.cur_step * config.TS_DURATION   #current moment in s
        #formula 15  time when packet will arrive at node j
        t3 = np.asarray([t2 + d for d in delays]) # hardcoded values. If each step is about 0.02 sec, then we are considernig t1 + 0.1 and t1 + 0.16 seconds after the current time.
                    # Each drone wil travel about 2.28m each timestep if he is travelling at 114 m/s (speed of a military drone)
                    # This should be an acceptable value as the delay is about 0.0019 and the time taken by a ping is then 0.0038 which is 1/5.26 of a step
                    # in s
        d_iD = utilities.euclidean_distance(drone_position, depot_position)
        
        # formula 12 (Requested Velocity to transmit the data packet)
        V = d_iD / ((packet.event_ref.deadline + 1 - self.simulator.cur_step) * config.TS_DURATION)  # m/s
        
        estimated_position = []
        for idx, hp in enumerate(hello_packets):
            p0 = hp.cur_pos
            p1 = hp.next_target
            all_distance = utilities.euclidean_distance(p0, p1)
            distance = (t3[idx]-t1[idx]) * hp.speed
            t = distance / all_distance
            pred = ((1 - t) * p0[0] + t * p1[0]), ((1 - t) * p0[1] + t * p1[1])
            estimated_position.append(pred)
        estimated_position = np.asarray(estimated_position)

        
        # formula 20
        dist_dr = lambda x: utilities.euclidean_distance(drone_position, x)
        distances = np.asarray([dist_dr(pos) for pos in estimated_position])

        # formula 16
        dist = lambda x: utilities.euclidean_distance(depot_position, x)
        neighbor_dist_from_depot =  np.asarray([dist(pos) if dist_dr(pos) <= config.COMMUNICATION_RANGE_DRONE else 69420 for pos in estimated_position])
        
        
        # formula 18 Link Quality
        d_f = np.exp(- 0.2 * (distances/config.COMMUNICATION_RANGE_DRONE))
        d_r = np.exp(- 0.1 * (distances/config.COMMUNICATION_RANGE_DRONE))
        LQ = d_f * d_r
        
        # formula 19 The coefficient of neighbor relationship
        M = np.asarray([1 - distance / config.COMMUNICATION_RANGE_DRONE if distance <= config.COMMUNICATION_RANGE_DRONE else 0 for distance in distances])
        # formula 17 weights
        k = M * LQ

        
        actual_velocities = (d_iD - neighbor_dist_from_depot) / delays
        
        where_v_greater_than_min_v = np.where((actual_velocities - V) > 0)[0]
        
        candidate_neighbors = neighbors_ids[where_v_greater_than_min_v]
        
        if len(candidate_neighbors) > 0:
            # get valid elements

            mask = np.ones(self.simulator.n_drones, dtype = bool)
            mask[candidate_neighbors] = False

            valid_q_values = k[where_v_greater_than_min_v]*self.qtable[candidate_neighbors]
            possible_actions = np.ma.array(self.qtable, mask=mask)
            possible_actions[possible_actions.mask == False] = valid_q_values

            # formula 21 choose the best drone
            random_tie_breaking = np.flatnonzero(possible_actions == possible_actions.max())
            
            action = self.random.choice(random_tie_breaking)            
            # send feedback to previous drone
            outcome = 0
            delay = drone2delay[action]

            for idx, (hp, drone) in enumerate(opt_neighbors):
                self.add_delay_to_history(drone.identifier, delays[idx])

        else:
            # No candidate neighbor: Y
            #if there are neighbors whose actual velocity is greater than 0
            penalty_condition = np.where(actual_velocities > 0)[0]
            if len(penalty_condition) > 0:
                n = np.argmax(actual_velocities)
                action = neighbors_ids[n]
                outcome = 0 
                delay = drone2delay[action]
            else:   #penalty mechanism
                outcome = -1 
                delay = None


        # send feedback to previous drone
        E_i = self.drone.residual_energy / self.drone.initial_energy
        delivery_delay = self.simulator.cur_step - packet.time_step_creation
        for drone in self.simulator.drones:
            drone.routing_algorithm.feedback(self.drone,
                                                        packet.event_ref.identifier,
                                                        delivery_delay,
                                                        outcome,
                                                        None,
                                                        E_i,
                                                        delay)         # self, drone, id_event, delay, outcome, reward, E_i, hop_delay
        
        if action is not None:
            self.taken_actions[packet.event_ref.identifier] = (self.drone.identifier, action, [drone.identifier for _, drone in opt_neighbors])   # save the taken action and the list of neighbors at this moment. 
                                                                                                                        # When the reward comes, I can use this to calculate the adaptive discount factor.
            # formula 11
            packet.decrease_deadline(delay)

            for _, drone in opt_neighbors:
                if drone.identifier == action:
                    return drone

