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
        self.taken_actions = {}  # id event : (old_state, old_action)
        self.look_back = 10 # to tune
        self.qtable = np.zeros(shape=(self.simulator.n_drones)) + 0.5
        self.rewards_history = np.zeros(shape=(self.simulator.n_drones, self.look_back))
        self.delay_history = np.zeros(shape=(self.simulator.n_drones, self.look_back))    # history delay used for normalized one-hop delay
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
            # print("got reward from action", id_event)
            del self.taken_actions[id_event]

            reward = self.get_reward(outcome, hop_delay, E_j)    # formula 3

            # calculate normalized one hop delay   formula 4

            if hop_delay is not None:
                normalized_one_hop_delay, variance = self.get_normalized_one_hop_delay(action, hop_delay)
                # print("normalized_one_hop_delay:", normalized_one_hop_delay)
                self.add_delay_to_history(drone.identifier, hop_delay)
            else:
                normalized_one_hop_delay, variance = self.get_normalized_one_hop_delay(action, self.delay_history[drone.identifier, 0])
            # calculate adaptive learning rate   formula 5
            adaptive_lr = 0.3 if variance == 0 else 1 - np.exp(-normalized_one_hop_delay)
            # print("normalized_one_hop_delay:", normalized_one_hop_delay)

            # calculate adaptive discount factor    formula 6
            cur_step = self.simulator.cur_step
            cur_neighbors = [self.hello_messages[hpk_id].src_drone.identifier for hpk_id in self.hello_messages if self.hello_messages[hpk_id].time_step_creation < cur_step - config.OLD_HELLO_PACKET]
            union_card = len(list(set(old_neighbors) | set(cur_neighbors)))
            intes_card = len(list(set(old_neighbors) & set(cur_neighbors)))
            disc_fact = (union_card - intes_card) / union_card

            # update the q_table    formula 1
            # print(f"Q table update for drone {self.drone.identifier} : {self.qtable[action]} + {adaptive_lr} * ({reward} + {disc_fact} * {np.max(drone.routing_algorithm.qtable)} - {self.qtable[action]})", "=" ,self.qtable[action] + adaptive_lr * (reward + disc_fact * np.max(drone.routing_algorithm.qtable) - self.qtable[action]))
            self.qtable[action] = self.qtable[action] + adaptive_lr * (reward + disc_fact * np.max(drone.routing_algorithm.qtable) - self.qtable[action])
            print(self.qtable)

    def get_reward(self, o, delay, E_j):

        rew = 0
        if o > 0:
            rew = 1
        elif o < 0:
            rew = -1
        else:
            rew = self.one_hop_del_weight * np.exp(-delay) + (1 - self.one_hop_del_weight) * E_j
            # print(self.one_hop_del_weight, "*", np.exp(-delay)," + ",  1, "-", self.one_hop_del_weight, "*", E_j, " = ", rew)
        # print(rew, self.drone.identifier)
        return rew


    def get_delay(self, hello_packet, packet):
        # this is not the same delay as the paper because we don't have access to the queue delay.
        drone_position = self.drone.coords
        neighbor_position = hello_packet.cur_pos
        wave_speed = 200 # m/s
        # for each neighbor I divide the difference in distance to the depot by the transmission time
        packet_size =  sys.getsizeof(packet) * 8
        distance = utilities.euclidean_distance(drone_position, neighbor_position)
        
        # consider transmission and propagation delay
        transmission_delay = packet_size / self.drone.transmission_rate
        propagation_delay = distance / wave_speed

        # print("delay:", transmission_delay + propagation_delay)

        return transmission_delay + propagation_delay

    def get_normalized_one_hop_delay(self, hop, delay):
        # get the one hop delay based on the history.
        history = self.delay_history[hop]
        var = np.var(history)
        print("delay", delay, "var", var)
        var = max(0.1, var)
        one_hop_delay = 0.3 if var == 0 else (delay - np.mean(history)) / var
        print("delay", delay, "delay history:", history, "one_hop_delay:", one_hop_delay)
        return one_hop_delay, var
    
    def add_delay_to_history(self, drone_id, delay):
        history = self.delay_history[drone_id]
        history = np.roll(history, 1)
        history[0] = delay
        self.delay_history[drone_id, :] = history


    def relay_selection(self, opt_neighbors: list, packet):
        """
        This function returns the best relay to send packets.
        @param packet:
        @param opt_neighbors: a list of tuple (hello_packet, source_drone)
        @return: The best drone to use as relay
        """

        if len(opt_neighbors) <= 0:
            return

        action = None
        outcome = None
        delay = None

        depot_position = self.simulator.depot.coords
        drone_position = self.drone.coords
        
        # distances = np.asarray([utilities.euclidean_distance(drone_position, depot_position) - utilities.euclidean_distance(hp.cur_pos, depot_position) for hp, n in opt_neighbors])  # check whether the current drone is the closest to the depot
        # candidate_neighbors = np.asarray(opt_neighbors)[np.where(distances < 0)[0]]
        hello_packets = np.asarray([hp for hp, _ in opt_neighbors])
        
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
        TTL = config.PACKETS_MAX_TTL - packet.get_TTL()
        deadlines = np.asarray([TTL * config.TS_DURATION - d for d in delays])

        # formula 12 (Requested Velocity to transmit the data packet)
        V = d_iD / deadlines  # m/s
        
        # Angles
        angles = np.asarray([np.arctan2(hp.next_target[1] -  hp.cur_pos[1], hp.next_target[0] -  hp.cur_pos[0]) for hp in hello_packets])
        # print(angles)

        # formula 13 & 14 predicted positions of neighbor
        estimated_position = np.asarray([
            np.asarray(
                [hp.cur_pos[0] + hp.speed * math.cos(angles[idx]) * (t3[idx]-t1[idx]),       # x
                hp.cur_pos[1] + hp.speed * math.sin(angles[idx]) * (t3[idx]-t1[idx])])       # y   
                                                                                    for idx, hp in enumerate(hello_packets)])   # for each hp I calculate x and y
        
        # print("Real" , [drone.coords for hp, drone in opt_neighbors], "\nEstimated", estimated_position, "-"*10, "\n")

        # formula 20
        dist = lambda x: utilities.euclidean_distance(drone_position, x)
        distances = np.asarray([dist(pos) for pos in estimated_position])


        # formula 16
        dist = lambda x: utilities.euclidean_distance(depot_position, x)
        neighbor_dist_from_depot =  np.asarray([dist(pos) for pos in estimated_position])
    
        actual_velocities = (d_iD - neighbor_dist_from_depot) / delays
        # print(actual_velocities)
        candidate_neighbors = np.where((actual_velocities - V) > 0)[0]
        
        if len(candidate_neighbors) > 0:
            
            # formula 18 Link Quality
            d_f = np.exp(- 0.2 * (distances/config.COMMUNICATION_RANGE_DRONE))
            d_r = np.exp(- 0.1 * (distances/config.COMMUNICATION_RANGE_DRONE))
            LQ = d_f * d_r
            # print("Link Quality", LQ)
            # formula 19 The coefficient of neighbor relationship
            M = np.asarray([1 - distance / config.COMMUNICATION_RANGE_DRONE if distance <= config.COMMUNICATION_RANGE_DRONE else 0 for distance in distances])
            # formula 17 weights
            k = M * LQ

            # get valid elements

            valid_q_values = self.qtable[candidate_neighbors]
            valid_k = k[candidate_neighbors]

            # formula 21 choose the best drone

            selected_valid_drone = np.argmax(valid_q_values)
            best_q = candidate_neighbors[selected_valid_drone]

            selected_valid_drone = np.argmax(valid_k * valid_q_values)
            action = candidate_neighbors[selected_valid_drone]

            # print("Q_value", best_q, "Weighted Q_value", action)
            
            # send feedback to previous drone
            outcome = 0
            delay = delays[action]

            for idx, (hp, drone) in enumerate(opt_neighbors):
                self.add_delay_to_history(drone.identifier, delays[idx])

        else:
            # No candidate neighbor: Y
            #if there are neighbors whose actual velocity is greater than 0
            penalty_condition = np.where(actual_velocities > 0)[0]
            if len(penalty_condition) > 0:
                action = np.argmax(actual_velocities)
                outcome = 0 
                delay = delays[action]
            else:   #penalty mechanism
                outcome = -1 
                delay = None
            pass


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
            self.taken_actions[packet.event_ref.identifier] = (self.drone.identifier, opt_neighbors[action][1].identifier, [drone.identifier for hp, drone in opt_neighbors])   # save the taken action and the list of neighbors at this moment. 
                                                                                                                        # When the reward comes, I can use this to calculate the adaptive discount factor.
            return opt_neighbors[action][1]

        

        
