from src.routing_algorithms.BASE_routing import BASE_routing
from src.utilities import utilities as util
import numpy as np
import sys
import math
from src.utilities import utilities
import src.utilities.config as config


class Q_Fanet_OUR(BASE_routing):

    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone=drone, simulator=simulator)
        self.taken_actions = {}  # id event : (old_state, old_action)
        self.look_back = 10 # to tune
        self.weights = np.asarray([i for i in range(self.look_back + 1, 1, -1)], dtype=np.float64) # not sure about this. The paper does not say how to calculate this
        self.weights /= np.sum(self.weights, dtype=np.float64)
        self.qtable = np.zeros(shape=(self.simulator.n_drones)) + 0.5
        self.rewards_history = np.zeros(shape=(self.simulator.n_drones, self.look_back))
        self.lr = 0.6
        self.epsilon = 0.1


    def feedback(self, drone, id_event, delay, outcome , reward = None, E_j = None, hop_delay = None):
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
        # - the discount factor: we need the list of neighbors at time t-1 and the current neighbors.

        if id_event in self.taken_actions:
            
            state, action = self.taken_actions[id_event]
            # print("got reward from action", id_event)
            del self.taken_actions[id_event]
            weights_reward_mult = np.matmul(self.weights.T, self.rewards_history[action, :].flatten())
            reward = self.get_reward(outcome)    # formula 3
            self.qtable[action] = (1 - self.lr) * (weights_reward_mult) + self.lr * reward

            # print(self.qtable)

            raw_slice = self.rewards_history[action, :]
            raw_slice = np.roll(raw_slice, 1)
            raw_slice[0] = reward
            self.rewards_history[action, :] = raw_slice

    def get_reward(self, o):
        rew = 0
        if o > 0:
            rew = 100
        elif o < 0:
            rew = -100
        else:
            rew = 50
        return rew


    def get_delay(self, hello_packet, packet):
        # this is not the same delay as the paper because we don't have access to the queue delay.
        drone_position = self.drone.coords
        neighbor_position = hello_packet.cur_pos
        wave_speed = 300000 # m/s
        # for each neighbor I divide the difference in distance to the depot by the transmission time
        packet_size =  sys.getsizeof(packet) * 8
        distance = utilities.euclidean_distance(drone_position, neighbor_position)
        
        # consider transmission and propagation delay
        transmission_delay = packet_size / self.drone.transmission_rate
        propagation_delay = distance / wave_speed

        # print("delay:", transmission_delay + propagation_delay)

        return transmission_delay + propagation_delay

    def relay_selection(self, opt_neighbors: list, packet):
        """
        This function returns the best relay to send packets.
        @param packet:
        @param opt_neighbors: a list of tuple (hello_packet, source_drone)
        @return: The best drone to use as relay
        """
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
        # estimated_position = [(hp.cur_pos[0], hp.cur_pos[1]) for hp in hello_packets]
        # print("Real" , [drone.coords for hp, drone in opt_neighbors], "\nEstimated", estimated_position, "-"*10, "\n")

        # formula 20
        dist_dr = lambda x: utilities.euclidean_distance(drone_position, x)
        distances = np.asarray([dist_dr(pos) for pos in estimated_position])


        # formula 16
        dist = lambda x: utilities.euclidean_distance(depot_position, x)
        neighbor_dist_from_depot =  np.asarray([dist(pos) if dist_dr(pos) <= config.COMMUNICATION_RANGE_DRONE else 69420 for pos in estimated_position])
        # print(f"neighbor_dist_from_depot {[hp.src_drone.identifier for hp in hello_packets]} (Drone: {self.drone.identifier})", neighbor_dist_from_depot)
    
        actual_velocities = (d_iD - neighbor_dist_from_depot) / delays
        # print(actual_velocities)
        candidate_neighbors = np.where((actual_velocities - V) > 0)[0] # np.where((neighbor_dist_from_depot <= d_iD))[0]

        
        if len(candidate_neighbors) > 0:
            #Q_Learning sub-module
            c_n = None
            # random value > greedy value
            if self.simulator.rnd_routing.rand() > self.epsilon:
                # select value with highest q
                c_n = np.argmax(self.qtable[candidate_neighbors])
            else:
                # select random value
                c_n = self.simulator.rnd_routing.randint(0, len(candidate_neighbors))
           
            action = candidate_neighbors[c_n]
            
            # apply reward function
            outcome = 0

        else:
            #QMR 
            # No candidate neighbor: Y
            #if there are neighbors whose actual velocity is greater than 0
            penalty_condition = np.where(actual_velocities > 0)[0]
            if len(penalty_condition) > 0:
                action = np.argmax(actual_velocities)
                outcome = 0 
            else:   #penalty mechanism
                outcome = -1 
                delay = None
            pass


        for drone in self.simulator.drones:
            delivery_delay = self.simulator.cur_step - packet.event_ref.current_time
            drone.routing_algorithm.feedback(self.drone,
                                                        packet.event_ref.identifier,
                                                        delivery_delay,
                                                        outcome,
                                                        None,
                                                        None,
                                                        delay)         # self, drone, id_event, delay, outcome, reward, E_i, hop_delay
            
        if action is not None:
            self.taken_actions[packet.event_ref.identifier] = (self.drone.identifier, opt_neighbors[action][1].identifier)   # save the taken action and the list of neighbors at this moment. 
                                                                                                                    
            return opt_neighbors[action][1]

        

        
