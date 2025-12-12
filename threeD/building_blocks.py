import math
import random

import numpy as np


class BuildingBlocks3D(object):
    '''
    @param resolution determines the resolution of the local planner(how many intermidiate configurations to check)
    @param p_bias determines the probability of the sample function to return the goal configuration
    '''

    def __init__(self, transform, ur_params, env, resolution=0.1):
        self.transform = transform
        self.ur_params = ur_params
        self.env = env
        self.resolution = resolution
        # self.p_bias = p_bias
        self.cost_weights = np.array([0.4, 0.3, 0.2, 0.1, 0.07, 0.05])

        self.single_mechanical_limit = list(self.ur_params.mechamical_limits.values())[-1][-1]

        # pairs of links that can collide during sampling
        self.possible_link_collisions = [['shoulder_link', 'forearm_link'],
                                         ['shoulder_link', 'wrist_1_link'],
                                         ['shoulder_link', 'wrist_2_link'],
                                         ['shoulder_link', 'wrist_3_link'],
                                         ['upper_arm_link', 'wrist_1_link'],
                                         ['upper_arm_link', 'wrist_2_link'],
                                         ['upper_arm_link', 'wrist_3_link'],
                                         ['forearm_link', 'wrist_2_link'],
                                         ['forearm_link', 'wrist_3_link']]

    def sample_random_config(self, goal_prob,  goal_conf) -> np.array:
        """
        sample random configuration
        @param goal_conf - the goal configuration
        :param goal_prob - the probability that goal should be sampled
        """
        if random.random() < goal_prob:
            return goal_conf
        return [random.uniform(limit[0], limit[1]) for limit in self.ur_params.mechamical_limits.values()]

    def config_validity_checker(self, conf) -> bool:
        """check for collision in given configuration, arm-arm and arm-obstacle
        return False if in collision
        @param conf - some configuration
        """
        spheres = self.transform.conf2sphere_coords(conf)
        # link link collision
        for plc in self.possible_link_collisions:
            obj_0_spheres = spheres[plc[0]]
            obj_1_spheres = spheres[plc[1]]
            for obj_1_sphere in obj_1_spheres:
                for obj_0_sphere in obj_0_spheres:
                    if math.dist(obj_0_sphere, obj_1_sphere) < 2*self.env.radius:
                        print("link link collision")
                        return False
        robot_spheres = []
        [robot_spheres.extend(spheres) for spheres in self.transform.conf2sphere_coords(conf).values()]
        #link obstacle collision
        for sphere in robot_spheres:
            for obstacle in self.env.obstacles:
                if math.dist(sphere,obstacle) < 2*self.env.radius:
                    print("link obstacle collision")
                    return False
        #link floor collision
        for sphere in robot_spheres:
            if np.array_equal(sphere,np.zeros(3)):
                continue
            if sphere[-1] - self.env.radius <0:
                print("link floor collision")
                return False

        return True

    def edge_validity_checker(self, prev_conf, current_conf) -> bool:
        '''check for collisions between two configurations - return True if trasition is valid
        @param prev_conf - some configuration
        @param current_conf - current configuration
        '''
        # TODO: HW2 5.2.4
        print(prev_conf, current_conf)
        length = self.compute_distance(current_conf, prev_conf)
        amount = int(max(length/self.resolution, 2))
        current = prev_conf
        increment = current_conf-prev_conf/amount
        for i in range(amount):
            print(current)
            if self.config_validity_checker(current):
                current += increment
            else:
                return False
        return True

    def compute_distance(self, conf1, conf2):
        '''
        Returns the Edge cost- the cost of transition from configuration 1 to configuration 2
        @param conf1 - configuration 1
        @param conf2 - configuration 2
        '''
        return np.dot(self.cost_weights, np.power(conf1 - conf2, 2)) ** 0.5
