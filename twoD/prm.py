import heapq
import random
import time
from collections import defaultdict

import numpy as np
import networkx as nx
from scipy.spatial import KDTree
import math
import matplotlib.pyplot as plt

class PRMController:
    def __init__(self, start, goal, bb):
        self.graph = nx.Graph()
        self.bb = bb
        self.start = start
        self.goal = goal

        self.coordinates_history = []
        # Feel free to add class variables as you wish

    def run_PRM(self, num_coords=100, k=5):
        """
            find a plan to get from current config to destination
            return-the found plan and None if couldn't find
        """
        path = []
        # TODO: HW2 4.3.4
        # Preprocessing
        coords = self.gen_coords(num_coords)
        self.add_to_graph(coords, k)
        # Planning part
        return self.shortest_path()

    
    def create_graph(self, base_number, how_many_to_add, num_searches):
        # TODO: HW2 4.3.5
        raise NotImplementedError()

    def gen_coords(self, n=5):
        """
        Generate 'n' random collision-free samples called milestones.
        n: number of collision-free configurations to generate
        """
        coord = []
        while len(coord) < n:
            config = np.array([random.uniform(-math.pi, math.pi) for _ in range(4)])
            if self.bb.config_validity_checker(config):
                coord.append(config)
        return coord

    def add_to_graph(self, configs, k):
        """
            add new configs to the graph.
        """

        self.graph.add_node(tuple(self.start))
        configs.append(self.goal)
        for config in configs:
            nns = self.find_nearest_neighbour(config, k)
            for nn in nns:
                neighbor = list(self.graph.nodes())[nn-1]
                if self.bb.edge_validity_checker(neighbor, config):
                    self.graph.add_edge(tuple(neighbor),tuple(config))
                    break


    def find_nearest_neighbour(self, config, k=5):
        """
            Find the k nearest neighbours to config
        """
        tree = KDTree(self.graph.nodes())
        distances, indices = tree.query((config), k)
        neighbours = indices[1:]
        return neighbours

    def shortest_path(self):
        """
            Find the shortest path from start to goal using Dijkstra's algorithm (you can use previous implementation from HW1)'
        """

        source = tuple(self.start)
        dest = tuple(self.goal)
        lines = self.graph.edges
        graph = defaultdict(list)
        # build graph
        for line in lines:
            length = self.bb.compute_distance(line[0], line[1])
            graph[tuple(line[0])].append((tuple(line[1]), length))
            graph[tuple(line[1])].append((tuple(line[0]), length))

        # run dijkstra
        heap = [(0, source)]
        distances = {point: float('inf') for point in graph.keys()}
        distances[source] = 0
        reached_from = dict()
        while heap:
            current_distance, current_point = heapq.heappop(heap)
            if current_point == dest:
                print("f")
                break
            if current_distance > distances[current_point]:
                continue
            for next_point, d in graph[current_point]:
                new_distance = current_distance + d
                if new_distance < distances[next_point]:
                    distances[next_point] = new_distance
                    heapq.heappush(heap, (current_distance + d, next_point))
                    reached_from[next_point] = current_point

        # get path
        path = []
        current = dest
        if distances[dest] == float('inf'):
            return path, float('inf')
        while current != source:
            path.append(current)
            current = reached_from[current]
        path.append(source)
        path.reverse()
        return path, distances
