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
        for i, c in enumerate(coords[:5]):
            print(f"[run_PRM] coords[{i}] =", c, " type:", type(c))

        self.add_to_graph(coords, k)
        # Planning part
        path , cost = self.shortest_path()

        plan = [np.array(cfg) for cfg in path]
        return plan

    
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
        print("[add_to_graph] received", len(configs), "configs")

        self.graph.add_node(tuple(self.start))
#        self.graph.add_node(tuple(self.goal))
        configs.append(self.start)
        configs.append(self.goal)
        for config in configs:
            self.graph.add_node(tuple(config))
            nns = self.find_nearest_neighbour(config, k)
            for nn in nns:
                if self.bb.edge_validity_checker(config, nn):
                    self.graph.add_edge(tuple(nn),tuple(config))



    def find_nearest_neighbour(self, config, k=5):
        """
            Find the k nearest neighbours to config
        """
        nodes = list(self.graph.nodes())
        if not nodes:
            return []

        pts = np.array(nodes)  # shape (N, d)
        m = min(k, len(pts))
        tree = KDTree(pts)

        # query with 2D input, then flatten indices
        dists, idx = tree.query([config], k=m)
        idx = np.asarray(idx).ravel()

        cfg_t = tuple(config)
        neighbours = []
        for i in idx:
            neighbour = tuple(pts[i])
            if neighbour == cfg_t:  # skip self if present
                continue
            neighbours.append(neighbour)

        return neighbours

    def shortest_path(self):
        """
            Find the shortest path from start to goal using Dijkstra's algorithm (you can use previous implementation from HW1)'
        """

        source = tuple(self.start)
        dest = tuple(self.goal)
        lines = self.graph.edges()
        graph = defaultdict(list)
        # build graph
        print(lines)
        for line in lines:
            length = self.bb.compute_distance(line[0], line[1])
            graph[tuple(line[0])].append((tuple(line[1]), length))
            graph[tuple(line[1])].append((tuple(line[0]), length))
        print(graph.items())
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
