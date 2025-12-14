import math
import time
import numpy as np
from twoD.environment import MapEnvironment
from twoD.building_blocks import BuildingBlocks2D
from twoD.prm import PRMController
from twoD.visualizer import Visualizer
from threeD.environment import Environment
from threeD.kinematics import UR5e_PARAMS, Transform
from threeD.building_blocks import BuildingBlocks3D
from threeD.visualizer import Visualize_UR
import matplotlib.pyplot as plt


def analyze_prm_performance(prm, bb):
    """
    Run PRM for various n and k, and plot:
    1) cost vs n  (separate curve per k type)
    2) cost vs runtime (runtime on x-axis, cost on y-axis, separate curve per k type)

    Parameters
    ----------
    prm : your PRM planner object (must have run_PRM(n, k))
    bb  : your BuildingBlocks2D (must have compute_path_cost(plan))
    """
    n_values = [100, 200, 300, 400, 500, 600, 700]
    k_types = ["5", "10", "log2(n)", "10*log2(n)", "n/10"]

    # Store results
    # results[k_type] = {'n': [...], 'cost': [...], 'runtime': [...]}
    results = {k_type: {'n': [], 'cost': [], 'runtime': []}
               for k_type in k_types}
    max_retries = 10
    def k_from_type(k_type, n):
        """Return integer k according to k_type string and n."""
        if k_type == "5":
            return 5
        if k_type == "10":
            return 10
        if k_type == "log2(n)":
            return max(1, int(math.log2(n)))
        if k_type == "10*log2(n)":
            return max(1, int(10 * math.log2(n)))
        if k_type == "n/10":
            return max(1, int(n / 10))
        raise ValueError(f"Unknown k_type: {k_type}")

    # Run experiments
    for n in n_values:
        for k_type in k_types:
            retries = 0
            cost = float('inf')
            runtime = None
            while retries <= max_retries and not math.isfinite(cost):
                k = k_from_type(k_type, n)
                prm.graph.clear()

                start_time = time.perf_counter()
                plan = prm.run_PRM(n, k)


                # If run_PRM can fail and return None, handle it:
                if plan is None or len(plan) == 0:
                    cost = float('inf')
                else:
                    cost = bb.compute_path_cost(plan)
                end_time = time.perf_counter()

                runtime = end_time - start_time
                if math.isfinite(cost):
                    print(f"[ OK ] n={n}, k={k_type}, retries={retries}, cost={cost}, runtime={runtime:.4f}s")
                    break
                else:
                    print(f"[FAIL] n={n}, k={k_type}, retry={retries}")
                    retries += 1
            if not math.isfinite(cost):
                print(f"[SKIP] n={n}, k={k_type} — no valid plan after {max_retries} retries")
                continue  # <-- DO NOT SAVE RESULT

            # Save results
            results[k_type]['n'].append(n)
            results[k_type]['cost'].append(cost)
            results[k_type]['runtime'].append(runtime)

            print(f"k={k_type:>8}, n={n:4d}, k_val={k:4d}, "
                  f"runtime={runtime:.4f}s, cost={cost}")

    # ===== Plot 1: cost vs n =====
    plt.figure(figsize=(8, 5))
    for k_type in k_types:
        plt.plot(results[k_type]['n'],
                 results[k_type]['cost'],
                 marker='o',
                 label=f"k = {k_type}")
    plt.xlabel("n (number of samples)")
    plt.ylabel("Path cost")
    plt.title("PRM: Cost vs n for different k")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.figure(figsize=(8, 5))
    # ===== Plot 2: cost vs runtime (runtime on x, cost on y) =====
    for k_type in k_types:
        runtimes = results[k_type]['runtime']
        costs = results[k_type]['cost']
        ns = results[k_type]['n']

        plt.scatter(runtimes, costs, label=f"k = {k_type}")

        # Annotate each point with n AND k-type
        for rt, ct, n in zip(runtimes, costs, ns):
            label = f"{n}, k={k_type}"
            plt.annotate(label, (rt, ct), textcoords="offset points",
                         xytext=(5, 5), fontsize=7)

    plt.xlabel("Runtime (seconds)")
    plt.ylabel("Path cost")
    plt.title("PRM: Cost vs Runtime (labeled by n and k)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()
def run_2d():
    conf = np.array([0.78, -0.78, 0.0, 0.0])

    # prepare the map
    planning_env = MapEnvironment(json_file="./twoD/map_mp.json")
    bb = BuildingBlocks2D(planning_env)
    visualizer = Visualizer(bb)

    robot_positions = bb.compute_forward_kinematics(given_config=conf)
    print(bb.validate_robot(robot_positions=robot_positions)) # check robot validity
    print(bb.config_validity_checker(config=conf)) # check robot and map validity

    visualizer.visualize_map(config=conf)


def run_prm():
    conf1 = np.array([0.78, -0.78, 0.0, 0.0])
    conf2 = np.array([0.8, -0.8, 0.8, 0.5])

    planning_env = MapEnvironment(json_file="./twoD/map_mp.json")
    bb = BuildingBlocks2D(planning_env)
    visualizer = Visualizer(bb)
    prm = PRMController(conf1, conf2, bb)
    analyze_prm_performance(prm, bb)
  #  plan = prm.run_PRM(700,70)
   # print(bb.compute_path_cost(plan))
   # visualizer.visualize_plan_as_gif(plan)

def generate_graph():
    conf1 = np.array([0.78, -0.78, 0.0, 0.0])
    conf2 = np.array([0.8, 0.8, 0.3, 0.5])
    planning_env = MapEnvironment(json_file="./twoD/map_mp.json")
    bb = BuildingBlocks2D(planning_env)
    prm = PRMController(conf1, conf2, bb)
    prm.create_graph()



def run_3d():
    ur_params = UR5e_PARAMS(inflation_factor=1)
    transform = Transform(ur_params)
    env = Environment(env_idx=1)
    bb = BuildingBlocks3D(transform=transform,
                          ur_params=ur_params,
                          resolution=0.1,
                          env=env)

    

    #visualizer = Visualize_UR(ur_params, env=env, transform=transform, bb=bb)

    # --------- configurations-------------
    conf1 = np.deg2rad([0, -90, 0, -90, 0, 0])
    no_col = np.deg2rad([0, -90, 90, -90, 0, 0])
    obs_col = np.deg2rad([0, 0, 0, 0, 0, 0])
    self_col = np.deg2rad([0, -90, 180, 0, 0, 0])
    conf_start = np.deg2rad( [80, -72, 101, -120, -90, -10])
    conf_goal = np.deg2rad([20, -90, 90, -90, -90, -10])
    conf2 = np.array([-0.694, -1.376, -2.212, -1.122, 1.570, -2.26])

    # ---------------------------------------

    # collision checking examples
    print(bb.config_validity_checker(conf_goal))
    #res = bb.edge_validity_checker(prev_conf=conf_start ,current_conf=conf_goal)
    #print("Edge between conf 1 and conf 2 is free collision:", res)



if __name__ == "__main__":
    #run_2d()
    #run_prm()
    run_3d()
    # generate_graph()
