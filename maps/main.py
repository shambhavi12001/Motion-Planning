import numpy as np
import time
import matplotlib.pyplot as plt; plt.ion()
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Rectangle
import Planner
import os
import csv

def tic():
  return time.time()
def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))
  

def load_map(fname):
  '''
  Loads the bounady and blocks from map file fname.
  
  boundary = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  
  blocks = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'],
            ...,
            ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  '''
  mapdata = np.loadtxt(fname,dtype={'names': ('type', 'xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'),\
                                    'formats': ('S8','f', 'f', 'f', 'f', 'f', 'f', 'f','f','f')})
  blockIdx = mapdata['type'] == b'block'
  boundary = mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  blocks = mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  return boundary, blocks


def draw_map(boundary, blocks, start, goal):
  '''
  Visualization of a planning problem with environment boundary, obstacle blocks, and start and goal points
  '''
  goal = np.atleast_2d(goal)
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  hb = draw_block_list(ax,blocks)
  hs = ax.plot(start[0:1],start[1:2],start[2:],'ro',markersize=7,markeredgecolor='k')
  hg = ax.plot(goal[:,0],goal[:,1],goal[:,2],'go',markersize=7,markeredgecolor='k')  
  for i, (x, y, z) in enumerate(goal):
    ax.text(x+0.7, y+0.2, z, str(i + 1), fontsize=10, color='k')
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  ax.set_xlim(boundary[0,0],boundary[0,3])
  ax.set_ylim(boundary[0,1],boundary[0,4])
  ax.set_zlim(boundary[0,2],boundary[0,5])
  return fig, ax, hb, hs, hg

def draw_block_list(ax,blocks):
  '''
  Subroutine used by draw_map() to display the environment blocks
  '''
  v = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]],dtype='float')
  f = np.array([[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[0,1,2,3],[4,5,6,7]])
  clr = blocks[:,6:]/255
  n = blocks.shape[0]
  d = blocks[:,3:6] - blocks[:,:3] 
  vl = np.zeros((8*n,3))
  fl = np.zeros((6*n,4),dtype='int64')
  fcl = np.zeros((6*n,3))
  for k in range(n):
    vl[k*8:(k+1)*8,:] = v * d[k] + blocks[k,:3]
    fl[k*6:(k+1)*6,:] = f + k*8
    fcl[k*6:(k+1)*6,:] = clr[k,:]
  
  if type(ax) is Poly3DCollection:
    ax.set_verts(vl[fl])
  else:
    pc = Poly3DCollection(vl[fl], alpha=0.25, linewidths=1, edgecolors='k')
    pc.set_facecolor(fcl)
    h = ax.add_collection3d(pc)
    return h


def animate_dynamic_target(ax, goal, paths, times, goal1_stay, goal_other_stay, slowdown=5):
  goal_stays = [goal1_stay] + [goal_other_stay] * (goal.shape[0] - 1)
  goal_event_times = np.cumsum(goal_stays)
  path_event_times = np.cumsum(times)

  goal_idx = 0
  path_idx = 0
  current_goal_handle = ax.plot(
      [goal[0,0]], [goal[0,1]], [goal[0,2]],
      'bo', markersize=9, markeredgecolor='k'
  )
  current_path_handle = None
  start_time = time.time()

  while goal_idx < len(goal) or path_idx < len(paths):
    sim_time = (time.time() - start_time) / slowdown

    if goal_idx < len(goal) and sim_time >= goal_event_times[goal_idx]:
      if current_goal_handle is not None:
        current_goal_handle[0].remove()
      if goal_idx < len(goal) - 1:
        current_goal = goal[goal_idx + 1]
        current_goal_handle = ax.plot(
            [current_goal[0]], [current_goal[1]], [current_goal[2]],
            'bo', markersize=9, markeredgecolor='k'
        )
      goal_idx += 1

    if path_idx < len(paths) and sim_time >= path_event_times[path_idx]:
      if current_path_handle is not None:
        current_path_handle[0].remove()
      path = paths[path_idx]
      current_path_handle = ax.plot(
          path[:, 0], path[:, 1], path[:, 2],
          'r-', linewidth=1.0
      )
      path_idx += 1

    plt.draw()
    plt.pause(0.01)

  if current_path_handle is not None:
    current_path_handle[0].remove()
  return path_event_times <= goal_event_times

def runtest(mapfile, start, goal, verbose = True, dynamic_target = False):
  '''
  This function:
   * loads the provided mapfile
   * creates a motion planner
   * plans a path from start to goal
   * checks whether the path is collision free and reaches the goal
   * computes the path length as a sum of the Euclidean norm of the path segments
  '''
  # Load a map and instantiate a motion planner
  boundary, blocks = load_map(mapfile)
  MP = Planner.MyPlanner(boundary, blocks) # TODO: replace this with your own planner implementation
  
  # Display the environment
  if verbose:
    fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal)

  # Call the motion planner
  if dynamic_target:
    paths = []
    times = []
    for i in range(goal.shape[0]):
      t0 = tic()
      path = MP.plan(start, goal[i])
      elapsed = time.time() - t0
      toc(t0,"Planning")
      times.append(elapsed)
      paths.append(path)  
  else:
      t0 = tic()
      path = MP.plan(start, goal)
      toc(t0,"Planning")

  # Visualize the dynamic moving process and check if the goal is reached in required time
  if dynamic_target:
    mapfile_lower = mapfile.lower()
    if 'window' in mapfile_lower:
      goal1_stay = 5
      goal_other_stay = 2
    else:
      goal1_stay = 3
      goal_other_stay = 2
    slowdown = 5 # Time slowdown factor for visualization, tune this value to slow down the visualization
    path_success = animate_dynamic_target(
        ax, goal, paths, times, goal1_stay, goal_other_stay, slowdown
    )

  # Plot the path
  if verbose:
    if dynamic_target:
      for i, path in enumerate(paths):
        if path_success[i]:
          ax.plot(path[:,0],path[:,1],path[:,2],'r-', linewidth=1.0)
        else:
          ax.plot(
              [goal[i,0]], [goal[i,1]], [goal[i,2]],
              'mo', markersize=9, markeredgecolor='k'
             )
    else:
      ax.plot(path[:,0],path[:,1],path[:,2],'r-', linewidth=1.0)

  # TODO: You should verify whether all paths actually intersects any of the obstacles in continuous space
  # TODO: You can implement your own algorithm or use an existing library for segment and 
  #       axis-aligned bounding box (AABB) intersection
  collision = False
  if dynamic_target:
    for path in paths:
        if MP.collision_check(path, blocks):
            collision = True
            break
  else:
    collision = MP.collision_check(path, blocks)
    
  if dynamic_target:
    goal_reached_list = []
    pathlength = []
    for i, path in enumerate(paths):
        reached = np.sum((path[-1] - goal[i])**2) <= 0.1
        goal_reached_list.append(reached)

        pathlength.append(np.sum(np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1))))
    goal_reached = np.all(goal_reached_list)
    success = (not collision) and goal_reached and np.all(path_success)
  else:
    goal_reached = np.sum((path[-1] - goal)**2) <= 0.1
    pathlength = np.sum(np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1)))
    success = (not collision) and goal_reached
    
    if dynamic_target:
      plot_plane_simulations(
          paths,
          start,
          goal,
          blocks=blocks,
          boundary=boundary,
          title="Dynamic Target Plane Projections"
      )
    else:
      plot_plane_simulations(
          path,
          start,
          goal,
          blocks=blocks,
          boundary=boundary,
          title="Static Target Plane Projections"
      )
  return success, pathlength


def test_single_cube(verbose = True, dynamic_target = False):
  print('Running single cube test...\n') 
  start = np.array([2.3, 2.3, 1.3])
  goal = np.array([7.0, 7.0, 5.5])
  success, pathlength = runtest('./maps/E4_single_cube.txt', start, goal, verbose, dynamic_target)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')
  
  
def test_maze(verbose = True, dynamic_target = False):
  print('Running maze test...\n') 
  start = np.array([0.0, 0.0, 1.0])
  goal = np.array([12.0, 12.0, 5.0])
  success, pathlength = runtest('./maps/E2_maze.txt', start, goal, verbose, dynamic_target)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

    
def test_window(verbose = True, dynamic_target = True):
  print('Running window test...\n') 
  start = np.array([0.2, -4.9, 0.2])
  goal = np.array([
    [8.800, 12.300, 3.800],
    [7.687, 13.227, 4.449],
    [5.000, 13.610, 4.718],
    [2.313, 13.227, 4.449],
    [1.200, 12.300, 3.800],
    [2.313, 11.373, 3.151],
    [5.000, 10.990, 2.882],
    [7.687, 11.373, 3.151],
  ])
  success, pathlength = runtest('./maps/E6_window.txt', start, goal, verbose, dynamic_target)
  print('Success: %r'%success)
  print('Path length: %s'%pathlength)
  print('\n')

  
def test_tower(verbose = True, dynamic_target = False):
  print('Running tower test...\n') 
  start = np.array([2.5, 4.0, 0.5])
  goal = np.array([4.0, 2.5, 19.5])
  success, pathlength = runtest('./maps/E5_tower.txt', start, goal, verbose, dynamic_target)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

     
def test_flappy_bird(verbose = True, dynamic_target = False):
  print('Running flappy bird test...\n') 
  start = np.array([0.5, 2.5, 5.5])
  goal = np.array([19.0, 2.5, 5.5])
  success, pathlength = runtest('./maps/E1_flappy_bird.txt', start, goal, verbose, dynamic_target)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength) 
  print('\n')

  
def test_room(verbose = True, dynamic_target = True):
  print('Running room test...\n') 
  start = np.array([1.0, 5.0, 1.5])
  goal = np.array([[1.7, 0.5 , 1.7],
           [8.0, 1.0, 1.5],
           [6.0, 4.0, 3.0],
           [3.0, 3.6, 0.5],
           [3.0, 7.0 , 1.0],
           [6.0, 8.0, 0.5],
           [8.0, 6.0 , 1.5],
           [9, 7.5, 0.5]])
  success, pathlength = runtest('./maps/E7_room.txt', start, goal, verbose, dynamic_target)
  print('Success: %r'%success)
  print('Path length: %s'%pathlength)
  print('\n')


def test_monza(verbose = True, dynamic_target = False):
  print('Running monza test...\n')
  start = np.array([0.5, 1.0, 4.9])
  goal = np.array([3.8, 1.0, 0.1])
  success, pathlength = runtest('./maps/E3_monza.txt', start, goal, verbose, dynamic_target)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

def experiment_table(mapfile, start, goal):
    boundary, blocks = load_map(mapfile)

    epsilons = [0.25, 0.5, 1.0]
    goal_biases = [0.05, 0.1, 0.2]
    max_iters = [10000, 50000, 100000]

    results = []

    for epsilon in epsilons:
        for goal_bias in goal_biases:
            for max_iter in max_iters:
                MP = Planner.MyPlanner(boundary, blocks)

                t0 = tic()
                path, info = MP.plan(
                    start,
                    goal,
                    epsilon=epsilon,
                    bias=goal_bias,
                    max_iter=max_iter,
                    return_info=True
                )
                runtime = time.time() - t0

                results.append([
                    mapfile,
                    epsilon,
                    goal_bias,
                    max_iter,
                    info["explored_nodes"],
                    info["iterations"],
                    info["success"],
                    info["path_length"],
                    runtime
                ])

    print("map, epsilon, goal_bias, max_iter, explored_nodes, actual_iterations, success, path_length, runtime")
    for row in results:
        print(row)

    return results

def dynamic_experiment_table(
    mapfile,
    start,
    goals,
    epsilon=0.5,
    bias=0.1,
    max_iter=100000,
    t1=3.0,
    t2=1.0,
    make_plots=True,
    title="Dynamic Goal Experiment"
):
    """
    Runs RRT* to each dynamic goal sequentially while reusing the same tree.
    Records path quality, explored nodes, runtime, and timing success.

    t1 = time available for first goal
    t2 = time available for each later goal
    """

    boundary, blocks = load_map(mapfile)
    MP = Planner.MyPlanner(boundary, blocks)

    paths = []
    results = []

    cumulative_runtime = 0.0
    stay_times = [t1] + [t2] * (len(goals) - 1)

    for i, goal in enumerate(goals):
        t0 = tic()

        path, info = MP.plan(
            start,
            goal,
            epsilon=epsilon,
            bias=bias,
            max_iter=max_iter,
            return_info=True
        )

        runtime = time.time() - t0
        cumulative_runtime += runtime

        if path is None:
            path_length = None
            reached = False
            collision_free = False
            path_success = False
        else:
            path_length = np.sum(np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1)))
            reached = np.sum((path[-1] - goal) ** 2) <= 0.1
            collision_free = not MP.collision_check(path, blocks)
            path_success = reached and collision_free

        timing_success = runtime <= stay_times[i]
        overall_success = path_success and timing_success

        paths.append(path)

        results.append({
            "goal_id": i + 1,
            "epsilon": epsilon,
            "goal_bias": bias,
            "max_iter": max_iter,
            "stay_time": stay_times[i],
            "explored_nodes": info["explored_nodes"],
            "iterations": info["iterations"],
            "path_success": path_success,
            "timing_success": timing_success,
            "overall_success": overall_success,
            "path_length": path_length,
            "runtime": runtime,
            "cumulative_runtime": cumulative_runtime
        })

    print("goal, eps, bias, max_iter, stay_time, nodes, iter, path_success, timing_success, overall_success, path_length, runtime, cumulative_runtime")
    for r in results:
        print([
            r["goal_id"],
            r["epsilon"],
            r["goal_bias"],
            r["max_iter"],
            r["stay_time"],
            r["explored_nodes"],
            r["iterations"],
            r["path_success"],
            r["timing_success"],
            r["overall_success"],
            r["path_length"],
            r["runtime"],
            r["cumulative_runtime"]
        ])

    if make_plots:
        plot_dynamic_3d(
            paths,
            start,
            goals,
            boundary,
            blocks,
            title=title + " 3-D Paths"
        )

        plot_plane_simulations(
            paths,
            start,
            goals,
            blocks=blocks,
            boundary=boundary,
            title=title + " Plane Projections"
        )

    return results, paths, boundary, blocks
    
def run_all_experiment_tables():
    experiments = [
        (
            "flappy_bird",
            "./maps/E1_flappy_bird.txt",
            np.array([0.5, 2.5, 5.5]),
            np.array([19.0, 2.5, 5.5])
        ),
        (
            "maze",
            "./maps/E2_maze.txt",
            np.array([0.0, 0.0, 1.0]),
            np.array([12.0, 12.0, 5.0])
        ),
        (
            "monza",
            "./maps/E3_monza.txt",
            np.array([0.5, 1.0, 4.9]),
            np.array([3.8, 1.0, 0.1])
        ),
        (
            "single_cube",
            "./maps/E4_single_cube.txt",
            np.array([2.3, 2.3, 1.3]),
            np.array([7.0, 7.0, 5.5])
        ),
        (
            "tower",
            "./maps/E5_tower.txt",
            np.array([2.5, 4.0, 0.5]),
            np.array([4.0, 2.5, 19.5])
        )
    ]

    all_results = []

    for name, mapfile, start, goal in experiments:
        print("\n===================================")
        print("Experiment:", name)
        print("===================================")

        results = experiment_table(mapfile, start, goal)

        for row in results:
            all_results.append([name] + row)

    return all_results


def plot_plane_simulations(paths, start, goal, blocks=None, boundary=None, title="Plane Projections"):
    if not isinstance(paths, list):
        paths = [paths]

    goal = np.atleast_2d(goal)

    fig, axs = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle(title)

    planes = [
        ("XY Plane", 0, 1, "X", "Y"),
        ("XZ Plane", 0, 2, "X", "Z"),
        ("YZ Plane", 1, 2, "Y", "Z")
    ]

    for ax, (plane_title, a, b, xlabel, ylabel) in zip(axs, planes):
        ax.set_title(plane_title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(True)

        if blocks is not None:
            for block in blocks:
                min_a = block[a]
                min_b = block[b]
                max_a = block[a + 3]
                max_b = block[b + 3]

                rect = Rectangle(
                    (min_a, min_b),
                    max_a - min_a,
                    max_b - min_b,
                    alpha=0.25,
                    edgecolor="k",
                    facecolor="gray"
                )
                ax.add_patch(rect)

        for path in paths:
            if path is None:
                continue
            ax.plot(path[:, a], path[:, b], linewidth=1.0)

        ax.plot(start[a], start[b], 'go', markersize=8, markeredgecolor='k', label="Start")
        ax.plot(goal[:, a], goal[:, b], 'bo', markersize=8, markeredgecolor='k', label="Goals")

        for i, g in enumerate(goal):
            ax.text(g[a], g[b], str(i + 1), fontsize=9)

        if boundary is not None:
            ax.set_xlim(boundary[0, a], boundary[0, a + 3])
            ax.set_ylim(boundary[0, b], boundary[0, b + 3])

        ax.axis("equal")

    axs[0].legend()
    plt.tight_layout()
    plt.show()
  
def plot_dynamic_3d(paths, start, goals, boundary, blocks, title="Dynamic Goal 3-D Paths"):
    goal_array = np.atleast_2d(goals)

    fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal_array)
    ax.set_title(title)

    for i, path in enumerate(paths):
        if path is None:
            ax.plot(
                [goal_array[i, 0]],
                [goal_array[i, 1]],
                [goal_array[i, 2]],
                'mo',
                markersize=8,
                markeredgecolor='k'
            )
            continue

        ax.plot(
            path[:, 0],
            path[:, 1],
            path[:, 2],
            linewidth=1.0,
            label=f"Goal {i+1}"
        )

    ax.legend(fontsize=7)
    plt.tight_layout()
    plt.show()

def dynamic_time_sweep(

    mapfile,
    start,
    goals,
    time_settings,
    epsilon=0.5,
    bias=0.1,
    max_iter=100000
):
    """
    Runs dynamic planning for different (t1, t2) values.

    t1 = time available for first goal
    t2 = time available for each later goal
    """

    sweep_rows = []

    for t1, t2 in time_settings:
        print(f"\n--- Dynamic time sweep: t1 = {t1}, t2 = {t2} ---")

        results, paths, boundary, blocks = dynamic_experiment_table(
            mapfile,
            start,
            goals,
            epsilon=epsilon,
            bias=bias,
            max_iter=max_iter,
            t1=t1,
            t2=t2,
            make_plots=False,
            title=f"Dynamic sweep t1={t1}, t2={t2}"
        )

        path_successes = sum(r["path_success"] for r in results)
        timing_successes = sum(r["timing_success"] for r in results)
        overall_successes = sum(r["overall_success"] for r in results)

        total_path_length = sum(
            r["path_length"] for r in results
            if r["path_length"] is not None
        )

        total_runtime = results[-1]["cumulative_runtime"] if len(results) > 0 else 0.0

        sweep_rows.append([
            t1,
            t2,
            path_successes,
            timing_successes,
            overall_successes,
            total_path_length,
            total_runtime
        ])

    print("\nt1, t2, path_successes, timing_successes, overall_successes, total_path_length, total_runtime")
    for row in sweep_rows:
        print(row)

    return sweep_rows
    
def run_dynamic_report_outputs():
    ensure_dir("figures")
    ensure_dir("results")

    dynamic_configs = [
        {
            "name": "window",
            "mapfile": "./maps/E6_window.txt",
            "start": np.array([0.2, -4.9, 0.2]),
            "goals": np.array([
                [8.800, 12.300, 3.800],
                [7.687, 13.227, 4.449],
                [5.000, 13.610, 4.718],
                [2.313, 13.227, 4.449],
                [1.200, 12.300, 3.800],
                [2.313, 11.373, 3.151],
                [5.000, 10.990, 2.882],
                [7.687, 11.373, 3.151],
            ]),
            "t1": 3.0,
            "t2": 1.0,
            "time_settings": [
                (0.1, 0.05),
                (0.25, 0.10),
                (0.5, 0.25),
                (1.0, 0.5),
                (3.0, 1.0)
            ]
        },
        {
            "name": "room",
            "mapfile": "./maps/E7_room.txt",
            "start": np.array([1.0, 5.0, 1.5]),
            "goals": np.array([
                [1.7, 0.5, 1.7],
                [8.0, 1.0, 1.5],
                [6.0, 4.0, 3.0],
                [3.0, 3.6, 0.5],
                [3.0, 7.0, 1.0],
                [6.0, 8.0, 0.5],
                [8.0, 6.0, 1.5],
                [9.0, 7.5, 0.5]
            ]),
            "t1": 0.3,
            "t2": 0.1,
            "time_settings": [
                (0.3, 0.1),
                (0.5, 0.25),
                (1.0, 0.5),
                (2.0, 1.0),
                (5.0, 2.0)
            ]
        }
    ]

    for cfg in dynamic_configs:
        print("\n===================================")
        print(f"Running dynamic report output for {cfg['name']}")
        print("===================================")

        results, paths, boundary, blocks = dynamic_experiment_table(
            cfg["mapfile"],
            cfg["start"],
            cfg["goals"],
            epsilon=0.5,
            bias=0.1,
            max_iter=100000,
            t1=cfg["t1"],
            t2=cfg["t2"],
            make_plots=False,
            title=f"E6/E7 {cfg['name']}"
        )

        save_dynamic_results_csv(
            results,
            f"results/{cfg['name']}_dynamic_per_goal.csv"
        )

        plot_dynamic_3d_save(
            paths,
            cfg["start"],
            cfg["goals"],
            boundary,
            blocks,
            f"figures/{cfg['name']}_dynamic_3d.png",
            f"{cfg['name'].capitalize()} Dynamic Goals: 3-D Paths"
        )

        plot_plane_simulations_save(
            paths,
            cfg["start"],
            cfg["goals"],
            blocks,
            boundary,
            f"figures/{cfg['name']}_dynamic_planes.png",
            f"{cfg['name'].capitalize()} Dynamic Goals: Plane Projections"
        )

        sweep_rows = dynamic_time_sweep(
            cfg["mapfile"],
            cfg["start"],
            cfg["goals"],
            cfg["time_settings"],
            epsilon=0.5,
            bias=0.1,
            max_iter=100000
        )

        save_time_sweep_csv(
            sweep_rows,
            f"results/{cfg['name']}_dynamic_time_sweep.csv"
        )

    print("\nSaved dynamic figures to ./figures")
    print("Saved dynamic tables to ./results")

def ensure_dir(dirname):
    if not os.path.exists(dirname):
        os.makedirs(dirname)


def save_dynamic_results_csv(results, filename):
    keys = [
        "goal_id",
        "epsilon",
        "goal_bias",
        "max_iter",
        "stay_time",
        "explored_nodes",
        "iterations",
        "path_success",
        "timing_success",
        "overall_success",
        "path_length",
        "runtime",
        "cumulative_runtime"
    ]

    with open(filename, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for r in results:
            writer.writerow(r)


def save_time_sweep_csv(rows, filename):
    headers = [
        "t1",
        "t2",
        "path_successes",
        "timing_successes",
        "overall_successes",
        "total_path_length",
        "total_runtime"
    ]

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        for row in rows:
            writer.writerow(row)


def plot_dynamic_3d_save(paths, start, goals, boundary, blocks, filename, title):
    goals = np.atleast_2d(goals)

    fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goals)
    ax.set_title(title)

    for i, path in enumerate(paths):
        if path is None:
            ax.plot(
                [goals[i, 0]], [goals[i, 1]], [goals[i, 2]],
                'mo', markersize=8, markeredgecolor='k'
            )
        else:
            ax.plot(
                path[:, 0], path[:, 1], path[:, 2],
                linewidth=1.0,
                label=f"Goal {i+1}"
            )

    ax.legend(fontsize=7)
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches="tight")
    plt.close(fig)


def plot_plane_simulations_save(paths, start, goal, blocks, boundary, filename, title):
    from matplotlib.patches import Rectangle

    if not isinstance(paths, list):
        paths = [paths]

    goal = np.atleast_2d(goal)

    fig, axs = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle(title)

    planes = [
        ("XY Plane", 0, 1, "X", "Y"),
        ("XZ Plane", 0, 2, "X", "Z"),
        ("YZ Plane", 1, 2, "Y", "Z")
    ]

    for ax, (plane_title, a, b, xlabel, ylabel) in zip(axs, planes):
        ax.set_title(plane_title)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(True)

        # obstacles
        for block in blocks:
            min_a = block[a]
            min_b = block[b]
            max_a = block[a + 3]
            max_b = block[b + 3]

            rect = Rectangle(
                (min_a, min_b),
                max_a - min_a,
                max_b - min_b,
                alpha=0.25,
                edgecolor="k",
                facecolor="gray"
            )
            ax.add_patch(rect)

        # paths
        for path in paths:
            if path is None:
                continue
            ax.plot(path[:, a], path[:, b], linewidth=1.0)

        # start and goals
        ax.plot(start[a], start[b], 'go', markersize=8, markeredgecolor='k', label="Start")
        ax.plot(goal[:, a], goal[:, b], 'bo', markersize=8, markeredgecolor='k', label="Goals")

        for i, g in enumerate(goal):
            ax.text(g[a], g[b], str(i + 1), fontsize=9)

        ax.set_xlim(boundary[0, a], boundary[0, a + 3])
        ax.set_ylim(boundary[0, b], boundary[0, b + 3])
        ax.axis("equal")

    axs[0].legend()
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches="tight")
    plt.close(fig)
    
if __name__=="__main__":
  # #Static target tests
  #test_single_cube()
  #test_maze()
  #test_flappy_bird()
  #test_monza()
  #test_tower()

  # #Dynamic target tests
  test_window()
  test_room()
  run_dynamic_report_outputs()
  plt.show(block=True)
  #run_all_experiment_tables()








