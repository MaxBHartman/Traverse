from src.environment_generator import *
from src.motion_planner import *

def experiment1(trials, granularity):
    number_obstacles_list = np.linspace(5, 25, granularity, dtype=int)
    bfs = {}
    dijkstra = {}
    a_star = {}
    dfs = {}
    inf_tracker = {'bfs': 0, 'dijkstra': 0, 'a_star': 0, 'dfs': 0}
    inf_count = {algorithm: dict.fromkeys(number_obstacles_list, 0) for algorithm in inf_tracker}

    for number_obstacles in number_obstacles_list:
        print(number_obstacles)
        #bfs_distances = []
        dijkstra_distances = []
        a_star_distances = []
        dfs_distances = []
        for _ in range(trials):
            env = Environment(screen_height=8, screen_width=8, number_obstacles=number_obstacles)
            env_image = env.generate_shapes_plot()
            planner = MotionPlanner(env_image)
            planner.find_obstacles()
            start, end = (0,0), (200,200)

            # Run Algorithms and Update Dictionaries
            for algorithm in ['a_star', 'dijkstra', 'dfs']:
                path, distance = getattr(planner, f'{algorithm}')(start, end)
                if np.isinf(distance):
                    inf_count[algorithm][number_obstacles] += 1
                else:
                    locals()[f"{algorithm}_distances"].append(distance)

        # Calculate the average distances for each algorithm, excluding inf values
        #bfs[number_obstacles] = np.mean(bfs_distances) if bfs_distances else np.inf
        dijkstra[number_obstacles] = np.mean(dijkstra_distances) if dijkstra_distances else np.inf
        a_star[number_obstacles] = np.mean(a_star_distances) if a_star_distances else np.inf
        dfs[number_obstacles] = np.mean(dfs_distances) if dfs_distances else np.inf
    
    # Output the results
    print('A*:', a_star)
    print('Dijkstra:', dijkstra)
    print('DFS:', dfs)
    print('BFS:', bfs)
    print('Inf Counts:', inf_count)
    
    # Plotting the results
    plt.figure(figsize=(10, 5))
    plt.plot(bfs.keys(), bfs.values(), label='BFS')
    plt.plot(dijkstra.keys(), dijkstra.values(), label='Dijkstra')
    plt.plot(a_star.keys(), a_star.values(), label='A*')
    plt.plot(dfs.keys(), dfs.values(), label='DFS')
    plt.title('Number of obstacles vs average distance traveled')
    plt.xlabel('Number of Obstacles')
    plt.ylabel('Average Distance')
    plt.legend()
    plt.show()

def a_star_efficency():
    average = 0
    for i in range(10):
        env = Environment(screen_height=12,screen_width=12,number_obstacles=10)
        env_image = env.generate_shapes_plot()
        paths = {}

        # Create motion planner object
        planner = MotionPlanner(env_image)
        planner.find_obstacles()

        start, end = planner.generate_start_end()
        actual_distance = np.sqrt((start[0] - end[0]) ** 2 + (start[1] - end[1]) ** 2)
        path, distance = planner.a_star(start, end)
        paths['a_star'] = (path,distance)
        print(f"A* Efficiency: {(distance / actual_distance)*100}%")
        average+=distance / actual_distance

        planner.plot_path_on_image(paths=paths)
    average /=10
    average*=100
    print(f"A* Efficiency: {average}%")