from src.environment_generator import *
from src.motion_planner import *
from src.experiments import *

def main():
    #Generate Environment
    # env = Environment(screen_height=12,screen_width=12,number_obstacles=5)
    # env_image = env.generate_shapes_plot()
    # paths = {}

    # # Create motion planner object
    # planner = MotionPlanner(env_image)
    # planner.find_obstacles()

    # start, end = planner.generate_start_end()
    # actual_distance = np.sqrt((start[0] - end[0]) ** 2 + (start[1] - end[1]) ** 2)
    # path, distance = planner.a_star(start, end)
    # paths['a_star'] = (path,distance)
    # #path, distance = planner.dijkstra(start, end)
    # print(f"A* Efficiency {(distance / actual_distance)*100}%")
    # #paths['dijkstra'] = (path,distance)
    # #path, distance = planner.bfs(start, end)
    # #paths['bfs'] = (path,distance)
    # #path, distance = planner.dfs(start, end)
    # #paths['dfs'] = (path,distance)
    # planner.plot_path_on_image(paths=paths)
    #experiment1(5,5)
    a_star_efficency()



main()