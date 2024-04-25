from src.environment_generator import *
from src.motion_planner import *

def main():
    # Generate Environment
    env = Environment(screen_height=8,screen_width=8,number_obstacles=10)
    env_image = env.generate_shapes_plot()

    # Create motion planner object
    planner = MotionPlanner(env_image)
    obstacles = planner.find_obstacles()
    bounds = planner._image.shape[:2]

    start, end = planner.generate_start_end()
    path = planner.a_star_algorithm(start, end)
    planner.plot_path_on_image(path=path)



main()