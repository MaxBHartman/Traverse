import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
from random import choice, randint

# GLOBAL VARIABLES
SCREEN_X = 5
SCREEN_Y = 5

class Environment():
    def __init__(self, screen_height, screen_width, number_obstacles):
        self._screen_height = screen_height
        self._screen_width = screen_width
        self._number_obstacles = number_obstacles
        self._obstacles = {}

    def create_random_circle(ax):
        radius = np.random.uniform(0.5, 1)  # random radius between 0.05 and 0.2
        x, y = np.random.uniform(0, SCREEN_X), np.random.uniform(0, SCREEN_Y)
        circle = patches.Circle((x, y), radius, color=np.random.rand(3,))
        ax.add_patch(circle)

    def create_random_rectangle(ax):
        width = np.random.uniform(0.5, 1)
        height = np.random.uniform(0.5, 1)
        x, y = np.random.uniform(0, SCREEN_X - width), np.random.uniform(0, SCREEN_Y - height)
        rectangle = patches.Rectangle((x, y), width, height, color=np.random.rand(3,))
        ax.add_patch(rectangle)

    def generate_shapes_plot(self):
        fig, ax = plt.subplots()
        ax.set_xlim(0, SCREEN_X)
        ax.set_ylim(0, SCREEN_Y)
        ax.set_aspect('equal')

        # Randomly choose shapes to draw
        for _ in range(self._number_obstacles):  # let's draw 10 shapes
            random_shape_creator = choice([create_random_circle, create_random_rectangle])
            random_shape_creator(ax)

        plt.show()


