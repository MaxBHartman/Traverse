import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
from random import choice, randint
from io import BytesIO
import PIL.Image
import cv2

# GLOBAL VARIABLES

class Environment():
    def __init__(self, screen_height, screen_width, number_obstacles):
        self._screen_height = screen_height
        self._screen_width = screen_width
        self._number_obstacles = number_obstacles
        self._obstacles = {}

    def create_random_circle(self, ax):
        radius = np.random.uniform(0.5, 1)  # random radius between 0.05 and 0.2
        x, y = np.random.uniform(0, self._screen_width), np.random.uniform(0, self._screen_height)
        circle = patches.Circle((x, y), radius, color=(1,0,0))
        ax.add_patch(circle)

    def create_random_rectangle(self, ax):
        width = np.random.uniform(0.5, 0.2*self._screen_width)
        height = np.random.uniform(0.5, 0.2*self._screen_height)
        x, y = np.random.uniform(0, self._screen_width - width*1.05), np.random.uniform(0, self._screen_height - height*1.05)
        rectangle = patches.Rectangle((x, y), width, height, color=(1,0,0))
        ax.add_patch(rectangle)

    def generate_shapes_plot(self):
        fig, ax = plt.subplots()
        ax.set_xlim(0, self._screen_width)
        ax.set_ylim(0, self._screen_height)
        ax.set_aspect('equal')
        ax.axis('off')

        # Randomly choose shapes to draw
        for _ in range(self._number_obstacles): 
            random_shape_creator = choice([self.create_random_rectangle])
            random_shape_creator(ax)

        buf = BytesIO()
        plt.savefig(buf, format='png', bbox_inches='tight', pad_inches=0)
        buf.seek(0)

        image = PIL.Image.open(buf)
        image_arr = np.array(image)
        image_arr = image_arr[:, :, ::-1]

        return image_arr
