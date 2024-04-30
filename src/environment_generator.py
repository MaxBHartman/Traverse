import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
from random import choice, randint
from io import BytesIO
import PIL.Image
import cv2
from matplotlib.colors import ListedColormap

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
        avoid_points = [(0, 0), (200, 200)]
    
        while True:
            width = np.random.uniform(0.5, 0.2 * self._screen_width)
            height = np.random.uniform(0.5, 0.2 * self._screen_height)
            x, y = np.random.uniform(0, self._screen_width - width), np.random.uniform(0, self._screen_height - height)
            rectangle_candidate = patches.Rectangle((x, y), width, height)
            overlaps = any(rectangle_candidate.contains_point(p) for p in avoid_points)
            if not overlaps:
                rectangle = patches.Rectangle((x, y), width, height, color=(1, 0, 0))
                ax.add_patch(rectangle)
                break

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
    
    def generate_maze_image(self, size=(10, 10), complexity=0.75, density=0.75):
        complexity = int(complexity * (5 * (size[0] + size[1])))
        density = int(density * (size[0] // 2 * size[1] // 2))

        maze = np.zeros(size, dtype=bool)
        maze[0, :] = maze[-1, :] = 1
        maze[:, 0] = maze[:, -1] = 1

        for _ in range(density):
            x, y = np.random.randint(0, size[1] // 2) * 2, np.random.randint(0, size[0] // 2) * 2
            maze[y, x] = 1
            for _ in range(complexity):
                neighbours = []
                if x > 1:
                    neighbours.append((y, x - 2))
                if x < size[1] - 2:
                    neighbours.append((y, x + 2))
                if y > 1:
                    neighbours.append((y - 2, x))
                if y < size[0] - 2:
                    neighbours.append((y + 2, x))
                if neighbours:
                    y_, x_ = neighbours[np.random.randint(0, len(neighbours))]
                    if maze[y_, x_] == 0:
                        maze[y_, x_] = 1
                        maze[y_ + (y - y_) // 2, x_ + (x - x_) // 2] = 1
                        x, y = x_, y_

        fig, ax = plt.subplots()
        ax.set_xlim(0, size[1])
        ax.set_ylim(0, size[0])
        ax.set_aspect('equal')
        ax.axis('off')
        cmap = ListedColormap(['white', '#0000FF'])  # HTML code for blue (0, 0, 255)
        ax.imshow(maze, cmap=cmap)

     # Save the image into a buffer
        buf = BytesIO()
        plt.savefig(buf, format='png', bbox_inches='tight', pad_inches=0)
        plt.close(fig)
        buf.seek(0)

    # Load the image from the buffer and convert it to a NumPy array
        image = PIL.Image.open(buf)
        image_arr = np.array(image)
        image_arr = image_arr[:, :, ::-1]  # Convert RGB to BGR to match the provided generator's color format

        return image_arr
