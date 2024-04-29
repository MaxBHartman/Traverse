import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
from random import choice, randint
from io import BytesIO
import PIL.Image
import cv2
from heapq import heappop, heappush
from collections import deque, defaultdict

COLORS = [(0, 0, 225), (0, 155, 155), (0, 225, 0), (0, 155, 0)]

class MotionPlanner():
    def __init__(self, image):
        self._image = image
        self._obstacles = []
        self._start = (0,0)
        self._end = (0,0)
        self.tree = []

    
    def find_obstacles(self):
        # load env
        
        #gray_image = cv2.cvtColor(self._image, cv2.COLOR_BGR2GRAY)

        hsv_image = cv2.cvtColor(self._image, cv2.COLOR_BGR2HSV)

        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])

        mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # find obstacles
        obstacles = []
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            obstacles.append((x, y, w, h))
        self._obstacles = obstacles
        return self._obstacles
    
    def manhattan_distance(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def euclidean_distance(self, a, b):
        return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
    
    def create_grid(self, obstacles, grid_size):
        grid = np.zeros(grid_size, dtype=np.uint8)
        for (x_b, y_b, w_b, h_b) in obstacles:
            for y in range(y_b, y_b + h_b):
                for x in range(x_b, x_b + w_b):
                    grid[y, x] = 1 

        # Display the grid as an image
        return grid

    def a_star_algorithm(self, start, end):
        grid = self.create_grid(obstacles=self._obstacles, grid_size=self._image.shape[:2])
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self.euclidean_distance(start, end)}
        oheap = []

        heappush(oheap, (fscore[start], start))

        while oheap:
            current = heappop(oheap)[1]
            if current == end:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                distance = sum(self.euclidean_distance(path[i], path[i+1]) for i in range(len(path)-1))
                return path[::-1], distance

            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + self.euclidean_distance(current, neighbor)
                if 0 <= neighbor[0] < grid.shape[1]:
                    if 0 <= neighbor[1] < grid.shape[0]:
                        if grid[neighbor[1]][neighbor[0]] != 0:
                            continue
                    else:
                        continue
                else:
                    continue
                
                if self.point_inside_obstacles(neighbor):
                    continue
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue

                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.euclidean_distance(neighbor, end)
                    heappush(oheap, (fscore[neighbor], neighbor))

        return False
    
    def dijkstra_algorithm(self, start, end):
        grid = self.create_grid(obstacles=self._obstacles, grid_size=self._image.shape[:2])
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        oheap = []

        heappush(oheap, (gscore[start], start))

        while oheap:
            current = heappop(oheap)[1]
            if current == end:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                distance = sum(self.euclidean_distance(path[i], path[i+1]) for i in range(len(path)-1))
                return path[::-1], distance

            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + self.euclidean_distance(current, neighbor)
                if 0 <= neighbor[0] < grid.shape[1]:
                    if 0 <= neighbor[1] < grid.shape[0]:
                        if grid[neighbor[1]][neighbor[0]] != 0:
                            continue
                    else:
                        continue
                else:
                    continue

                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue

                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    heappush(oheap, (gscore[neighbor], neighbor))

        return False


    def bfs(self, start, goal):
        # Create grid from the obstacle list and image size
        grid = self.create_grid(self._obstacles, self._image.shape[:2])
        queue = deque([(start, [start], 0)])
        visited = set()
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]  # Right, Down, Left, Up

        while queue:
            (x, y), path, distance = queue.popleft()
            if (x, y) == goal:
                return path, distance
            
            # Explore the neighbors in the grid within the boundary
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]:  # Check grid boundaries
                    if grid[ny][nx] == 0 and (nx, ny) not in visited:  # Check if the cell is open and not visited
                        visited.add((nx, ny))
                        queue.append(((nx, ny), path + [(nx, ny)], distance + 1))
        
        return False
    
    def plot_path_on_image(self, paths, line_thickness=2, line_color=(0, 0, 225)):
    # Create a copy of the image to draw the path and obstacles
        image_with_path = self._image.copy()

    # Draw each obstacle on the image
        for x, y, w, h in self._obstacles:
            cv2.rectangle(image_with_path, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green color for obstacles

        legend_pos = (10, 30)  # Starting position for the legend text
        color_index = 0  # To alternate colors for clarity
        color_palette = [(0, 0, 255), (255, 255, 0), (255, 0, 255), (0, 255, 255)]  # Predefined colors for paths

        for label, (path, distance) in paths.items():
        # Set a specific color for each path
            path_color = color_palette[color_index % len(color_palette)]
            color_index += 1

            # Draw the path
            if len(path) > 1:
                for i in range(1, len(path)):
                    cv2.line(image_with_path, path[i - 1], path[i], path_color, line_thickness)
            elif len(path) == 1:
                cv2.circle(image_with_path, path[0], radius=line_thickness, color=path_color, thickness=-1)

        # Add path label and distance to the legend
            legend_text = f"{label}: {distance:.2f}"
            cv2.putText(image_with_path, legend_text, (legend_pos[0], legend_pos[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, path_color, 1)
            legend_pos = (legend_pos[0], legend_pos[1] + 20)  # Move down for the next entry

        cv2.circle(image_with_path, self._start, radius=line_thickness, color=(0,155,155), thickness=-1)
        cv2.circle(image_with_path, self._end, radius=line_thickness, color=(155,155,0), thickness=-1)

        cv2.imshow('Path on Image', image_with_path)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    def generate_start_end(self):
        start = (0,0)
        end = (0,0)
        max_height, max_width = self._image.shape[:2]
        while np.sqrt((start[0] - end[0])**2 +  (start[1] - end[1])**2) < 250 or (self.point_inside_obstacles(start) or self.point_inside_obstacles(end)):
            start = (np.random.randint(0, max_width), np.random.randint(0, max_height))
            end = (np.random.randint(0, max_width), np.random.randint(0, max_height))
        
        self._start = start
        self._end = end
        return start, end
    
    def point_inside_obstacles(self, point):

        px, py = point
        for obs in self._obstacles:
            ox, oy, ow, oh = obs
            if ox <= px <= ox + ow and oy <= py <= oy + oh:
                return True 
        return False
    
