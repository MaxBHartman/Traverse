import matplotlib.pyplot as plt
import numpy as np
import matplotlib.patches as patches
from random import choice, randint
from io import BytesIO
import PIL.Image
import cv2
from heapq import heappop, heappush
from collections import deque, defaultdict
import time

COLORS = [(0, 0, 225), (0, 155, 155), (0, 225, 0), (0, 155, 0)]

class MotionPlanner():
    def __init__(self, image):
        self._image = image
        self._obstacles = []
        self._start = (0,0)
        self._end = (0,0)
        self.tree = []

    
    def find_obstacles(self):

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

        return grid

    def a_star(self, start, end):
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

        return False, np.inf
    
    def dijkstra(self, start, end):
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

        return False, np.inf


    def bfs(self, start, goal):
        # find viable nodes in graph
        grid = self.create_grid(self._obstacles, self._image.shape[:2])
        queue = deque([(start, [start], 0)])
        visited = set()
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)] 

        while queue:
            (x, y), path, distance = queue.popleft()
            if (x, y) == goal:
                return path, distance
            # Explore
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]: 
                    if grid[ny][nx] == 0 and (nx, ny) not in visited: 
                        visited.add((nx, ny))
                        queue.append(((nx, ny), path + [(nx, ny)], distance + 1))
        
        return False, np.inf
    
    def dfs(self, start, goal):
        grid = self.create_grid(self._obstacles, self._image.shape[:2])
        stack = [(start, [start], 0)]  # Use a list as a stack
        visited = set()
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]

        while stack:
            (x, y), path, distance = stack.pop()  # Pop from the stack
            if (x, y) == goal:
                return path, distance
        
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if 0 <= nx < grid.shape[1] and 0 <= ny < grid.shape[0]: 
                    if grid[ny][nx] == 0 and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        stack.append(((nx, ny), path + [(nx, ny)], distance + 1))
        
        return False, np.inf
    
    def plot_path_on_image(self, paths, line_thickness=2, line_color=(0, 0, 225), animation_delay=0.0005):
        image_with_path = self._image.copy()
        cv2.circle(image_with_path, self._start, radius=line_thickness, color=(0,155,155), thickness=-1)
        cv2.circle(image_with_path, self._end, radius=line_thickness, color=(155,155,0), thickness=-1)

        for x, y, w, h in self._obstacles:
            cv2.rectangle(image_with_path, (x, y), (x + w, y + h), (0, 255, 0), 2)

        max_length = max(len(path[1][0]) for path in paths.items() if path[1][1] != np.inf)

        path_indices = {label: 1 for label in paths}

        while any(index < max_length for index in path_indices.values()):
         for label, (path, distance) in paths.items():
                if path_indices[label] < len(path) and distance != np.inf:
                    cv2.line(image_with_path, path[path_indices[label] - 1], path[path_indices[label]], COLORS[path_indices[label] % len(COLORS)], line_thickness)
                    path_indices[label] += 1

                cv2.imshow('Path on Image', image_with_path)
                cv2.waitKey(1)
                time.sleep(animation_delay)

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
    
