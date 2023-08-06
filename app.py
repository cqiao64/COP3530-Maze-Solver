import pygame
import random
import numpy as np
from collections import deque
from pygame.locals import *
import time 
from queue import PriorityQueue

light_green = (144, 238, 144)  
green = (0, 255, 0)  
light_red = (255, 182, 193)  
red = (255, 0, 0)  
yellow = (255, 255, 0)  

class DisjointSet:
    def __init__(self, rows, cols):
        self.parent = {(i, j): (i, j) for i in range(rows) for j in range(cols)}
        self.rank = {(i, j): 0 for i in range(rows) for j in range(cols)}

    def find(self, cell):
        if self.parent[cell] != cell:
            self.parent[cell] = self.find(self.parent[cell])
        return self.parent[cell]

    def union(self, cell1, cell2):
        root1, root2 = self.find(cell1), self.find(cell2)
        if root1 != root2:
            if self.rank[root1] < self.rank[root2]:
                self.parent[root1] = root2
            elif self.rank[root1] > self.rank[root2]:
                self.parent[root2] = root1
            else:
                self.parent[root2] = root1
                self.rank[root1] += 1
            
class Maze:
    def __init__(self, rows, cols, screen):
        self.rows = rows
        self.cols = cols
        self.maze = None
        self.screen = screen
        self.cell_size = 2  
        self.directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        self.path_color = (255, 255, 255)
        self.wall_color = (0, 0, 0)
        self.solution_color = (255, 0, 0)
        self.solved = False
        self.nodes_expanded = 0
        self.length_of_path = 0
        self.time_to_solve = 0
        self.original_maze = None
        self.astar_stats = {"Nodes Expanded": 0, "Length of Path": 0, "Time to Solve": 0}
        self.dijkstra_stats = {"Nodes Expanded": 0, "Length of Path": 0, "Time to Solve": 0}
        

    def reset(self):
        self.maze = None
        self.solved = False
        self.nodes_expanded = 0
        self.length_of_path = 0
        self.time_to_solve = 0
        self.maze = np.copy(self.original_maze)

    def reset_current_stats(self):
        self.solved = False
        self.nodes_expanded = 0
        self.length_of_path = 0
        self.time_to_solve = 0
        self.maze = np.copy(self.original_maze)

    def show_current_stats(self):
        snapshot = pygame.display.get_surface().copy()  
        stats_screen = pygame.Surface((400, 150))  
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return snapshot  
            stats_screen.fill((0, 0, 0))
            flattened_stats = {f"{algo} {metric}": value for algo, stats in {"A*": self.astar_stats, "Dijkstra": self.dijkstra_stats}.items() for metric, value in stats.items()}
            draw_metrics(stats_screen, flattened_stats, 20, 20)
            if self.draw_back_button_and_handle_event(stats_screen):  
                return snapshot  
            self.screen.blit(stats_screen, (0, 0))  
            pygame.display.update()

    def solve_a_star(self, start, goal):
        self.astar_stats = {"Nodes Expanded": 0, "Length of Path": 0, "Time to Solve": 0}
        came_from, cost_so_far = self.a_star(start, goal)
        self.reconstruct_path(came_from, start, goal)
        self.astar_stats["Nodes Expanded"] = self.nodes_expanded
        self.astar_stats["Length of Path"] = self.length_of_path
        self.astar_stats["Time to Solve"] = self.time_to_solve

    def solve_dijkstra(self, start, goal):
        self.dijkstra_stats = {"Nodes Expanded": 0, "Length of Path": 0, "Time to Solve": 0}
        came_from, cost_so_far = self.dijkstra(start, goal)
        self.reconstruct_path(came_from, start, goal)
        self.dijkstra_stats["Nodes Expanded"] = self.nodes_expanded
        self.dijkstra_stats["Length of Path"] = self.length_of_path
        self.dijkstra_stats["Time to Solve"] = self.time_to_solve

    def draw_cell(self, x, y, color):
        pygame.draw.rect(self.screen, color, (y*self.cell_size, x*self.cell_size, self.cell_size, self.cell_size))

    def generate(self):
        self.maze = np.ones((self.rows, self.cols))  
        edges = []
        sets = DisjointSet(self.rows//2, self.cols//2)

        for i in range(self.rows//2):
            for j in range(self.cols//2):
                for dx, dy in [(0, 1), (1, 0)]:
                    nx, ny = i + dx, j + dy
                    if 0 <= nx < self.rows//2 and 0 <= ny < self.cols//2:
                        edges.append(((i, j), (nx, ny)))

        np.random.shuffle(edges)

        for (x1, y1), (x2, y2) in edges:
            if sets.find((x1, y1)) != sets.find((x2, y2)):
                sets.union((x1, y1), (x2, y2))
                self.maze[2*x1+1, 2*y1+1] = 0
                self.maze[2*x2+1, 2*y2+1] = 0
                self.maze[x1 + x2 + 1, y1 + y2 + 1] = 0

        self.maze[0, 1] = self.maze[-2, -1] = 0  

        self.draw_maze()

    def _find(self, cell, parent):
        if parent[cell] != cell:
            parent[cell] = self._find(parent[cell], parent)
        return parent[cell]

    def _union(self, cell1, cell2, parent, rank):
        root1 = self._find(cell1, parent)
        root2 = self._find(cell2, parent)

        if root1 != root2:
            if rank[root1] < rank[root2]:
                parent[root1] = root2
            elif rank[root1] > rank[root2]:
                parent[root2] = root1
            else:
                parent[root2] = root1
                rank[root1] += 1
        
    def solve(self):
        start_time = time.time()

        if self.solved:
            return

        prev = np.full((self.rows, self.cols, 2), -1, dtype=int)
        queue = deque([(0, 1)])
        while queue:
            x, y = queue.popleft()
            self.nodes_expanded += 1
            for dx, dy in self.directions:
                nx, ny = x+dx, y+dy
                if 0 <= nx < self.rows and 0 <= ny < self.cols and self.maze[nx, ny] == 0 and tuple(prev[nx, ny]) == (-1, -1):
                    prev[nx, ny] = (x, y)
                    queue.append((nx, ny))

            if self.nodes_expanded % 100 == 0:  
                pygame.display.update()

        path = []
        x, y = self.rows-2, self.cols-1  
        while (x, y) != (0, 1):
            path.append((x, y))
            x, y = prev[x, y]

        path.append((0, 1))  
        path = path[::-1]

        self.length_of_path = len(path)
        self.time_to_solve = time.time() - start_time

        self.solved = True
        self.draw_path(path)

    def heuristic(self, a, b):
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)

    def reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)  
        path.reverse()  
        self.draw_path(path)

    def a_star(self, start, goal):
        start_time = time.time()
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        visited = set()  

        while not frontier.empty():
            _, current = frontier.get()

            if current == goal:  
                break

            for dx, dy in self.directions:
                next_node = current[0] + dx, current[1] + dy
                if 0 <= next_node[0] < self.rows and 0 <= next_node[1] < self.cols and self.maze[next_node[0]][next_node[1]] != 1:
                    new_cost = cost_so_far[current] + 1
                    if next_node not in visited or new_cost < cost_so_far[next_node]:  
                        self.draw_cell(next_node[0], next_node[1], (0, 255, 0))  
                        pygame.display.update()
                        cost_so_far[next_node] = new_cost
                        priority = new_cost + self.heuristic(goal, next_node)
                        frontier.put((priority, next_node))
                        came_from[next_node] = current
                        visited.add(next_node)  

        self.length_of_path = cost_so_far[goal]
        self.nodes_expanded = len(visited)
        self.time_to_solve = time.time() - start_time

        self.astar_stats = {"Nodes Expanded": self.nodes_expanded, "Length of Path": self.length_of_path, "Time to Solve": round(self.time_to_solve, 2)}

        return came_from, cost_so_far

    def dijkstra(self, start, goal):
        start_time = time.time()  
        queue = PriorityQueue()
        queue.put((0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while not queue.empty():
            _, current = queue.get()

            if current == goal:
                break

            for dx, dy in self.directions:
                next_node = current[0] + dx, current[1] + dy
                if 0 <= next_node[0] < self.rows and 0 <= next_node[1] < self.cols and self.maze[next_node[0]][next_node[1]] != 1:
                    new_cost = cost_so_far[current] + 1
                    if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                        self.draw_cell(next_node[0], next_node[1], (0, 255, 0))  
                        pygame.display.update()
                        cost_so_far[next_node] = new_cost
                        queue.put((new_cost, next_node))
                        came_from[next_node] = current

        self.length_of_path = cost_so_far[goal]
        self.nodes_expanded = len(came_from)
        self.time_to_solve = time.time() - start_time

        self.dijkstra_stats = {"Nodes Expanded": self.nodes_expanded, "Length of Path": self.length_of_path, "Time to Solve": round(self.time_to_solve, 2)}

        return came_from, cost_so_far

    def reconstruct_path(self, came_from, start, goal):
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        self.draw_path(path)
    
    def draw_path(self, path, delay=True):
        for x, y in path:
            self.draw_cell(x, y, self.solution_color)
            pygame.display.update()
            if delay:
                pygame.time.wait(10)

    def draw_maze(self):
        for x in range(self.rows):
            for y in range(self.cols):
                color = self.path_color if self.maze[x, y] == 0 else self.wall_color
                self.draw_cell(x, y, color)
        pygame.display.update()
    
    def neighbors(self, node):
        (x, y) = node
        neighbors = [(x-1, y), (x, y-1), (x+1, y), (x, y+1)] 
        valid_neighbors = [n for n in neighbors if 0 <= n[0] < self.rows and 0 <= n[1] < self.cols] 
        return valid_neighbors
    
    def get_end_point(self):
        end_row, end_col = self.rows-2, self.cols-1  
        if self.maze[end_row, end_col] == 0:
            return end_row, end_col
        return None  
    
    def draw_back_button_and_handle_event(self, stats_screen):
        if draw_button(stats_screen, 'BACK', 280, 50, 50, 50, (200, 0, 0), (255, 0, 0)):
            return True

        return False
    
    def reset_all_stats(self):
        self.astar_stats = {"Nodes Expanded": 0, "Length of Path": 0, "Time to Solve": 0}
        self.dijkstra_stats = {"Nodes Expanded": 0, "Length of Path": 0, "Time to Solve": 0}
    
def draw_button(screen, message, x, y, w, h, ic, ac):
    mouse = pygame.mouse.get_pos()
    click = pygame.mouse.get_pressed()

    if x+w > mouse[0] > x and y+h > mouse[1] > y:
        pygame.draw.rect(screen, ac, (x, y, w, h))
        if click[0] == 1:
            return True
    else:
        pygame.draw.rect(screen, ic, (x, y, w, h))

    smallText = pygame.font.SysFont(None, 20)
    textSurf = smallText.render(message, True, (0, 0, 0))
    textRect = textSurf.get_rect()
    textRect.center = ((x+(w/2)), (y+(h/2)))
    screen.blit(textSurf, textRect)
    return False

def draw_metrics(screen, metrics, x, y):
    font = pygame.font.SysFont(None, 20)
    rendered_metrics = [font.render(f"• {key}: {value}", True, (255, 255, 255)) for key, value in metrics.items()]
    for i, metric in enumerate(rendered_metrics):
        screen.blit(metric, (x, y + 20*i))


def main():
    pygame.init()

    rows, cols = 317, 317
    cell_size = 2
    screen_width = cols * cell_size
    screen_height = rows * cell_size + 100  
    screen = pygame.display.set_mode((screen_width, screen_height))
    screen.fill((0, 0, 0))

    maze = Maze(rows, cols, screen)

    clock = pygame.time.Clock()  

    running = True
    solving = False
    algorithm_used = ""  

    while running:
        pygame.event.pump()  
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False

        pygame.draw.rect(screen, (0, 0, 0), (0, screen_height - 100, screen_width, 100))

        metrics = {
            "Nodes expanded": maze.nodes_expanded,
            "Path length": maze.length_of_path,
            "Time to solve": round(maze.time_to_solve, 2),
            "Algorithm": algorithm_used,  
        }

        draw_metrics(screen, metrics, screen_width - 200, screen_height - 100)  

        if draw_button(screen, 'Generate', 100, screen_height - 50, 80, 30, (0, 200, 0), (0, 255, 0)):  
            maze.reset_current_stats()  
            maze.reset_all_stats()  
            maze.generate()

        if draw_button(screen, 'Stats', 190, screen_height - 50, 80, 30, (0, 200, 0), (0, 255, 0)):  
            snapshot = maze.show_current_stats()  
            screen.blit(snapshot, (0, 0))  
            pygame.display.flip()  

        if draw_button(screen, 'Exit', 280, screen_height - 50, 80, 30, (200, 0, 0), (255, 0, 0)):  
            pygame.quit()
            quit()

        if not solving and draw_button(screen, 'Solve A*', 100, screen_height - 90, 80, 30, (0, 200, 0), (0, 255, 0)) and maze.maze is not None:
            solving = True
            start = (0, 1)
            goal = maze.get_end_point()
            came_from, cost_so_far = maze.a_star(start, goal)
            maze.reconstruct_path(came_from, start, goal)
            algorithm_used = "A*"  
            solving = False

        if not solving and draw_button(screen, 'Solve Dijkstra', 190, screen_height - 90, 120, 30, (0, 200, 0), (0, 255, 0)) and maze.maze is not None:
            solving = True
            start = (0, 1)
            goal = maze.get_end_point()
            came_from, cost_so_far = maze.dijkstra(start, goal)
            maze.reconstruct_path(came_from, start, goal)
            algorithm_used = "Dijkstra"  
            solving = False

        if not solving and draw_button(screen, 'Clear', 320, screen_height - 90, 80, 30, (200, 0, 0), (255, 0, 0)) and maze.maze is not None:
            maze.draw_maze()  
            algorithm_used = ""  
            maze.nodes_expanded = 0  
            maze.length_of_path = 0  
            maze.time_to_solve = 0  

        pygame.display.flip()
        clock.tick(60)  

    pygame.quit()

if __name__ == "__main__":
    main()