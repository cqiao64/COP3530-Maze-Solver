import pygame
import random
import numpy as np
from collections import deque
from pygame.locals import *
import time 
from queue import PriorityQueue

# Defining some colors
light_green = (144, 238, 144)  
green = (0, 255, 0)  
light_red = (255, 182, 193)  
red = (255, 0, 0)  
yellow = (255, 255, 0)  

# DisjointSet class helps to find and merge sets. Useful in Kruskal's algorithm for maze generation.
class DisjointSet:
    def __init__(self, rows, cols):
        self.parent = {(i, j): (i, j) for i in range(rows) for j in range(cols)}  # Each cell is its own parent initially.
        self.rank = {(i, j): 0 for i in range(rows) for j in range(cols)}  # Rank of each cell.

    def find(self, cell):  # Find the root of a cell.
        if self.parent[cell] != cell:
            self.parent[cell] = self.find(self.parent[cell])
        return self.parent[cell]

    def union(self, cell1, cell2):  # Union two cells into one set.
        root1, root2 = self.find(cell1), self.find(cell2)
        if root1 != root2:  # If the cells are from different sets.
            if self.rank[root1] < self.rank[root2]:  # Merge the set with smaller rank to the one with larger rank.
                self.parent[root1] = root2
            elif self.rank[root1] > self.rank[root2]:
                self.parent[root2] = root1
            else:  # If both have the same rank.
                self.parent[root2] = root1
                self.rank[root1] += 1

class Maze:
    def __init__(self, rows, cols, screen):
        # Initialize the dimensions of the maze (rows and columns)
        self.rows = rows
        self.cols = cols
        
        # This will store the actual maze representation
        self.maze = None
        
        # Screen to display the maze
        self.screen = screen
        
        # Size of each cell in the maze when displayed
        self.cell_size = 2
        
        # Possible directions a path can take within the maze
        self.directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        
        # Define the color for path, wall, and solution for visualization
        self.path_color = (255, 255, 255)
        self.wall_color = (0, 0, 0)
        self.solution_color = (255, 0, 0)
        
        # Flag to check if the maze has been solved
        self.solved = False
        
        # Keep track of nodes expanded during solving
        self.nodes_expanded = 0
        
        # Store the length of the final path taken to solve
        self.length_of_path = 0
        
        # Time taken to solve the maze
        self.time_to_solve = 0
        
        # Store the original maze for reset purposes
        self.original_maze = None
        
        # Stats specifically for A* and Dijkstra's algorithms
        self.astar_stats = {"Nodes Expanded": 0, "Length of Path": 0, "Time to Solve": 0}
        self.dijkstra_stats = {"Nodes Expanded": 0, "Length of Path": 0, "Time to Solve": 0}

    def reset(self):
        # Reset maze attributes to their initial state
        self.maze = None
        self.solved = False
        self.nodes_expanded = 0
        self.length_of_path = 0
        self.time_to_solve = 0
        
        # Reset the maze back to its original state
        self.maze = np.copy(self.original_maze)

    def reset_current_stats(self):
        # Reset only the stats for the current run, without modifying the maze itself
        self.solved = False
        self.nodes_expanded = 0
        self.length_of_path = 0
        self.time_to_solve = 0
        
        # Reset the maze back to its original state
        self.maze = np.copy(self.original_maze)

    def show_current_stats(self):
        # Capture the current state of the screen for later use
        snapshot = pygame.display.get_surface().copy()
        
        # Create a surface to display the stats
        stats_screen = pygame.Surface((400, 150))
        
        while True:
            # Handle events, e.g., closing the window
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return snapshot
            
            # Set the background color for the stats screen
            stats_screen.fill((0, 0, 0))
            
            # Combine the stats from A* and Dijkstra into one dictionary
            flattened_stats = {f"{algo} {metric}": value for algo, stats in {"A*": self.astar_stats, "Dijkstra": self.dijkstra_stats}.items() for metric, value in stats.items()}
            
            # Render the metrics onto the stats screen
            draw_metrics(stats_screen, flattened_stats, 20, 20)
            
            # Render the back button and handle its events
            if self.draw_back_button_and_handle_event(stats_screen):
                return snapshot
            
            # Display the stats screen on top of the main screen
            self.screen.blit(stats_screen, (0, 0))
            
            # Update the display to show the latest visuals
            pygame.display.update()

    def solve_a_star(self, start, goal):
        # Reset stats for A* algorithm
        self.astar_stats = {"Nodes Expanded": 0, "Length of Path": 0, "Time to Solve": 0}
        # Call A* algorithm and get the path from start to goal
        came_from, cost_so_far = self.a_star(start, goal)
        # Reconstruct and visualize the path
        self.reconstruct_path(came_from, start, goal)
        # Store the stats post solving
        self.astar_stats["Nodes Expanded"] = self.nodes_expanded
        self.astar_stats["Length of Path"] = self.length_of_path
        self.astar_stats["Time to Solve"] = self.time_to_solve

    def solve_dijkstra(self, start, goal):
        # Reset stats for Dijkstra's algorithm
        self.dijkstra_stats = {"Nodes Expanded": 0, "Length of Path": 0, "Time to Solve": 0}
        # Call Dijkstra's algorithm and get the path from start to goal
        came_from, cost_so_far = self.dijkstra(start, goal)
        # Reconstruct and visualize the path
        self.reconstruct_path(came_from, start, goal)
        # Store the stats post solving
        self.dijkstra_stats["Nodes Expanded"] = self.nodes_expanded
        self.dijkstra_stats["Length of Path"] = self.length_of_path
        self.dijkstra_stats["Time to Solve"] = self.time_to_solve

    def draw_cell(self, x, y, color):
        # Draw a cell at given (x,y) coordinates with the specified color
        pygame.draw.rect(self.screen, color, (y*self.cell_size, x*self.cell_size, self.cell_size, self.cell_size))

    def generate(self):
        # Initialize a maze filled with ones (walls)
        self.maze = np.ones((self.rows, self.cols))
        # List of edges for maze generation
        edges = []
        sets = DisjointSet(self.rows//2, self.cols//2)

        # Populate the edges for maze generation
        for i in range(self.rows//2):
            for j in range(self.cols//2):
                for dx, dy in [(0, 1), (1, 0)]:
                    nx, ny = i + dx, j + dy
                    if 0 <= nx < self.rows//2 and 0 <= ny < self.cols//2:
                        edges.append(((i, j), (nx, ny)))

        np.random.shuffle(edges)

        # Construct the maze using the randomized Kruskal's algorithm
        for (x1, y1), (x2, y2) in edges:
            if sets.find((x1, y1)) != sets.find((x2, y2)):
                sets.union((x1, y1), (x2, y2))
                self.maze[2*x1+1, 2*y1+1] = 0
                self.maze[2*x2+1, 2*y2+1] = 0
                self.maze[x1 + x2 + 1, y1 + y2 + 1] = 0

        # Set start and end points in the maze
        self.maze[0, 1] = self.maze[-2, -1] = 0
        # Visualize the generated maze
        self.draw_maze()

    def _find(self, cell, parent):
        # Helper function to find the root of a given cell (for disjoint set union-find)
        if parent[cell] != cell:
            parent[cell] = self._find(parent[cell], parent)
        return parent[cell]

    def _union(self, cell1, cell2, parent, rank):
        # Helper function to union two sets (for disjoint set union-find)
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
        # Start the timer to measure the time to solve
        start_time = time.time()

        if self.solved:
            return

        # Initialize the previous cell matrix and the queue for BFS traversal
        prev = np.full((self.rows, self.cols, 2), -1, dtype=int)
        queue = deque([(0, 1)])
        # Breadth-first traversal to solve the maze
        while queue:
            x, y = queue.popleft()
            self.nodes_expanded += 1
            for dx, dy in self.directions:
                nx, ny = x+dx, y+dy
                if 0 <= nx < self.rows and 0 <= ny < self.cols and self.maze[nx, ny] == 0 and tuple(prev[nx, ny]) == (-1, -1):
                    prev[nx, ny] = (x, y)
                    queue.append((nx, ny))
            # Update the display periodically during traversal
            if self.nodes_expanded % 100 == 0:
                pygame.display.update()

        # Reconstruct the path from end to start
        path = []
        x, y = self.rows-2, self.cols-1
        while (x, y) != (0, 1):
            path.append((x, y))
            x, y = prev[x, y]

        path.append((0, 1))
        path = path[::-1]

        # Store the length of the path and time taken to solve
        self.length_of_path = len(path)
        self.time_to_solve = time.time() - start_time

        self.solved = True
        # Visualize the solution path
        self.draw_path(path)

    def heuristic(self, a, b):
        # Simple Manhattan distance heuristic
        (x1, y1) = a
        (x2, y2) = b
        return abs(x1 - x2) + abs(y1 - y2)

    def reconstruct_path(self, came_from, start, goal):
        # Initializing the current node to the goal node
        current = goal
        path = []
        # Iterate back through the path from goal to start using the 'came_from' mapping
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)  # Append the starting node
        path.reverse()  # Reverse to get the path from start to goal
        self.draw_path(path)  # Visualize the reconstructed path

    def a_star(self, start, goal):
        # Record the start time for performance metrics
        start_time = time.time()

        # Initialize the priority queue, starting node, costs, and visited nodes
        frontier = PriorityQueue()
        frontier.put((0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}
        visited = set()  # Track visited nodes to avoid revisiting

        # Continue searching while there are nodes in the priority queue
        while not frontier.empty():
            _, current = frontier.get()

            # If the current node is the goal, break out of the loop
            if current == goal:
                break

            # Expand the current node in all possible directions
            for dx, dy in self.directions:
                next_node = current[0] + dx, current[1] + dy
                # Ensure next node is valid and is not a wall in the maze
                if 0 <= next_node[0] < self.rows and 0 <= next_node[1] < self.cols and self.maze[next_node[0]][next_node[1]] != 1:
                    new_cost = cost_so_far[current] + 1
                    # Update costs and queue if the next node has not been visited or offers a shorter path
                    if next_node not in visited or new_cost < cost_so_far[next_node]:
                        self.draw_cell(next_node[0], next_node[1], (0, 255, 0))  # Visualization of the node being considered
                        pygame.display.update()
                        cost_so_far[next_node] = new_cost
                        # Calculate priority using heuristic and add to the queue
                        priority = new_cost + self.heuristic(goal, next_node)
                        frontier.put((priority, next_node))
                        came_from[next_node] = current  # Record the current node as the predecessor of the next node
                        visited.add(next_node)  # Mark next node as visited

        # Update statistics for the A* algorithm
        self.length_of_path = cost_so_far[goal]
        self.nodes_expanded = len(visited)
        self.time_to_solve = time.time() - start_time
        self.astar_stats = {"Nodes Expanded": self.nodes_expanded, "Length of Path": self.length_of_path, "Time to Solve": round(self.time_to_solve, 2)}

        return came_from, cost_so_far

    def dijkstra(self, start, goal):
        # Record the start time for performance metrics
        start_time = time.time()

        # Initialize the priority queue, starting node, costs
        queue = PriorityQueue()
        queue.put((0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        # Continue searching while there are nodes in the priority queue
        while not queue.empty():
            _, current = queue.get()

            # If the current node is the goal, break out of the loop
            if current == goal:
                break

            # Expand the current node in all possible directions
            for dx, dy in self.directions:
                next_node = current[0] + dx, current[1] + dy
                # Ensure next node is valid and is not a wall in the maze
                if 0 <= next_node[0] < self.rows and 0 <= next_node[1] < self.cols and self.maze[next_node[0]][next_node[1]] != 1:
                    new_cost = cost_so_far[current] + 1
                    # Update costs and queue if the next node offers a shorter path
                    if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                        self.draw_cell(next_node[0], next_node[1], (0, 255, 0))  # Visualization of the node being considered
                        pygame.display.update()
                        cost_so_far[next_node] = new_cost
                        queue.put((new_cost, next_node))
                        came_from[next_node] = current

        # Update statistics for the Dijkstra algorithm
        self.length_of_path = cost_so_far[goal]
        self.nodes_expanded = len(came_from)
        self.time_to_solve = time.time() - start_time
        self.dijkstra_stats = {"Nodes Expanded": self.nodes_expanded, "Length of Path": self.length_of_path, "Time to Solve": round(self.time_to_solve, 2)}

        return came_from, cost_so_far

    def reconstruct_path(self, came_from, start, goal):
        # Begin from the goal and backtrack to the start using the 'came_from' mapping
        current = goal
        path = []
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()
        self.draw_path(path)

    def draw_path(self, path, delay=True):
        # Visualize the computed path in the maze
        for x, y in path:
            self.draw_cell(x, y, self.solution_color)
            pygame.display.update()
            # Optional delay to visualize path progression
            if delay:
                pygame.time.wait(10)

    def draw_maze(self):
        # Visualize the entire maze
        for x in range(self.rows):
            for y in range(self.cols):
                color = self.path_color if self.maze[x, y] == 0 else self.wall_color
                self.draw_cell(x, y, color)
        pygame.display.update()

    def neighbors(self, node):
        # Generate the four potential neighboring nodes
        (x, y) = node
        neighbors = [(x-1, y), (x, y-1), (x+1, y), (x, y+1)]
        # Filter out neighbors that are out of the maze boundaries
        valid_neighbors = [n for n in neighbors if 0 <= n[0] < self.rows and 0 <= n[1] < self.cols]
        return valid_neighbors

    def get_end_point(self):
        # Calculate a potential end point based on maze dimensions
        end_row, end_col = self.rows-2, self.cols-1
        # Check if the potential end point is a valid path (not a wall)
        if self.maze[end_row, end_col] == 0:
            return end_row, end_col
        return None

    def draw_back_button_and_handle_event(self, stats_screen):
        # Draw the back button and handle the event when the button is pressed
        if draw_button(stats_screen, 'BACK', 280, 50, 50, 50, (200, 0, 0), (255, 0, 0)):
            return True
        return False

    def reset_all_stats(self):
        # Reset all algorithm statistics
        self.astar_stats = {"Nodes Expanded": 0, "Length of Path": 0, "Time to Solve": 0}
        self.dijkstra_stats = {"Nodes Expanded": 0, "Length of Path": 0, "Time to Solve": 0}

    
def draw_button(screen, message, x, y, w, h, ic, ac):
    # Get current mouse position and mouse button states
    mouse = pygame.mouse.get_pos()
    click = pygame.mouse.get_pressed()

    # Check if the mouse is over the button
    if x+w > mouse[0] > x and y+h > mouse[1] > y:
        # Draw button with the active color if mouse is hovering over it
        pygame.draw.rect(screen, ac, (x, y, w, h))
        # If the button is clicked, return True
        if click[0] == 1:
            return True
    else:
        # If the mouse is not over the button, draw with the inactive color
        pygame.draw.rect(screen, ic, (x, y, w, h))

    # Render the text of the button and center it
    smallText = pygame.font.SysFont(None, 20)
    textSurf = smallText.render(message, True, (0, 0, 0))
    textRect = textSurf.get_rect()
    textRect.center = ((x+(w/2)), (y+(h/2)))
    # Draw the centered text on the button
    screen.blit(textSurf, textRect)
    return False

def draw_metrics(screen, metrics, x, y):
    # Prepare the font for rendering metrics
    font = pygame.font.SysFont(None, 20)
    # Render each metric with the corresponding value
    rendered_metrics = [font.render(f"• {key}: {value}", True, (255, 255, 255)) for key, value in metrics.items()]
    # Display each rendered metric on the screen, spaced by 20 pixels vertically
    for i, metric in enumerate(rendered_metrics):
        screen.blit(metric, (x, y + 20*i))

def main():
    # Initialize pygame
    pygame.init()

    # Maze configuration
    rows, cols = 317, 317
    cell_size = 2
    screen_width = cols * cell_size
    screen_height = rows * cell_size + 100  # additional 100 pixels for the interface
    screen = pygame.display.set_mode((screen_width, screen_height))
    screen.fill((0, 0, 0))

    # Create a new Maze object
    maze = Maze(rows, cols, screen)

    # Initialize game clock for controlling frame rate
    clock = pygame.time.Clock()  

    # Main loop control variables
    running = True
    solving = False
    algorithm_used = ""  

    while running:
        pygame.event.pump()  
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False

        # Reset the bottom 100 pixels (interface area)
        pygame.draw.rect(screen, (0, 0, 0), (0, screen_height - 100, screen_width, 100))

        # Display the maze metrics
        metrics = {
            "Nodes expanded": maze.nodes_expanded,
            "Path length": maze.length_of_path,
            "Time to solve": round(maze.time_to_solve, 2),
            "Algorithm": algorithm_used,  
        }
        draw_metrics(screen, metrics, screen_width - 200, screen_height - 100)  

        # Generate maze button
        if draw_button(screen, 'Generate', 100, screen_height - 50, 80, 30, (0, 200, 0), (0, 255, 0)):
            maze.reset_current_stats()  
            maze.reset_all_stats()  
            maze.generate()

        # Display current maze stats button
        if draw_button(screen, 'Stats', 190, screen_height - 50, 80, 30, (0, 200, 0), (0, 255, 0)):
            snapshot = maze.show_current_stats()  
            screen.blit(snapshot, (0, 0))  
            pygame.display.flip()  

        # Exit button
        if draw_button(screen, 'Exit', 280, screen_height - 50, 80, 30, (200, 0, 0), (255, 0, 0)):
            pygame.quit()
            quit()

        # Solve maze using A* algorithm button
        if not solving and draw_button(screen, 'Solve A*', 100, screen_height - 90, 80, 30, (0, 200, 0), (0, 255, 0)) and maze.maze is not None:
            solving = True
            start = (0, 1)
            goal = maze.get_end_point()
            came_from, cost_so_far = maze.a_star(start, goal)
            maze.reconstruct_path(came_from, start, goal)
            algorithm_used = "A*"  
            solving = False

        # Solve maze using Dijkstra's algorithm button
        if not solving and draw_button(screen, 'Solve Dijkstra', 190, screen_height - 90, 120, 30, (0, 200, 0), (0, 255, 0)) and maze.maze is not None:
            solving = True
            start = (0, 1)
            goal = maze.get_end_point()
            came_from, cost_so_far = maze.dijkstra(start, goal)
            maze.reconstruct_path(came_from, start, goal)
            algorithm_used = "Dijkstra"  
            solving = False

        # Clear the maze solution and reset metrics
        if not solving and draw_button(screen, 'Clear', 320, screen_height - 90, 80, 30, (200, 0, 0), (255, 0, 0)) and maze.maze is not None:
            maze.draw_maze()  
            algorithm_used = ""  
            maze.nodes_expanded = 0  
            maze.length_of_path = 0  
            maze.time_to_solve = 0  

        # Update the entire screen
        pygame.display.flip()
        # Limit frame rate to 60 FPS
        clock.tick(60)  

    pygame.quit()

if __name__ == "__main__":
    main()
