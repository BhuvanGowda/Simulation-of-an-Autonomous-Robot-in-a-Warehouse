import numpy as np
import matplotlib.pyplot as plt
import heapq
import time  # Import time module for delay

# Constants
WAREHOUSE_SIZE = 10  # Warehouse dimensions (10m x 10m)
OBSTACLE_COUNT = 20  # Number of obstacles to place
ROBOT_SPEED = 0.1    # Robot speed in meters per second
MOVE_DISTANCE = 0.01  # Robot movement increment in meters (1 cm)
TRAVEL_TIME = MOVE_DISTANCE / ROBOT_SPEED  # Time taken to move 0.01 m
STOP_TIME = 2       # Time the robot stops for after each movement
START = (0, 0)       # Starting point of the robot
DESTINATION = (7, 9) # Destination point of the robot

# Directions for 4-way movement (up, right, down, left)
DIRECTIONS = [(0, 1), (1, 0), (0, -1), (-1, 0)]

class AStar:
    """A* Pathfinding Algorithm Implementation."""
    
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.rows = grid.shape[0]
        self.cols = grid.shape[1]

    def heuristic(self, a, b):
        """Calculate the Manhattan distance between two points."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

    def astar(self):
        """Perform A* pathfinding to find the shortest path from start to goal."""
        open_set = []
        heapq.heappush(open_set, (0, self.start))  # Initialize open set with the start point
        came_from = {}
        
        # Initialize scores for g and f
        g_score = {cell: float('inf') for cell in np.ndindex(self.rows, self.cols)}
        g_score[self.start] = 0
        f_score = {cell: float('inf') for cell in np.ndindex(self.rows, self.cols)}
        f_score[self.start] = self.heuristic(self.start, self.goal)

        while open_set:
            current = heapq.heappop(open_set)[1]  # Get the cell with the lowest f_score

            if current == self.goal:  # Check if the goal has been reached
                return self.reconstruct_path(came_from, current)

            # Check each direction for neighbors
            for direction in DIRECTIONS:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                if (0 <= neighbor[0] < self.rows and 
                    0 <= neighbor[1] < self.cols and 
                    self.grid[neighbor] == 0):  # Ensure the neighbor is within bounds and free

                    tentative_g_score = g_score[current] + 1  # Assume moving to neighbor costs 1

                    # Update scores and path if a better path is found
                    if tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = g_score[neighbor] + self.heuristic(neighbor, self.goal)

                        if neighbor not in [i[1] for i in open_set]:
                            heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # No path found

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from start to goal."""
        total_path = [current]
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        return total_path[::-1]  # Return reversed path

def create_warehouse():
    
    #"""Create a warehouse grid and randomly place obstacles."""
    grid = np.zeros((WAREHOUSE_SIZE, WAREHOUSE_SIZE))
    obstacle_positions = set()

    # Randomly generate unique obstacle positions
    while len(obstacle_positions) < OBSTACLE_COUNT:
        pos = (np.random.randint(0, WAREHOUSE_SIZE), np.random.randint(0, WAREHOUSE_SIZE))
        if pos != START and pos != DESTINATION:
            obstacle_positions.add(pos)

    # Mark obstacles in the grid
    for pos in obstacle_positions:
        grid[pos] = 1  # 1 indicates an obstacle

    return grid, obstacle_positions
    

def visualize_movement(path, obstacles):
    """Visualize the robot's movement through the warehouse."""
    plt.figure(figsize=(8, 8))
    plt.xlim(-1, WAREHOUSE_SIZE + 1)
    plt.ylim(-1, WAREHOUSE_SIZE + 1)
    plt.title("Robot Navigation with Pathfinding")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")

    # Plot obstacles as gray boxes, centered on their coordinates
    for obs in obstacles:
        plt.gca().add_patch(plt.Rectangle((obs[0] - 0.5, obs[1] - 0.5), 1, 1, color='gray', alpha=0.7,
                                            label='Obstacle' if obs == next(iter(obstacles)) else ""))

    # Mark start and destination with larger markers
    plt.plot(START[0], START[1], 'go', markersize=10, label='Start (0, 0)')  # Start point larger
    plt.plot(DESTINATION[0], DESTINATION[1], 'ro', markersize=10, label='Destination (7, 9)')  # Destination larger

    # Placeholder for current position in legend
    plt.plot([], [], 'bo', markersize=5, label='Current Position')  
    plt.legend(loc='upper left')  # Show legend at the start

    # Initialize the robot's position
    robot_position = np.array(START, dtype=float)

    # Move along the calculated path
    for position in path:
        # Move robot towards the next position in increments of MOVE_DISTANCE
        while np.linalg.norm(robot_position - np.array(position)) > MOVE_DISTANCE:
            direction = (np.array(position) - robot_position)
            step = direction / np.linalg.norm(direction) * MOVE_DISTANCE  # Calculate normalized step
            robot_position += step
            
            # Plot current robot position (smaller dot)
            plt.plot(robot_position[0], robot_position[1], 'bo', markersize=5)  # Current position in blue
            plt.pause(TRAVEL_TIME)  # Pause for travel time

            # Delay for 2 seconds after moving for 0.1 seconds
            time.sleep(STOP_TIME)  # Introduce the delay

        # Mark the exact position in the path
        plt.plot(position[0], position[1], 'bo', markersize=5)  # Exact position in blue
        plt.pause(STOP_TIME)  # Stop for 2 seconds

    plt.grid()
    plt.show()

def main():
    """Main function to run the robot navigation simulation."""
    # Create the warehouse with obstacles
    grid, obstacles = create_warehouse()
    print("Warehouse Grid (0 = free, 1 = obstacle):")
    print(grid)

    # Initialize A* pathfinding
    astar = AStar(grid, START, DESTINATION)
    path = astar.astar()  # Find the path using A*

    if path:
        print("Path found:", path)
        visualize_movement(path, obstacles)  # Visualize the found path
    else:
        print("No path found to the destination.")

if __name__ == "__main__":
    main()  # Run the simulation
