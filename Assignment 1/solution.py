#AHMED ALI 22i-0825 E

from queue import PriorityQueue
import numpy as np

class Coordinates:
    def __init__(self, x, y):
        self.x = x
        self.y = y

robot_coordinates = []
goal_coordinates = []
agent_coordinates = []
agent_times = []

# Function to return all the indexes of a character in a string as an unpacked tuple
def index(str, char):
    return tuple(i for i, letter in enumerate(str) if letter == char)

def print_coordinates(l):
    for i in range(len(l)):
        print(l[i].x, l[i].y)

# Initialize the grid from txt file
with open('Data/data2.txt', 'r') as file:
    N = int(file.readline().strip())
    grid = []
   
    # Read character by character, if the character='x' then put a 1 in the array otherwise put 0, 
    # if there is a /n then it means a new row
    for line in file:
        local_grid = []
        for char in line:
            if char == 'X':
                local_grid.append('X')
            else:
                local_grid.append('0')
        grid.append(local_grid)
    # Print the grid:
    for i in range(len(grid)):
        print(grid[i])

# Read robot and goal coordinates
with open('Data/Robots2.txt', 'r') as file:
    for line in file:
        start_x_index, goal_x_index = index(line, '(')
        start_y_index, goal_y_index = index(line, ')')
        start_x = line[start_x_index+1:index(line, ',')[0]]
        goal_x = line[goal_x_index+1:index(line, ',')[1]]
        start_y = line[index(line, ',')[0]+2:start_y_index]
        goal_y = line[index(line, ',')[1]+2:goal_y_index]

        robot_coordinates.append(Coordinates(int(start_x), int(start_y)))
        goal_coordinates.append(Coordinates(int(goal_x), int(goal_y)))

# Read agent coordinates and times
with open('Data/Agent2.txt', 'r') as file:
    for line in file:
        if "Agent" in line:  # Check if the line contains agent data
            # Extract agent number
            agent_number = line.split("Agent")[1].split(":")[0].strip()
            
            # Extract coordinates and times
            coords_part, times_part = line.split(" at times ")
            coords = eval(coords_part.split(":")[1].strip())
            times = eval(times_part)
            
            # Store coordinates and times
            agent_coordinates.append(coords)
            agent_times.append(times)

# Print the grid
print(grid)

# Print robot and goal coordinates
print_coordinates(robot_coordinates)
print_coordinates(goal_coordinates)

# Print agent coordinates and times
for i in range(len(agent_coordinates)):
    print(f"Agent {i+1} Coordinates:", agent_coordinates[i])
    print(f"Agent {i+1} Times:", agent_times[i])




# Start Implementing A*
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def node(position, g, h, parent):
    return {
        'position': position,
        'g': g,
        'h': h,
        'f': g + h,
        'parent': parent
    }

def print_grid(grid):
    for i in range(len(grid)):
        print(grid[i])

def construct_path(node):
    path = []
    while node is not None:
        path.append((node['position']))
        node = node['parent']
    return path[::-1]

def check_collision(robots):
    colliding_robots = []
    for i in range(len(robots)):
        for j in range(i+1, len(robots)):
            if robots[i]['position'] == robots[j]['position']:
                colliding_robots.append(i)
                colliding_robots.append(j)
    return colliding_robots

def resolve_collision(robots, colliding_robots,t):
    # each colliding robot will go a valid neighbour randomly
    dont_go_here = []
    for i in colliding_robots:
        neighbours = get_valid_neighbours(grid, robots[i]['position'],t)
        
        random_neighbour = neighbours[np.random.randint(0, len(neighbours))]
        while random_neighbour in dont_go_here:
            random_neighbour = neighbours[np.random.randint(0, len(neighbours))]
        robots[i]['position'] = neighbours[np.random.randint(0, len(neighbours))]
        dont_go_here.append(robots[i]['position'])
    return robots


def get_valid_neighbours(grid, position, t,dynamic_agents):
    neighbours = []
    N, M = len(grid), len(grid[0])
    for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
        nx, ny = position[0] + dx, position[1] + dy
        if 0 <= nx < N and 0 <= ny < M and grid[nx][ny] != 'X':
            # Check if any dynamic agent will occupy this cell at time t+1
            occupied = False
            for agent in dynamic_agents:
                if t + 1 in agent['times']:
                    agent_index = agent['times'].index(t + 1)
                    if (nx, ny) == agent['path'][agent_index]:
                        occupied = True
                        break
            if not occupied:
                neighbours.append((nx, ny))
    return neighbours

def a_star(robot,goal,grid,dynamic_agents):
    frontier = []    # Priority Queue for the robot
    open_dict = []   # Dictionary that map positions to nodes
    closed_list = set() # List of nodes that have been visited

    frontier = PriorityQueue()
    frontier.put((0, robot['position']))
    open_dict = { robot['position']: node(robot['position'], 0, heuristic(robot['position'], goal), None)}
    
   
    iter=0
    t = -1
    force_Stop = False
    paths = {}
    found = False
    # A* for single robot
    while(not found or not frontier.empty()):
            t += 1
            
            current = frontier.get()[1]    # Get the current position of the robot
        
            robot['position'] = current # Update the robot's position in the robots list

            grid[current[0]][current[1]] = 'R'    # Mark the position of the robot on the grid
            # print("Robot", i, "Position:", current)
            # #print_grid(grid)
            # print("Frontier", i, ":", frontiers[i].queue)
            # print("-------------------------------------")
            # print("-------------------------------------")
            grid[current[0]][current[1]] = '0'    # Unmark the position of the robot on the grid
            current_node = open_dict[current]   # Get the current node of the robot
            if current == goal:
                print("Goal Reached by Robot")
                path = construct_path(current_node)
                print("Path:", path)
                found = True
                return path
                
                
            closed_list.add(current)
            
            for neighbour in get_valid_neighbours(grid, current, t,dynamic_agents):
                if neighbour not in closed_list:
                    g = current_node['g'] + 1
                    h = heuristic(neighbour, goal)
                    f = g + h
                    new_node = node(neighbour, g, h, current_node)
                    # If the node is not in the frontier or the new f value is less than the previous f value
                    if neighbour not in open_dict or f < open_dict[neighbour]['f']:
                        frontier.put((f, neighbour))
                        open_dict[neighbour] = new_node
        
    if not found:
        print("No Solutions Found") 
        return None           
    
        
def simulate():
    robots = [{'position': (robot.x, robot.y), 'time': 0} for robot in robot_coordinates]
    goals = [(goal.x, goal.y) for goal in goal_coordinates]

    dynamic_agents = []
    for i in range(len(agent_coordinates)):
        dynamic_agents.append({
            'path': list(agent_coordinates[i][0]),
            'times': agent_times[i]
        })
    
    print("Dynamic Agents:", dynamic_agents)
    print("Initial Robot Positions:", robots)
    print("Goals:", goals)
    
    # Precompute paths for all robots
    paths = {}
    for i in range(len(robots)):
        if grid[goals[i][0]][goals[i][1]] == 'X':
            print(f"Goal {i} is in an obstacle")
            paths[i] = []
            robots[i]['position'] = goals[i]
            continue
        # If starting position is an obstacle, move starting to a valid neighbour
        if grid[robots[i]['position'][0]][robots[i]['position'][1]] == 'X':
            print(f"Robot {i} is in an obstacle, Choosing a random neighbour")
            neighbours = get_valid_neighbours(grid, robots[i]['position'], 0, dynamic_agents)
            robots[i]['position'] = neighbours[np.random.randint(0, len(neighbours))]

        path = a_star(robots[i], goals[i], grid, dynamic_agents)
        if path:
            paths[i] = path
        else:
            paths[i] = []  # If no path found, set an empty path

   
    t = 0
    # Keep running until all robots reach their goals
    while not all(robots[i]['position'] == goals[i] for i in range(len(robots))):
        t += 1

        # Move robots one step ahead in their path
        for i in range(len(robots)):
            if paths[i]:  # If path is not empty, move robot
                robots[i]['position'] = paths[i].pop(0)

        # Check for collisions
        colliding_robots = check_collision(robots)
        if colliding_robots:
            print(f"Collision Detected at t={t}")
            print("Colliding Robots:", colliding_robots)
            print("Colliding Positions:", [robots[i]['position'] for i in colliding_robots])

            # Resolve collision and update paths for affected robots
            robots = resolve_collision(robots, colliding_robots, t)

            # Recompute paths only for colliding robots
            for i in colliding_robots:
                new_path = a_star(robots[i], goals[i], grid, dynamic_agents)
                if new_path:
                    paths[i] = new_path
                else:
                    paths[i] = []  # If no path is found, robot stays in place
        
    return paths

paths = simulate()
print("Final Paths:", paths)
print("Path lengths:", [len(path) for path in paths.values()])











