# Import any libraries required
import random


# The main path planning function. Additional functions, classes, 
# variables, libraries, etc. can be added to the file, but this
# function must always be defined with these arguments and must 
# return an array ('list') of coordinates (col,row).
#DO NOT EDIT THIS FUNCTION DECLARATION  输入和输出（return）不要改动
def do_a_star(grid, start, end, display_message):
    #EDIT ANYTHING BELOW HERE
    
    # Get the size of the grid
    COL = len(grid)
    ROW = len(grid[0])

    # Make a list of 10 random cell coordinates
    path = []
    for i in range(1,10):
        path.append((random.randint(0, COL-1),random.randint(0, ROW-1)))
        
    # Print an example debug message
    display_message("Created random path points")
    display_message("Start location is " + str(start))

    # Send the path points back to the gui to be displayed
    #FUNCTION MUST ALWAYS RETURN A LIST OF (col,row) COORDINATES
    return path

#end of file




"""代码1"""
# Import any libraries required
# Note: No additional libraries are used as per requirements

def do_a_star(grid, start, end, display_message):
    # Initialize grid dimensions
    COL = len(grid)
    ROW = len(grid[0]) if COL > 0 else 0
    start_col, start_row = start
    end_col, end_row = end

    # Validate start and end points
    if grid[start_col][start_row] == 0:
        display_message("[ERROR] Start point is blocked")
        return []
    if grid[end_col][end_row] == 0:
        display_message("[ERROR] End point is blocked")
        return []

    # A* algorithm implementation
    open_list = []
    closed_set = set()
    g_scores = {}  # Actual movement cost from start
    parents = {}    # Parent nodes for path reconstruction

    # Initialize starting node
    g_scores[start] = 0
    h_score = ((end_col - start_col)**2 + (end_row - start_row)**2)**0.5  # Euclidean heuristic
    open_list.append((h_score + g_scores[start], start_col, start_row))
    parents[start] = None

    path_found = False

    while open_list:
        # Get node with lowest f-score (g + h)
        current_node = min(open_list, key=lambda x: x[0])
        open_list.remove(current_node)
        current_f, current_col, current_row = current_node
        current_pos = (current_col, current_row)

        # Early exit if goal reached
        if current_pos == end:
            path_found = True
            break

        closed_set.add(current_pos)

        # Generate valid neighbors (4-direction movement)
        neighbors = []
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            n_col = current_col + dx
            n_row = current_row + dy
            if 0 <= n_col < COL and 0 <= n_row < ROW:
                if grid[n_col][n_row] == 1:  # Check if traversable
                    neighbors.append((n_col, n_row))

        # Process neighbors
        for neighbor in neighbors:
            n_pos = neighbor
            if n_pos in closed_set:
                continue

            # Calculate tentative g-score
            tentative_g = g_scores.get(current_pos, float('inf')) + 1

            # Update if better path found
            if tentative_g < g_scores.get(n_pos, float('inf')):
                parents[n_pos] = current_pos
                g_scores[n_pos] = tentative_g
                h = ((end_col - n_pos[0])**2 + (end_row - n_pos[1])**2)**0.5
                f = tentative_g + h

                # Update open list efficiently
                found = False
                for idx, item in enumerate(open_list):
                    if item[1:] == n_pos:
                        if f < item[0]:
                            open_list[idx] = (f, n_pos[0], n_pos[1])
                        found = True
                        break
                if not found:
                    open_list.append((f, n_pos[0], n_pos[1]))

    # Path reconstruction
    if not path_found:
        display_message("[WARN] No valid path exists")
        return []

    path = []
    current = end
    while current is not None:
        path.append(current)
        current = parents.get(current)
    path.reverse()

    # Validate path endpoints
    if path[0] != start or path[-1] != end:
        display_message("[WARN] Path endpoint mismatch")
        return []

    display_message("[INFO] Path found with {} nodes".format(len(path)))
    return path
    

"""代码2"""
def do_a_star(grid, start, end, display_message):
    """A* pathfinding implementation with 4-directional movement and Euclidean heuristic."""
    
    # Grid dimensions validation
    COL = len(grid)
    if COL == 0:
        display_message("[ERROR] Empty grid")
        return []
    ROW = len(grid[0])
    
    # Unpack coordinates
    start_col, start_row = start
    end_col, end_row = end
    
    # Check if start/end are blocked
    if grid[start_col][start_row] == 0:
        display_message("[ERROR] Start point is blocked")
        return []
    if grid[end_col][end_row] == 0:
        display_message("[ERROR] End point is blocked")
        return []
    
    # Heuristic function (Euclidean distance)
    def heuristic(col, row):
        return ((end_col - col)**2 + (end_row - row)**2)**0.5
    
    # Initialize data structures
    open_list = []
    closed_set = set()
    g_scores = {start: 0}
    parents = {}
    
    # Add start node: (f, col, row)
    initial_h = heuristic(start_col, start_row)
    open_list.append((initial_h + 0, start_col, start_row))
    
    path_found = False
    
    while open_list:
        # Get node with minimal f-score
        current_f, current_col, current_row = min(open_list, key=lambda x: x[0])
        current_pos = (current_col, current_row)
        open_list.remove((current_f, current_col, current_row))
        
        # Early termination if goal reached
        if current_pos == end:
            path_found = True
            break
        
        closed_set.add(current_pos)
        
        # Generate 4-direction neighbors
        neighbors = []
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:  # Left, Right, Down, Up
            n_col = current_col + dx
            n_row = current_row + dy
            # Boundary check
            if 0 <= n_col < COL and 0 <= n_row < ROW:
                # Obstacle check
                if grid[n_col][n_row] == 1:
                    neighbors.append((n_col, n_row))
        
        # Process neighbors
        for neighbor in neighbors:
            n_pos = neighbor
            if n_pos in closed_set:
                continue
            
            # Calculate tentative g-score (current g + 1 step)
            tentative_g = g_scores.get(current_pos, float('inf')) + 1
            
            # Update if better path found
            if tentative_g < g_scores.get(n_pos, float('inf')):
                parents[n_pos] = current_pos
                g_scores[n_pos] = tentative_g
                h = heuristic(*n_pos)
                f = tentative_g + h
                
                # Update existing entry in open list
                existing_entry = next((item for item in open_list if item[1:] == n_pos), None)
                if existing_entry:
                    if f < existing_entry[0]:
                        open_list.remove(existing_entry)
                        open_list.append((f, *n_pos))
                else:
                    open_list.append((f, *n_pos))
    
    # Path reconstruction
    if not path_found:
        display_message("[WARN] No valid path exists")
        return []
    
    path = []
    current = end
    while current is not None:
        path.append(current)
        current = parents.get(current)
    path.reverse()
    
    # Final validation
    if not path or path[0] != start or path[-1] != end:
        display_message("[WARN] Path endpoint mismatch")
        return []
    
    display_message(f"[INFO] Path found with {len(path)} nodes")
    return path