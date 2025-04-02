def do_a_star(grid, start, end, display_message):
    """
    A* Pathfinding Algorithm with 4-directional movement and Euclidean heuristic
    
    Implements the A* search algorithm for pathfinding on a 2D grid with 
    obstacles. Uses Manhattan movement (4-directional) and Euclidean distance
    for heuristic calculation.
    
    Args:
        grid (list[list[int]]): 2D grid where 1=walkable, 0=obstacle
        start (tuple[int, int]): (column, row) starting position
        end (tuple[int, int]): (column, row) target position
        display_message (function): Callback for status messages
        
    Returns:
        list[tuple[int, int]]: Path from start to end, or empty list if no path
    """
    
    # === 1. INPUT VALIDATION ================================================
    # Validate grid structure
    if not grid or not grid[0]:     # Empty grid or empty first row check
        display_message("[ERROR] Empty grid structure")
        return []

    # Get grid dimensions
    max_col = len(grid)             # Number of columns (x-axis size)
    max_row = len(grid[0])          # Number of rows (y-axis size)
    start_col, start_row = start    # Unpack start coordinates
    end_col, end_row = end          # Unpack end coordinates

    # Validate start position boundaries
    if not (0 <= start_col < max_col and 0 <= start_row < max_row):
        display_message("[ERROR] Start position out of grid bounds")
        return []
    
    # Validate end position boundaries
    if not (0 <= end_col < max_col and 0 <= end_row < max_row):
        display_message("[ERROR] End position out of grid bounds")
        return []
    
    # Check start position accessibility
    if grid[start_col][start_row] == 0:
        display_message("[ERROR] Start position is obstructed")
        return []
    
    # Check end position accessibility
    if grid[end_col][end_row] == 0:
        display_message("[ERROR] End position is obstructed")
        return []

    # === 2. HEURISTIC FUNCTION ==============================================
    def heuristic(col, row):
        """
        Calculate Euclidean distance to target
        
        Args:
            col (int): Current column position
            row (int): Current row position
            
        Returns:
            float: Straight-line distance to target
        """
        return ((end_col - col)**2 + (end_row - row)**2)**0.5  # √(Δx² + Δy²)

    # === 3. ALGORITHM INITIALIZATION ========================================
    open_list = []                                      # Exploration frontier: (total_cost, heuristic, col, row)
    closed_set = set()                                  # Processed nodes: set((col, row))
    g_scores = {start: 0}                               # Movement costs: {(col, row): cost}
    parents = {}                                        # Path reconstruction: {(col, row): parent}

    # Initialize starting node
    initial_h = heuristic(start_col, start_row)         # Initial heuristic
    open_list.append((initial_h + 0, initial_h,         # f = g + h (g=0 at start)
                     start_col, start_row))
    
    path_found = False                                  # Path discovery flag
    expanded_nodes = 0                                  # Processed nodes counter

    # === 4. MAIN SEARCH LOOP ================================================
    while open_list:
        # --- Node Selection ---
        # Find node with minimum f-cost (primary) and h-cost (secondary)
        min_index = 0                                   # Track current minimum index
        min_f = open_list[0][0]                         # Initialize with first node's f-cost
        min_h = open_list[0][1]                         # Initialize with first node's h-cost
        
        # Linear search through open list (O(n) complexity)
        for i in range(1, len(open_list)):
            current_f = open_list[i][0]                 # Current node's total cost
            current_h = open_list[i][1]                 # Current node's heuristic
            
            # Update minimum if:
            # - Lower f-cost found, or
            # - Same f-cost but lower h-cost (tiebreaker)
            if (current_f < min_f) or \
               (current_f == min_f and current_h < min_h):
                min_f, min_h = current_f, current_h
                min_index = i

        # --- Current Node Processing ---
        current_node = open_list.pop(min_index)         # Remove from frontier
        current_f, current_h, current_col, current_row = current_node
        current_pos = (current_col, current_row)
        expanded_nodes += 1                             # Increment processed counter

        # Early exit if target reached
        if current_pos == end:
            path_found = True
            break

        closed_set.add(current_pos)                     # Mark as processed

        # --- Neighbor Generation ---
        neighbors = []
        # Define 4-directional movement vectors: left/right/up/down
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            n_col = current_col + dx                    # New column position
            n_row = current_row + dy                    # New row position
            
            # Validate neighbor position:
            # - Within grid boundaries
            # - Not obstructed (grid value == 1)
            if (0 <= n_col < max_col and 
                0 <= n_row < max_row and 
                grid[n_col][n_row] == 1):
                neighbors.append((n_col, n_row))

        # --- Neighbor Processing ---
        for n_col, n_row in neighbors:
            n_pos = (n_col, n_row)
            
            # Skip already processed nodes
            if n_pos in closed_set:
                continue

            # Calculate tentative movement cost:
            # Current node's cost + uniform step cost (1)
            tentative_g = g_scores[current_pos] + 1

            # Update if new path is better than existing
            if tentative_g < g_scores.get(n_pos, float('inf')):
                # Update path tracking
                parents[n_pos] = current_pos            # Set parent for path reconstruction
                g_scores[n_pos] = tentative_g           # Update movement cost
                
                # Calculate heuristic and total cost
                h = heuristic(n_col, n_row)             # Future cost estimate
                f = tentative_g + h                     # Total cost (f = g + h)

                # Check if node exists in open list
                existing = None
                for i, (_, _, col, row) in enumerate(open_list):
                    if (col, row) == n_pos:
                        existing = i
                        break

                # Update or add node
                if existing is not None:
                    if f < open_list[existing][0]:      # Better path found
                        open_list.pop(existing)
                        open_list.append((f, h, n_col, n_row))
                else:                                   # New node discovered
                    open_list.append((f, h, n_col, n_row))

    # === 5. PATH RECONSTRUCTION =============================================
    path = []
    if path_found:
        current = end                                   # Start backtracking from target
        
        # Follow parent pointers until reaching start
        while current in parents:
            path.append(current)
            current = parents[current]
        
        path.append(start)                              # Ensure start position is included
        path.reverse()                                  # Reverse to get start->end order

    # Validate path continuity
    if not path or path[0] != start or path[-1] != end:
        return []                                       # Invalid path detected

    # === 6. RESULT REPORTING ================================================
    display_message(
        f"[INFO] Expanded nodes: {expanded_nodes}, "
        f"Path length: {len(path)}, "
        f"Remaining open nodes: {len(open_list)}"
    )
    
    return path