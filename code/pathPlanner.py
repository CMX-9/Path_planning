def do_a_star(grid, start, end, display_message):
    """
    Adaptive A* Path Planning Algorithm (4-directional movement)
    
    Parameters:
    grid -- 2D list where 1=traversable, 0=obstacle
    start -- Tuple (col, row) start coordinates
    end -- Tuple (col, row) target coordinates
    display_message -- Function to display messages in GUI
    
    Returns:
    list -- Path as [(col1,row1), (col2,row2)...] or empty list
    
    Algorithm Features:
    1、Three Adaptive Selection Mechanisms:
        - Small grids (≤30x30): Dictionary + Set implementation, optimized for simplicity in small-scale scenarios.
        - Medium grids (≤500x500): 2D array with precomputed heuristics, balancing performance for complex scenarios.
        - Large grids (>500x500): 1D memory-optimized implementation, prioritizing memory efficiency for massive grids.
    2、Key Features:
        - 4-directional movement (up, down, left, right).
        - Euclidean distance heuristic for cost calculation.
        - Six-layer error-checking mechanism (boundary, collision, continuity, etc.).
        - Path continuity validation to ensure valid step transitions.
        - Dynamic memory management tailored to grid size.
    """

    # ====================== SECTION 1: INITIALIZATION ======================
    # Get grid dimensions
    max_col = len(grid)                             # Number of columns (vertical/Y-axis)
    max_row = len(grid[0]) if max_col > 0 else 0    # Number of rows (horizontal/X-axis)
    grid_size = max_col * max_row                   # Total cells for algorithm selection
    
    # Unpack coordinates
    start_col, start_row = start                    # Start column and row 
    end_col, end_row = end                          # End column and row
    
    path = []                                       # Final path storage


    # ====================== SECTION 2: ERROR CHECKING ======================
    # Case 1: Start and end are same point
    if (start_col, start_row) == (end_col, end_row):
        display_message("[INFO] Start and end positions are identical")
        return [start]                              # Return single-element path

    # Case 2: Invalid grid structure
    if not grid or max_row == 0:
        display_message("[ERROR] Invalid grid: empty or zero rows")
        return []

    # Case 3: Start out of bounds
    if not (0 <= start_col < max_col and 0 <= start_row < max_row):
        display_message("[ERROR] Start coordinates out of grid bounds")
        return []

    # Case 4: End out of bounds 
    if not (0 <= end_col < max_col and 0 <= end_row < max_row):
        display_message("[ERROR] End coordinates out of grid bounds")
        return []

    # Case 5: Start blocked
    if grid[start_col][start_row] == 0:
        display_message("[ERROR] Start position is obstructed")
        return []

    # Case 6: End blocked
    if grid[end_col][end_row] == 0:
        display_message("[ERROR] End position is obstructed")
        return []


    # ====================== SECTION 3: CORE ALGORITHM ======================
    # --------------- VERSION A: Dictionary-based (Small grids ≤30x30) ---------------
    if grid_size <= 900:                                            # 30x30=900 cells threshold
        """
        Version A: Dictionary-Driven Implementation
        Features and Advantages: 
        - Simplified code structure for rapid iteration, optimal for small grids.
        - Performance degrades in complex/large-scale scenarios due to computational inefficiency, thus recommended only for small grids.

        Data Structures:
        - open_list: Frontier queue storing nodes as (f, col, row) tuples.
        - closed_set: Explored node registry using set for O(1) lookups.
        - g_scores: Dictionary tracking actual movement costs from start node.
        - parents: Dictionary mapping child nodes to their optimal parents.
        """
        display_message(f"A算法")
        # Initialize data structures
        open_list = []                                              # Frontier nodes: list of (f, col, row)
        closed_set = set()                                          # Explored nodes: set of (col, row)
        g_scores = {start: 0}                                       # Movement costs: {(col,row): g_value}
        parents = {}                                                # Parent relationships: {(col,row): parent_pos}
        path_found = False                                          # Path discovery flag

        # Heuristic function (Euclidean distance)
        def heuristic(col, row):
            """Calculate straight-line distance to target"""
            return ((end_col - col)**2 + (end_row - row)**2)**0.5   # √(Δx² + Δy²)

        # Initialize start node
        initial_h = heuristic(start_col, start_row)                 # h(start)
        open_list.append((initial_h, start_col, start_row))         # (f=h+0, col, row)

        # Main search loop
        while open_list:
            # Step 1: Select node with minimum f-value
            current_node = min(open_list, key=lambda x: x[0])       # linear search
            current_f, current_col, current_row = current_node
            current_pos = (current_col, current_row)
            open_list.remove(current_node)                          # Remove from frontier

            # Step 2: Check termination condition
            if current_pos == end:
                path_found = True
                display_message("[DEBUG] Target reached, reconstructing path")
                break

            # Step 3: Mark current node as explored
            closed_set.add(current_pos)

            # Step 4: Generate 4-directional neighbors
            neighbors = []
            # Define movement vectors: up/down/left/right
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                n_col = current_col + dx                            # Calculate new column
                n_row = current_row + dy                            # Calculate new row
                # Validate neighbor position
                if (0 <= n_col < max_col and                        # Check column bounds
                    0 <= n_row < max_row and                        # Check row bounds
                    grid[n_col][n_row] == 1):                       # Check traversability
                    neighbors.append((n_col, n_row))

            # Step 5: Process each neighbor
            for n_col, n_row in neighbors:
                n_pos = (n_col, n_row)
                
                # Skip already explored nodes
                if n_pos in closed_set:
                    continue

                # Calculate tentative movement cost
                tentative_g = g_scores[current_pos] + 1             # Current g + 1 step

                # Update path if new path is better
                if tentative_g < g_scores.get(n_pos, float('inf')):
                    # Update parent-child relationship
                    parents[n_pos] = current_pos
                    # Update movement cost
                    g_scores[n_pos] = tentative_g
                    # Calculate heuristic
                    h = heuristic(n_col, n_row)
                    # Total cost
                    f = tentative_g + h

                    # Check if node exists in frontier
                    existing = next((x for x in open_list if x[1:] == n_pos), None)
                    if existing:
                        # Replace if new path is better
                        if f < existing[0]:
                            open_list.remove(existing)
                            open_list.append((f, n_col, n_row))
                    else:
                        # Add new node to frontier
                        open_list.append((f, n_col, n_row))

        # Path reconstruction
        if path_found:
            current = end                                           # Start from end
            # Backtrack through parent pointers
            while current in parents:
                path.append(current)
                current = parents[current]
            path.append(start)                                      # Add start position
            path.reverse()                                          # Reverse to get start->end order


    # --------------- VERSION B: 2D Array Optimization (Medium grids ≤500x500) ---------------
    elif 900 < grid_size <= 9500:                                 # 500x500=250,000 cells threshold
        """
        Version B: Matrix Precomputation Optimization
        Features and Advantages:
        - Sacrifices memory for speed by reducing redundant calculations.
        - Outperforms Version A in medium-scale complex scenarios.

        Key Technologies:
        - h_cache: Precomputed heuristic matrix storing Euclidean distances for all nodes.
        - 2D Array State Storage: Matrix-based tracking of exploration status and costs.
        - Ordered Frontier Insertion: Maintains open list sorting through intelligent insertion.
        """
        display_message(f"B算法")
        # Precompute heuristic matrix to avoid redundant calculations
        h_cache = [
            [((end_col - c)**2 + (end_row - r)**2)**0.5             # Precompute for each cell
             for r in range(max_row)] 
            for c in range(max_col)
        ]

        # Initialize 2D state arrays
        closed_set = [[False]*max_row for _ in range(max_col)]      # Exploration status
        g_scores = [[float('inf')]*max_row for _ in range(max_col)] # Movement costs
        parents = [[None]*max_row for _ in range(max_col)]          # Parent coordinates
        open_list = []                                              # Frontier: list of (f, col, row)
        path_found = False

        # Initialize start node
        g_scores[start_col][start_row] = 0                          # g(start) = 0
        # f = g + h = 0 + h_cache[start]
        open_list.append((h_cache[start_col][start_row], start_col, start_row))

        # Main search loop
        while open_list:
            # Extract node with minimum f-value (frontier is kept sorted)
            current_f, current_col, current_row = open_list.pop(0)
            
            # Skip already processed nodes (duplicate entries possible)
            if closed_set[current_col][current_row]:
                continue
            closed_set[current_col][current_row] = True             # Mark as explored

            # Check if current node is target
            if (current_col, current_row) == (end_col, end_row):
                path_found = True
                break

            # Generate 4-directional neighbors
            neighbors = []
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:           # Movement vectors
                n_col = current_col + dx
                n_row = current_row + dy
                if (0 <= n_col < max_col and 
                    0 <= n_row < max_row and 
                    grid[n_col][n_row] == 1):
                    neighbors.append((n_col, n_row))

            # Process neighbors
            current_g = g_scores[current_col][current_row]
            for n_col, n_row in neighbors:
                # Skip explored neighbors
                if closed_set[n_col][n_row]:
                    continue

                # Calculate new movement cost
                tentative_g = current_g + 1

                # Update if better path found
                if tentative_g < g_scores[n_col][n_row]:
                    # Remove existing entries for this node
                    for i in reversed(range(len(open_list))):
                        if open_list[i][1] == n_col and open_list[i][2] == n_row:
                            del open_list[i]
                            break  # Only remove first occurrence

                    # Calculate new f-value
                    new_f = tentative_g + h_cache[n_col][n_row]
                    
                    # Find insertion position to maintain sorted order
                    insert_pos = 0
                    while (insert_pos < len(open_list) and 
                          (new_f > open_list[insert_pos][0] or      # Compare f-values
                          (new_f == open_list[insert_pos][0] and    # Tiebreaker: heuristic
                           h_cache[n_col][n_row] > h_cache[open_list[insert_pos][1]][open_list[insert_pos][2]]))):
                        insert_pos += 1
                    
                    # Insert at calculated position
                    open_list.insert(insert_pos, (new_f, n_col, n_row))

                    # Update state matrices
                    g_scores[n_col][n_row] = tentative_g
                    parents[n_col][n_row] = (current_col, current_row)

        # Path reconstruction
        if path_found:
            current = (end_col, end_row)
            while current != (start_col, start_row):
                path.append(current)
                current_col, current_row = current
                parent = parents[current_col][current_row]
                if parent is None:  # Path discontinuity detected
                    path = []
                    break
                current = parent
            if path:
                path.append((start_col, start_row))
                path.reverse()


    # --------------- VERSION C: 1D Memory Optimization (Large grids >500x500) ---------------
    else:  
        """
        Version C: 1D Compression Optimization
        Features and Advantages:
        - Maximizes memory efficiency for ultra-large grids (>500x500).
        - Optimized for modern CPU cache architectures.

        Key Technologies:
        - Coordinate Linearization: 2D-to-1D mapping via row-major indexing (col*max_row + row).
        - 1D State Arrays: Unified memory blocks for exploration status and node costs.
        - Batch Memory Operations: Vectorized updates leveraging contiguous memory layout.
        """   
        display_message(f"C算法")
        # Convert 2D coordinates to 1D index (row-major order)
        def coord_to_index(col, row):
            """Linearize 2D coordinates for 1D array access"""
            return col * max_row + row                              # Formula: column * total_rows + row

        # Precompute heuristic matrix (same as Version B)
        h_cache = [
            [((end_col - c)**2 + (end_row - r)**2)**0.5 
             for r in range(max_row)] 
            for c in range(max_col)
        ]

        # Initialize 1D state arrays
        closed_set = [False] * (max_col * max_row)                  # Exploration status
        g_scores = [float('inf')] * (max_col * max_row)             # Movement costs
        parents = [None] * (max_col * max_row)                      # Parent indices
        open_list = []  # Frontier: list of (f, col, row)
        path_found = False

        # Initialize start node
        start_index = coord_to_index(start_col, start_row)
        g_scores[start_index] = 0
        open_list.append((h_cache[start_col][start_row], start_col, start_row))

        # Main search loop
        while open_list:
            current_f, current_col, current_row = open_list.pop(0)
            current_index = coord_to_index(current_col, current_row)
            
            # Skip processed nodes
            if closed_set[current_index]:
                continue
            closed_set[current_index] = True                        # Mark as explored

            # Target check
            if (current_col, current_row) == (end_col, end_row):
                path_found = True
                break

            # Generate 4-directional neighbors
            neighbors = []
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                n_col = current_col + dx
                n_row = current_row + dy
                if (0 <= n_col < max_col and 
                    0 <= n_row < max_row and 
                    grid[n_col][n_row] == 1):
                    neighbors.append((n_col, n_row))

            # Process neighbors using 1D indices
            current_g = g_scores[current_index]
            for n_col, n_row in neighbors:
                n_index = coord_to_index(n_col, n_row)
                
                # Skip explored nodes
                if closed_set[n_index]:
                    continue

                # Calculate new cost
                tentative_g = current_g + 1

                # Update if better path found
                if tentative_g < g_scores[n_index]:
                    # Remove existing entries
                    for i in reversed(range(len(open_list))):
                        if open_list[i][1] == n_col and open_list[i][2] == n_row:
                            del open_list[i]
                            break

                    # Calculate and insert sorted by f-value
                    new_f = tentative_g + h_cache[n_col][n_row]
                    insert_pos = 0
                    while (insert_pos < len(open_list)) and (
                        new_f > open_list[insert_pos][0] or 
                        (new_f == open_list[insert_pos][0] and 
                         h_cache[n_col][n_row] > h_cache[open_list[insert_pos][1]][open_list[insert_pos][2]])):
                        insert_pos += 1
                    open_list.insert(insert_pos, (new_f, n_col, n_row))

                    # Update 1D array states
                    g_scores[n_index] = tentative_g
                    parents[n_index] = (current_col, current_row)

        # Path reconstruction using 1D indices
        if path_found:
            current = (end_col, end_row)
            while current != (start_col, start_row):
                path.append(current)
                current_index = coord_to_index(*current)
                parent = parents[current_index]
                if parent is None:                          # Path break detected
                    path = []
                    break
                current = parent
            if path:
                path.append((start_col, start_row))
                path.reverse()


    # ====================== SECTION 4: PATH VALIDATION ======================
    # Validate path endpoints
    if not path or path[0] != start or path[-1] != end:
        display_message("[WARNING] Invalid path endpoints")
        return []

    # Validate path continuity (4-directional movement only)
    for i in range(1, len(path)):
        prev_col, prev_row = path[i-1]
        curr_col, curr_row = path[i]
        # Calculate Manhattan distance between consecutive nodes
        if abs(prev_col - curr_col) + abs(prev_row - curr_row) != 1:
            display_message("[ERROR] Non-adjacent nodes in path")
            return []

    display_message(f"[SUCCESS] Path found with {len(path)} steps")
    return path