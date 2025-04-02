# Adaptive A* Path Planning Algorithm(4-directional movement)
In developing the Adaptive A* path planning algorithm, I optimized search efficiency across different grid sizes by implementing three adaptive selection mechanisms using dictionaries, 2D arrays, 
and 1D memory-optimized structures. The algorithm supports efficient four-directional movement, leveraging the Euclidean heuristic and a six-layer error-checking mechanism to enhance path continuity 
and robustness. 
Through this project, I deepened my understanding of A* optimization strategies, strengthened my application of data structures in performance and memory management, and improved my ability to design intelligent navigation in complex environments.
    
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