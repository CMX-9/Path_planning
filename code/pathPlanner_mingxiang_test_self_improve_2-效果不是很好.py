import time

def do_a_star(grid, start, end, display_message):
    start_time = time.perf_counter()
    
    # === 1. 错误检查 === 
    if not grid or not grid[0]:
        display_message("[ERROR] 网格为空")
        return []
    
    max_col, max_row = len(grid), len(grid[0])
    start_col, start_row = start
    end_col, end_row = end

    if not (0 <= start_col < max_col and 0 <= start_row < max_row):
        display_message("[ERROR] 起点坐标无效")
        return []
    if not (0 <= end_col < max_col and 0 <= end_row < max_row):
        display_message("[ERROR] 终点坐标无效")
        return []
    if grid[start_col][start_row] == 0:
        display_message("[ERROR] 起点被阻挡")
        return []
    if grid[end_col][end_row] == 0:
        display_message("[ERROR] 终点被阻挡")
        return []

    # === 2. 数据结构优化 ===
    closed_set = [[False]*max_row for _ in range(max_col)]
    g_scores = [[float('inf')]*max_row for _ in range(max_col)]
    parents = [[None]*max_row for _ in range(max_col)]
    
    # 预计算启发值缓存
    h_cache = [[((end_col - c)**2 + (end_row - r)**2)**0.5 
               for r in range(max_row)] for c in range(max_col)]
    
    # 维护排序的开放列表（手动实现优先队列逻辑）
    open_list = []
    g_scores[start_col][start_row] = 0
    initial_f = h_cache[start_col][start_row]
    open_list.append((initial_f, start_col, start_row))
    
    path_found = False
    expanded_nodes = 0

    # === 3. 主循环优化 ===
    while open_list:
        # 通过反向排序+pop(0)实现快速获取最小值
        open_list.sort(reverse=True)
        current_f, current_col, current_row = open_list.pop(0)
        
        if closed_set[current_col][current_row]:
            continue
            
        closed_set[current_col][current_row] = True
        expanded_nodes += 1

        if (current_col, current_row) == (end_col, end_row):
            path_found = True
            break

        # 四方向邻居生成
        neighbors = []
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            n_col, n_row = current_col + dx, current_row + dy
            if 0 <= n_col < max_col and 0 <= n_row < max_row:
                if grid[n_col][n_row] == 1:
                    neighbors.append((n_col, n_row))

        # 邻居处理优化
        current_g = g_scores[current_col][current_row]
        for n_col, n_row in neighbors:
            if closed_set[n_col][n_row]:
                continue
                
            tentative_g = current_g + 1
            if tentative_g < g_scores[n_col][n_row]:
                g_scores[n_col][n_row] = tentative_g
                parents[n_col][n_row] = (current_col, current_row)
                new_f = tentative_g + h_cache[n_col][n_row]
                
                # 插入时保持开放列表有序
                insert_pos = 0
                while insert_pos < len(open_list) and open_list[insert_pos][0] < new_f:
                    insert_pos += 1
                open_list.insert(insert_pos, (new_f, n_col, n_row))

    # === 4. 路径重构 ===
    path = []
    if path_found:
        current = (end_col, end_row)
        while current != start:
            path.append(current)
            current = parents[current[0]][current[1]]
            if current is None:
                path = []
                break
        path.append(start)
        path.reverse()

    # === 5. 时间稳定化处理 ===
    elapsed_time = time.perf_counter() - start_time
    elapsed_time_ms = elapsed_time * 1000
    
    # 通过多次采样平滑时间波动（可选）
    '''
    sample_times = []
    for _ in range(3):  # 采样3次取最小值
        t = time.perf_counter()
        # 执行简单操作（如1+1）测量基础开销
        dummy = 1+1
        base_time = time.perf_counter() - t
        sample_times.append(base_time)
    min_base = min(sample_times)
    elapsed_time_ms = max(0, elapsed_time_ms - min_base*1000)
    '''
    
    time_str = f"{elapsed_time_ms:.3f} 毫秒" if elapsed_time_ms >= 0.001 else "< 0.001 毫秒"
    
    display_message(
        f"[信息] 算法运行时间：{time_str}，"
        f"扩展节点数：{expanded_nodes}，"
        f"路径长度：{len(path)},"
        f"剩余开放节点：{len(open_list)}"
    )
    return path if path and path[0] == start and path[-1] == end else []