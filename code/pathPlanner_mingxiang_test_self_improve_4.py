import time
import tracemalloc
def do_a_star(grid, start, end, display_message):
    tracemalloc.start()  # 开启内存跟踪
    start_time = time.perf_counter()
    max_col, max_row = len(grid), len(grid[0])
    start_col, start_row = start
    end_col, end_row = end

    # === 1. 错误检查 === 
    # 新增：起点终点重合判断
    if (start_col, start_row) == (end_col, end_row):
        display_message("[INFO] 起点终点重合")
        tracemalloc.stop()
        return [start]
    if not grid or not grid[0]:
        display_message("[ERROR] 网格为空")
        return []

    if not (0 <= start_col < max_col and 0 <= start_row < max_row):
        display_message("[ERROR] 起点坐标无效")
        return []
    if not (0 <= end_col < max_col and 0 <= end_row < max_row):
        display_message("[ERROR] 终点坐标无效")
        return []
    if grid[start_col][start_row] == 0:
        display_message("[ERROR] 起点被阻挡")
        tracemalloc.stop()  # 新增此行
        return []
    if grid[end_col][end_row] == 0:
        display_message("[ERROR] 终点被阻挡")
        return []

    # === 预计算启发式缓存（保持不变） ===
    h_cache = [
        [((end_col - c)**2 + (end_row - r)**2)**0.5 
        for r in range(max_row)] 
        for c in range(max_col)
    ]

    # === 一维数组优化 ===
    grid_size = max_col * max_row
    closed_set = [False] * grid_size
    g_scores = [float('inf')] * grid_size
    parents = [None] * grid_size

    def coord_to_index(col, row):
        return col * max_row + row

    # === 初始化开放列表 ===
    open_list = []
    start_index = coord_to_index(start_col, start_row)
    g_scores[start_index] = 0
    initial_f = h_cache[start_col][start_row]
    open_list.append((initial_f, start_col, start_row))

    path_found = False
    expanded_nodes = 0
    DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 预定义方向

    # === 主循环（优化后） ===
    while open_list:
        current_f, current_col, current_row = open_list.pop(0)
        current_index = coord_to_index(current_col, current_row)
        
        if closed_set[current_index]:
            continue
        closed_set[current_index] = True
        expanded_nodes += 1

        if (current_col, current_row) == (end_col, end_row):
            path_found = True
            break

        # 生成邻居
        neighbors = []
        for dx, dy in DIRECTIONS:
            n_col, n_row = current_col + dx, current_row + dy
            if 0 <= n_col < max_col and 0 <= n_row < max_row and grid[n_col][n_row] == 1:
                neighbors.append((n_col, n_row))

        # 处理邻居
        current_g = g_scores[current_index]
        for n_col, n_row in neighbors:
            n_index = coord_to_index(n_col, n_row)
            if closed_set[n_index]:
                continue
                
            tentative_g = current_g + 1
            if tentative_g < g_scores[n_index]:
                # 清理旧节点
                for i in reversed(range(len(open_list))):
                    if open_list[i][1] == n_col and open_list[i][2] == n_row:
                        del open_list[i]
                        break
                
                # 插入新节点到有序位置
                new_f = tentative_g + h_cache[n_col][n_row]
                insert_pos = 0
                while insert_pos < len(open_list) and (
                    new_f > open_list[insert_pos][0] or (
                        new_f == open_list[insert_pos][0] and 
                        h_cache[n_col][n_row] > h_cache[open_list[insert_pos][1]][open_list[insert_pos][2]]
                    )
                ):
                    insert_pos += 1
                open_list.insert(insert_pos, (new_f, n_col, n_row))
                g_scores[n_index] = tentative_g
                parents[n_index] = (current_col, current_row)

    # === 路径重构（优化后） ===
    path = []
    if path_found:
        current = (end_col, end_row)
        while current != (start_col, start_row):
            path.append(current)
            current_col, current_row = current
            current_index = coord_to_index(current_col, current_row)
            parent = parents[current_index]
            if parent is None:
                path = []
                break
            current = parent
        if path:
            path.append((start_col, start_row))
            path.reverse()

    # 验证路径有效性
    if not path or path[0] != start or path[-1] != end:
        return []
    
    # === 统计总内存并输出到终端 ===
    elapsed_time = time.perf_counter() - start_time
    elapsed_time_ms = elapsed_time * 1000
    time_str = f"{elapsed_time_ms:.3f} 毫秒" if elapsed_time_ms >= 0.001 else "< 0.001 毫秒"
    
    # 获取内存快照并计算总和
    snapshot = tracemalloc.take_snapshot()
    top_stats = snapshot.statistics('lineno')
    
    # 计算总内存占用（单位：字节）
    total_memory = sum(stat.size for stat in top_stats)
    total_memory_kb = total_memory / 1024
    total_memory_mb = total_memory_kb / 1024
    
    # 格式化内存信息
    if total_memory_mb >= 1:
        memory_str = f"{total_memory_mb:.2f} MB"
    else:
        memory_str = f"{total_memory_kb:.2f} KB"
    
    # 输出到GUI和终端
    display_message(
        f"[信息] 算法运行时间：{time_str}，"
        f"扩展节点数：{expanded_nodes}，"
        f"路径长度：{len(path)}, "
        f"总内存占用：{memory_str}"
    )
    
    # 终端输出详细信息
    print(f"\n=== C算法统计 ===")
    print(f"运行时间: {time_str}")
    print(f"扩展节点数: {expanded_nodes}")
    print(f"路径长度: {len(path)}")
    print(f"总内存占用: {memory_str}")
    
    print("\n=== 内存详情（Top 10）===")
    for stat in top_stats[:10]:
        traceback = stat.traceback.format()[-1]  # 获取最后一行代码位置
        print(f"代码行: {traceback} | 内存: {stat.size/1024:.2f} KB")
    
    tracemalloc.stop()
    return path