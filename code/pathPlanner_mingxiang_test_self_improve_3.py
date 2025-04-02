import time
import tracemalloc

def do_a_star(grid, start, end, display_message):
    tracemalloc.start()  # 开启内存跟踪
    start_time = time.perf_counter()  # 记录开始时间
    
    max_col = len(grid)
    max_row = len(grid[0]) if max_col > 0 else 0
    start_col, start_row = start
    end_col, end_row = end

    # === 1. 错误检查 ===
    # 新增：起点终点重合判断
    if (start_col, start_row) == (end_col, end_row):
        display_message("[INFO] 起点终点重合")
        tracemalloc.stop()
        return [start]
    if not grid or max_row == 0:
        display_message("[ERROR] 网格为空")
        tracemalloc.stop()
        return []
    if not (0 <= start_col < max_col and 0 <= start_row < max_row):
        display_message("[ERROR] 起点坐标无效")
        tracemalloc.stop()
        return []
    if not (0 <= end_col < max_col and 0 <= end_row < max_row):
        display_message("[ERROR] 终点坐标无效")
        tracemalloc.stop()
        return []
    if grid[start_col][start_row] == 0:
        display_message("[ERROR] 起点被阻挡")
        tracemalloc.stop()
        return []
    if grid[end_col][end_row] == 0:
        display_message("[ERROR] 终点被阻挡")
        tracemalloc.stop()
        return []

    # === 2. 算法核心数据结构 ===
    def heuristic(c, r):
        return ((end_col - c)**2 + (end_row - r)**2)**0.5  # 实时计算启发式值

    closed_set = [[False for _ in range(max_row)] for _ in range(max_col)]
    g_scores = [[float('inf') for _ in range(max_row)] for _ in range(max_col)]
    parents = [[None for _ in range(max_row)] for _ in range(max_col)]
    open_list = []
    DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 四方向移动

    # 初始化起点
    g_scores[start_col][start_row] = 0
    initial_f = heuristic(start_col, start_row)
    open_list.append((initial_f, start_col, start_row))

    path_found = False
    expanded_nodes = 0  # 扩展节点计数器

    # === 3. 主循环（优化开放列表管理） ===
    while open_list:
        # 取出当前最优节点（列表已保持有序）
        current_f, current_col, current_row = open_list.pop(0)
        
        if closed_set[current_col][current_row]:
            continue
        closed_set[current_col][current_row] = True
        expanded_nodes += 1

        # 终点检测
        if (current_col, current_row) == (end_col, end_row):
            path_found = True
            break

        # 生成有效邻居
        neighbors = []
        for dx, dy in DIRECTIONS:
            n_col = current_col + dx
            n_row = current_row + dy
            if 0 <= n_col < max_col and 0 <= n_row < max_row and grid[n_col][n_row] == 1:
                neighbors.append((n_col, n_row))

        # 处理邻居节点
        current_g = g_scores[current_col][current_row]
        for n_col, n_row in neighbors:
            if closed_set[n_col][n_row]:
                continue
                
            tentative_g = current_g + 1
            if tentative_g < g_scores[n_col][n_row]:
                # 清理旧节点（逆向遍历提高效率）
                for i in reversed(range(len(open_list))):
                    if open_list[i][1] == n_col and open_list[i][2] == n_row:
                        del open_list[i]
                        break  # 找到即退出循环
                
                # 更新节点信息
                g_scores[n_col][n_row] = tentative_g
                parents[n_col][n_row] = (current_col, current_row)
                new_f = tentative_g + heuristic(n_col, n_row)
                
                # 有序插入开放列表（代替全排序）
                insert_pos = 0
                while insert_pos < len(open_list) and (
                    new_f > open_list[insert_pos][0] or (
                        new_f == open_list[insert_pos][0] and 
                        heuristic(n_col, n_row) > heuristic(
                            open_list[insert_pos][1], 
                            open_list[insert_pos][2]
                        )
                    )
                ):
                    insert_pos += 1
                open_list.insert(insert_pos, (new_f, n_col, n_row))

    # === 4. 路径重构与验证 ===
    path = []
    if path_found:
        current = (end_col, end_row)
        while current != (start_col, start_row):
            path.append(current)
            current_col, current_row = current
            parent = parents[current_col][current_row]  # 直接二维访问
            
            if parent is None:  # 路径断裂保护
                path = []
                break
            current = parent
        
        if path:  # 有效路径处理
            path.append((start_col, start_row))
            path.reverse()

    # === 5. 性能统计 ===
    elapsed_time = time.perf_counter() - start_time
    elapsed_time_ms = elapsed_time * 1000
    time_str = f"{elapsed_time_ms:.3f} 毫秒" if elapsed_time_ms >= 0.001 else "< 0.001 毫秒"
    
    # 内存统计（确保仅统计算法部分）
    snapshot = tracemalloc.take_snapshot()
    top_stats = snapshot.statistics('lineno')
    total_memory = sum(stat.size for stat in top_stats)
    total_memory_kb = total_memory / 1024
    memory_str = f"{total_memory_kb:.2f} KB" if total_memory_kb < 1024 else f"{total_memory_kb/1024:.2f} MB"
    
    # 输出结果
    display_message(
        f"[信息] 运行时间: {time_str}, "
        f"扩展节点: {expanded_nodes}, "
        f"路径长度: {len(path)}, "
        f"内存占用: {memory_str}"
    )
    
    print("\n=== B算法性能统计 ===")
    print(f"运行时间: {time_str}")
    print(f"扩展节点数: {expanded_nodes}")
    print(f"路径长度: {len(path)}")
    print(f"总内存占用: {memory_str}")
    
    print("\n=== 内存分配详情（Top 10）===")
    for stat in top_stats[:10]:
        trace_line = stat.traceback.format()[-1].strip()
        print(f"{trace_line}: {stat.size/1024:.2f} KB")
    
    tracemalloc.stop()
    
    return path if path and path[0] == start and path[-1] == end else []