import time
import tracemalloc

def do_a_star(grid, start, end, display_message):
    """使用A*算法实现四方向路径规划，采用欧几里得距离作为启发式函数"""
    tracemalloc.start()  # 开启内存跟踪
    start_time = time.perf_counter()  # 记录开始时间
    
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
    
    # === 2. 启发式函数 ===
    def heuristic(col, row):
        return ((end_col - col)**2 + (end_row - row)**2)**0.5
    
    # === 3. 初始化数据结构 ===
    open_list = []
    closed_set = set()
    g_scores = {start: 0}
    parents = {}
    
    initial_h = heuristic(start_col, start_row)
    open_list.append((initial_h + 0, start_col, start_row))
    
    path_found = False
    expanded_nodes = 0
    
    # === 4. 主循环 ===
    while open_list:
        current_node = min(open_list, key=lambda x: x[0])
        current_f, current_col, current_row = current_node
        current_pos = (current_col, current_row)
        open_list.remove(current_node)
        expanded_nodes += 1
        
        if current_pos == end:
            path_found = True
            break
        
        closed_set.add(current_pos)
        
        neighbors = []
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            n_col = current_col + dx
            n_row = current_row + dy
            if 0 <= n_col < max_col and 0 <= n_row < max_row and grid[n_col][n_row] == 1:
                neighbors.append((n_col, n_row))
        
        for n_col, n_row in neighbors:
            n_pos = (n_col, n_row)
            if n_pos in closed_set:
                continue
            
            tentative_g = g_scores[current_pos] + 1
            if tentative_g < g_scores.get(n_pos, float('inf')):
                parents[n_pos] = current_pos
                g_scores[n_pos] = tentative_g
                h = heuristic(n_col, n_row)
                f = tentative_g + h
                
                existing = next((x for x in open_list if x[1:] == n_pos), None)
                if existing:
                    if f < existing[0]:
                        open_list.remove(existing)
                        open_list.append((f, n_col, n_row))
                else:
                    open_list.append((f, n_col, n_row))
    
    # === 5. 路径重构 ===
    if not path_found:
        display_message("[警告] 无有效路径")
        return []
    
    path = []
    current = end
    while current in parents:
        path.append(current)
        current = parents[current]
    path.append(start)
    path.reverse()
    
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
    print(f"\n=== A算法统计 ===")
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
    # # === 6. 统计时间 ===
    # elapsed_time = time.time() - start_time
    # display_message(f"[信息] 算法运行时间：{elapsed_time:.4f}秒，扩展节点数：{expanded_nodes}")
    # # === 5. 时间稳定化处理 ===
    # elapsed_time = time.perf_counter() - start_time
    # elapsed_time_ms = elapsed_time * 1000
    
    # time_str = f"{elapsed_time_ms:.3f} 毫秒" if elapsed_time_ms >= 0.001 else "< 0.001 毫秒"
    
    # display_message(
        # f"[信息] 算法运行时间：{time_str}，"
        # f"扩展节点数：{expanded_nodes}，"
        # f"路径长度：{len(path)},"
        # f"剩余开放节点：{len(open_list)}"
    # )
    # # 获取内存快照
    # snapshot = tracemalloc.take_snapshot()
    # top_stats = snapshot.statistics('lineno')
    
    # # 输出内存消耗Top 10
    # display_message("[内存分析] 关键内存消耗：")
    # for stat in top_stats[:10]:
        # display_message(f"{stat.traceback}: {stat.size/1024:.2f} KB")
    
    # tracemalloc.stop()
    # return path