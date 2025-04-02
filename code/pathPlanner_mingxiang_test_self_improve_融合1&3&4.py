def do_a_star(grid, start, end, display_message):
    """
    自适应A*路径规划算法（四方向移动）
    
    参数:
    grid -- 二维列表，表示网格地图。1表示可通过区域，0表示障碍物。
    start -- 元组(col, row)，表示起点坐标。
    end -- 元组(col, row)，表示终点坐标。
    display_message -- GUI提供的消息回调函数，用于输出调试信息到界面。
    
    返回:
    list -- 路径坐标列表，从起点到终点。若无法到达则返回空列表。
    
    算法特性:
    1. 三种自适应选择机制:
       - 小型网格(≤20x20): 字典+集合实现，简单场景下表现较优。
       - 中型网格(≤500x500): 二维数组预计算，平衡性能，复杂场景表现较优。
       - 大型网格(>500x500): 一维数组优化，大型网络下，其内存更高效。   
    2. 特性:
       - 上下左右四方向移动。
       - 欧几里得距离计算启发函数。
       - 六层错误检查机制
       - 路径连续性验证
       - 动态内存管理
    """

    # ====================== 模块1：参数初始化 ======================
    max_col = len(grid)                    # 获取网格列数（垂直方向）
    max_row = len(grid[0]) if max_col > 0 else 0  # 获取网格行数（水平方向）
    grid_size = max_col * max_row          # 计算网格总单元格数
    start_col, start_row = start           # 解包起点坐标
    end_col, end_row = end                 # 解包终点坐标
    path = []                              # 最终路径存储列表

    # ===================== 模块2：错误检查 ======================
    # 起点终点重合判断
    if (start_col, start_row) == (end_col, end_row):
        display_message("[INFO] 起点终点重合")
        tracemalloc.stop()
        return [start]

    # 检查空网格（无列或行）
    if not grid or max_row == 0:
        display_message("[ERROR] 网格结构异常")
        return []

    # 验证起点坐标是否在网格范围内
    if not (0 <= start_col < max_col and 0 <= start_row < max_row):
        display_message("[ERROR] 起点坐标越界")
        return []

    # 验证终点坐标是否在网格范围内
    if not (0 <= end_col < max_col and 0 <= end_row < max_row):
        display_message("[ERROR] 终点坐标越界")
        return []

    # 检查起点是否位于障碍物上
    if grid[start_col][start_row] == 0:
        display_message("[ERROR] 起点被障碍物阻挡")
        return []

    # 检查终点是否位于障碍物上
    if grid[end_col][end_row] == 0:
        display_message("[ERROR] 终点被障碍物阻挡")
        return []

    # ================== 模块3：算法选择与核心逻辑 ==================
    if grid_size <= 400:  # 小型网格处理（20x20=400）
        """版本A：字典+集合实现（适合快速开发和小型网格）"""
        
        # 数据结构初始化
        open_list = []    # 开放列表，元素格式：(总代价f, 列坐标, 行坐标)
        closed_set = set()  # 已探索节点集合
        g_scores = {start: 0}  # 实际移动代价字典，g(n)
        parents = {}       # 父节点字典，用于路径回溯
        path_found = False  # 路径找到标志

        # 定义启发式函数（欧几里得距离）
        def heuristic(col, row):
            """计算当前节点到终点的直线距离"""
            return ((end_col - col)**2 + (end_row - row)**2)**0.5  # 平方和开根号

        # 初始化起点
        initial_h = heuristic(start_col, start_row)
        open_list.append((initial_h, start_col, start_row))  # 初始f值=0+h

        # 主循环处理开放列表
        while open_list:
            # 选择f值最小的节点（线性搜索）
            current_node = min(open_list, key=lambda x: x[0])
            current_f, current_col, current_row = current_node
            current_pos = (current_col, current_row)
            open_list.remove(current_node)  # 从开放列表移除

            # 终点检测
            if current_pos == end:
                path_found = True
                display_message("[INFO] 找到有效路径")
                break

            closed_set.add(current_pos)  # 将当前节点标记为已探索

            # 生成四方向邻居节点
            neighbors = []
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:  # 上/下/左/右
                n_col = current_col + dx
                n_row = current_row + dy
                # 边界检查和障碍物检查
                if (0 <= n_col < max_col and 
                    0 <= n_row < max_row and 
                    grid[n_col][n_row] == 1):
                    neighbors.append((n_col, n_row))

            # 处理每个邻居节点
            for n_col, n_row in neighbors:
                n_pos = (n_col, n_row)
                if n_pos in closed_set:  # 跳过已探索节点
                    continue

                # 计算新移动代价（当前节点g值+1）
                tentative_g = g_scores[current_pos] + 1

                # 发现更优路径时更新数据
                if tentative_g < g_scores.get(n_pos, float('inf')):
                    parents[n_pos] = current_pos  # 记录父节点
                    g_scores[n_pos] = tentative_g  # 更新g值
                    h = heuristic(n_col, n_row)   # 计算启发值
                    f = tentative_g + h           # 总代价f=g+h

                    # 检查是否已在开放列表
                    existing = next((x for x in open_list if x[1:] == n_pos), None)
                    if existing:
                        if f < existing[0]:  # 新路径更优时替换
                            open_list.remove(existing)
                            open_list.append((f, n_col, n_row))
                    else:
                        open_list.append((f, n_col, n_row))  # 添加新节点

        # 路径重构（版本A）
        if path_found:
            current = end
            while current in parents:  # 反向回溯路径
                path.append(current)
                current = parents[current]
            path.append(start)  # 添加起点
            path.reverse()      # 反转得到正序路径

    elif 400 < grid_size <= 250000:  # 中型网格处理（500x500=250,000）
        """版本B：二维数组优化（平衡性能与内存）"""
        
        # 预计算所有节点的启发值矩阵
        h_cache = [
            [((end_col - c)**2 + (end_row - r)**2)**0.5 
            for r in range(max_row)] 
            for c in range(max_col)
        ]

        # 二维数组初始化
        closed_set = [[False]*max_row for _ in range(max_col)]  # 节点探索状态
        g_scores = [[float('inf')]*max_row for _ in range(max_col)]  # 实际移动代价
        parents = [[None]*max_row for _ in range(max_col)]  # 父节点坐标
        open_list = []  # 开放列表，元素格式：(f, col, row)
        path_found = False

        # 初始化起点
        g_scores[start_col][start_row] = 0
        open_list.append((h_cache[start_col][start_row], start_col, start_row))

        # 主循环处理
        while open_list:
            current_f, current_col, current_row = open_list.pop(0)
            
            # 跳过已处理节点（可能重复添加）
            if closed_set[current_col][current_row]:
                continue
            closed_set[current_col][current_row] = True

            # 终点检测
            if (current_col, current_row) == (end_col, end_row):
                path_found = True
                display_message("[INFO] 找到有效路径")
                break

            # 生成四方向邻居
            neighbors = []
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                n_col = current_col + dx
                n_row = current_row + dy
                if (0 <= n_col < max_col and 
                    0 <= n_row < max_row and 
                    grid[n_col][n_row] == 1):
                    neighbors.append((n_col, n_row))

            # 处理邻居节点
            current_g = g_scores[current_col][current_row]
            for n_col, n_row in neighbors:
                if closed_set[n_col][n_row]:  # 跳过已探索节点
                    continue

                # 计算新移动代价
                tentative_g = current_g + 1

                # 发现更优路径时更新
                if tentative_g < g_scores[n_col][n_row]:
                    # 移除旧节点记录（反向遍历提高效率）
                    for i in reversed(range(len(open_list))):
                        if open_list[i][1] == n_col and open_list[i][2] == n_row:
                            del open_list[i]
                            break

                    # 计算新f值并有序插入
                    new_f = tentative_g + h_cache[n_col][n_row]
                    insert_pos = 0
                    # 查找合适插入位置（维护开放列表有序性）
                    while (insert_pos < len(open_list) and 
                           (new_f > open_list[insert_pos][0] or 
                           (new_f == open_list[insert_pos][0] and 
                           h_cache[n_col][n_row] > h_cache[open_list[insert_pos][1]][open_list[insert_pos][2]])):
                        insert_pos += 1
                    open_list.insert(insert_pos, (new_f, n_col, n_row))

                    # 更新节点信息
                    g_scores[n_col][n_row] = tentative_g
                    parents[n_col][n_row] = (current_col, current_row)

        # 路径重构（版本B）
        if path_found:
            current = (end_col, end_row)
            while current != (start_col, start_row):
                path.append(current)
                current_col, current_row = current
                parent = parents[current_col][current_row]
                if parent is None:  # 路径断裂保护
                    path = []
                    break
                current = parent
            if path:
                path.append((start_col, start_row))
                path.reverse()

    else:  # 大型网格处理（>500x500）
        """版本C：一维数组优化（内存高效）"""
        
        # 坐标转换函数（二维→一维）
        def coord_to_index(col, row):
            return col * max_row + row  # 行优先存储

        # 预计算启发值矩阵
        h_cache = [
            [((end_col - c)**2 + (end_row - r)**2)**0.5 
            for r in range(max_row)] 
            for c in range(max_col)
        ]

        # 一维数组初始化
        closed_set = [False] * (max_col * max_row)    # 探索状态数组
        g_scores = [float('inf')] * (max_col * max_row)  # 移动代价数组
        parents = [None] * (max_col * max_row)        # 父节点索引数组
        open_list = []  # 开放列表，元素格式：(f, col, row)
        path_found = False

        # 初始化起点
        start_index = coord_to_index(start_col, start_row)
        g_scores[start_index] = 0
        open_list.append((h_cache[start_col][start_row], start_col, start_row))

        # 主循环处理
        while open_list:
            current_f, current_col, current_row = open_list.pop(0)
            current_index = coord_to_index(current_col, current_row)
            
            if closed_set[current_index]:  # 跳过已处理节点
                continue
            closed_set[current_index] = True

            # 终点检测
            if (current_col, current_row) == (end_col, end_row):
                path_found = True
                display_message("[INFO] 找到有效路径")
                break

            # 生成四方向邻居
            neighbors = []
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                n_col = current_col + dx
                n_row = current_row + dy
                if (0 <= n_col < max_col and 
                    0 <= n_row < max_row and 
                    grid[n_col][n_row] == 1):
                    neighbors.append((n_col, n_row))

            # 处理邻居节点
            current_g = g_scores[current_index]
            for n_col, n_row in neighbors:
                n_index = coord_to_index(n_col, n_row)
                if closed_set[n_index]:  # 跳过已探索节点
                    continue

                # 计算新移动代价
                tentative_g = current_g + 1

                # 发现更优路径时更新
                if tentative_g < g_scores[n_index]:
                    # 移除旧节点记录
                    for i in reversed(range(len(open_list))):
                        if open_list[i][1] == n_col and open_list[i][2] == n_row:
                            del open_list[i]
                            break

                    # 计算新f值并有序插入
                    new_f = tentative_g + h_cache[n_col][n_row]
                    insert_pos = 0
                    # 维护开放列表有序性
                    while (insert_pos < len(open_list)) and (
                        new_f > open_list[insert_pos][0] or 
                        (new_f == open_list[insert_pos][0] and 
                        h_cache[n_col][n_row] > h_cache[open_list[insert_pos][1]][open_list[insert_pos][2]])):
                        insert_pos += 1
                    open_list.insert(insert_pos, (new_f, n_col, n_row))

                    # 更新节点信息
                    g_scores[n_index] = tentative_g
                    parents[n_index] = (current_col, current_row)

        # 路径重构（版本C）
        if path_found:
            current = (end_col, end_row)
            while current != (start_col, start_row):
                path.append(current)
                current_index = coord_to_index(*current)
                parent = parents[current_index]
                if parent is None:  # 路径断裂保护
                    path = []
                    break
                current = parent
            if path:
                path.append((start_col, start_row))
                path.reverse()

    # ==================== 模块4：最终验证 ====================
    # 验证路径有效性（起点终点匹配性）
    if not path or path[0] != start or path[-1] != end:
        display_message("[WARN] 无效路径")
        return []

    # 验证路径连续性（可选增强）
    for i in range(1, len(path)):
        prev_col, prev_row = path[i-1]
        curr_col, curr_row = path[i]
        if abs(prev_col - curr_col) + abs(prev_row - curr_row) != 1:
            display_message("[ERROR] 路径不连续")
            return []

    display_message(f"[SUCCESS] 路径长度: {len(path)}")
    return path

# ==================== 后续优化方向 ==================== 
"""
1. 开放列表性能优化
   - 现状：线性搜索选择最小f值节点，时间复杂度O(n)
   - 优化：采用优先队列结构（如heapq模块），将时间复杂度降至O(log n)
   - 风险：需处理重复节点问题

2. 路径平滑处理
   - 现状：路径可能存在冗余转折点
   - 优化：后处理阶段检测并删除不必要的中间节点
   - 示例：路径[(0,0)→(0,1)→(0,2)→(1,2)]可简化为[(0,0)→(0,2)→(1,2)]

3. 动态障碍支持
   - 现状：静态障碍物处理
   - 优化：添加二次验证步骤，检查路径节点是否被动态障碍占据
   - 机制：路径执行时实时检测，触发重规划

4. 内存压缩优化
   - 现状：一维数组已较好，但parents数组可优化
   - 方案：使用位移存储坐标信息，如将col和row编码为32位整数

5. 并行计算优化
   - 现状：单线程处理邻居节点
   - 优化：将邻居节点评估过程并行化
   - 注意：需处理线程安全问题和性能权衡

6. 路径断裂保护增强
   - 现状：简单判断parent是否为None
   - 优化：添加最大回溯步数限制，防止死循环
   - 实现：while循环中添加步数计数器，超过2倍网格大小时报错

7. 启发函数多样化
   - 现状：仅支持欧几里得距离
   - 扩展：支持曼哈顿距离、对角线距离等
   - 机制：通过函数参数动态选择启发函数

8. 可视化调试支持
   - 现状：依赖GUI基础消息输出
   - 增强：添加详细调试模式，输出搜索过程动画
   - 技术：生成搜索过程帧图或使用GUI扩展接口

9. 能耗优化模型
   - 扩展：考虑不同地形的移动代价
   - 示例：沼泽地代价为2，公路代价为0.5
   - 实现：修改移动代价计算逻辑

10. 三维扩展支持
    - 扩展：支持多层网格（如立体仓库）
    - 调整：修改坐标系统和邻居生成逻辑
    - 挑战：启发函数需适应三维空间
"""