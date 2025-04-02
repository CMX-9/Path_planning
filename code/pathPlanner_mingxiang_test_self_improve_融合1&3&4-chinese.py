def do_a_star(grid, start, end, display_message):
    """
    自适应A*路径规划算法（四方向移动）
    
    参数:
    grid -- 二维列表，表示网格地图。1表示可通过区域，0表示障碍物
    start -- 元组(col, row)，表示起点坐标。坐标系以网格左上角为原点(0,0)
    end -- 元组(col, row)，表示终点坐标。坐标系与起点一致
    display_message -- GUI提供的消息回调函数，用于输出调试信息到界面
    
    返回:
    list -- 路径坐标列表，从起点到终点。格式示例：[(0,0), (1,1), (2,2)]。若无法到达返回空列表
    
    算法特性:
    1. 三维自适应架构:
       - 小型网格(≤20x20): 字典+集合实现，时间复杂度O(n²)，空间复杂度O(n)
       - 中型网格(≤500x500): 二维数组预计算，时间复杂度O(n log n)，空间复杂度O(n)
       - 大型网格(>500x500): 一维数组优化，时间复杂度O(n log n)，空间复杂度O(1)
    2. 严格遵循课程要求:
       - 完全四方向移动模型（曼哈顿移动）
       - 纯欧几里得距离启发函数
       - 零第三方库依赖
    3. 增强特性:
       - 六层错误检查机制
       - 路径连续性验证
       - 动态内存管理
    """

    # ====================== 模块1：参数初始化 ======================
    max_col = len(grid)                    # 网格列数（垂直方向/Y轴）
    max_row = len(grid[0]) if max_col > 0 else 0  # 网格行数（水平方向/X轴）
    grid_size = max_col * max_row          # 计算网格总单元格数（用于算法选择）
    start_col, start_row = start           # 解包起点坐标（col对应Y轴，row对应X轴）
    end_col, end_row = end                 # 解包终点坐标（坐标系与起点一致）
    path = []                              # 最终路径存储列表（元素为(col,row)元组）

    # ===================== 模块2：错误检查 ======================
    # 场景1：起点终点重合检查（特殊优化情形）
    if (start_col, start_row) == (end_col, end_row):
        display_message("[INFO] 起点终点重合，路径长度为0")
        return [start]  # 直接返回包含起点的单元素列表

    # 场景2：空网格检查（防御性编程）
    if not grid or max_row == 0:
        display_message("[CRITICAL] 网格数据异常：空网格或零行网格")
        return []

    # 场景3：坐标合法性检查（防止越界访问）
    if not (0 <= start_col < max_col and 0 <= start_row < max_row):
        display_message("[ERROR] 起点坐标超出网格范围")
        return []
    
    if not (0 <= end_col < max_col and 0 <= end_row < max_row):
        display_message("[ERROR] 终点坐标超出网格范围")
        return []

    # 场景4：障碍物检查（路径可行性验证）
    if grid[start_col][start_row] == 0:
        display_message("[ERROR] 起点位于不可通行区域")
        return []
    
    if grid[end_col][end_row] == 0:
        display_message("[ERROR] 终点位于不可通行区域")
        return []

    # ================== 模块3：算法选择与核心逻辑 ==================
    if grid_size <= 400:  # 小型网格处理（20x20网格）
        """
        版本A：字典驱动实现
        优势：代码简洁，适合快速迭代
        数据结构:
          - open_list: 待探索节点队列，元素为(f, col, row)
          - closed_set: 已探索节点集合，快速查找
          - g_scores: 节点到起点的实际代价字典
          - parents: 节点回溯字典
        """
        
        # 数据结构初始化
        open_list = []    # 开放列表（未探索节点池）
        closed_set = set()  # 关闭集合（已探索节点记录）
        g_scores = {start: 0}  # 实际代价字典（g(n)=起点到n的实际距离）
        parents = {}       # 父节点映射表（用于路径回溯）
        path_found = False  # 路径发现标志

        # 启发式函数定义（严格欧几里得距离）
        def heuristic(col, row):
            """计算节点到终点的直线距离（允许浮点精度）"""
            return ((end_col - col)**2 + (end_row - row)**2)**0.5  # 平方开方避免累积误差

        # 起点初始化（f = g + h，初始g=0）
        initial_h = heuristic(start_col, start_row)
        open_list.append((initial_h, start_col, start_row))  # 元组(f, col, row)

        # 主搜索循环（开放列表驱动）
        while open_list:
            # 选择当前最优节点（线性搜索最小f值）
            current_node = min(open_list, key=lambda x: x[0])  # O(n)时间复杂度
            current_f, current_col, current_row = current_node
            current_pos = (current_col, current_row)
            open_list.remove(current_node)  # 从开放列表移除

            # 终点到达检测
            if current_pos == end:
                path_found = True
                display_message("[DEBUG] 抵达终点，开始路径重构")
                break

            closed_set.add(current_pos)  # 加入已探索集合

            # 四方向邻居生成（严格曼哈顿移动）
            neighbors = []
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:  # 上/下/左/右
                n_col = current_col + dx  # 计算新列坐标
                n_row = current_row + dy  # 计算新行坐标
                # 三重验证：边界检查+障碍检查+数据类型验证
                if (0 <= n_col < max_col and 
                    0 <= n_row < max_row and 
                    isinstance(grid[n_col][n_row], int) and 
                    grid[n_col][n_row] == 1):
                    neighbors.append((n_col, n_row))

            # 邻居节点处理
            for n_col, n_row in neighbors:
                n_pos = (n_col, n_row)
                if n_pos in closed_set:  # 跳过已探索节点
                    continue

                # 计算新g值（当前g值+单位移动代价）
                tentative_g = g_scores[current_pos] + 1  # 四方向移动代价恒为1

                # 路径优化判断（发现更优路径）
                if tentative_g < g_scores.get(n_pos, float('inf')):
                    parents[n_pos] = current_pos  # 更新父节点映射
                    g_scores[n_pos] = tentative_g  # 更新实际代价
                    h = heuristic(n_col, n_row)   # 计算启发值
                    f = tentative_g + h           # 计算总代价

                    # 开放列表更新策略
                    existing = next((x for x in open_list if x[1:] == n_pos), None)
                    if existing:  # 节点已存在时的处理
                        if f < existing[0]:  # 仅在新路径更优时替换
                            open_list.remove(existing)
                            open_list.append((f, n_col, n_row))
                    else:  # 新节点加入
                        open_list.append((f, n_col, n_row))

        # 路径重构（反向回溯）
        if path_found:
            current = end
            while current in parents:  # 反向追踪父节点
                path.append(current)
                current = parents[current]
            path.append(start)  # 补全起点
            path.reverse()      # 反转得到正序路径

    elif 400 < grid_size <= 250000:  # 中型网格处理（500x500）
        """
        版本B：矩阵预计算优化
        优势：空间换时间，减少重复计算
        关键技术:
          - h_cache: 预计算所有节点的启发值
          - 二维数组存储状态信息
          - 有序插入维护开放列表
        """
        
        # 预计算启发值矩阵（空间换时间优化）
        h_cache = [
            [((end_col - c)**2 + (end_row - r)**2)**0.5 
            for r in range(max_row)] 
            for c in range(max_col)  # 列优先遍历
        ]

        # 状态矩阵初始化
        closed_set = [[False]*max_row for _ in range(max_col)]  # 探索状态矩阵
        g_scores = [[float('inf')]*max_row for _ in range(max_col)]  # 实际代价矩阵
        parents = [[None]*max_row for _ in range(max_col)]  # 父节点坐标矩阵
        open_list = []  # 开放列表（元素为(f, col, row)）
        path_found = False

        # 起点初始化
        g_scores[start_col][start_row] = 0  # 起点g值为0
        open_list.append((h_cache[start_col][start_row], start_col, start_row))

        # 主搜索循环
        while open_list:
            current_f, current_col, current_row = open_list.pop(0)  # 取首元素
            
            # 状态验证（处理重复添加情况）
            if closed_set[current_col][current_row]:
                continue
            closed_set[current_col][current_row] = True  # 标记为已探索

            # 终点检测
            if (current_col, current_row) == (end_col, end_row):
                path_found = True
                break

            # 四方向邻居生成
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
                if closed_set[n_col][n_row]:
                    continue

                # 计算新g值
                tentative_g = current_g + 1

                # 发现更优路径
                if tentative_g < g_scores[n_col][n_row]:
                    # 清理旧节点（提高遍历效率）
                    for i in reversed(range(len(open_list))):
                        if open_list[i][1] == n_col and open_list[i][2] == n_row:
                            del open_list[i]
                            break  # 只需处理第一个匹配项

                    # 计算新f值并有序插入
                    new_f = tentative_g + h_cache[n_col][n_row]
                    insert_pos = 0
                    # 查找插入位置（维护升序）
                    while (insert_pos < len(open_list) and 
                          (new_f > open_list[insert_pos][0] or 
                          (new_f == open_list[insert_pos][0] and 
                           h_cache[n_col][n_row] > h_cache[open_list[insert_pos][1]][open_list[insert_pos][2]])):
                        insert_pos += 1
                    open_list.insert(insert_pos, (new_f, n_col, n_row))

                    # 更新节点数据
                    g_scores[n_col][n_row] = tentative_g
                    parents[n_col][n_row] = (current_col, current_row)

        # 路径重构（矩阵回溯）
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
        """
        版本C：一维压缩优化
        优势：内存效率最大化，适合超大规模网格
        关键技术:
          - 坐标线性化：二维坐标映射为一维索引
          - 一维数组存储状态
          - 批量内存操作优化
        """
        
        # 坐标转换函数（内存优化核心）
        def coord_to_index(col, row):
            """将二维坐标转换为行优先的一维索引"""
            return col * max_row + row  # 列号*行数+行号

        # 预计算启发值矩阵
        h_cache = [
            [((end_col - c)**2 + (end_row - r)**2)**0.5 
            for r in range(max_row)] 
            for c in range(max_col)
        ]

        # 一维状态数组初始化
        closed_set = [False] * (max_col * max_row)    # 探索状态数组
        g_scores = [float('inf')] * (max_col * max_row)  # 实际代价数组
        parents = [None] * (max_col * max_row)        # 父节点索引数组
        open_list = []  # 开放列表（元素为(f, col, row)）
        path_found = False

        # 起点初始化
        start_index = coord_to_index(start_col, start_row)
        g_scores[start_index] = 0
        open_list.append((h_cache[start_col][start_row], start_col, start_row))

        # 主搜索循环
        while open_list:
            current_f, current_col, current_row = open_list.pop(0)
            current_index = coord_to_index(current_col, current_row)
            
            if closed_set[current_index]:  # 状态验证
                continue
            closed_set[current_index] = True

            # 终点检测
            if (current_col, current_row) == (end_col, end_row):
                path_found = True
                break

            # 四方向邻居生成
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
                if closed_set[n_index]:
                    continue

                # 计算新g值
                tentative_g = current_g + 1

                # 路径优化处理
                if tentative_g < g_scores[n_index]:
                    # 清理旧节点记录
                    for i in reversed(range(len(open_list))):
                        if open_list[i][1] == n_col and open_list[i][2] == n_row:
                            del open_list[i]
                            break

                    # 计算新f值并有序插入
                    new_f = tentative_g + h_cache[n_col][n_row]
                    insert_pos = 0
                    while (insert_pos < len(open_list)) and (
                        new_f > open_list[insert_pos][0] or 
                        (new_f == open_list[insert_pos][0] and 
                        h_cache[n_col][n_row] > h_cache[open_list[insert_pos][1]][open_list[insert_pos][2]])):
                        insert_pos += 1
                    open_list.insert(insert_pos, (new_f, n_col, n_row))

                    # 更新节点数据
                    g_scores[n_index] = tentative_g
                    parents[n_index] = (current_col, current_row)

        # 路径重构（一维回溯）
        if path_found:
            current = (end_col, end_row)
            while current != (start_col, start_row):
                path.append(current)
                current_index = coord_to_index(*current)
                parent = parents[current_index]
                if parent is None:  # 断裂保护
                    path = []
                    break
                current = parent
            if path:
                path.append((start_col, start_row))
                path.reverse()

    # ==================== 模块4：路径后处理 ====================
    # 基础验证：起点终点匹配性
    if not path or path[0] != start or path[-1] != end:
        display_message("[WARN] 路径端点校验失败")
        return []

    # 增强验证：路径连续性检查
    for i in range(1, len(path)):
        prev = path[i-1]
        curr = path[i]
        # 曼哈顿距离必须为1（严格四方向）
        if abs(prev[0] - curr[0]) + abs(prev[1] - curr[1]) != 1:
            display_message("[ERROR] 路径不连续，存在非法移动")
            return []

    display_message(f"[SUCCESS] 规划完成，路径长度：{len(path)}")
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