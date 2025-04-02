import sys
import math
from PyQt5 import QtGui
import importlib
import pathPlanner
from PyQt5.QtWidgets import (
    QApplication,
    QMainWindow,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QWidget,
    QPushButton,
    QLineEdit,
    QScrollArea,
)
from PyQt5.QtGui import (
    QPen,
    QFont,
    QIntValidator,
    QPainter,
    QResizeEvent,
    QColor,
)
from PyQt5.QtCore import Qt, QPoint, QTimer

class MainWindow(QMainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.layout = QVBoxLayout()
        self.title = "AMR Coursework 2 - Path Planning"
        self.top = 200
        self.left = 500
        self.width = 600
        self.height = 500
        self.obstacle_mode = False
        self.start_mode = False
        self.end_mode = False
        self.start_set = False
        self.end_set = False
        self.grid_dimensions = [20, 10]
        self.checked_path = []
        self.algorithm_generator = None
        self.current_open = set()
        self.current_closed = set()
        self.current_neighbors = set()

        self.init_window()
        self.container_widget = QWidget()
        self.container_widget.setLayout(self.layout)
        self.setCentralWidget(self.container_widget)

    def init_window(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)
        self.draw_console(self.layout)
        self.canvas = CanvasWidget(self)
        self.layout.addWidget(self.canvas)
        self.draw_control_panel(self.layout)

    def draw_console(self, parentLayout):
        console_layout = QHBoxLayout()
        self.message_display = ScrollableLabel(self)
        self.message_display.setText("")
        self.message_display.setMaximumHeight(80)

        self.clear_button = QPushButton("CLEAR")
        self.clear_button.setStyleSheet("QPushButton { background-color: white }")
        self.clear_button.clicked.connect(self.on_click_clear)

        self.indicator = QLabel("")
        self.indicator.setStyleSheet("QLabel { background-color : grey; }")
        self.indicator.setMinimumWidth(40)
        self.indicator.setMaximumHeight(40)

        console_layout.addWidget(self.message_display)
        console_layout.addWidget(self.clear_button)
        console_layout.addWidget(self.indicator)
        parentLayout.addLayout(console_layout)

    def draw_control_panel(self, parentLayout):
        control_panel_layout = QHBoxLayout()
        self.width_input = LabelledIntField("Width", 2, self.grid_dimensions[0])
        self.height_input = LabelledIntField("Height", 2, self.grid_dimensions[1])
        self.width_input.show()

        self.step_button = QPushButton("Step")
        self.step_button.setStyleSheet("QPushButton { background-color: white }")
        self.step_button.clicked.connect(self.on_click_step)

        self.reset_button = QPushButton("Reset")
        self.reset_button.setStyleSheet("QPushButton { background-color: white }")
        self.reset_button.setMaximumHeight(self.width_input.height())
        self.reset_button.clicked.connect(self.on_click_reset)

        self.obstacle_button = QPushButton("Add\nObstacles")
        self.obstacle_button.setStyleSheet("QPushButton { background-color: white }")
        self.obstacle_button.setMaximumHeight(self.width_input.height())
        self.obstacle_button.clicked.connect(self.on_click_obstacle)

        self.obstacle_undo_button = QPushButton("Undo\nObstacle")
        self.obstacle_undo_button.setStyleSheet("QPushButton { background-color: white }")
        self.obstacle_undo_button.setMaximumHeight(self.width_input.height())
        self.obstacle_undo_button.clicked.connect(self.on_click_obstacle_undo)

        self.start_button = QPushButton("Add\nStart")
        self.start_button.setStyleSheet("QPushButton { background-color: white }")
        self.start_button.setMaximumHeight(self.width_input.height())
        self.start_button.clicked.connect(self.on_click_start)

        self.end_button = QPushButton("Add\nEnd")
        self.end_button.setStyleSheet("QPushButton { background-color: white }")
        self.end_button.setMaximumHeight(self.width_input.height())
        self.end_button.clicked.connect(self.on_click_end)

        self.run_button = QPushButton("Run")
        self.run_button.setStyleSheet("QPushButton { background-color: white }")
        self.run_button.setMaximumHeight(self.width_input.height())
        self.run_button.clicked.connect(self.on_click_run)

        control_panel_layout.addWidget(self.width_input)
        control_panel_layout.addWidget(self.height_input)
        control_panel_layout.addWidget(self.reset_button)
        control_panel_layout.addWidget(self.obstacle_button)
        control_panel_layout.addWidget(self.obstacle_undo_button)
        control_panel_layout.addWidget(self.start_button)
        control_panel_layout.addWidget(self.end_button)
        control_panel_layout.addWidget(self.step_button)
        control_panel_layout.addStretch(1)
        control_panel_layout.addWidget(self.run_button)

        control_panel_layout.setSpacing(2)
        parentLayout.addLayout(control_panel_layout)

    def on_click_step(self):
        """单步执行按钮点击事件"""
        if not self.start_set or not self.end_set:
            self.display_message("需要先设置起点和终点", "WARN")
            return
        
        try:
            if self.algorithm_generator is None:
                self.algorithm_generator = self.a_star_stepwise(
                    self.create_grid(),
                    self.canvas.start,
                    self.canvas.end,
                    self.display_message,
                )
            
            result = next(self.algorithm_generator, None)
            
            if result is None:
                self.display_message("算法已完成", "INFO")
                self.step_button.setEnabled(False)
                return
                
            self.current_open, self.current_closed, self.current_neighbors, self.current_f_values = result
            self.canvas.open_nodes = self.current_open
            self.canvas.closed_nodes = self.current_closed
            self.canvas.current_neighbors = self.current_neighbors
            self.canvas.f_values = self.current_f_values  # 传递f值到画布
            self.canvas.update()
            
        except Exception as e:
            self.display_message(f"执行出错: {str(e)}", "ERROR")
            self.algorithm_generator = None

    def a_star_stepwise(self, grid, start, end, display_message):
        """分步执行A*算法"""
        # === 错误检查 ===
        # 检查网格是否为空
        if not grid or not grid[0]:
            display_message("[ERROR] 网格为空")
            yield None
            return

        # 解包坐标并验证合法性
        start_col, start_row = start
        end_col, end_row = end
        max_col, max_row = len(grid), len(grid[0])

        # 检查起点/终点是否在网格内
        if not (0 <= start_col < max_col and 0 <= start_row < max_row):
            display_message("[ERROR] 起点坐标无效")
            yield None
            return
        if not (0 <= end_col < max_col and 0 <= end_row < max_row):
            display_message("[ERROR] 终点坐标无效")
            yield None
            return
        
        # 检查起点/终点是否可行走
        if grid[start_col][start_row] == 0:
            display_message("[ERROR] 起点被阻挡")
            yield None
            return
        if grid[end_col][end_row] == 0:
            display_message("[ERROR] 终点被阻挡")
            yield None
            return

        # === 初始化数据结构 ===
        open_list = [(self.heuristic(start, end), start_col, start_row)]
        f_values = {start: self.heuristic(start, end)}  # 新增：记录所有节点的f值
        closed_set = set()
        g_scores = {start: 0}
        parents = {}
        
        while open_list:
            # 获取当前节点信息
            current_node = min(open_list, key=lambda x: x[0])
            current_f, current_col, current_row = current_node
            current_pos = (current_col, current_row)
            open_list.remove(current_node)
            
            # 提前终止：找到终点
            if current_pos == end:
                path = self.reconstruct_path(parents, end)
                self.canvas.path = path
                self.canvas.update()
                display_message(f"[信息] 找到路径，最终路径节点数：{len(path)}，"
                f"总共探索节点数目：{len(closed_set) + 1},"
                f"剩余待探索节点：{len(open_list)}"
                )
                yield None
                return

            closed_set.add(current_pos)
            
            # 生成四方向邻居
            neighbors = []
            for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                n_col = current_col + dx
                n_row = current_row + dy
                if 0 <= n_col < max_col and 0 <= n_row < max_row:
                    if grid[n_col][n_row] == 1:
                        neighbors.append((n_col, n_row))
            
            # 第一次yield：显示待处理的邻居
            current_open = {(x[1], x[2]) for x in open_list}
            # 返回新增的f_values
            yield (current_open, closed_set.copy(), set(neighbors), f_values.copy())
            
            # 处理邻居节点
            for n_col, n_row in neighbors:
                n_pos = (n_col, n_row)
                if n_pos in closed_set:
                    continue
                    
                tentative_g = g_scores[current_pos] + 1

                if tentative_g < g_scores.get(n_pos, float('inf')):
                    parents[n_pos] = current_pos
                    g_scores[n_pos] = tentative_g
                    f = tentative_g + self.heuristic(n_pos, end)
                    f_values[n_pos] = f  # 记录f值
                    
                    # 更新开放列表
                    existing = next((x for x in open_list if x[1:] == n_pos), None)
                    if existing:
                        if f < existing[0]:
                            open_list.remove(existing)
                            open_list.append((f, n_col, n_row))
                    else:
                        open_list.append((f, n_col, n_row))
            
            # 第二次yield：更新后的状态
            yield (current_open, closed_set.copy(), set(), f_values.copy())
            # # 显示当前节点的 g 值和 h 值
            # display_message(
            #     f"[DEBUG] 当前黄色关闭列表节点: {current_pos}, "
            #     f"g值: {g_scores[current_pos]}, "
            #     f"h值: {self.heuristic(current_pos, end)}"
            # )
            # for neighbor in neighbors:
            #     n_pos = (neighbor[0], neighbor[1])
            #     display_message(
            #             f"[DEBUG] 邻居节点: {n_pos}, "
            #             f"g值: {g_scores[current_pos]}, "
            #             f"h值: {f_values[n_pos]-g_scores[current_pos]}"
            #         )


        # 循环结束未找到路径
        # === 未找到路径处理 ===
        display_message(
            f"[警告] 无有效路径，"
            f"总共探索节点数：{len(closed_set)}"
        )
        yield None

    def heuristic(self, pos, end):
        return ((end[0]-pos[0])**2 + (end[1]-pos[1])**2)**0.5  # 欧几里得距离
        # dx = abs(end[0]-pos[0])
        # dy = abs(end[1]-pos[1])
        # return dx + dy  # 曼哈顿距离

    def reconstruct_path(self, parents, end):
        path = []
        current = end
        while current in parents:
            path.append(current)
            current = parents[current]
        path.append(current)  # 添加起点
        path.reverse()
        return path

    def resizeEvent(self, e: QResizeEvent):
        self.canvas.draw_grid(self.grid_dimensions[0], self.grid_dimensions[1])

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_R:
            self.reset()

    def reset(self):
        self.grid_dimensions = (
            self.width_input.get_value(),
            self.height_input.get_value(),
        )
        self.display_message(
            "Resetting grid to {} x {}".format(
                self.grid_dimensions[0], self.grid_dimensions[1]
            ),
            "INFO",
        )
        self.indicator.setStyleSheet("QLabel { background-color : grey; }")
        self.canvas.draw_grid(self.grid_dimensions[0], self.grid_dimensions[1])
        self.canvas.obstacles = []
        self.canvas.path = None
        self.start_set = False
        self.canvas.start = None
        self.start_set = False
        self.canvas.end = None

    def on_click_clear(self):
        self.message_display.setText("")

    def on_click_reset(self):
        self.reset()

    def on_click_obstacle(self):
        self.obstacle_mode = not self.obstacle_mode

        self.start_mode = False
        self.start_button.setStyleSheet("QPushButton { background-color: white }")
        self.end_mode = False
        self.end_button.setStyleSheet("QPushButton { background-color: white }")

        self.obstacle_button.setStyleSheet(
            "QPushButton { background-color: %s }"
            % ("grey" if self.obstacle_mode else "white")
        )

    def on_click_obstacle_undo(self):
        if len(self.canvas.obstacles) > 0:
            self.canvas.path = None
            self.indicator.setStyleSheet("QLabel { background-color : grey; }")
            self.canvas.obstacles.pop()
            self.canvas.update()

    def on_click_start(self):
        self.start_mode = not self.start_mode

        self.obstacle_mode = False
        self.obstacle_button.setStyleSheet("QPushButton { background-color: white }")
        self.end_mode = False
        self.end_button.setStyleSheet("QPushButton { background-color: white }")

        self.start_button.setStyleSheet(
            "QPushButton { background-color: %s }"
            % ("green" if self.start_mode else "white")
        )

    def on_click_end(self):
        self.end_mode = not self.end_mode

        self.start_mode = False
        self.start_button.setStyleSheet("QPushButton { background-color: white }")
        self.obstacle_mode = False
        self.obstacle_button.setStyleSheet("QPushButton { background-color: white }")

        self.end_button.setStyleSheet(
            "QPushButton { background-color: %s }"
            % ("red" if self.end_mode else "white")
        )

    def on_click_run(self):
        if not self.start_set or not self.end_set:
            self.display_message("Start and End must be set", "WARN")
            self.indicator.setStyleSheet("QLabel { background-color : red; }")
            return

        self.display_message("Running algorithm", "INFO")
        try:
            importlib.reload(pathPlanner)
            unchecked_path = pathPlanner.do_a_star(
                self.create_grid(),
                self.canvas.start,
                self.canvas.end,
                self.display_message,
            )
        except Exception as e:
            self.display_message('Python: "{}"'.format(str(e)), "ERROR")
            self.indicator.setStyleSheet("QLabel { background-color : red; }")
            return

        if len(unchecked_path) == 0:
            self.display_message("No path returned", "ERROR")
            self.indicator.setStyleSheet("QLabel { background-color : red; }")
            return
        else:
            for cell in unchecked_path:
                if not self.check_inside_grid(cell):
                    self.indicator.setStyleSheet("QLabel { background-color : red; }")
                    return
                if self.check_obstacle_intersection(cell):
                    self.indicator.setStyleSheet("QLabel { background-color : red; }")
                    break
                self.indicator.setStyleSheet("QLabel { background-color : green; }")

        if self.canvas.start in unchecked_path:
            unchecked_path.remove(self.canvas.start)
        if self.canvas.end in unchecked_path:
            unchecked_path.remove(self.canvas.end)

        if len(unchecked_path) > 0:
            self.display_message("Drawing path", "INFO")
            self.checked_path = unchecked_path

            self.canvas.path = []
            self.system_timer = QTimer()
            self.system_timer.setInterval(
                int(1000 / len(self.checked_path))
            )  # Convert to milliseconds
            self.system_timer.timeout.connect(self.animate_path)
            self.system_timer.start()

    def check_inside_grid(self, cell):
        if (
            cell[0] < 0
            or cell[0] >= self.grid_dimensions[0]
            or cell[1] < 0
            or cell[1] >= self.grid_dimensions[1]
        ):
            self.display_message("Path outside grid", "ERROR")
            return False
        return True

    def check_obstacle_intersection(self, cell):
        if cell in self.canvas.obstacles:
            self.display_message("Path intersects obstacle", "WARN")
            return True
        return False

    def create_grid(self):
        grid = [
            [1 for x in range(self.grid_dimensions[1])]
            for y in range(self.grid_dimensions[0])
        ]
        for obstacle in self.canvas.obstacles:
            grid[obstacle[0]][obstacle[1]] = 0
        return grid

    def animate_path(self):
        if len(self.checked_path) > 0 and self.canvas.path != None:
            self.canvas.path.append(self.checked_path.pop(0))
            self.canvas.update()
        else:
            self.system_timer.stop()

    def display_message(self, message, type="DEBUG"):
        message = "[{}] {}".format(type, message)
        if type == "DEBUG":
            self.message_display.appendBlueText(message)
        elif type == "ERROR":
            self.message_display.appendRedText(message)
        elif type == "INFO":
            self.message_display.appendBlackText(message)
        elif type == "WARN":
            self.message_display.appendOrangeText(message)
        else:
            return
        self.message_display.scrollToTop()


class CanvasWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.setMinimumSize(500, 300)
        self.setAutoFillBackground(True)
        self.setPalette(QtGui.QPalette(QtGui.QColor(255, 255, 255)))
        self.mouse_pressed = False
        self.grid = []
        self.obstacles = []
        self.path = None
        self.start = None
        self.end = None
        self.cell_width = 0
        self.cell_height = 0
        self.column_offset = 0
        self.row_offset = 0
        self.open_nodes = set()
        self.closed_nodes = set()
        self.current_neighbors = set()
        self.f_values = {}  # 新增：存储f值

    def draw_grid(self, columns, rows):
        width = self.width()
        height = self.height()
        self.cell_width = math.floor(width / columns)
        self.column_offset = math.floor((width % self.cell_width) / 2)
        self.cell_height = math.floor(height / rows)
        self.row_offset = math.floor((height % self.cell_height) / 2)
        
        self.grid = []
        for x in range(columns + 1):
            xc = x * self.cell_width
            self.grid.append((
                xc + self.column_offset,
                self.row_offset,
                xc + self.column_offset,
                rows * self.cell_height + self.row_offset,
            ))
        for y in range(rows + 1):
            yc = y * self.cell_height
            self.grid.append((
                self.column_offset,
                yc + self.row_offset,
                columns * self.cell_width + self.column_offset,
                yc + self.row_offset,
            ))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        
        # 绘制动态元素
        painter.setBrush(QColor(0, 255, 255, 80))
        for node in self.open_nodes:
            painter.drawRect(*self.cell_to_coords(node))
            
        painter.setBrush(QColor(255, 255, 0, 50))
        for node in self.closed_nodes:
            painter.drawRect(*self.cell_to_coords(node))
            
        painter.setBrush(QColor(255, 0, 255, 100))
        for node in self.current_neighbors:
            painter.drawRect(*self.cell_to_coords(node))
            
        # 绘制静态元素
        painter.setBrush(Qt.black)
        for obstacle in self.obstacles:
            painter.drawRect(*self.cell_to_coords(obstacle))
            
        if self.start:
            painter.fillRect(*self.cell_to_coords(self.start), Qt.green)
        if self.end:
            painter.fillRect(*self.cell_to_coords(self.end), Qt.red)
            
        if self.path:
            painter.setBrush(Qt.blue)
            for cell in self.path:
                painter.drawRect(*self.cell_to_coords(cell))
                
        # 绘制网格线
        painter.setPen(QPen(Qt.black, 2))
        for line in self.grid:
            painter.drawLine(*line)
        
        # 绘制f值（在所有元素之后绘制，确保文字在最上层）
        painter.setPen(QPen(Qt.black, 1))
        painter.setFont(QFont("Arial", 8))
        for pos in self.f_values:
            x, y, w, h = self.cell_to_coords(pos)
            text = f"{self.f_values[pos]:.1f}"
            text_rect = painter.fontMetrics().boundingRect(text)
            
            # 将坐标转换为整数
            draw_x = int(x + (w - text_rect.width()) / 2)
            draw_y = int(y + (h + text_rect.height()) / 2)
            
            painter.drawText(draw_x, draw_y, text)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.mouse_pressed = True

            if self.parent.start_mode:

                # Remove the path and reset the indicator
                self.path = None
                self.parent.indicator.setStyleSheet(
                    "QLabel { background-color : grey; }"
                )

                start_pos = event.pos()
                start_cell = self.get_selected_cell(start_pos)
                if (
                    start_cell not in self.obstacles
                    and start_cell != self.end
                    and start_cell[0] >= 0
                    and start_cell[0] < self.parent.grid_dimensions[0]
                    and start_cell[1] >= 0
                    and start_cell[1] < self.parent.grid_dimensions[1]
                ):
                    self.start = start_cell
                    self.parent.start_set = True
                    self.update()
            elif self.parent.end_mode:

                # Remove the path and reset the indicator
                self.path = None
                self.parent.indicator.setStyleSheet(
                    "QLabel { background-color : grey; }"
                )

                end_pos = event.pos()
                end_cell = self.get_selected_cell(end_pos)
                if (
                    end_cell not in self.obstacles
                    and end_cell != self.start
                    and end_cell[0] >= 0
                    and end_cell[0] < self.parent.grid_dimensions[0]
                    and end_cell[1] >= 0
                    and end_cell[1] < self.parent.grid_dimensions[1]
                ):
                    self.end = end_cell
                    self.parent.end_set = True
                    self.update()

    def mouseMoveEvent(self, event):
        if self.mouse_pressed:
            if self.parent.obstacle_mode:

                # Remove the path and reset the indicator
                self.path = None
                self.parent.indicator.setStyleSheet(
                    "QLabel { background-color : grey; }"
                )

                obstacle_pos = event.pos()
                obstacle_cell = self.get_selected_cell(obstacle_pos)
                if (
                    obstacle_cell not in self.obstacles
                    and obstacle_cell != self.start
                    and obstacle_cell != self.end
                    and obstacle_cell[0] >= 0
                    and obstacle_cell[0] < self.parent.grid_dimensions[0]
                    and obstacle_cell[1] >= 0
                    and obstacle_cell[1] < self.parent.grid_dimensions[1]
                ):
                    self.obstacles.append(obstacle_cell)
                    self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self.parent.obstacle_mode:

                # Remove the path and reset the indicator
                self.path = None
                self.parent.indicator.setStyleSheet(
                    "QLabel { background-color : grey; }"
                )
                obstacle_pos = event.pos()
                obstacle_cell = self.get_selected_cell(obstacle_pos)
                if (
                    obstacle_cell not in self.obstacles
                    and obstacle_cell != self.start
                    and obstacle_cell != self.end
                    and obstacle_cell[0] >= 0
                    and obstacle_cell[0] < self.parent.grid_dimensions[0]
                    and obstacle_cell[1] >= 0
                    and obstacle_cell[1] < self.parent.grid_dimensions[1]
                ):
                    self.obstacles.append(obstacle_cell)
            self.mouse_pressed = False
            self.update()

    #     # 添加新的单步执行方法
    # def on_click_step(self):
    #     """单步执行按钮点击事件"""
    #     if not self.start_set or not self.end_set:
    #         self.display_message("需要先设置起点和终点", "WARN")
    #         return
        
    #     try:
    #         if self.algorithm_generator is None:
    #             # 初始化算法
    #             self.algorithm_generator = self.a_star_stepwise(
    #                 self.create_grid(),
    #                 self.canvas.start,
    #                 self.canvas.end,
    #                 self.display_message,
    #             )
            
    #         # 执行单步
    #         result = next(self.algorithm_generator, None)
            
    #         if result is None:
    #             self.display_message("算法已完成", "INFO")
    #             self.step_button.setEnabled(False)
    #             return
                
    #         # 更新界面状态
    #         self.current_open, self.current_closed, self.current_neighbors = result
    #         self.canvas.open_nodes = self.current_open
    #         self.canvas.closed_nodes = self.current_closed
    #         self.canvas.current_neighbors = self.current_neighbors
    #         self.canvas.update()
        
    #     except Exception as e:
    #         self.display_message(f"执行出错: {str(e)}", "ERROR")
    #         self.algorithm_generator = None

    # def init_algorithm(self):
    #     # 初始化算法参数
    #     self.step_gen = self.a_star_stepwise(
    #         self.create_grid(),
    #         self.canvas.start,
    #         self.canvas.end,
    #         self.display_message,
    #     )
    #     self.algorithm_state = {
    #         'open_set': [],
    #         'closed_set': set(),
    #         'g_scores': {self.canvas.start: 0},
    #         'parents': {},
    #         'path_found': False
    #     }
    #     self.step_button.setEnabled(True)
    #     self.canvas.path = None

    # # 修改后的A*算法（生成器版本）
    # def a_star_stepwise(self, grid, start, end):
    #     open_set = [(self.heuristic(start, end), start[0], start[1])]
    #     closed_set = set()
    #     g_scores = {start: 0}
    #     parents = {}
        
    #     while open_set:
    #         current_open = {(col, row) for (_, col, row) in open_set}
    #         current_node = min(open_set, key=lambda x: x[0])
    #         current_f, current_col, current_row = current_node
    #         current_pos = (current_col, current_row)
    #         open_set.remove(current_node)
            
    #         if current_pos == end:
    #             path = self.reconstruct_path(parents, end)
    #             self.canvas.path = path
    #             self.canvas.update()
    #             yield None  # 算法完成
    #             return
                
    #         closed_set.add(current_pos)
            
    #         neighbors = []
    #         for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
    #             n_col = current_col + dx
    #             n_row = current_row + dy
    #             if 0 <= n_col < len(grid) and 0 <= n_row < len(grid[0]):
    #                 if grid[n_col][n_row] == 1:
    #                     neighbors.append((n_col, n_row))
            
    #         # 准备要显示的邻居节点
    #         display_neighbors = set(neighbors)
            
    #         # 返回当前状态
    #         yield (current_open, closed_set.copy(), display_neighbors)
            
    #         # 处理邻居节点
    #         for n_col, n_row in neighbors:
    #             n_pos = (n_col, n_row)
    #             if n_pos in closed_set:
    #                 continue
                    
    #             tentative_g = g_scores[current_pos] + 1
                
    #             if tentative_g < g_scores.get(n_pos, float('inf')):
    #                 parents[n_pos] = current_pos
    #                 g_scores[n_pos] = tentative_g
    #                 f = tentative_g + self.heuristic(n_pos, end)
                    
    #                 existing = next((x for x in open_set if x[1:] == n_pos), None)
    #                 if existing:
    #                     if f < existing[0]:
    #                         open_set.remove(existing)
    #                         open_set.append((f, n_col, n_row))
    #                 else:
    #                     open_set.append((f, n_col, n_row))
            
    #         # 再次yield让界面更新
    #         yield (current_open, closed_set.copy(), set())
        
    #     yield None  # 没有找到路径

    # def heuristic(self, pos, end):
    #     return ((end[0]-pos[0])**2 + (end[1]-pos[1])**2)**0.5

    # def reconstruct_path(self, parents, end):
    #     path = []
    #     current = end
    #     while current in parents:
    #         path.append(current)
    #         current = parents[current]
    #     path.reverse()
    #     return path

    # # 修改CanvasWidget的paintEvent方法
    # def paintEvent(self, event):
    #     painter = QPainter(self)
        
    #     # 绘制开放节点（青色）
    #     painter.setBrush(QColor(0, 255, 255, 80))
    #     for node in self.open_nodes:
    #         painter.drawRect(*self.cell_to_coords(node))
            
    #     # 绘制封闭节点（黄色）
    #     painter.setBrush(QColor(255, 255, 0, 50))
    #     for node in self.closed_nodes:
    #         painter.drawRect(*self.cell_to_coords(node))
            
    #     # 绘制当前邻居（品红）
    #     painter.setBrush(QColor(255, 0, 255, 100))
    #     for node in self.current_neighbors:
    #         painter.drawRect(*self.cell_to_coords(node))
        
    #     # 绘制障碍物（黑色）
    #     painter.setBrush(Qt.black)
    #     for obstacle in self.obstacles:
    #         painter.drawRect(*self.cell_to_coords(obstacle))
        
    #     # 绘制起点终点
    #     if self.start:
    #         painter.fillRect(*self.cell_to_coords(self.start), Qt.green)
    #     if self.end:
    #         painter.fillRect(*self.cell_to_coords(self.end), Qt.red)
        
    #     # 绘制路径
    #     if self.path:
    #         painter.setBrush(Qt.blue)
    #         for cell in self.path:
    #             painter.drawRect(*self.cell_to_coords(cell))
        
    #     # 绘制网格线
    #     painter.setPen(QPen(Qt.black, 2))
    #     for line in self.grid:
    #         painter.drawLine(*line)

    def get_selected_cell(self, pos):
        return (
            math.floor((pos.x() - self.column_offset) / self.cell_width),
            math.floor((pos.y() - self.row_offset) / self.cell_height),
        )

    def cell_to_coords(self, obstacle_cell):
        return (
            obstacle_cell[0] * self.cell_width + self.column_offset,
            obstacle_cell[1] * self.cell_height + self.row_offset,
            self.cell_width,
            self.cell_height,
        )


class LabelledIntField(QWidget):
    def __init__(self, title, max_length, initial_value=None):
        QWidget.__init__(self)
        layout = QVBoxLayout()
        self.setLayout(layout)

        self.label = QLabel()
        self.label.setText(title)
        self.label.setFont(QFont("Arial", weight=QFont.Bold))
        layout.addWidget(self.label)

        self.lineEdit = QLineEdit(self)
        self.lineEdit.setValidator(QIntValidator())
        self.lineEdit.setMaxLength(max_length)
        if initial_value != None:
            self.lineEdit.setText(str(initial_value))
        layout.addWidget(self.lineEdit)
        layout.setContentsMargins(0, 0, 0, 0)

    def set_label_width(self, width):
        self.label.setFixedWidth(width)

    def set_input_width(self, width):
        self.lineEdit.setFixedWidth(width)

    def get_value(self):
        return int(self.lineEdit.text())


class ScrollableLabel(QScrollArea):

    def __init__(self, *args, **kwargs):
        QScrollArea.__init__(self, *args, **kwargs)

        self.setWidgetResizable(True)
        content = QWidget(self)
        self.setWidget(content)

        layout = QHBoxLayout(content)

        self.label = QLabel(content)

        self.label.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        self.label.setWordWrap(True)

        layout.addWidget(self.label)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)

    def setText(self, text):
        self.label.setText(text)

    def appendBlackText(self, text):
        self.label.setText(text + "<br>" + self.label.text())

    def appendRedText(self, text):
        self.label.setText(
            "<font color='Red'>" + text + "</font> " + "<br>" + self.label.text()
        )

    def appendGreenText(self, text):
        self.label.setText(
            "<font color='Green'>" + text + "</font> " + "<br>" + self.label.text()
        )

    def appendBlueText(self, text):
        self.label.setText(
            "<font color='Blue'>" + text + "</font> " + "<br>" + self.label.text()
        )

    def appendOrangeText(self, text):
        self.label.setText(
            "<font color='Orange'>" + text + "</font> " + "<br>" + self.label.text()
        )

    def scrollToBottom(self):
        self.verticalScrollBar().setValue(self.verticalScrollBar().maximum())

    def scrollToTop(self):
        self.verticalScrollBar().setValue(self.verticalScrollBar().minimum())


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    w = MainWindow()
    w.show()
    w.canvas.draw_grid(w.grid_dimensions[0], w.grid_dimensions[1])
    sys.exit(app.exec_())