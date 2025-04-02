import sys
import math
from PyQt5 import QtGui
import importlib
import pathPlanner
import algorithmB
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
        
        # 界面初始化
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

        # 初始化主界面
        self.init_window()
        
        # 设置中心控件
        self.container_widget = QWidget()
        self.container_widget.setLayout(self.layout)
        self.setCentralWidget(self.container_widget)
        self.run_button_algorithm_b = QPushButton("Run Algorithm B")

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
        self.width_input = LabelledIntField("Width", 2, 20)
        self.height_input = LabelledIntField("Height", 2, 10)
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
        
        self.run_button_algorithm_b = QPushButton("Run Algorithm B")
        self.run_button_algorithm_b.setStyleSheet("QPushButton { background-color: white }")
        self.run_button_algorithm_b.setMaximumHeight(self.width_input.height())
        self.run_button_algorithm_b.clicked.connect(self.on_click_run_algorithm_b)

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
        control_panel_layout.addWidget(self.run_button_algorithm_b)
        control_panel_layout.addStretch(1)

        control_panel_layout.setSpacing(2)
        parentLayout.addLayout(control_panel_layout)

    # ========== 核心功能方法 ==========
    def on_click_step(self):
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
            self.canvas.f_values = self.current_f_values
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
        self.display_message(f"重置网格: {self.grid_dimensions[0]}x{self.grid_dimensions[1]}", "INFO")
        self.indicator.setStyleSheet("QLabel { background-color : grey; }")
        self.canvas.draw_grid(*self.grid_dimensions)
        self.canvas.obstacles = []
        self.canvas.path = None
        self.start_set = False
        self.canvas.start = None
        self.canvas.end = None
        self.algorithm_generator = None

    def on_click_clear(self):
        self.message_display.setText("")
        self.checked_path = []
        if hasattr(self, 'system_timer'):
            self.system_timer.stop()
        self.algorithm_generator = None

    def on_click_reset(self):
        self.reset()

    def on_click_obstacle(self):
        self.obstacle_mode = not self.obstacle_mode
        self.start_mode = False
        self.end_mode = False
        self.obstacle_button.setStyleSheet(
            "QPushButton { background-color: %s }" % ("grey" if self.obstacle_mode else "white")
        )

    def on_click_obstacle_undo(self):
        if self.canvas.obstacles:
            self.canvas.obstacles.pop()
            self.canvas.update()

    def on_click_start(self):
        self.start_mode = not self.start_mode
        self.obstacle_mode = False
        self.end_mode = False
        self.start_button.setStyleSheet(
            "QPushButton { background-color: %s }" % ("green" if self.start_mode else "white")
        )

    def on_click_end(self):
        self.end_mode = not self.end_mode
        self.start_mode = False
        self.obstacle_mode = False
        self.end_button.setStyleSheet(
            "QPushButton { background-color: %s }" % ("red" if self.end_mode else "white")
        )

    def on_click_run(self):
        self.canvas.path = []
        self.canvas.path_algorithm_b = None
        self.canvas.update()
        
        if not self.start_set or not self.end_set:
            self.display_message("请先设置起点和终点", "WARN")
            return

        try:
            importlib.reload(pathPlanner)
            path = pathPlanner.do_a_star(
                self.create_grid(),
                self.canvas.start,
                self.canvas.end,
                self.display_message,
            )
            
            if not path:
                self.display_message("未找到路径", "ERROR")
                return

            self.checked_path = path
            self.canvas.path = self.checked_path.copy()
            self.canvas.update()

        except Exception as e:
            self.display_message(f"错误: {str(e)}", "ERROR")

    def on_click_run_algorithm_b(self):
        self.canvas.path_algorithm_b = None
        self.canvas.update()
        
        if not self.start_set or not self.end_set:
            self.display_message("请先设置起点和终点", "WARN")
            return

        try:
            importlib.reload(algorithmB)
            path = algorithmB.do_a_star(
                self.create_grid(),
                self.canvas.start,
                self.canvas.end,
                self.display_message,
            )
            
            if not path:
                self.display_message("未找到路径", "ERROR")
                return

            self.canvas.path_algorithm_b = path
            self.canvas.update()

        except Exception as e:
            self.display_message(f"错误: {str(e)}", "ERROR")

    def check_inside_grid(self, cell):
        return 0 <= cell[0] < self.grid_dimensions[0] and 0 <= cell[1] < self.grid_dimensions[1]

    def check_obstacle_intersection(self, cell):
        return cell in self.canvas.obstacles

    def create_grid(self):
        grid = [[1]*self.grid_dimensions[1] for _ in range(self.grid_dimensions[0])]
        for obstacle in self.canvas.obstacles:
            grid[obstacle[0]][obstacle[1]] = 0
        return grid

    def display_message(self, message, type="DEBUG"):
        formatted = f"[{type}] {message}"
        if type == "DEBUG":
            self.message_display.appendBlueText(formatted)
        elif type == "ERROR":
            self.message_display.appendRedText(formatted)
        elif type == "INFO":
            self.message_display.appendBlackText(formatted)
        elif type == "WARN":
            self.message_display.appendOrangeText(formatted)
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
        self.path_algorithm_b = None
        self.start = None
        self.end = None
        self.cell_width = 0
        self.cell_height = 0
        self.column_offset = 0
        self.row_offset = 0
        self.open_nodes = set()
        self.closed_nodes = set()
        self.current_neighbors = set()
        self.f_values = {}

    def draw_grid(self, columns, rows):
        width = self.width()
        height = self.height()
        self.cell_width = math.floor(width / columns)
        self.column_offset = math.floor((width % self.cell_width) / 2)
        self.cell_height = math.floor(height / rows)
        self.row_offset = math.floor((height % self.cell_height) / 2)
        
        self.grid = [
            (x * self.cell_width + self.column_offset, self.row_offset, 
             x * self.cell_width + self.column_offset, rows * self.cell_height + self.row_offset)
            for x in range(columns + 1)
        ] + [
            (self.column_offset, y * self.cell_height + self.row_offset,
             columns * self.cell_width + self.column_offset, y * self.cell_height + self.row_offset)
            for y in range(rows + 1)
        ]
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
            painter.setBrush(Qt.blue)  # A* 路径
            for cell in self.path:
                painter.drawRect(*self.cell_to_coords(cell))

        if self.path_algorithm_b:
            painter.setBrush(Qt.magenta)  # Algorithm B 路径
            for cell in self.path_algorithm_b:
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
        
        # # 新增网格坐标标注
        # painter.setPen(QPen(Qt.gray, 1))
        # painter.setFont(QFont("Arial", 8))
        # for x in range(self.parent.grid_dimensions[0]):
            # for y in range(self.parent.grid_dimensions[1]):
                # x_pos = x * self.cell_width + self.column_offset + 5
                # y_pos = y * self.cell_height + self.row_offset + 15
                # painter.drawText(x_pos, y_pos, f"({x},{y})")

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.mouse_pressed = True  # 新增：设置鼠标按下标志
            cell = self.get_selected_cell(event.pos())
            
            if self.parent.obstacle_mode:
                self.handle_obstacle(cell)
            elif self.parent.start_mode:
                self.handle_start(cell)
            elif self.parent.end_mode:
                self.handle_end(cell)
                
            self.update()

    def mouseMoveEvent(self, event):
        if self.mouse_pressed and self.parent.obstacle_mode:
            cell = self.get_selected_cell(event.pos())
            self.handle_obstacle(cell)
            self.update()

    def mouseReleaseEvent(self, event):
        self.mouse_pressed = False  # 新增：清除鼠标按下标志

    def handle_obstacle(self, cell):
        # 添加防重复检查
        if (self.is_valid_cell(cell) 
            and cell not in self.obstacles 
            and cell != self.start 
            and cell != self.end):
            
            self.obstacles.append(cell)

    def handle_start(self, cell):
        if self.is_valid_cell(cell) and cell != self.end:
            self.start = cell
            self.parent.start_set = True
            self.update()

    def handle_end(self, cell):
        if self.is_valid_cell(cell) and cell != self.start:
            self.end = cell
            self.parent.end_set = True
            self.update()

    def handle_obstacle(self, cell):
        if self.is_valid_cell(cell) and cell not in (self.start, self.end):
            self.obstacles.append(cell)
            self.update()

    def is_valid_cell(self, cell):
        return (0 <= cell[0] < self.parent.grid_dimensions[0] and 
                0 <= cell[1] < self.parent.grid_dimensions[1])

    def get_selected_cell(self, pos):
        return (
            math.floor((pos.x() - self.column_offset) / self.cell_width),
            math.floor((pos.y() - self.row_offset) / self.cell_height)
        )

    def cell_to_coords(self, cell):
        return (
            cell[0] * self.cell_width + self.column_offset,
            cell[1] * self.cell_height + self.row_offset,
            self.cell_width,
            self.cell_height
        )

class LabelledIntField(QWidget):
    def __init__(self, title, max_length, initial_value=None):
        super().__init__()
        layout = QVBoxLayout()
        self.label = QLabel(title)
        self.lineEdit = QLineEdit()
        self.lineEdit.setValidator(QIntValidator())
        self.lineEdit.setMaxLength(max_length)
        if initial_value:
            self.lineEdit.setText(str(initial_value))
        layout.addWidget(self.label)
        layout.addWidget(self.lineEdit)
        self.setLayout(layout)

    def get_value(self):
        return int(self.lineEdit.text())

class ScrollableLabel(QScrollArea):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWidgetResizable(True)
        content = QWidget()
        self.setWidget(content)
        self.label = QLabel(content)
        self.label.setWordWrap(True)
        layout = QHBoxLayout(content)
        layout.addWidget(self.label)
        
        # 添加滚动条策略
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)

    def setText(self, text):
        self.label.setText(text)

    def scrollToTop(self):
        self.verticalScrollBar().setValue(0)

    def scrollToBottom(self):
        self.verticalScrollBar().setValue(self.verticalScrollBar().maximum())

    def appendBlueText(self, text):
        self.label.setText(f'<font color="blue">{text}</font><br>{self.label.text()}')
        self.scrollToTop()

    def appendRedText(self, text):
        self.label.setText(f'<font color="red">{text}</font><br>{self.label.text()}')
        self.scrollToTop()

    def appendBlackText(self, text):
        self.label.setText(f'{text}<br>{self.label.text()}')
        self.scrollToTop()

    def appendOrangeText(self, text):
        self.label.setText(f'<font color="orange">{text}</font><br>{self.label.text()}')
        self.scrollToTop()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    window.canvas.draw_grid(20, 10)
    sys.exit(app.exec_())