import sys
from PyQt5 import QtGui
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
)
from PyQt5.QtCore import Qt, QPoint, QTimer
import math
import importlib
import pathPlanner


class MainWindow(QMainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.layout = QVBoxLayout()
        self.title = "AMR Coursework 2 - Path Planning"
        self.top = 200
        self.left = 500
        self.width = 600
        self.height = 500
        self.obstacle_mode = False  # Flag to indicate if obstacle mode is on
        self.start_mode = False  # Flag to indicate if start mode is on
        self.end_mode = False  # Flag to indicate if end mode is on
        self.start_set = False  # Flag to indicate if start has been set
        self.end_set = False  # Flag to indicate if end has been set
        self.grid_dimensions = [20, 10]
        self.checked_path = []

        self.init_window()
        # Container widget to position widgets
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
        self.width_input.show()  # Widget dimensions are not set until it is shown

        self.reset_button = QPushButton("Reset")
        self.reset_button.setStyleSheet("QPushButton { background-color: white }")
        self.reset_button.setMaximumHeight(self.width_input.height())
        self.reset_button.clicked.connect(self.on_click_reset)

        self.obstacle_button = QPushButton("Add\nObstacles")
        self.obstacle_button.setStyleSheet("QPushButton { background-color: white }")
        self.obstacle_button.setMaximumHeight(self.width_input.height())
        self.obstacle_button.clicked.connect(self.on_click_obstacle)

        self.obstacle_undo_button = QPushButton("Undo\nObstacle")
        self.obstacle_undo_button.setStyleSheet(
            "QPushButton { background-color: white }"
        )
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
        control_panel_layout.addStretch(1)
        control_panel_layout.addWidget(self.run_button)

        control_panel_layout.setSpacing(2)
        parentLayout.addLayout(control_panel_layout)

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

    def draw_grid(self, columns, rows):
        self.grid = []
        width = self.width()
        height = self.height()
        self.cell_width = math.floor(width / columns)
        self.column_offset = math.floor(
            (width % self.cell_width) / 2
        )  # Used to center the grid
        self.cell_height = math.floor(height / rows)
        self.row_offset = math.floor(
            (height % self.cell_height) / 2
        )  # used to center the grid
        for x in range(0, columns + 1):
            xc = x * self.cell_width
            self.grid.append(
                (
                    xc + self.column_offset,
                    self.row_offset,
                    xc + self.column_offset,
                    rows * self.cell_height + self.row_offset,
                )
            )

        for y in range(0, rows + 1):
            yc = y * self.cell_height
            self.grid.append(
                (
                    self.column_offset,
                    yc + self.row_offset,
                    columns * self.cell_width + self.column_offset,
                    yc + self.row_offset,
                )
            )
        self.update()

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

    def paintEvent(self, event):
        painter = QPainter(self)
        pen = QPen()
        pen.setWidth(2)
        pen.setColor(Qt.black)
        painter.setPen(pen)

        for obstacle in self.obstacles:
            painter.fillRect(*self.cell_to_coords(obstacle), Qt.black)

        if self.start:
            painter.fillRect(*self.cell_to_coords(self.start), Qt.green)
        if self.end:
            painter.fillRect(*self.cell_to_coords(self.end), Qt.red)

        if self.path:
            for cell in self.path:
                if cell not in self.obstacles:
                    painter.fillRect(*self.cell_to_coords(cell), Qt.blue)
                else:
                    painter.fillRect(*self.cell_to_coords(cell), Qt.gray)

        for line in self.grid:
            painter.drawLine(*line)

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


app = QApplication(sys.argv)
app.setStyle("Fusion")
w = MainWindow()
w.show()
w.canvas.draw_grid(w.grid_dimensions[0], w.grid_dimensions[1])
sys.exit(app.exec_())
