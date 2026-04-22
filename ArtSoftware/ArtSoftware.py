import sys
import math
import time
import copy
import json
import threading
from collections import deque
from pathlib import Path
from PySide6.QtWidgets import (
    QApplication, QWidget, QScrollArea, QVBoxLayout,
    QHBoxLayout, QSlider, QLabel, QMainWindow, QToolBar,
    QSpinBox, QFrame, QPushButton, QStatusBar, QMenuBar,
    QButtonGroup, QSpacerItem, QSizePolicy, QMessageBox,
    QLineEdit, QDoubleSpinBox, QPlainTextEdit
)
from PySide6.QtGui import (
    QPainter, QColor, QImage, QTabletEvent, QWheelEvent,
    QAction, QIcon, QPen, QBrush, QCursor, QTransform,
    QPixmap, QKeySequence
)
from PySide6.QtCore import Qt, QPointF, QSize, QRectF, QPoint, QTimer

try:
    serial = __import__("serial")
except ImportError:
    serial = None

DRAWING_WIDTH_MM = 195.0
DRAWING_HEIGHT_MM = 185.0
PIXELS_PER_MM = 12.0
WIDTH = int(DRAWING_WIDTH_MM * PIXELS_PER_MM)
HEIGHT = int(DRAWING_HEIGHT_MM * PIXELS_PER_MM)
LINE_THICKNESS_MM = 0.3
PEN_STATE_UP = 0
PEN_STATE_DOWN = 1
MAX_SKETCH_HISTORY = 100

# --- STYLING ---
DARK_STYLE = """
QMainWindow, QWidget {
    background-color: #1e1e1e;
    color: #e0e0e0;
}
QToolBar {
    background-color: #2d2d2d;
    border: 1px solid #3d3d3d;
    border-radius: 10px;
    spacing: 10px;
    padding: 5px;
}
QPushButton {
    background-color: #3d3d3d;
    border-radius: 10px;
    min-width: 40px;
    min-height: 40px;
    font-weight: bold;
}
QLineEdit, QSpinBox, QDoubleSpinBox, QPlainTextEdit {
    background-color: #2a2a2a;
    border: 1px solid #3d3d3d;
    border-radius: 8px;
    padding: 4px 6px;
    min-height: 24px;
}
QMenuBar {
    background-color: #202020;
}
QMenuBar::item {
    border-radius: 6px;
    padding: 4px 8px;
}
QMenuBar::item:selected {
    background-color: #303030;
}
QMenu {
    background-color: #242424;
    border: 1px solid #3a3a3a;
    border-radius: 10px;
    padding: 4px;
}
QMenu::item {
    border-radius: 6px;
    padding: 6px 18px;
}
QMenu::item:selected {
    background-color: #3498db;
    color: #ffffff;
}
QPushButton:hover {
    background-color: #505050;
}
QPushButton:checked {
    background-color: #3498db;
    color: white;
    border: 1px solid #2980b9;
}
QPushButton:disabled {
    background-color: #2a2a2a;
    color: #555555;
    border: 1px solid #2a2a2a;
}
QSlider::groove:horizontal {
    border: 1px solid #3d3d3d;
    height: 8px;
    background: #1a1a1a;
    margin: 2px 0;
    border-radius: 4px;
}
QSlider::handle:horizontal {
    background: #3498db;
    border: 1px solid #3498db;
    width: 18px;
    margin: -5px 0;
    border-radius: 9px;
}
"""

# Tool Constants
TOOL_INK = "INK"
TOOL_SKETCH = "SKETCH"
TOOL_ERASER_SKETCH = "ERASER_SKETCH"


class MachineBridge:
    def __init__(self, output_file):
        self.output_file = Path(output_file)
        self.serial_conn = None
        self.command_queue = []
        self.connected_port = ""
        self.connected_baud = 115200

        self._lock = threading.Lock()
        self._sender_thread = None
        self._stop_event = threading.Event()
        self._has_work = threading.Event()
        self._pause_requested = False
        self._paused = False

        self.clear_output_log()

    def connect_serial(self, port, baud):
        if serial is None:
            return False, "pyserial is not installed"
        try:
            self.disconnect_serial()
            self.serial_conn = serial.Serial(port=port, baudrate=baud, timeout=1.0)
            self.connected_port = port
            self.connected_baud = baud
            time.sleep(0.25)
            try:
                self.serial_conn.reset_input_buffer()
            except Exception:
                pass
            self._start_sender()
            return True, f"Connected to {port} @ {baud}"
        except Exception as exc:
            self.serial_conn = None
            return False, f"Serial connection failed: {exc}"

    def disconnect_serial(self):
        self._stop_sender()
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.serial_conn = None
        self.connected_port = ""

    def enqueue_stroke(self, stroke_data_mm):
        if not stroke_data_mm:
            return
        stroke_data_mm = self._compress_stroke(stroke_data_mm)

        commands = []
        start = stroke_data_mm[0]
        commands.append(f"START X{start['x']:.2f} Y{start['y']:.2f}")

        for point in stroke_data_mm[1:-1]:
            commands.append(f"MOVE X{point['x']:.2f} Y{point['y']:.2f}")

        end = stroke_data_mm[-1]
        commands.append(f"END X{end['x']:.2f} Y{end['y']:.2f}")
        with self._lock:
            self.command_queue.extend(commands)
        self._persist_stroke_json(stroke_data_mm)

    def _compress_stroke(self, stroke_data_mm):
        if len(stroke_data_mm) <= 2:
            return stroke_data_mm

        # 1. RDP Line Simplification - Fine Tuned
        def rdp_simplify(points, epsilon_mm):
            if len(points) < 3:
                return points

            def point_line_distance(p, a, b):
                den = math.hypot(b['y'] - a['y'], b['x'] - a['x'])
                if den == 0:
                    return math.hypot(p['x'] - a['x'], p['y'] - a['y'])
                return abs((b['y'] - a['y'])*p['x'] - (b['x'] - a['x'])*p['y'] + b['x']*a['y'] - b['y']*a['x']) / den

            dmax = 0.0
            index = 0
            end = len(points) - 1
            for i in range(1, end):
                d = point_line_distance(points[i], points[0], points[end])
                if d > dmax:
                    index = i
                    dmax = d

            if dmax > epsilon_mm:
                left = rdp_simplify(points[:index+1], epsilon_mm)
                right = rdp_simplify(points[index:], epsilon_mm)
                return left[:-1] + right
            else:
                return [points[0], points[end]]

        # FINE TUNED: 0.08 preserves beautiful curves while shedding useless tablet jitter
        simplified = rdp_simplify(stroke_data_mm, 0.08)

        # 2. Minimum Distance Filter - Fine Tuned
        final_stroke = [simplified[0]]
        last = simplified[0]

        for point in simplified[1:-1]:
            dist = math.hypot(point['x'] - last['x'], point['y'] - last['y'])

            # FINE TUNED: 0.3mm distance forces the machine to process evenly spaced vectors 
            # without making them look blocky on paper.
            if dist >= 0.3:
                final_stroke.append(point)
                last = point

        final_stroke.append(simplified[-1])
        return final_stroke

    def flush_queue(self):
        self._has_work.set()

    def request_pause(self):
        with self._lock:
            self._pause_requested = True
            # If idle, pause immediately.
            if not self.command_queue:
                self._paused = True
        self._has_work.set()

    def resume_queue(self):
        with self._lock:
            self._pause_requested = False
            self._paused = False
        self._has_work.set()

    def is_paused(self):
        with self._lock:
            return self._paused

    def is_pause_requested(self):
        with self._lock:
            return self._pause_requested

    def clear_pending_queue(self):
        with self._lock:
            self.command_queue.clear()

    def get_pending_count(self):
        with self._lock:
            return len(self.command_queue)

    def get_queue_preview(self, max_items=40):
        with self._lock:
            if not self.command_queue:
                return []
            return list(self.command_queue[-max_items:])

    def clear_output_log(self):
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        with self.output_file.open("w", encoding="utf-8"):
            pass

    def clear_all(self):
        self.clear_pending_queue()
        self.clear_output_log()

    def _start_sender(self):
        self._stop_event.clear()
        if self._sender_thread and self._sender_thread.is_alive():
            return
        self._sender_thread = threading.Thread(
            target=self._sender_loop, daemon=True, name="machine-sender"
        )
        self._sender_thread.start()

    def _stop_sender(self):
        self._stop_event.set()
        self._has_work.set()
        if self._sender_thread and self._sender_thread.is_alive():
            self._sender_thread.join(timeout=2.0)

    def _sender_loop(self):
        in_flight = 0
        in_flight_cmds = deque()
        # PIPELINE FIX: We keep exactly 2 commands active.
        # 1 executes on the Arduino, while the 2nd sits in the Arduino's 64-byte RAM buffer.
        # This completely eliminates USB latency lag between moves.
        MAX_IN_FLIGHT = 2 

        while not self._stop_event.is_set():
            with self._lock:
                queue_has_items = len(self.command_queue) > 0
                paused = self._paused
                pause_requested = self._pause_requested

            if paused:
                self._has_work.wait()
                self._has_work.clear()
                continue

            if not queue_has_items and in_flight == 0:
                self._has_work.wait()
                self._has_work.clear()
                continue

            conn = self.serial_conn
            if conn is None or not conn.is_open:
                in_flight = 0
                time.sleep(0.1)
                continue

            # Fill the pipeline buffer
            while in_flight < MAX_IN_FLIGHT:
                with self._lock:
                    pause_requested = self._pause_requested
                if pause_requested:
                    break

                with self._lock:
                    if not self.command_queue:
                        break
                    command = self.command_queue.pop(0)

                try:
                    conn.write((command + "\n").encode("ascii", errors="ignore"))
                    conn.flush()
                    in_flight += 1
                    in_flight_cmds.append(command)
                except Exception as exc:
                    print(f"[BRIDGE] send error: {exc}")
                    with self._lock:
                        self.command_queue.insert(0, command)
                    break

            # Wait for at least ONE command to finish to clear space in the pipeline
            if in_flight > 0:
                if self._wait_for_ok(conn, timeout_seconds=20.0):
                    in_flight -= 1

                    completed_cmd = in_flight_cmds.popleft() if in_flight_cmds else ""
                    if completed_cmd.startswith("END"):
                        with self._lock:
                            if self._pause_requested:
                                self._paused = True
                                self._pause_requested = False
                else:
                    in_flight = 0
                    in_flight_cmds.clear()

        with self._lock:
            if not self.command_queue:
                self.clear_output_log()

    def _wait_for_ok(self, conn, timeout_seconds=20.0):
        deadline = time.time() + timeout_seconds
        while time.time() < deadline and not self._stop_event.is_set():
            try:
                # Polling approach allows us to yield without locking the serial buffer
                if conn.in_waiting > 0:
                    response = conn.readline().decode("ascii", errors="ignore").strip()
                    if not response:
                        continue
                    if response == "OK":
                        return True
                    print(f"[MACHINE] {response}")
                else:
                    time.sleep(0.001)
            except Exception as exc:
                print(f"[BRIDGE] ack read error: {exc}")
                return False

        print("[BRIDGE] timeout waiting for OK")
        return False

    def _persist_stroke_json(self, stroke_data_mm):
        payload = {
            "ts": time.time(),
            "stroke": stroke_data_mm,
        }
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        with self.output_file.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(payload) + "\n")


class Navigator(QWidget):
    def __init__(self, canvas_ref, scroll_area):
        super().__init__()
        self.canvas_ref = canvas_ref
        self.scroll_area = scroll_area
        self.setFixedWidth(220)
        self.setMinimumHeight(300)
        self.target_rect = QRectF()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(30, 30, 30))

        padding = 15
        available_w = self.width() - (padding * 2)
        available_h = self.height() - (padding * 2)
        aspect_ratio = WIDTH / HEIGHT
        draw_w = available_w
        draw_h = int(draw_w / aspect_ratio)
        if draw_h > available_h:
            draw_h = available_h
            draw_w = int(draw_h * aspect_ratio)

        offset_x = (self.width() - draw_w) / 2
        offset_y = 20
        self.target_rect = QRectF(offset_x, offset_y, draw_w, draw_h)

        thumb = QImage(self.target_rect.size().toSize(), QImage.Format_ARGB32)
        thumb.fill(Qt.white)
        
        thumb_painter = QPainter(thumb)
        thumb_painter.setRenderHint(QPainter.SmoothPixmapTransform, False)
        
        target_size = self.target_rect.size().toSize()
        dest_rect = QRectF(0,0, target_size.width(), target_size.height())
        
        thumb_painter.drawImage(dest_rect, self.canvas_ref.layer_sketch)
        thumb_painter.drawImage(dest_rect, self.canvas_ref.layer_ink)
        thumb_painter.drawImage(dest_rect, self.canvas_ref.layer_feedback)
        
        thumb_painter.end()

        painter.fillRect(self.target_rect.adjusted(2, 2, 2, 2), QColor(0, 0, 0, 80))
        painter.fillRect(self.target_rect, Qt.white)

        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
        painter.setOpacity(0.9)
        painter.drawImage(self.target_rect, thumb)
        painter.setOpacity(1.0)

        painter.setPen(QPen(QColor(80, 80, 80), 1))
        painter.drawRect(self.target_rect)

        vis_rect = self.scroll_area.viewport().rect()
        zoom = self.canvas_ref.zoom_level
        
        scrollbar_h = self.scroll_area.horizontalScrollBar().value()
        scrollbar_v = self.scroll_area.verticalScrollBar().value()

        view_x = scrollbar_h / zoom
        view_y = scrollbar_v / zoom
        view_w = vis_rect.width() / zoom
        view_h = vis_rect.height() / zoom

        scale_x = self.target_rect.width() / WIDTH
        scale_y = self.target_rect.height() / HEIGHT

        box_w = min(view_w * scale_x, self.target_rect.width())
        box_h = min(view_h * scale_y, self.target_rect.height())
        box_x = self.target_rect.x() + (view_x * scale_x)
        box_y = self.target_rect.y() + (view_y * scale_y)

        painter.setPen(QPen(QColor(52, 152, 219), 1.5))
        painter.setBrush(QColor(52, 152, 219, 20))
        painter.drawRect(QRectF(box_x, box_y, box_w, box_h))

    def mousePressEvent(self, event):
        if self.target_rect.contains(event.position()):
            self.teleport_to(event.position())

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton:
            if self.target_rect.contains(event.position()):
                self.teleport_to(event.position())

    def teleport_to(self, pos):
        rel_x = (pos.x() - self.target_rect.x()) / self.target_rect.width()
        rel_y = (pos.y() - self.target_rect.y()) / self.target_rect.height()
        h_bar = self.scroll_area.horizontalScrollBar()
        v_bar = self.scroll_area.verticalScrollBar()
        h_bar.setValue(int(rel_x * h_bar.maximum()))
        v_bar.setValue(int(rel_y * v_bar.maximum()))
        self.update()

class Inkstroke(QWidget):
    def __init__(self):
        super().__init__()
        self.zoom_level = 0.25
        
        self.setFixedSize(int(WIDTH * self.zoom_level), int(HEIGHT * self.zoom_level))
        
        self.layer_sketch = QImage(WIDTH, HEIGHT, QImage.Format_ARGB32)
        self.layer_sketch.fill(Qt.transparent)
        
        self.layer_ink = QImage(WIDTH, HEIGHT, QImage.Format_ARGB32)
        self.layer_ink.fill(Qt.transparent)

        self.layer_feedback = QImage(WIDTH, HEIGHT, QImage.Format_ARGB32)
        self.layer_feedback.fill(Qt.transparent)

        self.sketch_strokes = [] 
        self.sketch_history = [] 
        self.sketch_redo_stack = []

        self.current_stroke_data = []
        self.current_tool = TOOL_INK
        
        self.drawing = False
        self.stabilization = 0.1
        self.current_pos = None
        self.nav_ref = None
        
        self.eraser_size_mm = 6.0
        self.flow = 0.8
        self.spacing_mm = 0.2
        self.current_pen_state = PEN_STATE_DOWN
        self._last_pan_pos = QPoint()

        self.cursor_screen_pos = QPointF(-100, -100)
        self.is_hovering = False
        
        self.machine_bridge = MachineBridge(Path(__file__).with_name("machine_queue.ndjson"))

        self.setAttribute(Qt.WA_TabletTracking)
        self.setMouseTracking(True)
        self.setCursor(Qt.BlankCursor)

    def set_zoom(self, factor):
        self.zoom_level = max(0.05, min(factor, 8.0))
        self.setFixedSize(int(WIDTH * self.zoom_level), int(HEIGHT * self.zoom_level))
        self.update()

    def mm_to_px(self, mm_value):
        return mm_value * PIXELS_PER_MM

    def px_to_mm(self, px_value):
        return px_value / PIXELS_PER_MM

    def px_to_machine_mm(self, x_px, y_px):
        x_mm = self.px_to_mm(x_px)
        y_mm = DRAWING_HEIGHT_MM - self.px_to_mm(y_px)
        x_mm = max(0.0, min(DRAWING_WIDTH_MM, x_mm))
        y_mm = max(0.0, min(DRAWING_HEIGHT_MM, y_mm))
        return x_mm, y_mm

    def machine_mm_to_px(self, x_mm, y_mm):
        x_px = self.mm_to_px(x_mm)
        y_px = self.mm_to_px(DRAWING_HEIGHT_MM - y_mm)
        return x_px, y_px

    def px_point_to_mm(self, point):
        x_mm, y_mm = self.px_to_machine_mm(point.x(), point.y())
        return {
            'x': x_mm,
            'y': y_mm,
            'p': point.get('p', PEN_STATE_DOWN) if isinstance(point, dict) else PEN_STATE_DOWN,
        }

    def _capture_sketch_state(self):
        return {
            'strokes': copy.deepcopy(self.sketch_strokes),
            'layer': self.layer_sketch.copy(),
        }

    def _restore_sketch_state(self, state):
        self.sketch_strokes = copy.deepcopy(state['strokes'])
        self.layer_sketch = state['layer'].copy()
        self.update()
    
    def save_sketch_state(self):
        if len(self.sketch_history) >= MAX_SKETCH_HISTORY:
            self.sketch_history.pop(0)
        self.sketch_history.append(self._capture_sketch_state())
        self.sketch_redo_stack.clear()

    def perform_undo(self):
        if self.current_tool == TOOL_SKETCH or self.current_tool == TOOL_ERASER_SKETCH:
            if self.sketch_history:
                if len(self.sketch_redo_stack) >= MAX_SKETCH_HISTORY:
                    self.sketch_redo_stack.pop(0)
                self.sketch_redo_stack.append(self._capture_sketch_state())
                self._restore_sketch_state(self.sketch_history.pop())
        elif self.current_tool == TOOL_INK:
            pass

    def perform_redo(self):
        if self.current_tool == TOOL_SKETCH or self.current_tool == TOOL_ERASER_SKETCH:
            if self.sketch_redo_stack:
                if len(self.sketch_history) >= MAX_SKETCH_HISTORY:
                    self.sketch_history.pop(0)
                self.sketch_history.append(self._capture_sketch_state())
                self._restore_sketch_state(self.sketch_redo_stack.pop())
        elif self.current_tool == TOOL_INK:
            pass

    def redraw_sketch_layer(self):
        self.layer_sketch.fill(Qt.transparent)
        
        painter = QPainter(self.layer_sketch)
        painter.setRenderHint(QPainter.Antialiasing)
        c = QColor(100, 149, 237)
        c.setAlpha(int(255 * self.flow))
        painter.setBrush(c)
        painter.setPen(Qt.NoPen)

        for stroke in self.sketch_strokes:
            if len(stroke) < 2: continue
            
            for i in range(len(stroke) - 1):
                p1 = stroke[i]
                p2 = stroke[i+1]
                
                pt1 = QPointF(p1['x'], p1['y'])
                pt2 = QPointF(p2['x'], p2['y'])

                radius = self.mm_to_px(LINE_THICKNESS_MM / 2.0)
                dist = math.hypot(pt2.x() - pt1.x(), pt2.y() - pt1.y())
                step = max(1, int(dist / self.mm_to_px(self.spacing_mm)))
                
                for s in range(step + 1):
                    t = s / step
                    x = pt1.x() + (pt2.x() - pt1.x()) * t
                    y = pt1.y() + (pt2.y() - pt1.y()) * t
                    painter.drawEllipse(QPointF(x, y), radius, radius)
        painter.end()
        self.update()

    def commit_ink_stroke(self, stroke_data):
        if not stroke_data: return
        stroke_data_mm = [
            {'x': x_mm, 'y': y_mm, 'p': PEN_STATE_DOWN}
            for p in stroke_data
            for x_mm, y_mm in [self.px_to_machine_mm(p['x'], p['y'])]
        ]
        
        print(f"\n[COMMIT] Sending stroke to machine ({len(stroke_data_mm)} pts, mm)...")
        start = stroke_data_mm[0]
        print(f"[ START ] X: {start['x']:>8.2f}, Y: {start['y']:>8.2f}")
        
        for i in range(len(stroke_data_mm)-1):
            p2 = stroke_data_mm[i+1]
            print(f"[ MOVE  ] X: {p2['x']:>8.2f}, Y: {p2['y']:>8.2f}")
            
        end = stroke_data_mm[-1]
        print(f"[  END  ] X: {end['x']:>8.2f}, Y: {end['y']:>8.2f}")

        self.machine_bridge.enqueue_stroke(stroke_data_mm)
        self.machine_bridge.flush_queue()

        painter = QPainter(self.layer_ink)
        painter.setRenderHint(QPainter.Antialiasing)
        c = QColor(0, 0, 0)
        c.setAlpha(int(255 * self.flow))
        painter.setBrush(c)
        painter.setPen(Qt.NoPen)
        
        for i in range(len(stroke_data) - 1):
            p1 = stroke_data[i]
            p2 = stroke_data[i+1]
            
            pt1 = QPointF(p1['x'], p1['y'])
            pt2 = QPointF(p2['x'], p2['y'])
            radius = self.mm_to_px(LINE_THICKNESS_MM / 2.0)
            
            dist = math.hypot(pt2.x() - pt1.x(), pt2.y() - pt1.y())
            step = max(1, int(dist / self.mm_to_px(self.spacing_mm)))
            
            for s in range(step + 1):
                t = s / step
                x = pt1.x() + (pt2.x() - pt1.x()) * t
                y = pt1.y() + (pt2.y() - pt1.y()) * t
                painter.drawEllipse(QPointF(x, y), radius, radius)
        painter.end()

    def refresh_feedback_layer(self):
        self.layer_feedback.fill(Qt.transparent)
        
        painter = QPainter(self.layer_feedback)
        painter.setRenderHint(QPainter.Antialiasing)
        c = QColor(0, 0, 0)
        c.setAlpha(int(255 * self.flow)) 
        painter.setBrush(c)
        painter.setPen(Qt.NoPen)
        
        def paint_path(data):
            if len(data) < 2: return
            for i in range(len(data) - 1):
                p1 = data[i]
                p2 = data[i+1]
                pt1 = QPointF(p1['x'], p1['y'])
                pt2 = QPointF(p2['x'], p2['y'])
                dist = math.hypot(pt2.x() - pt1.x(), pt2.y() - pt1.y())
                radius = self.mm_to_px(LINE_THICKNESS_MM / 2.0)
                step = max(1, int(dist / self.mm_to_px(self.spacing_mm)))
                for s in range(step + 1):
                    t = s / step
                    x = pt1.x() + (pt2.x() - pt1.x()) * t
                    y = pt1.y() + (pt2.y() - pt1.y()) * t
                    painter.drawEllipse(QPointF(x, y), radius, radius)

        if self.drawing and self.current_tool == TOOL_INK and self.current_stroke_data:
            paint_path(self.current_stroke_data)

        painter.end()
        self.update()

    def get_canvas_coords(self, pos):
        return QPointF(pos.x() / self.zoom_level, pos.y() / self.zoom_level)

    def process_input(self, input_pos_canvas):
        if self.current_pos is None:
            self.current_pos = input_pos_canvas
            return
        
        weight = 1.0 - self.stabilization
        nx = self.current_pos.x() + (input_pos_canvas.x() - self.current_pos.x()) * weight
        ny = self.current_pos.y() + (input_pos_canvas.y() - self.current_pos.y()) * weight
        new_stabilized_pos = QPointF(nx, ny)

        if self.drawing:
            self.draw_line(self.current_pos, new_stabilized_pos)
            
            if self.current_tool == TOOL_SKETCH or self.current_tool == TOOL_INK:
                self.current_stroke_data.append({
                    'x': new_stabilized_pos.x(),
                    'y': new_stabilized_pos.y(),
                    'p': PEN_STATE_DOWN
                })
        
        self.current_pos = new_stabilized_pos

    def draw_line(self, p1, p2):
        dist = math.hypot(p2.x() - p1.x(), p2.y() - p1.y())
        step = max(1, int(dist / self.mm_to_px(self.spacing_mm)))
        
        target_layer = None
        brush_color = Qt.black
        comp_mode = QPainter.CompositionMode_SourceOver
        
        radius = self.mm_to_px(LINE_THICKNESS_MM / 2.0)
        
        if self.current_tool == TOOL_INK:
            target_layer = self.layer_feedback
            brush_color = QColor(0, 0, 0)
            radius = self.mm_to_px(LINE_THICKNESS_MM / 2.0)
            
        elif self.current_tool == TOOL_SKETCH:
            target_layer = self.layer_sketch
            brush_color = QColor(100, 149, 237)
            radius = self.mm_to_px(LINE_THICKNESS_MM / 2.0)
            
        elif self.current_tool == TOOL_ERASER_SKETCH:
            target_layer = self.layer_sketch 
            brush_color = Qt.transparent
            comp_mode = QPainter.CompositionMode_Clear 
            base_r = self.mm_to_px(float(self.eraser_size_mm) / 2.0)
            radius = base_r
        
        if target_layer is None: return

        painter = QPainter(target_layer)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setCompositionMode(comp_mode)
        
        alpha = int(255 * self.flow)
        
        for i in range(step + 1):
            t = i / step
            x = p1.x() + (p2.x() - p1.x()) * t
            y = p1.y() + (p2.y() - p1.y()) * t
            
            painter.setPen(Qt.NoPen)
            
            if comp_mode != QPainter.CompositionMode_Clear:
                c = QColor(brush_color)
                c.setAlpha(alpha)
                painter.setBrush(c)
            else:
                painter.setBrush(Qt.black) 

            painter.drawEllipse(QPointF(x, y), radius, radius)
            
        painter.end()

    def tabletEvent(self, event: QTabletEvent):
        self.cursor_screen_pos = event.position()
        self.is_hovering = True
        
        pos_canvas = self.get_canvas_coords(event.position())
        
        if event.type() == QTabletEvent.TabletPress:
            self.drawing = True
            self.current_pos = pos_canvas
            
            self.current_stroke_data = []
            
            if self.current_tool == TOOL_SKETCH or self.current_tool == TOOL_ERASER_SKETCH:
                self.save_sketch_state()
            
            self.current_stroke_data.append({
                'x': pos_canvas.x(), 
                'y': pos_canvas.y(), 
                'p': PEN_STATE_DOWN
            })
        
        elif event.type() == QTabletEvent.TabletRelease:
            if self.current_pos:
                 self.current_stroke_data.append({
                    'x': self.current_pos.x(), 
                    'y': self.current_pos.y(), 
                          'p': PEN_STATE_UP
                })
            
            if self.current_stroke_data:
                if self.current_tool == TOOL_SKETCH:
                    self.sketch_strokes.append(self.current_stroke_data)
                
                elif self.current_tool == TOOL_INK:
                    self.commit_ink_stroke(self.current_stroke_data)
                    self.layer_feedback.fill(Qt.transparent)
            
            self.current_stroke_data = []
            self.drawing = False
        
        elif event.type() == QTabletEvent.TabletMove:
            if self.drawing:
                self.process_input(pos_canvas)
            
        event.accept()
        self.update()

    def mousePressEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self.setCursor(Qt.ClosedHandCursor)
            self._last_pan_pos = event.globalPosition().toPoint()
            return

        if event.button() == Qt.LeftButton:
            self.drawing = True
            self.current_pen_state = PEN_STATE_DOWN
            pos = self.get_canvas_coords(event.position())
            self.current_pos = pos
            
            self.current_stroke_data = []
            if self.current_tool == TOOL_SKETCH or self.current_tool == TOOL_ERASER_SKETCH:
                self.save_sketch_state()
            
            self.current_stroke_data.append({'x': pos.x(), 'y': pos.y(), 'p': PEN_STATE_DOWN})

    def mouseMoveEvent(self, event):
        self.cursor_screen_pos = event.position()
        self.is_hovering = True

        if event.buttons() & Qt.MiddleButton:
            delta = event.globalPosition().toPoint() - self._last_pan_pos
            self._last_pan_pos = event.globalPosition().toPoint()
            scroll_area = self.parent().parent() 
            if isinstance(scroll_area, QScrollArea):
                h_bar = scroll_area.horizontalScrollBar()
                v_bar = scroll_area.verticalScrollBar()
                h_bar.setValue(h_bar.value() - delta.x())
                v_bar.setValue(v_bar.value() - delta.y())
            self.update() 
            return

        if self.drawing:
            pos = self.get_canvas_coords(event.position())
            self.process_input(pos)
        self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self.setCursor(Qt.BlankCursor)
        if event.button() == Qt.LeftButton:
            if self.current_pos:
                 self.current_stroke_data.append({'x': self.current_pos.x(), 'y': self.current_pos.y(), 'p': PEN_STATE_UP})

            if self.current_stroke_data:
                if self.current_tool == TOOL_SKETCH:
                    self.sketch_strokes.append(self.current_stroke_data)
                elif self.current_tool == TOOL_INK:
                    self.commit_ink_stroke(self.current_stroke_data)
                    self.layer_feedback.fill(Qt.transparent)
            
            self.current_stroke_data = []
            self.drawing = False
            self.update()

    def leaveEvent(self, event):
        self.is_hovering = False
        self.update()
        super().leaveEvent(event)

    def paintEvent(self, event):
        painter = QPainter(self)
        
        if self.drawing:
            painter.setRenderHint(QPainter.SmoothPixmapTransform, False)
        else:
            painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
            
        painter.fillRect(self.rect(), Qt.white)
        painter.drawImage(self.rect(), self.layer_sketch)
        painter.drawImage(self.rect(), self.layer_ink)
        painter.drawImage(self.rect(), self.layer_feedback)

        if self.is_hovering:
            painter.setTransform(QTransform())
            cx, cy = self.cursor_screen_pos.x(), self.cursor_screen_pos.y()
            
            tool_radius = 0.0
            if self.current_tool == TOOL_ERASER_SKETCH:
                tool_radius = self.mm_to_px(float(self.eraser_size_mm) / 2.0)
            else:
                tool_radius = self.mm_to_px(LINE_THICKNESS_MM / 2.0)
            
            screen_radius = tool_radius * self.zoom_level
            gap_size = 4    
            arm_length = 6 
            
            pen = QPen(Qt.black, 0) 
            pen.setCosmetic(True)
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            painter.setRenderHint(QPainter.Antialiasing, True) 

            show_crosshair = True
            if screen_radius >= gap_size: show_crosshair = False
            if self.current_tool == TOOL_ERASER_SKETCH and self.eraser_size_mm > 4.0: show_crosshair = False

            if show_crosshair:
                painter.drawLine(QPointF(cx - gap_size - arm_length, cy), QPointF(cx - gap_size, cy))
                painter.drawLine(QPointF(cx + gap_size, cy), QPointF(cx + gap_size + arm_length, cy))
                painter.drawLine(QPointF(cx, cy - gap_size - arm_length), QPointF(cx, cy - gap_size))
                painter.drawLine(QPointF(cx, cy + gap_size), QPointF(cx, cy + gap_size + arm_length))

            painter.drawEllipse(QPointF(cx, cy), screen_radius, screen_radius)
            
            pen_white = QPen(Qt.white, 0)
            pen_white.setCosmetic(True)
            painter.setPen(pen_white)
            painter.setOpacity(0.5)
            painter.drawEllipse(QPointF(cx, cy), screen_radius + 1, screen_radius + 1)
            painter.setOpacity(1.0)
            
    def convert_sketch_to_ink(self):
        print("\n" + "="*40)
        print("    ROBOT COORDINATES MM (CONVERSION OUTPUT)")
        print("="*40)
        
        final_robot_paths = []
        for stroke in self.sketch_strokes:
            current_segment = []
            for point_data in stroke:
                x, y = point_data['x'], point_data['y']
                ix, iy = int(x), int(y)
                is_visible = False
                if 0 <= ix < WIDTH and 0 <= iy < HEIGHT:
                    if self.layer_sketch.pixelColor(ix, iy).alpha() > 20:
                        is_visible = True
                if is_visible:
                    current_segment.append(point_data)
                else:
                    if current_segment:
                        final_robot_paths.append(current_segment)
                        current_segment = []
            if current_segment:
                final_robot_paths.append(current_segment)

        painter = QPainter(self.layer_ink)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(Qt.NoPen)
        c = QColor(0, 0, 0)
        c.setAlpha(int(255 * self.flow)) 
        painter.setBrush(c)
        
        for path in final_robot_paths:
            if not path: continue
            path_mm = [
                {'x': x_mm, 'y': y_mm, 'p': PEN_STATE_DOWN}
                for v in path
                for x_mm, y_mm in [self.px_to_machine_mm(v['x'], v['y'])]
            ]
            start = path_mm[0]
            print(f"[ START ] X: {start['x']:>8.2f}, Y: {start['y']:>8.2f}")
            if len(path_mm) > 1:
                for i in range(len(path_mm) - 1):
                    p1_dat = path_mm[i]
                    p2_dat = path_mm[i+1]
                    p1 = QPointF(p1_dat['x'], p1_dat['y'])
                    p2 = QPointF(p2_dat['x'], p2_dat['y'])
                    print(f"[ MOVE  ] X: {p2.x():>8.2f}, Y: {p2.y():>8.2f}")
                    p1_px_x, p1_px_y = self.machine_mm_to_px(p1.x(), p1.y())
                    p2_px_x, p2_px_y = self.machine_mm_to_px(p2.x(), p2.y())
                    p1_px = QPointF(p1_px_x, p1_px_y)
                    p2_px = QPointF(p2_px_x, p2_px_y)
                    dist = math.hypot(p2_px.x() - p1_px.x(), p2_px.y() - p1_px.y())
                    step = max(1, int(dist / self.mm_to_px(self.spacing_mm)))
                    radius = self.mm_to_px(LINE_THICKNESS_MM / 2.0)
                    for s in range(step + 1):
                        t = s / step
                        lx = p1_px.x() + (p2_px.x() - p1_px.x()) * t
                        ly = p1_px.y() + (p2_px.y() - p1_px.y()) * t
                        painter.drawEllipse(QPointF(lx, ly), radius, radius)
            end = path_mm[-1]
            print(f"[  END  ] X: {end['x']:>8.2f}, Y: {end['y']:>8.2f}")

            self.machine_bridge.enqueue_stroke(path_mm)
            self.machine_bridge.flush_queue()

        painter.end()
        print("="*40 + "\n")
        self.layer_sketch.fill(Qt.transparent)
        self.sketch_strokes = [] 
        self.sketch_history = []
        self.sketch_redo_stack = []
        self.update()

class AppWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Inkstroke Studio")
        self.resize(1400, 900)
        self.setStyleSheet(DARK_STYLE)

        self.canvas_widget = Inkstroke()
        self.scroll = QScrollArea()
        self.scroll.setWidget(self.canvas_widget)
        self.scroll.setAlignment(Qt.AlignCenter)
        self.scroll.setStyleSheet("QScrollArea { border: none; background-color: #121212; }")
        
        self.scroll.verticalScrollBar().valueChanged.connect(self.update_nav_immediate)
        self.scroll.horizontalScrollBar().valueChanged.connect(self.update_nav_immediate)
        self.scroll.installEventFilter(self)
        self.scroll.viewport().installEventFilter(self)

        self.navigator = Navigator(self.canvas_widget, self.scroll)
        self.canvas_widget.nav_ref = self.navigator
        self.setup_right_panel()

        self.setup_menus()
        self.setup_top_toolbar()
        self.setup_side_toolbar()

        central = QWidget()
        layout = QHBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.scroll)
        layout.addWidget(self.right_panel)
        self.setCentralWidget(central)

        self.setStatusBar(QStatusBar(self))
        self.statusBar().showMessage("Ready", 2000)

        self.set_tool(TOOL_INK)
        self.btn_brush.setChecked(True)

        self.queue_timer = QTimer(self)
        self.queue_timer.timeout.connect(self.update_queue_monitor)
        self.queue_timer.start(300)
        self.update_queue_monitor()
        
        self.nav_timer = QTimer(self)
        self.nav_timer.timeout.connect(self.update_nav_background)
        self.nav_timer.start(250)

    def setup_right_panel(self):
        self.right_panel = QWidget()
        self.right_panel.setFixedWidth(260)
        panel_layout = QVBoxLayout(self.right_panel)
        panel_layout.setContentsMargins(8, 8, 8, 8)
        panel_layout.setSpacing(8)

        panel_layout.addWidget(self.navigator, 2)

        machine_panel = QFrame()
        machine_panel.setStyleSheet("QFrame { border: 1px solid #3d3d3d; border-radius: 10px; }")
        machine_layout = QVBoxLayout(machine_panel)
        machine_layout.setContentsMargins(8, 8, 8, 8)
        machine_layout.setSpacing(6)

        machine_layout.addWidget(QLabel("Machine"))
        machine_layout.addWidget(QLabel(f"Area: {DRAWING_WIDTH_MM:.0f} x {DRAWING_HEIGHT_MM:.0f} mm"))
        machine_layout.addWidget(QLabel("Origin: (0,0) bottom-left"))

        serial_row = QHBoxLayout()
        serial_row.setSpacing(6)
        self.serial_port = QLineEdit("COM3")
        self.serial_port.setPlaceholderText("COM")
        self.serial_port.setFixedWidth(62)
        serial_row.addWidget(self.serial_port)

        self.serial_baud = QSpinBox()
        self.serial_baud.setRange(9600, 2000000)
        self.serial_baud.setValue(115200)
        self.serial_baud.setSingleStep(9600)
        self.serial_baud.setFixedWidth(98)
        serial_row.addWidget(self.serial_baud)

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.setMinimumWidth(80)
        self.btn_connect.clicked.connect(self.toggle_serial_connection)
        serial_row.addWidget(self.btn_connect)
        machine_layout.addLayout(serial_row)

        self.lbl_serial_state = QLabel("Serial: disconnected")
        machine_layout.addWidget(self.lbl_serial_state)

        self.lbl_queue_state = QLabel("Pending commands: 0")
        machine_layout.addWidget(self.lbl_queue_state)

        self.queue_view = QPlainTextEdit()
        self.queue_view.setReadOnly(True)
        self.queue_view.setPlaceholderText("Pending serial commands will appear here")
        self.queue_view.setMinimumHeight(140)
        self.queue_view.document().setMaximumBlockCount(120)
        machine_layout.addWidget(self.queue_view)

        btn_clear_queue = QPushButton("Clear Pending")
        btn_clear_queue.clicked.connect(self.clear_pending_queue)
        machine_layout.addWidget(btn_clear_queue)

        self.btn_pause_queue = QPushButton("Pause Sending")
        self.btn_pause_queue.clicked.connect(self.toggle_pause_queue)
        machine_layout.addWidget(self.btn_pause_queue)

        panel_layout.addWidget(machine_panel, 3)

    def setup_menus(self):
        menubar = self.menuBar()
        logo = QLabel(" [INK] ")
        logo.setStyleSheet("font-weight: bold; color: #3498db; padding-left: 10px;")
        menubar.setCornerWidget(logo, Qt.TopLeftCorner)
        
        file_menu = menubar.addMenu("File")
        edit_menu = menubar.addMenu("Edit")
        view_menu = menubar.addMenu("View")

        action_new = QAction("New Canvas", self)
        action_new.setShortcut(QKeySequence.New)
        action_new.triggered.connect(self.new_canvas)
        file_menu.addAction(action_new)

        action_toggle_serial = QAction("Connect/Disconnect Serial", self)
        action_toggle_serial.setShortcut("Ctrl+Shift+S")
        action_toggle_serial.triggered.connect(self.toggle_serial_connection)
        file_menu.addAction(action_toggle_serial)

        file_menu.addSeparator()

        action_exit = QAction("Exit", self)
        action_exit.setShortcut(QKeySequence.Quit)
        action_exit.triggered.connect(self.close)
        file_menu.addAction(action_exit)
        
        action_undo = QAction("Undo", self)
        action_undo.setShortcut(QKeySequence.Undo) 
        action_undo.triggered.connect(self.canvas_widget.perform_undo)
        edit_menu.addAction(action_undo)

        action_redo = QAction("Redo", self)
        action_redo.setShortcut("Ctrl+Shift+Z")
        action_redo.triggered.connect(self.canvas_widget.perform_redo)
        edit_menu.addAction(action_redo)
        
        edit_menu.addSeparator()

        action_convert = QAction("Convert Sketch", self)
        action_convert.setShortcut("Ctrl+Shift+K")
        action_convert.setStatusTip("Convert visible sketch strokes into ink")
        action_convert.triggered.connect(self.canvas_widget.convert_sketch_to_ink)
        edit_menu.addAction(action_convert)

        action_clear_sketch = QAction("Clear Sketch Layer", self)
        action_clear_sketch.setShortcut("Ctrl+Shift+Backspace")
        action_clear_sketch.triggered.connect(self.clear_sketch_layer)
        edit_menu.addAction(action_clear_sketch)

        action_clear_ink = QAction("Clear Ink Layer", self)
        action_clear_ink.setShortcut("Ctrl+Alt+Backspace")
        action_clear_ink.triggered.connect(self.clear_ink_layer)
        edit_menu.addAction(action_clear_ink)

        action_zoom_in = QAction("Zoom In", self)
        action_zoom_in.setShortcut(QKeySequence.ZoomIn)
        action_zoom_in.triggered.connect(self.zoom_in)
        view_menu.addAction(action_zoom_in)

        action_zoom_out = QAction("Zoom Out", self)
        action_zoom_out.setShortcut(QKeySequence.ZoomOut)
        action_zoom_out.triggered.connect(self.zoom_out)
        view_menu.addAction(action_zoom_out)

        action_zoom_reset = QAction("Reset Zoom", self)
        action_zoom_reset.setShortcut("Ctrl+0")
        action_zoom_reset.triggered.connect(self.reset_zoom)
        view_menu.addAction(action_zoom_reset)

        action_fit = QAction("Fit Canvas", self)
        action_fit.setShortcut("Ctrl+9")
        action_fit.triggered.connect(self.fit_canvas)
        view_menu.addAction(action_fit)

        view_menu.addSeparator()

        self.action_show_navigator = QAction("Show Navigator", self)
        self.action_show_navigator.setCheckable(True)
        self.action_show_navigator.setChecked(True)
        self.action_show_navigator.toggled.connect(self.toggle_navigator)
        view_menu.addAction(self.action_show_navigator)

    def setup_top_toolbar(self):
        self.top_bar = QToolBar("Settings")
        self.top_bar.setMovable(False)
        self.addToolBar(Qt.TopToolBarArea, self.top_bar)
        
        btn_undo = QPushButton("↺")
        btn_undo.setToolTip("Undo (Ctrl+Z)")
        btn_undo.clicked.connect(self.canvas_widget.perform_undo)
        self.top_bar.addWidget(btn_undo)
        
        btn_redo = QPushButton("↻")
        btn_redo.setToolTip("Redo (Ctrl+Shift+Z)")
        btn_redo.clicked.connect(self.canvas_widget.perform_redo)
        self.top_bar.addWidget(btn_redo)
        
        self.top_bar.addSeparator()

        self.top_bar.addWidget(QLabel("  Stabilization:  "))
        self.slider_stab = QSlider(Qt.Horizontal)
        self.slider_stab.setRange(0, 95)
        self.slider_stab.setValue(10)
        self.slider_stab.setFixedWidth(120)
        self.top_bar.addWidget(self.slider_stab)

        self.spin_stab = QDoubleSpinBox()
        self.spin_stab.setRange(0.00, 0.95)
        self.spin_stab.setSingleStep(0.01)
        self.spin_stab.setDecimals(2)
        self.spin_stab.setValue(0.10)
        self.spin_stab.setPrefix("v ")
        self.spin_stab.setFixedWidth(80)
        self.top_bar.addWidget(self.spin_stab)

        self.slider_stab.valueChanged.connect(self.update_stabilization_from_slider)
        self.spin_stab.valueChanged.connect(self.update_stabilization_from_spin)
        
        self.top_bar.addSeparator()

        self.lbl_eraser = QLabel("  Eraser (mm):  ")
        self.top_bar.addWidget(self.lbl_eraser)
        
        self.slider_eraser = QSlider(Qt.Horizontal)
        self.slider_eraser.setRange(10, 300)
        self.slider_eraser.setValue(60)
        self.slider_eraser.setFixedWidth(150)
        self.slider_eraser.valueChanged.connect(self.update_eraser_size)
        self.top_bar.addWidget(self.slider_eraser)

    def setup_side_toolbar(self):
        side_bar = QToolBar("Tools")
        side_bar.setMovable(False)
        side_bar.setOrientation(Qt.Vertical)
        self.addToolBar(Qt.LeftToolBarArea, side_bar)
        
        self.tool_group = QButtonGroup(self)

        def make_icon(tool_name):
            pix = QPixmap(24, 24)
            pix.fill(Qt.transparent)
            p = QPainter(pix)
            p.setRenderHint(QPainter.Antialiasing)
            if tool_name == TOOL_INK:
                p.setBrush(QColor(15, 15, 15))
                p.setPen(QPen(QColor(15, 15, 15), 2))
                p.drawEllipse(QPointF(8, 8), 3.5, 3.5)
                p.drawLine(11, 11, 19, 19)
            elif tool_name == TOOL_SKETCH:
                p.setPen(QPen(QColor(100, 149, 237), 3))
                p.drawLine(4, 18, 19, 5)
                p.setBrush(QColor(100, 149, 237))
                p.drawPolygon([QPoint(18, 4), QPoint(21, 3), QPoint(20, 6)])
            else:
                p.setPen(QPen(QColor(220, 220, 220), 2))
                p.setBrush(QColor(220, 220, 220))
                p.drawRoundedRect(5, 8, 14, 8, 2, 2)
                p.setPen(QPen(QColor(120, 120, 120), 1))
                p.drawLine(5, 14, 19, 14)
            p.end()
            return QIcon(pix)
        
        self.btn_brush = QPushButton("")
        self.btn_brush.setIcon(make_icon(TOOL_INK))
        self.btn_brush.setIconSize(QSize(22, 22))
        self.btn_brush.setToolTip("Ink Brush [Hotkey: B]")
        self.btn_brush.setCheckable(True)
        self.btn_brush.clicked.connect(lambda: self.set_tool(TOOL_INK))
        self.tool_group.addButton(self.btn_brush)
        side_bar.addWidget(self.btn_brush)
        
        self.btn_sketch = QPushButton("")
        self.btn_sketch.setIcon(make_icon(TOOL_SKETCH))
        self.btn_sketch.setIconSize(QSize(22, 22))
        self.btn_sketch.setToolTip("Sketch Pencil [Hotkey: P]")
        self.btn_sketch.setCheckable(True)
        self.btn_sketch.clicked.connect(lambda: self.set_tool(TOOL_SKETCH))
        self.tool_group.addButton(self.btn_sketch)
        side_bar.addWidget(self.btn_sketch)
        
        self.btn_eraser = QPushButton("")
        self.btn_eraser.setIcon(make_icon(TOOL_ERASER_SKETCH))
        self.btn_eraser.setIconSize(QSize(22, 22))
        self.btn_eraser.setToolTip("Sketch Eraser [Hotkey: E]")
        self.btn_eraser.setCheckable(True)
        self.btn_eraser.clicked.connect(lambda: self.set_tool(TOOL_ERASER_SKETCH))
        self.tool_group.addButton(self.btn_eraser)
        side_bar.addWidget(self.btn_eraser)

    def set_tool(self, tool_name):
        self.canvas_widget.current_tool = tool_name
        
        is_eraser = (tool_name == TOOL_ERASER_SKETCH)
        self.lbl_eraser.setVisible(is_eraser)
        self.slider_eraser.setVisible(is_eraser)
        
        self.canvas_widget.setCursor(Qt.BlankCursor)
        self.canvas_widget.update()

    def update_stabilization_from_slider(self, value):
        st = value / 100.0
        self.canvas_widget.stabilization = st
        self.spin_stab.blockSignals(True)
        self.spin_stab.setValue(st)
        self.spin_stab.blockSignals(False)

    def update_stabilization_from_spin(self, value):
        self.canvas_widget.stabilization = value
        self.slider_stab.blockSignals(True)
        self.slider_stab.setValue(int(round(value * 100.0)))
        self.slider_stab.blockSignals(False)
        
    def update_eraser_size(self, value):
        self.canvas_widget.eraser_size_mm = value / 10.0
        self.canvas_widget.update()

    def toggle_serial_connection(self):
        bridge = self.canvas_widget.machine_bridge
        if bridge.serial_conn and bridge.serial_conn.is_open:
            bridge.disconnect_serial()
            self.btn_connect.setText("Connect")
            self.statusBar().showMessage("Serial disconnected", 3000)
            self.update_queue_monitor()
            return

        ok, msg = bridge.connect_serial(self.serial_port.text().strip(), self.serial_baud.value())
        if ok:
            self.btn_connect.setText("Disconnect")
            self.statusBar().showMessage(msg, 4000)
        else:
            QMessageBox.warning(self, "Serial", msg)

        self.update_queue_monitor()

    def update_nav_immediate(self):
        self.navigator.update()
        
    def update_nav_background(self):
        if self.navigator.isVisible():
            self.navigator.update()

    def clear_pending_queue(self):
        self.canvas_widget.machine_bridge.clear_all()
        self.update_queue_monitor()
        self.statusBar().showMessage("Pending command queue cleared", 2500)

    def update_queue_monitor(self):
        bridge = self.canvas_widget.machine_bridge
        pending = bridge.get_pending_count()
        connected = bridge.serial_conn and bridge.serial_conn.is_open
        paused = bridge.is_paused()
        pause_requested = bridge.is_pause_requested()

        if connected:
            serial_text = f"Serial: {bridge.connected_port} @ {bridge.connected_baud}"
        else:
            serial_text = "Serial: disconnected"

        self.lbl_serial_state.setText(serial_text)
        queue_state = f"Pending commands: {pending}"
        if paused:
            queue_state += " (PAUSED)"
        elif pause_requested:
            queue_state += " (Pausing at next lift...)"
        self.lbl_queue_state.setText(queue_state)

        if paused:
            self.btn_pause_queue.setText("Resume Sending")
        elif pause_requested:
            self.btn_pause_queue.setText("Pausing...")
        else:
            self.btn_pause_queue.setText("Pause Sending")

        if pending:
            preview = "\n".join(bridge.get_queue_preview(40))
        else:
            preview = "Queue empty"
        self.queue_view.setPlainText(preview)

    def toggle_pause_queue(self):
        bridge = self.canvas_widget.machine_bridge
        if bridge.is_paused():
            bridge.resume_queue()
            self.statusBar().showMessage("Queue resumed", 2500)
        else:
            bridge.request_pause()
            self.statusBar().showMessage("Pause requested: waiting for pen-lift boundary", 3000)
        self.update_queue_monitor()

    def clear_sketch_layer(self):
        self.canvas_widget.save_sketch_state()
        self.canvas_widget.layer_sketch.fill(Qt.transparent)
        self.canvas_widget.sketch_strokes = []
        self.canvas_widget.update()
        self.navigator.update()
        self.statusBar().showMessage("Sketch layer cleared", 2500)

    def clear_ink_layer(self):
        self.canvas_widget.layer_ink.fill(Qt.transparent)
        self.canvas_widget.layer_feedback.fill(Qt.transparent)
        self.canvas_widget.update()
        self.navigator.update()
        self.statusBar().showMessage("Ink layer cleared", 2500)

    def new_canvas(self):
        answer = QMessageBox.question(
            self,
            "New Canvas",
            "Clear sketch, ink, and queued commands?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return

        self.canvas_widget.layer_sketch.fill(Qt.transparent)
        self.canvas_widget.layer_ink.fill(Qt.transparent)
        self.canvas_widget.layer_feedback.fill(Qt.transparent)
        self.canvas_widget.sketch_strokes = []
        self.canvas_widget.sketch_history = []
        self.canvas_widget.sketch_redo_stack = []
        self.canvas_widget.current_stroke_data = []
        self.canvas_widget.machine_bridge.clear_all()
        self.canvas_widget.update()
        self.navigator.update()
        self.update_queue_monitor()
        self.statusBar().showMessage("New canvas ready", 3000)

    def closeEvent(self, event):
        bridge = self.canvas_widget.machine_bridge
        bridge.clear_all()
        bridge.disconnect_serial()
        super().closeEvent(event)

    def toggle_navigator(self, visible):
        self.navigator.setVisible(visible)

    def _viewport_center(self):
        viewport = self.scroll.viewport()
        return QPointF(viewport.width() / 2.0, viewport.height() / 2.0)

    def zoom_at_cursor(self, factor, cursor_pos):
        old_zoom = self.canvas_widget.zoom_level
        new_zoom = max(0.05, min(old_zoom * factor, 8.0))
        if abs(new_zoom - old_zoom) < 0.0001:
            return

        h_bar = self.scroll.horizontalScrollBar()
        v_bar = self.scroll.verticalScrollBar()

        canvas_x = (h_bar.value() + cursor_pos.x()) / old_zoom
        canvas_y = (v_bar.value() + cursor_pos.y()) / old_zoom

        self.canvas_widget.set_zoom(new_zoom)

        h_bar.setValue(int(canvas_x * new_zoom - cursor_pos.x()))
        v_bar.setValue(int(canvas_y * new_zoom - cursor_pos.y()))
        self.navigator.update()

    def zoom_in(self):
        self.zoom_at_cursor(1.1, self._viewport_center())

    def zoom_out(self):
        self.zoom_at_cursor(0.9, self._viewport_center())

    def reset_zoom(self):
        current = self.canvas_widget.zoom_level
        if current <= 0:
            return
        self.zoom_at_cursor(0.25 / current, self._viewport_center())

    def fit_canvas(self):
        viewport = self.scroll.viewport().size()
        if viewport.width() <= 0 or viewport.height() <= 0:
            return
        target_zoom = min(viewport.width() / WIDTH, viewport.height() / HEIGHT)
        current = self.canvas_widget.zoom_level
        if current <= 0:
            return
        self.zoom_at_cursor(target_zoom / current, self._viewport_center())

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_B:
            self.btn_brush.setChecked(True)
            self.set_tool(TOOL_INK)
            return
        elif event.key() == Qt.Key_P:
            self.btn_sketch.setChecked(True)
            self.set_tool(TOOL_SKETCH)
            return
        elif event.key() == Qt.Key_E:
            self.btn_eraser.setChecked(True)
            self.set_tool(TOOL_ERASER_SKETCH)
            return
        elif event.key() == Qt.Key_BracketLeft:
            self.slider_eraser.setValue(self.slider_eraser.value() - 5)
            return
        elif event.key() == Qt.Key_BracketRight:
            self.slider_eraser.setValue(self.slider_eraser.value() + 5)
            return
        elif event.key() == Qt.Key_Delete:
            if self.canvas_widget.current_tool == TOOL_SKETCH:
                 self.canvas_widget.save_sketch_state()
                 self.canvas_widget.layer_sketch.fill(Qt.transparent)
                 self.canvas_widget.sketch_strokes = []
                 self.canvas_widget.update()
                 self.navigator.update()
                 return

        super().keyPressEvent(event)

    def eventFilter(self, source, event):
        if event.type() == QWheelEvent.Type.Wheel:
            modifiers = event.modifiers()
            angle = event.angleDelta().y()
            if angle == 0:
                angle = event.pixelDelta().y()

            if angle == 0:
                return super().eventFilter(source, event)

            if source is self.scroll.viewport():
                cursor_pos = event.position()
            else:
                mapped = self.scroll.viewport().mapFrom(self.scroll, event.position().toPoint())
                cursor_pos = QPointF(mapped)

            if modifiers & Qt.ControlModifier:
                factor = 1.12 if angle > 0 else 0.89
                self.zoom_at_cursor(factor, cursor_pos)
                return True
            elif modifiers & Qt.ShiftModifier:
                h_bar = self.scroll.horizontalScrollBar()
                h_bar.setValue(h_bar.value() - angle)
                return True
        return super().eventFilter(source, event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = AppWindow()
    window.show()
    sys.exit(app.exec())