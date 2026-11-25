"""SML Arm hardware test console - hardware control workspace.

Uses the existing SerialManager and CommandClient interfaces. No motion is
issued on startup or on connect. Angles are firmware step estimates, not encoder
measurements. Software STOP/ESTOP are not hardware safety functions.
"""
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from time import monotonic
import math
import re

from PySide6.QtCore import Qt, QTimer, QRectF, QPointF
from PySide6.QtGui import QColor, QTextCharFormat, QTextCursor, QPainter, QPen, QFont, QPalette
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QLabel, QPushButton, QComboBox, QLineEdit, QCheckBox, QPlainTextEdit,
    QTabWidget, QSplitter, QMessageBox, QFileDialog, QSizePolicy, QFrame, QApplication,
)
from serial_manager import SerialManager
from command_client import CommandClient, JointState
from appearance import AppearanceManager, ThemeDialog, PRESETS, LOG_COLORS, stylesheet, readable

JOINT_NAMES = {1: 'Base', 2: 'Shoulder', 3: 'Elbow', 4: 'Wrist 1',
               5: 'Wrist 2', 6: 'Tool'}
LINKED_JOINTS = tuple(JOINT_NAMES)  # Availability is confirmed by STATUS.
DEFAULTS = {'PPR': '1600', 'GEAR': '100', 'INVERT': '0', 'MIN': '-180',
            'MAX': '180', 'SPEED': '9', 'ACCEL': '6.75', 'LINEAR': '0.45'}
CONFIG_COMMANDS = {'PPR', 'GEAR', 'INVERT', 'LIMIT', 'SPEED', 'ACCEL', 'LINEAR'}
MOTION_COMMANDS = {'GOTO', 'JREL'}
TIMEOUT = 3.0


def numeric(text, integer=False):
    """Validate explicitly; never substitute a default for invalid input."""
    text = text.strip()
    if integer:
        if not re.fullmatch(r'[0-9]+', text):
            raise ValueError('Enter a positive whole number.')
        value = int(text)
        if not 1 <= value <= 4294967295:
            raise ValueError('PPR must be between 1 and 4294967295.')
        return str(value)
    if not re.fullmatch(r'[+-]?(?:\d+(?:\.\d*)?|\.\d+)', text, re.ASCII):
        raise ValueError('Enter a number, using a decimal point if needed.')
    value = float(text)
    if not math.isfinite(value) or abs(value) > 3.402823e38:
        raise ValueError('Number is outside the supported range.')
    return text


@dataclass
class Reply:
    """One normal request at a time: firmware has no request identifiers."""
    verb: str
    joint: int | None
    values: tuple = ()
    quiet: bool = False
    reload: bool = False
    deadline: float = field(default_factory=lambda: monotonic() + TIMEOUT)
    phase: int = 0
    data: dict = field(default_factory=dict)
    draft_snapshot: dict = field(default_factory=dict)

    def feed(self, line):
        """Return 'more', 'ok', 'error', or None for an unrelated line."""
        if self.phase == 0:
            expected = '[OK] PONG' if self.verb == 'PING' else f'[OK] {self.verb}'
            if self.joint is not None:
                expected += f' J{self.joint}'
            if line.startswith('[ERROR]'):
                return 'error'
            if line != expected and not (self.verb == 'ANGLE' and line.startswith(expected + ' ')):
                return None
            if self.verb in ('STATUS', 'CONFIG'):
                self.phase = 1
                return 'more'
            if self.verb == 'ANGLE':
                self.data['ANGLE'] = line.rsplit(' ', 1)[-1]
            return 'ok'
        if line.startswith('[ERROR]'):
            return 'error'
        fields = CommandClient.parse_fields(line)
        if self.verb == 'STATUS':
            if self.phase == 1 and {'AVAILABLE', 'RUNNING', 'POSITION_VALID'} <= fields.keys():
                if any(fields[k] not in ('0', '1') for k in ('AVAILABLE', 'RUNNING', 'POSITION_VALID')):
                    return 'error'
                self.data.update(fields)
                self.phase = 2
                return 'more'
            if self.phase == 2:
                if 'ANGLE' in fields and self.data['POSITION_VALID'] == '1':
                    self.data['ANGLE'] = fields['ANGLE']
                    return 'ok'
                if line == f'[WARN] POSITION_UNKNOWN J{self.joint}' and self.data['POSITION_VALID'] == '0':
                    return 'ok'
            return None
        if self.verb == 'CONFIG':
            keys = ({'PPR', 'GEAR', 'INVERT'}, {'MIN', 'MAX'}, {'SPEED', 'ACCEL', 'LINEAR'})[self.phase - 1]
            if keys <= fields.keys():
                self.data.update({k: fields[k] for k in keys})
                self.phase += 1
                return 'ok' if self.phase == 4 else 'more'
        return None


class AngleGauge(QWidget):
    """Read-only gauge of the firmware estimate, scaled to confirmed limits."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(104)
        self.setMaximumHeight(175)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.colors = PRESETS['Graphite Pro']
        self.angle = None
        self.limits = None

    def set_data(self, angle, limits):
        if (angle, limits) != (self.angle, self.limits):
            self.angle, self.limits = angle, limits
            self.update()

    def set_colors(self, colors):
        self.colors = colors
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        c = self.colors
        w, h = self.width(), self.height()
        r = min(w * .36, (h - 34) / 1.65)
        if r < 12:
            return
        cx, cy = w / 2, r + 9
        arc = QRectF(cx-r, cy-r, 2*r, 2*r)
        def pen(color, width):
            a = QPen(QColor(color), width)
            a.setCapStyle(Qt.PenCapStyle.RoundCap)
            return a
        p.setPen(pen(c['surface'], 9))
        p.drawArc(arc, 210*16, -240*16)
        for i in range(13):
            a = math.radians(210-240*i/12)
            p.setPen(pen(c['muted'], 1.2))
            p.drawLine(QPointF(cx+(r-10)*math.cos(a),cy-(r-10)*math.sin(a)),
                       QPointF(cx+(r-16)*math.cos(a),cy-(r-16)*math.sin(a)))
        if self.limits is not None and self.angle is not None:
            low, high = self.limits
            frac = max(0., min(1., (self.angle-low)/(high-low)))
            p.setPen(pen(c['accent'], 9))
            p.drawArc(arc, 210*16, round(-240*16*frac))
            a = math.radians(210-240*frac)
            p.setPen(pen(c['accent'], 2.5))
            p.drawLine(QPointF(cx,cy),QPointF(cx+(r-23)*math.cos(a),cy-(r-23)*math.sin(a)))
            p.setBrush(QColor(c['accent']))
            p.drawEllipse(QPointF(cx,cy),4,4)
        p.setPen(QColor(c['muted']))
        font = QFont('Segoe UI'); font.setPixelSize(11); p.setFont(font)
        if self.limits is not None:
            low, high = self.limits
            p.drawText(QRectF(0,h-18,w/2-5,18), Qt.AlignmentFlag.AlignCenter, f'{low:g}°')
            p.drawText(QRectF(w/2+5,h-18,w/2-5,18), Qt.AlignmentFlag.AlignCenter, f'{high:g}°')
        else:
            p.drawText(QRectF(0,h-18,w,18), Qt.AlignmentFlag.AlignCenter,'Limits not confirmed')


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('SML Arm Studio | Hardware Control')
        self.appearance = AppearanceManager()
        self.theme_dialog = None
        QApplication.setStyle('Fusion')
        self.setMinimumSize(1100, 730)
        screen = self.screen()
        if screen is not None:
            area = screen.availableGeometry()
            self.resize(min(1200, area.width() - 24), min(850, area.height() - 45))
        else:
            self.resize(1200, 850)
        # Use the real backend; no fake connection state or guessed methods.
        self.serial = SerialManager(self)
        self.client = CommandClient(self.serial)
        self.connected = False
        self.ready = False
        self.protocol_fault = False
        self.selected_joint = 1
        self.states = {j: JointState() for j in JOINT_NAMES}
        self.status_seen = set()
        self.configs = {}
        self.dirty = {j: set() for j in JOINT_NAMES}
        self.drafts = {j: DEFAULTS.copy() for j in JOINT_NAMES}
        self.pending = None
        self.stops = {}  # STOP/ESTOP can bypass a pending normal request.
        self.need_status = set()
        self.need_config = {}  # joint -> force reload editable fields
        self.tracking = set()
        self.next_poll = 0.0
        self.poll_cursor = 0
        self._build_ui()
        QApplication.styleHints().colorSchemeChanged.connect(self._system_theme_changed)
        self.serial.received.connect(self._on_received)
        self.serial.transmitted.connect(self._on_transmitted)
        self.serial.connection_changed.connect(self._on_connection)
        self.serial.failure.connect(self._on_failure)
        self.serial.finished.connect(self._on_thread_finished)
        self.timer = QTimer(self)
        self.timer.setInterval(200)
        self.timer.timeout.connect(self._tick)
        self.refresh_ports()
        self._render()
        self._log('INFO', 'Ready. Default values are local only; connect to read firmware.')

    # ---------------- UI: native, compact widgets ----------------
    def _button(self, text, action, color=None):
        b = QPushButton(text)
        b.setMinimumHeight(34)
        b.clicked.connect(action)
        if color:
            b.setProperty('role', {'#f3d4e6': 'primary', '#ffe2aa': 'stop', '#ffc2c5': 'estop'}.get(color, 'primary'))
        return b

    def _number(self, default):
        edit = QLineEdit(str(default))
        edit.setMinimumHeight(34)
        edit.setMaximumWidth(170)
        edit.setMaxLength(32)
        edit.setAlignment(Qt.AlignmentFlag.AlignLeft)
        edit.setToolTip('Type a number. Apply or Move sends it to firmware.')
        return edit

    def _style(self, palette=None):
        self.active_palette = palette or self.appearance.palette()
        p = self.active_palette
        palette = QPalette(self.palette())
        for role, color in ((QPalette.ColorRole.Window, p['bg']),
                            (QPalette.ColorRole.WindowText, p['text']),
                            (QPalette.ColorRole.Base, p['panel']),
                            (QPalette.ColorRole.AlternateBase, p['surface']),
                            (QPalette.ColorRole.Text, p['text']),
                            (QPalette.ColorRole.Button, p['surface']),
                            (QPalette.ColorRole.ButtonText, p['text']),
                            (QPalette.ColorRole.Highlight, p['accent']),
                            (QPalette.ColorRole.HighlightedText, readable(p['accent'])),
                            (QPalette.ColorRole.PlaceholderText, p['muted']),
                            (QPalette.ColorRole.ToolTipBase, p['panel']),
                            (QPalette.ColorRole.ToolTipText, p['text'])):
            palette.setColor(role, QColor(color))
        self.setPalette(palette)
        self.setStyleSheet(stylesheet(p))
        if hasattr(self, 'gauge'):
            self.gauge.set_colors(self.active_palette)

    def _system_theme_changed(self, scheme):
        if self.appearance.name == 'System' and self.theme_dialog is None:
            self._style()

    def _change_theme(self, name):
        if self.theme_dialog is not None and self.theme_dialog.isVisible():
            self.theme_dialog.reject()
        self.appearance.save(name)
        self._style()

    def _custom_theme(self):
        if self.theme_dialog is not None and self.theme_dialog.isVisible():
            self.theme_dialog.raise_()
            self.theme_dialog.activateWindow()
            return
        dialog = ThemeDialog(self.appearance, self)
        self.theme_dialog = dialog
        dialog.preview.connect(self._style)
        dialog.saved.connect(self._custom_saved)
        self.theme_select.setEnabled(False)
        dialog.finished.connect(self._custom_closed)
        dialog.show()

    def _custom_saved(self, palette):
        self.theme_select.blockSignals(True)
        self.theme_select.setCurrentText('Custom')
        self.theme_select.blockSignals(False)
        self._style(palette)

    def _custom_closed(self, result):
        self.theme_select.setEnabled(True)
        dialog, self.theme_dialog = self.theme_dialog, None
        if dialog is not None:
            dialog.deleteLater()

    def _build_ui(self):
        central = QWidget()
        central.setObjectName("root")
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 10, 12, 8)
        root.setSpacing(9)
        self._style()
        branding = QHBoxLayout()
        brand = QLabel('SML ARM STUDIO')
        brand.setProperty('role', 'brand')
        branding.addWidget(brand)
        branding.addStretch()
        subtitle = QLabel('ROBOTICS ENGINEERING WORKSPACE')
        subtitle.setProperty('role', 'muted')
        branding.addWidget(subtitle)
        self.theme_select = QComboBox()
        self.theme_select.addItems([*PRESETS, 'System', 'Custom'])
        self.theme_select.setCurrentText(self.appearance.name)
        self.theme_select.setMinimumWidth(160)
        self.theme_select.currentTextChanged.connect(self._change_theme)
        branding.addWidget(self.theme_select)
        branding.addWidget(self._button('Appearance…', self._custom_theme))
        root.addLayout(branding)
        header_frame = QFrame()
        header_frame.setProperty('role', 'panel')
        header = QHBoxLayout(header_frame)
        header.setContentsMargins(10, 6, 10, 6)
        self.port_box = QComboBox()
        self.port_box.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.port_box.setMinimumContentsLength(12)
        self.port_box.setMaximumWidth(320)
        header.addWidget(QLabel('Serial'))
        header.addWidget(self.port_box, 1)
        self.ports_btn = self._button('Refresh ports', self.refresh_ports)
        header.addWidget(self.ports_btn)
        header.addWidget(QLabel('115200 baud'))
        self.connect_btn = self._button('Connect', self._toggle_connection, '#f3d4e6')
        header.addWidget(self.connect_btn)
        self.connection_label = QLabel('Disconnected')
        header.addWidget(self.connection_label, 1)
        self.ping_btn = self._button('PING', lambda: self._request('PING'))
        self.stop_all_btn = self._button('STOP ALL', lambda: self._stop('STOP', 'ALL'), '#ffe2aa')
        self.estop_all_btn = self._button('ESTOP ALL', lambda: self._stop('ESTOP', 'ALL'), '#ffc2c5')
        for b in (self.ping_btn, self.stop_all_btn, self.estop_all_btn):
            header.addWidget(b)
        root.addWidget(header_frame)

        self.splitter = QSplitter(Qt.Orientation.Vertical)
        self.splitter.setChildrenCollapsible(False)
        root.addWidget(self.splitter, 1)
        workspace = QWidget()
        workspace.setObjectName("workspace")
        upper = QVBoxLayout(workspace)
        upper.setContentsMargins(0, 0, 0, 0)
        upper.setSpacing(5)
        self.selector = QWidget()
        selector = QVBoxLayout(self.selector)
        selector.setContentsMargins(0, 0, 0, 0)
        heading = QLabel('SELECT JOINT')
        heading.setProperty('role', 'heading')
        selector.addWidget(heading)
        self.selector.setFixedWidth(155)
        self.joint_buttons = {}
        for j, name in JOINT_NAMES.items():
            b = self._button(f'J{j} {name}', lambda checked=False, jid=j: self._select_joint(jid))
            b.setCheckable(True)
            b.setEnabled(j in LINKED_JOINTS)
            b.setChecked(j == self.selected_joint)
            if j not in LINKED_JOINTS:
                b.setToolTip('Not linked to firmware')
            b.setMinimumHeight(39)
            selector.addWidget(b)
            self.joint_buttons[j] = b
        selector.addStretch()
        body = QHBoxLayout()
        body.addWidget(self.selector)
        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_control(), 'Control')
        self.tabs.addTab(self._build_multi(), 'Multi-Joint')
        self.tabs.addTab(self._build_config(), 'Configure')
        self.tabs.addTab(self._build_monitor(), 'Monitor')
        self.tabs.currentChanged.connect(lambda idx: self.selector.setVisible(idx in (0, 2)))
        body.addWidget(self.tabs, 1)
        upper.addLayout(body, 1)
        self.splitter.addWidget(workspace)
        self.splitter.addWidget(self._build_terminal())
        self.splitter.setStretchFactor(0, 3)
        self.splitter.setStretchFactor(1, 2)
        self.splitter.setSizes([450, 180])
        self.statusBar().showMessage('6-joint hardware test | Software ESTOP is not a physical E-stop')

    def _build_control(self):
        page = QWidget()
        root = QHBoxLayout(page)
        root.setContentsMargins(12, 8, 12, 8)
        root.setSpacing(12)
        status = QGroupBox('Joint position')
        left = QVBoxLayout(status)
        left.addWidget(QLabel('Current angle · firmware estimate'))
        self.angle_label = QLabel('— °')
        self.angle_label.setProperty('role', 'angle')
        self.angle_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left.addWidget(self.angle_label)
        self.state_label = QLabel('Offline')
        self.state_label.setWordWrap(True)
        self.state_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        left.addWidget(self.state_label)
        self.gauge = AngleGauge()
        self.gauge.set_colors(self.active_palette)
        left.addWidget(self.gauge, 1)
        self.zero_btn = self._button('Set software zero', lambda: self._zero(self.selected_joint))
        self.read_status = self._button('Read status', lambda: self._request('STATUS', self.selected_joint))
        self.read_config = self._button('Read config', lambda: self._request('CONFIG', self.selected_joint, reload=True))
        for button in (self.zero_btn, self.read_status, self.read_config):
            left.addWidget(button)
        root.addWidget(status, 2)
        right = QVBoxLayout()
        box = QGroupBox('Motion control')
        g = QGridLayout(box)
        self.target = self._number('0')
        self.relative = self._number('1')
        self.move_btn = self._button('Move absolute', lambda: self._move('GOTO', self.selected_joint, self.target), '#f3d4e6')
        self.relative_btn = self._button('Move relative', lambda: self._move('JREL', self.selected_joint, self.relative))
        for row, (label, edit, button) in enumerate((('Target angle (°)', self.target, self.move_btn), ('Relative angle (°)', self.relative, self.relative_btn))):
            g.addWidget(QLabel(label), row, 0)
            g.addWidget(edit, row, 1)
            g.addWidget(button, row, 2)
        g.setColumnStretch(2, 1)
        g.setHorizontalSpacing(10)
        g.setVerticalSpacing(8)
        right.addWidget(box)
        jog = QGroupBox('Jog · one step per click')
        v = QVBoxLayout(jog)
        row = QHBoxLayout()
        self.jog_step = self._number('1')
        self.jog_step.setMaximumWidth(80)
        row.addWidget(QLabel('Step (°)'))
        row.addWidget(self.jog_step)
        for n in ('0.1', '1', '5', '10'):
            row.addWidget(self._button(n, lambda checked=False, value=n: self.jog_step.setText(value)), 1)
        v.addLayout(row)
        row = QHBoxLayout()
        self.jog_minus = self._button('← Jog −', lambda: self._jog(-1))
        self.jog_plus = self._button('Jog + →', lambda: self._jog(1))
        row.addWidget(self.jog_minus, 1)
        row.addWidget(self.jog_plus, 1)
        v.addLayout(row)
        right.addWidget(jog)
        row = QHBoxLayout()
        self.stop_btn = self._button('STOP', lambda: self._stop('STOP', self.selected_joint), '#ffe2aa')
        self.estop_btn = self._button('ESTOP', lambda: self._stop('ESTOP', self.selected_joint), '#ffc2c5')
        row.addWidget(self.stop_btn, 1)
        row.addWidget(self.estop_btn, 1)
        right.addLayout(row)
        right.addStretch()
        root.addLayout(right, 3)
        return page

    def _build_multi(self):
        page = QWidget()
        root = QVBoxLayout(page)
        hint = QLabel('Independent profiles · Group starts multiple joints without waiting; finish times may differ.')
        hint.setProperty('role', 'muted')
        root.addWidget(hint)
        g = QGridLayout()
        self.multi = {}
        for col, text in enumerate(('Joint', 'Current angle', 'Target (°)', 'State', 'Action')):
            heading = QLabel(text)
            heading.setProperty('role','tableHeader')
            g.addWidget(heading, 0, col)
        for j, name in JOINT_NAMES.items():
            angle, target, state = QLabel('—'), self._number('0'), QLabel('Offline')
            b = self._button(f'Move J{j}', lambda checked=False, jid=j: self._move('GOTO', jid, self.multi[jid][1]))
            axis = QLabel(f'J{j}  {name}')
            axis.setProperty('role','heading')
            for col, w in enumerate((axis, angle, target, state, b)):
                g.addWidget(w, j, col)
            self.multi[j] = (angle, target, state, b)
        g.setColumnStretch(0, 2)
        g.setColumnStretch(1, 2)
        g.setColumnStretch(2, 1)
        g.setColumnStretch(3, 2)
        g.setColumnStretch(4, 2)
        g.setVerticalSpacing(6)
        root.addLayout(g)
        self.multi_refresh = self._button('Refresh linked joints', self._refresh_all)
        row = QHBoxLayout()
        row.addWidget(self.multi_refresh)
        self.group_btn = self._button('Move all 6 (independent)', self._move_all, '#f3d4e6')
        row.addWidget(self.group_btn)
        root.addLayout(row)
        root.addStretch(1)
        return page

    def _build_config(self):
        page = QWidget()
        root = QVBoxLayout(page)
        self.config_note = QLabel('Local defaults — not yet read from firmware')
        self.config_note.setWordWrap(True)
        root.addWidget(self.config_note)
        columns = QHBoxLayout()
        self.config_fields = {}
        self.config_buttons = {}
        for title, rows in (('Mechanical', [('PPR', 'Pulses / motor revolution'), ('GEAR', 'Gear ratio (:1)')]),
                            ('Motion', [('SPEED', 'Speed (°/s)'), ('ACCEL', 'Acceleration (°/s²)'), ('LINEAR', 'Linear acceleration (°)')]),
                            ('Software limits', [('MIN', 'Minimum angle (°)'), ('MAX', 'Maximum angle (°)')])):
            group = QGroupBox(title)
            v = QVBoxLayout(group)
            v.setSpacing(5)
            for key, label in rows:
                v.addWidget(QLabel(label))
                row = QHBoxLayout()
                edit = self._number(DEFAULTS[key])
                edit.textEdited.connect(lambda text, k=key: self._edit_config(k, text))
                self.config_fields[key] = edit
                row.addWidget(edit, 1)
                if key not in ('MIN', 'MAX'):
                    button = self._button('Apply', lambda checked=False, k=key: self._apply(k))
                    self.config_buttons[key] = button
                    row.addWidget(button)
                v.addLayout(row)
            if title == 'Mechanical':
                self.invert = QCheckBox('Invert direction')
                self.invert.clicked.connect(lambda checked: self._edit_config('INVERT', '1' if checked else '0'))
                row = QHBoxLayout()
                row.addWidget(self.invert, 1)
                button = self._button('Apply', lambda: self._apply('INVERT'))
                self.config_buttons['INVERT'] = button
                row.addWidget(button)
                v.addLayout(row)
            if title == 'Software limits':
                self.config_buttons['LIMIT'] = self._button('Apply limits', lambda: self._apply('LIMIT'), '#f3d4e6')
                v.addWidget(self.config_buttons['LIMIT'])
            v.addStretch()
            columns.addWidget(group, 1)
        root.addLayout(columns, 1)
        row = QHBoxLayout()
        note = QLabel('Changes are sent only when you press Apply.')
        note.setWordWrap(True)
        row.addWidget(note, 1)
        self.reload_btn = self._button('Reload from firmware', lambda: self._request('CONFIG', self.selected_joint, reload=True))
        row.addWidget(self.reload_btn)
        root.addLayout(row)
        return page

    def _build_monitor(self):
        page = QWidget()
        root = QVBoxLayout(page)
        grid = QGridLayout()
        self.monitor = {}
        for j, name in JOINT_NAMES.items():
            card = QGroupBox(f'J{j} · {name}')
            v = QVBoxLayout(card)
            angle = QLabel('—')
            angle.setProperty('role', 'monitorAngle')
            label = QLabel('Offline')
            label.setWordWrap(True)
            label.setProperty('role', 'muted')
            v.addWidget(angle)
            v.addWidget(label)
            self.monitor[j] = (angle, label)
            grid.addWidget(card, (j - 1) // 3, (j - 1) % 3)
        root.addLayout(grid, 1)
        self.monitor_refresh = self._button('Refresh linked joints', self._refresh_all)
        root.addWidget(self.monitor_refresh)
        return page

    def _build_terminal(self):
        box = QWidget()
        box.setMinimumHeight(150)
        layout = QVBoxLayout(box)
        layout.setContentsMargins(0, 2, 0, 0)
        layout.setSpacing(4)
        row = QHBoxLayout()
        heading = QLabel('SERIAL TERMINAL')
        heading.setProperty('role', 'terminalTitle')
        row.addWidget(heading)
        row.addStretch()
        self.auto_refresh = QCheckBox('Auto refresh')
        self.auto_refresh.setChecked(False)
        self.interval = QComboBox()
        for ms in (500, 1000, 1500, 2000, 3000):
            self.interval.addItem(f'{ms} ms', ms)
        self.interval.setCurrentIndex(2)
        self.hide_polling = QCheckBox('Hide polling logs')
        self.hide_polling.setChecked(True)
        self.auto_scroll = QCheckBox('Auto scroll')
        self.auto_scroll.setChecked(True)
        for w in (self.auto_refresh, self.interval, self.hide_polling, self.auto_scroll):
            row.addWidget(w)
        row.addWidget(self._button('Clear', lambda: self.terminal.clear()))
        row.addWidget(self._button('Export', self._export))
        layout.addLayout(row)
        self.terminal = QPlainTextEdit()
        self.terminal.setReadOnly(True)
        self.terminal.setMinimumHeight(65)
        self.terminal.document().setMaximumBlockCount(4000)
        layout.addWidget(self.terminal, 1)
        row = QHBoxLayout()
        self.manual = QLineEdit()
        self.manual.setMaxLength(127)
        self.manual.setPlaceholderText('Manual command, e.g. STATUS 1')
        self.manual.returnPressed.connect(self._manual_send)
        row.addWidget(self.manual, 1)
        self.send_btn = self._button('Send', self._manual_send)
        row.addWidget(self.send_btn)
        layout.addLayout(row)
        return box

    # ---------------- Transport and command lifecycle ----------------
    def refresh_ports(self):
        selected = self.port_box.currentData()
        self.port_box.clear()
        try:
            for port, description in self.serial.ports():
                self.port_box.addItem(f'{port} — {description}', port)
        except OSError as exc:
            self._log('ERROR', str(exc))
        index = self.port_box.findData(selected or 'COM9')
        if index >= 0:
            self.port_box.setCurrentIndex(index)

    def _toggle_connection(self):
        if self.connected:
            if not self._confirm_disconnect():
                return
            self.connect_btn.setEnabled(False)
            self.serial.disconnect_port()
            return
        port = self.port_box.currentData()
        if not port:
            self._log('WARN', 'No serial port selected.')
            return
        if self.serial.connect_port(port, 115200):
            self.connect_btn.setEnabled(False)
            self.connection_label.setText('Opening port…')
        else:
            self._log('WARN', 'Serial worker is busy.')

    def _reset_session(self):
        self.ready = False
        self.protocol_fault = False
        self.pending = None
        self.stops.clear()
        self.need_status.clear()
        self.need_config.clear()
        self.tracking.clear()
        self.status_seen.clear()
        self.configs.clear()
        self.states = {j: JointState() for j in JOINT_NAMES}
        self.next_poll = 0.0
        self.poll_cursor = 0

    def _on_connection(self, connected, port):
        self.connected = connected
        self._reset_session()
        self.connect_btn.setText('Disconnect' if connected else 'Connect')
        self.connect_btn.setEnabled(connected or not self.serial.isRunning())
        self._log('INFO', f'Port opened: {port}' if connected else f'Port closed: {port}')
        if connected:
            self.timer.start()
            # Opening COM is not equivalent to a firmware ACK.
            self._request('PING')
        else:
            self.timer.stop()
        self._render()

    def _on_thread_finished(self):
        if not self.connected:
            self.connect_btn.setEnabled(True)

    def _on_failure(self, message):
        self._log('ERROR', message)
        self.ready = False
        self.status_seen.clear()
        self._render()

    def _busy(self):
        return self.pending is not None or bool(self.stops)

    def _request(self, verb, joint=None, *values, quiet=False, reload=False):
        if not self.connected or self.protocol_fault or self._busy():
            return False
        if verb not in ('PING', 'HELP') and not self.ready:
            return False
        if joint is not None and joint not in LINKED_JOINTS:
            self._log('WARN', f'J{joint} is not linked.')
            return False
        if verb == 'PING':
            sent = self.client.ping()
        elif verb == 'HELP':
            sent = self.serial.send('HELP')
        elif verb == 'GROUP':
            sent = self.serial.send('GROUP ' + ' '.join(str(v) for v in values))
        else:
            sent = self.client.send(verb, joint, *values)
        if not sent:
            self._log('ERROR', 'Command was not queued: serial unavailable.')
            return False
        self.pending = Reply(verb, joint, tuple(str(v) for v in values), quiet, reload)
        if verb == 'CONFIG' and reload:
            self.pending.draft_snapshot = dict(self.drafts[joint])
        if verb == 'GROUP':
            for j in map(int, values[::2]):
                self.status_seen.discard(j)
                self.tracking.add(j)
        elif verb in MOTION_COMMANDS | CONFIG_COMMANDS | {'ZERO'}:
            self.status_seen.discard(joint)
        if verb in CONFIG_COMMANDS:
            self.configs.pop(joint, None)  # Require fresh readback before motion.
        self._render()
        return True

    def _stop(self, verb, target):
        if not self.connected:
            return False
        key = (verb, target)
        if key in self.stops:
            return False
        sent = self.client.stop_all(verb == 'ESTOP') if target == 'ALL' else self.client.send(verb, target)
        if not sent:
            self._log('ERROR', f'{verb} was not queued.')
            return False
        self.stops[key] = monotonic() + TIMEOUT
        affected = LINKED_JOINTS if target == 'ALL' else (target,)
        for j in affected:
            self.status_seen.discard(j)
            if verb == 'ESTOP':
                self.states[j].position_valid = False
                self.states[j].angle = None
            self.need_status.add(j)
        self._render()
        return True

    def _on_transmitted(self, command):
        pending = self.pending
        if pending and pending.quiet and self.hide_polling.isChecked() and command == f'STATUS {pending.joint}':
            return
        self._log('TX', command)

    def _on_received(self, line):
        line = line.strip()
        if not line:
            return
        kind = 'ERROR' if line.startswith('[ERROR]') else 'WARN' if line.startswith('[WARN]') else 'OK' if line.startswith('[OK]') else 'RX'
        # Stops have independent ACKs so they never consume a normal request.
        stop = re.match(r'^\[(OK|ERROR)\] (STOP|ESTOP) (ALL|J[1-6])(?:\s|$)', line)
        if stop:
            self._log(kind, line)
            target = 'ALL' if stop[3] == 'ALL' else int(stop[3][1:])
            self.stops.pop((stop[2], target), None)
            for j in LINKED_JOINTS if target == 'ALL' else (target,):
                self.status_seen.discard(j)
                self.need_status.add(j)
                self.tracking.add(j)
            self._dispatch()
            self._render()
            return
        if line.startswith('[OK] SML_ARM_READY') or line.startswith('[READY]'):
            self._log('INFO', line)
            # A controller reset invalidates every previously read position.
            self._reset_session()
            self._request('PING')
            self._render()
            return
        pending = self.pending
        result = pending.feed(line) if pending else None
        hidden = bool(pending and pending.quiet and result is not None and self.hide_polling.isChecked())
        # Even silent polling must expose genuine errors. Its expected UNKNOWN
        # warning is represented in the status card instead of repeated in log.
        if not hidden or kind == 'ERROR':
            self._log(kind, line)
        if line.startswith('[WARN] POSITION_UNKNOWN J'):
            try:
                j = int(line.rsplit('J', 1)[1])
                self.states[j].position_valid = False
                self.states[j].angle = None
            except (ValueError, KeyError):
                pass
        if result in ('ok', 'error'):
            self.pending = None
            self._complete(pending, result == 'ok')
            self._dispatch()
        self._render()

    def _complete(self, p, success):
        j = p.joint
        if p.verb == 'GROUP':
            # Refresh every involved joint on either ACK or error: a failed
            # launch may have stopped and invalidated some coordinates.
            for axis in map(int, p.values[::2]):
                self.status_seen.discard(axis)
                self.need_status.add(axis)
                self.tracking.add(axis)
            return
        if not success:
            if j is not None:
                self.status_seen.discard(j)
                self.tracking.discard(j)
                if p.verb in MOTION_COMMANDS | CONFIG_COMMANDS | {'ZERO'}:
                    self.need_status.add(j)
                if p.verb in CONFIG_COMMANDS:
                    self.need_config[j] = False
                if p.verb == 'CONFIG':
                    self.configs.pop(j, None)
            return
        try:
            if p.verb == 'PING':
                if not self.ready:
                    self.ready = True
                    self.need_status.update(LINKED_JOINTS)
                    self.need_config.update({i: True for i in LINKED_JOINTS})
            elif p.verb == 'STATUS':
                d = p.data
                state = self.states[j]
                angle = float(d['ANGLE']) if d['POSITION_VALID'] == '1' else None
                if angle is not None and not math.isfinite(angle):
                    raise ValueError('Invalid reported angle')
                state.available = d['AVAILABLE'] == '1'
                if not state.available:
                    self.need_config.pop(j, None)
                    self.configs.pop(j, None)
                state.running = d['RUNNING'] == '1'
                state.position_valid = d['POSITION_VALID'] == '1'
                state.angle = angle
                # A STOP sent while STATUS was in flight makes that reply stale.
                if not self.stops and j not in self.need_status:
                    self.status_seen.add(j)
                    if state.running:
                        self.tracking.add(j)
                    else:
                        self.tracking.discard(j)
            elif p.verb == 'CONFIG':
                d = p.data
                for k in DEFAULTS:
                    if k == 'INVERT':
                        if d[k] not in ('0', '1'):
                            raise ValueError('Invalid invert')
                    elif k == 'PPR':
                        numeric(d[k], True)
                    elif not math.isfinite(float(d[k])):
                        raise ValueError('Invalid config value')
                self.configs[j] = dict(d)
                if p.reload:
                    for k in tuple(self.dirty[j]):
                        if self.drafts[j][k] == p.draft_snapshot.get(k):
                            self.dirty[j].discard(k)
                for k, value in d.items():
                    if k not in self.dirty[j]:
                        self.drafts[j][k] = value
                if j == self.selected_joint:
                    self._show_config()
            elif p.verb == 'ANGLE':
                a = float(p.data['ANGLE'])
                if not math.isfinite(a):
                    raise ValueError('Invalid angle')
                if j in self.status_seen and self.states[j].position_valid:
                    self.states[j].angle = a
            elif p.verb in MOTION_COMMANDS:
                # ACK means accepted, never means the movement is finished.
                self.tracking.add(j)
                self.need_status.add(j)
            elif p.verb == 'ZERO':
                self.need_status.add(j)
            elif p.verb in CONFIG_COMMANDS:
                keys = ('MIN', 'MAX') if p.verb == 'LIMIT' else (p.verb,)
                for key, value in zip(keys, p.values):
                    # Keep unrelated edits and edits made after sending.
                    try:
                        same = float(self.drafts[j][key]) == float(value)
                    except ValueError:
                        same = False
                    if same:
                        self.dirty[j].discard(key)
                self.need_status.add(j)
                self.need_config[j] = False
        except (ValueError, KeyError) as exc:
            self.status_seen.discard(j)
            self.configs.pop(j, None)
            self.tracking.discard(j)
            self._log('ERROR', f'Invalid {p.verb} response: {exc}')

    def _dispatch(self):
        if not self.connected or not self.ready or self.protocol_fault or self._busy():
            return
        if self.need_status:
            j = min(self.need_status)
            self.need_status.remove(j)
            self._request('STATUS', j)
        elif self.need_config:
            j = min(self.need_config)
            force = self.need_config.pop(j)
            self._request('CONFIG', j, reload=force)

    def _tick(self):
        now = monotonic()
        if self.pending and now >= self.pending.deadline:
            p = self.pending
            self.pending = None
            self.protocol_fault = True
            self.status_seen.clear()
            self.tracking.clear()
            self._log('ERROR', f'{p.verb} response timed out. Reconnect before normal commands.')
        for key, deadline in list(self.stops.items()):
            if now >= deadline:
                del self.stops[key]
                self.protocol_fault = True
                self.status_seen.clear()
                self._log('ERROR', f'{key[0]} response timed out. Motor state unconfirmed.')
        self._dispatch()
        if not self._busy() and self.connected and self.ready and not self.protocol_fault and now >= self.next_poll:
            joints = sorted(self.tracking or (set(LINKED_JOINTS) if self.auto_refresh.isChecked() else set()))
            if joints:
                # Round-robin by joint ID, even as the tracking set changes.
                j = next((axis for axis in joints if axis > self.poll_cursor), joints[0])
                self.poll_cursor = j
                self.next_poll = now + (0.5 if self.tracking else self.interval.currentData() / 1000)
                self._request('STATUS', j, quiet=True)
        self._render()

    # ---------------- User actions and validation ----------------
    def _select_joint(self, joint):
        if joint not in LINKED_JOINTS:
            return
        self.selected_joint = joint
        for j, b in self.joint_buttons.items():
            b.setChecked(j == joint)
        self._show_config()
        self._render()

    def _edit_config(self, key, text):
        j = self.selected_joint
        self.drafts[j][key] = text
        self.dirty[j].add(key)
        self._render()

    def _show_config(self):
        d = self.drafts[self.selected_joint]
        for key, edit in self.config_fields.items():
            # Do not replace a dirty edit while the user is typing.
            if edit.text() != d[key]:
                edit.setText(d[key])
        self.invert.setChecked(d['INVERT'] == '1')

    def _can_operate(self, j):
        return (self.connected and self.ready and not self.protocol_fault and not self._busy()
                and j in LINKED_JOINTS and j in self.status_seen and self.states[j].available
                and not self.states[j].running and j not in self.tracking)

    def _can_move(self, j):
        return self._can_operate(j) and self.states[j].position_valid and j in self.configs

    def _validate_args(self, verb, values):
        n = 2 if verb == 'LIMIT' else 1
        if len(values) != n:
            raise ValueError(f'{verb} needs {n} value(s).')
        if verb == 'INVERT':
            if values[0] not in ('0', '1'):
                raise ValueError('Invert must be 0 or 1.')
            return tuple(values)
        args = tuple(numeric(x, verb == 'PPR') for x in values)
        x = float(args[0])
        if verb in ('GEAR', 'SPEED', 'ACCEL') and x <= 0:
            raise ValueError(f'{verb} must be greater than zero.')
        if verb == 'LINEAR' and x < 0:
            raise ValueError('Linear acceleration cannot be negative.')
        if verb == 'LIMIT' and not (x <= 0 <= float(args[1]) and x < float(args[1])):
            raise ValueError('Limits must be ordered and contain zero.')
        return args

    def _issue_values(self, verb, j, values):
        try:
            args = self._validate_args(verb, values)
            if verb in MOTION_COMMANDS:
                if not self._can_move(j):
                    raise ValueError('Read status/config and establish a valid, idle joint first.')
                d = self.configs[j]
                target = float(args[0])
                if verb == 'JREL':
                    target += self.states[j].angle
                if not float(d['MIN']) <= target <= float(d['MAX']):
                    raise ValueError('Target is outside the firmware angle limits.')
            elif not self._can_operate(j):
                raise ValueError('Read status and wait for the joint to be idle first.')
            return self._request(verb, j, *args)
        except (ValueError, TypeError) as exc:
            self._log('WARN', str(exc))
            return False

    def _move(self, verb, j, edit):
        self._issue_values(verb, j, (edit.text(),))

    def _issue_group(self, pairs):
        try:
            if not 2 <= len(pairs) <= 6 or len({j for j, _ in pairs}) != len(pairs):
                raise ValueError('GROUP requires 2–6 distinct joints.')
            args = []
            for j, text in pairs:
                if j not in LINKED_JOINTS or not self._can_move(j):
                    raise ValueError(f'J{j} must be available, idle and have valid zero/config.')
                value = numeric(text)
                d = self.configs[j]
                if not float(d['MIN']) <= float(value) <= float(d['MAX']):
                    raise ValueError(f'J{j} target is outside the firmware limits.')
                args.extend((str(j), value))
            summary = ', '.join(f'J{j}: {v}°' for j, v in zip(args[::2], args[1::2]))
            answer = QMessageBox.question(self, 'Start group motion',
                summary + '\nIndependent profiles; verify the mechanism and clear the workspace.',
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No)
            if answer != QMessageBox.StandardButton.Yes:
                return False
            # Modal dialog may process status/connection events. Recheck readiness.
            if not all(self._can_move(j) for j, _ in pairs):
                raise ValueError('Joint readiness changed; refresh and retry.')
            return self._request('GROUP', None, *args)
        except (ValueError, TypeError, KeyError) as exc:
            self._log('WARN', str(exc))
            return False

    def _move_all(self):
        self._issue_group([(j, self.multi[j][1].text()) for j in LINKED_JOINTS])

    def _jog(self, sign):
        try:
            step = numeric(self.jog_step.text())
            if float(step) <= 0:
                raise ValueError('Jog step must be positive.')
            delta = ('-' if sign < 0 else '') + step.lstrip('+-')
            self._issue_values('JREL', self.selected_joint, (delta,))
        except ValueError as exc:
            self._log('WARN', str(exc))

    def _zero(self, j):
        if not self._can_operate(j):
            return False
        answer = QMessageBox.question(self, 'Set software zero',
            f'Is J{j} at the known mechanical zero? This sets a coordinate only; it does not home the robot.',
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No, QMessageBox.StandardButton.No)
        if answer == QMessageBox.StandardButton.Yes and self._can_operate(j):
            return self._request('ZERO', j)
        return False

    def _apply(self, verb):
        j = self.selected_joint
        if verb == 'INVERT':
            args = ('1' if self.invert.isChecked() else '0',)
        elif verb == 'LIMIT':
            args = (self.config_fields['MIN'].text(), self.config_fields['MAX'].text())
        else:
            args = (self.config_fields[verb].text(),)
        self._issue_values(verb, j, args)

    def _refresh_all(self):
        self.need_status.update(LINKED_JOINTS)
        self._dispatch()

    def _manual_send(self):
        parts = self.manual.text().strip().split()
        if not parts:
            return
        verb = parts[0].upper()
        sent = False
        if verb in ('PING', 'HELP') and len(parts) == 1:
            sent = self._request(verb)
        elif verb == 'GROUP':
            if 5 <= len(parts) <= 13 and len(parts) % 2 == 1 and all(x.isdigit() for x in parts[1::2]):
                sent = self._issue_group([(int(j), v) for j, v in zip(parts[1::2], parts[2::2])])
            else:
                self._log('WARN', 'Use GROUP <id> <angle> <id> <angle> ...')
        elif verb in ('STOP', 'ESTOP') and len(parts) == 2 and parts[1].upper() == 'ALL':
            sent = self._stop(verb, 'ALL')
        elif len(parts) >= 2 and parts[1].isdigit() and int(parts[1]) in LINKED_JOINTS:
            j = int(parts[1])
            if verb in ('STATUS', 'CONFIG', 'ANGLE') and len(parts) == 2:
                sent = self._request(verb, j, reload=(verb == 'CONFIG'))
            elif verb == 'ZERO' and len(parts) == 2:
                sent = self._zero(j)
            elif verb in ('STOP', 'ESTOP') and len(parts) == 2:
                sent = self._stop(verb, j)
            elif verb in MOTION_COMMANDS | CONFIG_COMMANDS:
                sent = self._issue_values(verb, j, tuple(parts[2:]))
            else:
                self._log('WARN', 'Unsupported command or invalid syntax. Use HELP.')
        else:
            self._log('WARN', 'Invalid syntax or joint not linked. Use HELP.')
        if sent:
            self.manual.clear()

    # ---------------- Status and log rendering ----------------
    def _describe(self, j):
        if j not in LINKED_JOINTS:
            return 'Not linked', '—'
        if not self.connected:
            return 'Offline', '—'
        if j not in self.status_seen:
            return ('Motion / status pending' if j in self.tracking else 'Status not confirmed'), '—'
        s = self.states[j]
        if not s.available:
            return 'Not available', '—'
        state = ('Running' if s.running else 'Idle') + (' · VALID' if s.position_valid else ' · UNKNOWN')
        angle = f'{s.angle:.4f}°' if s.position_valid and s.angle is not None else '—'
        return state, angle

    @staticmethod
    def _tone(widget, tone):
        if widget.property('tone') != tone:
            widget.setProperty('tone', tone)
            widget.style().unpolish(widget)
            widget.style().polish(widget)
            widget.update()

    def _render(self):
        j = self.selected_joint
        normal = self.connected and self.ready and not self.protocol_fault and not self._busy()
        self.connection_label.setText('Reconnect required' if self.protocol_fault else 'Firmware ready' if self.connected and self.ready else 'Port open / waiting for PONG' if self.connected else 'Disconnected')
        self._tone(self.connection_label, 'error' if self.protocol_fault else 'good' if self.connected and self.ready else 'warn' if self.connected else '')
        self.ping_btn.setEnabled(self.connected and not self.protocol_fault and not self._busy())
        self.ports_btn.setEnabled(not self.connected)
        self.port_box.setEnabled(not self.connected)
        for b in (self.stop_all_btn, self.estop_all_btn, self.stop_btn, self.estop_btn):
            b.setEnabled(self.connected)
        for b in (self.move_btn, self.relative_btn, self.jog_minus, self.jog_plus):
            b.setEnabled(self._can_move(j))
        self.zero_btn.setEnabled(self._can_operate(j))
        for b in self.config_buttons.values():
            b.setEnabled(self._can_operate(j))
        for b in (self.read_status, self.read_config, self.reload_btn, self.multi_refresh, self.monitor_refresh):
            b.setEnabled(normal)
        # Numeric fields stay editable offline; no command is sent by typing.
        self.send_btn.setEnabled(self.connected)
        self.manual.setEnabled(self.connected)
        state, angle = self._describe(j)
        self.state_label.setText(f'J{j} · {state}')
        self._tone(self.state_label, 'good' if state == 'Idle · VALID' else 'warn' if self.connected else '')
        self.angle_label.setText(angle)
        limits = None
        if j in self.configs:
            try:
                low, high = float(self.configs[j]['MIN']), float(self.configs[j]['MAX'])
                if math.isfinite(low) and math.isfinite(high) and low < high:
                    limits = (low, high)
            except (KeyError, ValueError):
                pass
        estimate = self.states[j].angle if angle != '—' else None
        self.gauge.set_data(estimate, limits)
        self.config_note.setText(f'J{j} · ' + ('Firmware config read' if j in self.configs else 'Local values — not confirmed by firmware') + (' · Unapplied edits' if self.dirty[j] else ''))
        self.group_btn.setEnabled(all(self._can_move(i) for i in LINKED_JOINTS))
        for i, (a, t, s, b) in self.multi.items():
            description, value = self._describe(i)
            a.setText(value)
            s.setText(description)
            t.setEnabled(i in LINKED_JOINTS)
            b.setEnabled(self._can_move(i))
            self.monitor[i][0].setText(value)
            self.monitor[i][1].setText(description)
            self._tone(self.monitor[i][1], 'good' if description == 'Idle · VALID' else 'warn' if i in LINKED_JOINTS and self.connected else '')

    def _log(self, kind, message):
        colors = LOG_COLORS
        stamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        scrollbar = self.terminal.verticalScrollBar()
        previous = scrollbar.value()
        cursor = QTextCursor(self.terminal.document())
        cursor.movePosition(QTextCursor.MoveOperation.End)
        fmt = QTextCharFormat()
        fmt.setForeground(QColor(colors.get(kind, LOG_COLORS['RX'])))
        cursor.insertText(f'[{stamp}] {kind:<5} {message}\n', fmt)
        if self.auto_scroll.isChecked():
            self.terminal.setTextCursor(cursor)
            self.terminal.ensureCursorVisible()
        else:
            scrollbar.setValue(previous)

    def _export(self):
        path, _ = QFileDialog.getSaveFileName(self, 'Export terminal log', 'sml_arm_log.txt', 'Text files (*.txt)')
        if path:
            try:
                Path(path).write_text(self.terminal.toPlainText(), encoding='utf-8')
            except OSError as exc:
                self._log('ERROR', str(exc))

    def _confirm_disconnect(self):
        if self.connected and (self.protocol_fault or self.tracking or self.stops or any(s.running for s in self.states.values()) or
                               (self.pending and self.pending.verb in MOTION_COMMANDS | {'GROUP'})):
            return QMessageBox.question(self, 'Motion may be active',
                'Disconnecting does not stop the robot. Verify that the mechanism is safe before disconnecting.',
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No, QMessageBox.StandardButton.No) == QMessageBox.StandardButton.Yes
        return True

    def closeEvent(self, event):
        if not self._confirm_disconnect():
            event.ignore()
            return
        self.timer.stop()
        self.serial.disconnect_port()
        if self.serial.isRunning() and not self.serial.wait(2000):
            self._log('ERROR', 'Serial worker did not close yet.')
            event.ignore()
            return
        event.accept()
