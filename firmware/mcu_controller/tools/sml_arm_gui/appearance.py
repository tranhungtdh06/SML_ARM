"""SML Arm Studio appearance: palettes, contrast, settings and live editor.

This module has no access to the robot or serial transport.
"""
import json
from pathlib import Path
from PySide6.QtCore import Qt, QSettings, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (QApplication, QDialog, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QComboBox, QLineEdit, QColorDialog,
    QCheckBox, QFileDialog, QMessageBox)

# Background, panel, secondary, border, accent, text, muted, terminal, dark.
_RAW = {
    'Graphite Pro': ('#101114','#1B1D22','#25282F','#363B45','#3978F6','#F4F5F7','#A1A8B5','#090B0E',True),
    'Industrial Control': ('#E2E7E5','#FFFFFF','#E9EEEB','#B4C0BA','#00633C','#142820','#53675D','#10251D',False),
    'Sakura Signature': ('#F0E8ED','#FFFFFF','#F2DFE9','#D5B5C7','#C01865','#30212A','#79566A','#24151F',False),
    'Black Titanium': ('#0B0C0E','#17191C','#26292E','#3E4146','#D4AC60','#F5F2EB','#A6A39D','#08090A',True),
    'Midnight Robotics': ('#080F1C','#111E30','#1B2D45','#2C435F','#00C6E0','#EAF5FF','#92A9C3','#050B13',True),
    'Precision White': ('#E7EBF0','#FFFFFF','#EDF0F5','#C1CAD6','#174FC5','#172334','#58697F','#182333',False),
    'Crimson Motorsport': ('#101011','#1C1C1F','#29292D','#414147','#DF243D','#F6F6F7','#AAAAB2','#09090B',True),
    'Emerald Matrix': ('#0B1512','#14231E','#20352D','#345347','#20C997','#E7F5EF','#8EAEA0','#060E0B',True),
}
KEYS = ('bg','panel','surface','border','accent','text','muted','terminal')
PRESETS = {name: dict(zip((*KEYS,'dark'), values), font=13, radius=7)
           for name, values in _RAW.items()}
DEFAULT = 'Graphite Pro'
# Semantic colors cannot be overridden by an imported or custom palette.
STOP, ESTOP = '#E7AC32', '#B91D35'
LOG_COLORS = {'TX':'#8DBAFF','RX':'#E0E6EF','OK':'#60E69B',
              'WARN':'#FFD078','ERROR':'#FF8295','INFO':'#C5B4FA'}


def mix(a,b,k):
    a,b = QColor(a),QColor(b)
    return QColor(*(round(x*(1-k)+y*k) for x,y in zip(a.getRgb()[:3], b.getRgb()[:3]))).name()


def luminance(color):
    rgb = QColor(color).getRgbF()[:3]
    def linear(x):
        return x/12.92 if x <= .04045 else ((x+.055)/1.055)**2.4
    return sum(w*linear(x) for w,x in zip((.2126,.7152,.0722),rgb))


def contrast(a,b):
    x,y = sorted((luminance(a),luminance(b)))
    return (y+.05)/(x+.05)


def readable(bg):
    return max(('#000000','#FFFFFF'), key=lambda c: contrast(c,bg))


def normalized(data):
    """Validate imported palette; do not accept arbitrary stylesheet text."""
    import re
    if not isinstance(data,dict):
        raise ValueError('Theme must be a JSON object.')
    p = {}
    for key in KEYS:
        value = data.get(key)
        if not isinstance(value,str) or not re.fullmatch(r'#[0-9a-fA-F]{6}',value):
            raise ValueError(f'{key}: expected #RRGGBB.')
        p[key] = value.upper()
    p['dark'] = luminance(p['bg']) < .18
    p['font'] = max(12,min(16,int(data.get('font',13))))
    p['radius'] = max(3,min(12,int(data.get('radius',7))))
    # Guarantee text contrast on the common surfaces, even with custom colors.
    for key in ('text','muted'):
        if min(contrast(p[key],p[s]) for s in ('bg','panel','surface')) < 4.5:
            p[key] = readable(p['panel'])
        if min(contrast(p[key],p[s]) for s in ('bg','panel','surface')) < 4.5:
            raise ValueError('Background, panel and secondary must have compatible brightness.')
    # Terminal is always dark so existing semantic log colors stay legible.
    if any(contrast(c,p['terminal']) < 4.5 for c in LOG_COLORS.values()):
        p['terminal'] = '#090B0E'
    return p


def generate(accent, dark, strength=80, font=13, radius=7):
    if not QColor(accent).isValid():
        raise ValueError('Invalid accent color.')
    accent=QColor(accent).name()
    p=PRESETS['Graphite Pro' if dark else 'Precision White'].copy()
    p.update(accent=accent,font=font,radius=radius)
    tint=(100-strength)/100*.17
    for key in ('bg','panel','surface','border'):
        p[key]=mix(p[key],accent,tint)
    return normalized(p)


def stylesheet(p):
    bg,panel,surface,border,accent,text,muted,terminal=(p[k] for k in KEYS)
    fg=readable(accent)
    title=accent if contrast(accent,panel)>=4.5 and contrast(accent,bg)>=4.5 else text
    hover=mix(surface,accent,.13)
    disabled=mix(panel,surface,.7)
    f,r=p['font'],p['radius']
    return f"""
    QMainWindow, QWidget#workspace, QWidget#root {{ background:{bg}; }}
    QWidget {{ font-family:'Segoe UI'; font-size:{f}px; color:{text}; }}
    QDialog, QMessageBox, QMenu {{ background:{panel}; color:{text}; }}
    QLabel {{ background:transparent; border:none; }}
    QLabel[role='brand'] {{ font-size:{f+10}px; font-weight:700; color:{title}; }}
    QLabel[role='heading'] {{ font-size:{f+2}px; font-weight:700; }}
    QLabel[role='muted'] {{ color:{muted}; }}
    QLabel[role='angle'] {{ font-size:{f+24}px; font-weight:700; color:{title}; }}
    QLabel[role='monitorAngle'] {{ font-size:{f+12}px; font-weight:700; }}
    QLabel[role='tableHeader'] {{ font-weight:700; color:{text}; background:{surface}; padding:6px; border-radius:{r}px; }}
    QLabel[tone='good'] {{ color:{'#60E69B' if p['dark'] else '#006B36'}; font-weight:600; }}
    QLabel[tone='warn'] {{ color:{'#FFD078' if p['dark'] else '#865000'}; font-weight:600; }}
    QLabel[tone='error'] {{ color:{'#FF8295' if p['dark'] else '#AD1731'}; font-weight:600; }}
    QLabel[role='terminalTitle'] {{ font-weight:700; color:{title}; }}
    QFrame[role='panel'] {{ background:{panel}; border:1px solid {border}; border-radius:{r+2}px; }}
    QGroupBox {{ background:{panel}; border:1px solid {border}; border-radius:{r+2}px; margin-top:15px; padding:9px 8px 7px; font-weight:700; }}
    QGroupBox::title {{ subcontrol-origin:margin; left:12px; padding:0 5px; color:{title}; }}
    QPushButton {{ background:{surface}; color:{text}; border:1px solid {border}; border-radius:{r}px; padding:5px 10px; font-weight:600; }}
    QPushButton:hover {{ background:{hover}; border-color:{accent}; }}
    QPushButton:pressed {{ background:{mix(surface,accent,.22)}; }}
    QPushButton:checked, QPushButton[role='primary'] {{ background:{accent}; color:{fg}; border-color:{accent}; }}
    QPushButton[role='primary']:hover {{ background:{mix(accent,fg,.09)}; }}
    QPushButton[role='stop'] {{ background:{STOP}; color:#231A08; border-color:#C38B20; font-weight:700; }}
    QPushButton[role='estop'] {{ background:{ESTOP}; color:white; border-color:#941329; font-weight:700; }}
    QPushButton[role='stop']:hover {{ background:#F3C260; }}
    QPushButton[role='estop']:hover {{ background:#D32640; }}
    QPushButton:disabled {{ background:{disabled}; color:{muted}; border-color:{border}; }}
    QLineEdit, QComboBox {{ background:{panel}; color:{text}; border:1px solid {border}; border-radius:{r}px; padding:4px 8px; selection-background-color:{accent}; selection-color:{fg}; }}
    QLineEdit:focus, QComboBox:focus {{ border-color:{accent}; }}
    QLineEdit:disabled, QComboBox:disabled {{ background:{disabled}; color:{muted}; }}
    QComboBox QAbstractItemView {{ background:{panel}; color:{text}; selection-background-color:{accent}; selection-color:{fg}; }}
    QTabWidget::pane {{ border:1px solid {border}; border-radius:{r}px; background:{bg}; }}
    QTabBar::tab {{ background:{surface}; color:{text}; padding:8px 18px; margin-right:4px; border-top-left-radius:{r}px; border-top-right-radius:{r}px; font-weight:600; }}
    QTabBar::tab:selected {{ background:{accent}; color:{fg}; }}
    QSplitter::handle {{ background:{border}; height:6px; }}
    QPlainTextEdit {{ background:{terminal}; color:#E0E6EF; border:1px solid {border}; border-radius:{r}px; font-family:Consolas,monospace; font-size:{max(12,f-1)}px; selection-background-color:#36588B; selection-color:white; }}
    QScrollBar:vertical {{ background:{surface}; width:12px; margin:0; }}
    QScrollBar::handle:vertical {{ background:{muted}; min-height:24px; border-radius:5px; }}
    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height:0; }}
    QStatusBar {{ background:{bg}; color:{muted}; }}
    """


class AppearanceManager:
    def __init__(self):
        self.settings=QSettings('SML','SML Arm Studio')
        self.name=self.settings.value('appearance/name',DEFAULT,type=str)
        try:
            self.custom=normalized(json.loads(self.settings.value('appearance/custom','{}',type=str)))
        except (ValueError,TypeError):
            self.custom=generate('#3978F6',True)
        if self.name not in PRESETS and self.name not in ('Custom','System'):
            self.name=DEFAULT

    def palette(self,name=None):
        name=name or self.name
        if name=='System':
            name='Graphite Pro' if QApplication.styleHints().colorScheme()==Qt.ColorScheme.Dark else 'Precision White'
        return self.custom.copy() if name=='Custom' else normalized(PRESETS[name])

    def save(self,name,custom=None):
        if custom is not None:
            self.custom=normalized(custom)
        self.name=name
        self.settings.setValue('appearance/name',name)
        self.settings.setValue('appearance/custom',json.dumps(self.custom))
        self.settings.sync()


class ThemeDialog(QDialog):
    preview=Signal(dict)
    saved=Signal(dict)

    def __init__(self,manager,parent=None):
        super().__init__(parent)
        self.manager=manager
        self.setWindowTitle('Appearance — Custom theme')
        self.setWindowModality(Qt.WindowModality.NonModal)
        self.resize(510,510)
        self.original=manager.palette()
        self.current=manager.custom.copy() if manager.name!='Custom' else self.original.copy()
        root=QVBoxLayout(self)
        heading=QLabel('Smart Theme Generator'); heading.setProperty('role','heading'); root.addWidget(heading)
        row=QHBoxLayout(); self.mode=QComboBox(); self.mode.addItems(['Dark','Light','System (current)']); self.mode.setCurrentText('Dark' if self.current['dark'] else 'Light')
        row.addWidget(QLabel('Mode')); row.addWidget(self.mode,1)
        self.strength=QComboBox(); self.strength.addItems(['100','80','60','40','20']); self.strength.setCurrentText('80'); row.addWidget(QLabel('Neutrality %')); row.addWidget(self.strength); root.addLayout(row)
        grid=QGridLayout(); self.edits={}
        for i,key in enumerate(KEYS):
            label=QLabel({'bg':'Background','panel':'Panel','surface':'Secondary','border':'Border','accent':'Accent','text':'Text','muted':'Muted text','terminal':'Terminal'}[key]); edit=QLineEdit(self.current[key]); edit.setMaxLength(7); edit.setMaximumWidth(110)
            btn=QPushButton('Pick'); btn.clicked.connect(lambda checked=False,k=key:self.pick(k)); grid.addWidget(label,i,0); grid.addWidget(edit,i,1); grid.addWidget(btn,i,2); self.edits[key]=edit; edit.editingFinished.connect(self.try_preview)
        root.addLayout(grid)
        row=QHBoxLayout(); self.font=QComboBox(); self.font.addItems([str(i) for i in range(12,17)]); self.font.setCurrentText(str(self.current['font'])); self.radius=QComboBox(); self.radius.addItems([str(i) for i in range(3,13)]); self.radius.setCurrentText(str(self.current['radius'])); row.addWidget(QLabel('Font px')); row.addWidget(self.font); row.addWidget(QLabel('Radius px')); row.addWidget(self.radius); row.addStretch(); root.addLayout(row)
        self.note=QLabel('Text contrast is checked automatically. STOP / ESTOP colors are fixed.'); self.note.setWordWrap(True); root.addWidget(self.note)
        row=QHBoxLayout()
        for text,fn in [('Generate',self.auto_generate),('Preview',self.try_preview),('Import JSON',self.import_theme),('Export JSON',self.export_theme)]:
            b=QPushButton(text); b.clicked.connect(fn); row.addWidget(b)
        root.addLayout(row)
        row=QHBoxLayout(); reset=QPushButton('Reset custom'); reset.clicked.connect(self.reset); row.addWidget(reset); row.addStretch(); cancel=QPushButton('Cancel'); cancel.clicked.connect(self.reject); row.addWidget(cancel); save=QPushButton('Save & apply'); save.setProperty('role','primary'); save.clicked.connect(self.commit); row.addWidget(save); root.addLayout(row)
        self.font.currentTextChanged.connect(self.try_preview); self.radius.currentTextChanged.connect(self.try_preview)
        self.mode.currentTextChanged.connect(self.auto_generate); self.strength.currentTextChanged.connect(self.auto_generate)

    def load(self,p):
        self.current=p.copy()
        for k,e in self.edits.items(): e.setText(p[k])
        for box,value in ((self.font,p['font']),(self.radius,p['radius'])):
            box.blockSignals(True); box.setCurrentText(str(value)); box.blockSignals(False)

    def read(self):
        return normalized(dict({k:e.text().strip() for k,e in self.edits.items()},font=int(self.font.currentText()),radius=int(self.radius.currentText())))

    def try_preview(self,*args):
        try:
            self.current=self.read()
            self.note.setText('Preview applied. Text contrast checked; safety colors protected.')
            self.preview.emit(self.current.copy())
            return True
        except (ValueError,TypeError) as exc:
            self.note.setText(str(exc)); return False

    def auto_generate(self,*args):
        try:
            mode=self.mode.currentText()
            dark=mode=='Dark' or (mode=='System (current)' and QApplication.styleHints().colorScheme()==Qt.ColorScheme.Dark)
            self.load(generate(self.edits['accent'].text(),dark,int(self.strength.currentText()),int(self.font.currentText()),int(self.radius.currentText())))
            self.try_preview()
        except ValueError as exc: self.note.setText(str(exc))

    def pick(self,key):
        color=QColorDialog.getColor(QColor(self.edits[key].text()),self,'Choose '+key)
        if color.isValid():
            self.edits[key].setText(color.name()); self.try_preview()

    def reset(self):
        self.load(generate('#3978F6',True)); self.try_preview()

    def import_theme(self):
        path,_=QFileDialog.getOpenFileName(self,'Import theme','','Theme JSON (*.json)')
        if path:
            try: self.load(normalized(json.loads(Path(path).read_text(encoding='utf-8')))); self.try_preview()
            except (OSError,ValueError,TypeError) as exc: QMessageBox.warning(self,'Invalid theme',str(exc))

    def export_theme(self):
        if not self.try_preview(): return
        path,_=QFileDialog.getSaveFileName(self,'Export theme','sml_theme.json','Theme JSON (*.json)')
        if path:
            try: Path(path).write_text(json.dumps(self.current,indent=2),encoding='utf-8')
            except OSError as exc: QMessageBox.warning(self,'Export failed',str(exc))

    def commit(self):
        if self.try_preview():
            self.manager.save('Custom',self.current); self.saved.emit(self.current.copy()); self.accept()

    def reject(self):
        self.preview.emit(self.original.copy()); super().reject()
