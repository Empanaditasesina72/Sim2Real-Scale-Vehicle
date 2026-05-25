# -*- coding: utf-8 -*-
"""Diagrama de clases UML del sistema TMR 2026."""
from PIL import Image, ImageDraw, ImageFont, ImageFilter
import os

os.chdir(r'C:\Users\Angel\Documents\GitHub\Carrito')

S = 3
W, H = 1560, 1230
img = Image.new('RGBA', (W * S, H * S), (255, 255, 255, 255))

def load_font(options, size):
    for p in options:
        try:
            return ImageFont.truetype(p, int(size * S))
        except Exception:
            continue
    return ImageFont.load_default()

f_title = load_font(['segoeuib.ttf', 'arialbd.ttf'], 26)
f_pkg   = load_font(['segoeuib.ttf', 'arialbd.ttf'], 15)
f_cls   = load_font(['segoeuib.ttf', 'arialbd.ttf'], 16)
f_mem   = load_font(['consola.ttf', 'cour.ttf', 'segoeui.ttf'], 12)
f_leg   = load_font(['segoeui.ttf', 'arial.ttf'], 12)

INK = '#1E1E1E'
PKG_BD = '#8C8C9E'
PKG_FILL = '#F7F7FB'
HDR_FILL = '#FBF3D4'
BODY_FILL = '#FFFFFF'

def scb(b):
    return [c * S for c in b]

def rrect(d, b, r, fill=None, outline=None, width=1):
    kw = {'radius': r * S}
    if fill:
        kw['fill'] = fill
    if outline:
        kw['outline'] = outline
        kw['width'] = max(1, int(width * S))
    d.rounded_rectangle(scb(b), **kw)

def rect(d, b, fill=None, outline=None, width=1):
    kw = {}
    if fill:
        kw['fill'] = fill
    if outline:
        kw['outline'] = outline
        kw['width'] = max(1, int(width * S))
    d.rectangle(scb(b), **kw)

def line(d, p1, p2, color, w):
    d.line([p1[0] * S, p1[1] * S, p2[0] * S, p2[1] * S], fill=color, width=max(1, int(w * S)))

def text_l(d, x, y, s, fnt, fill):
    d.text((x * S, y * S), s, font=fnt, fill=fill)

def text_c(d, cx, cy, s, fnt, fill):
    l, t, r, b = d.textbbox((0, 0), s, font=fnt)
    d.text((cx * S - (r - l) / 2 - l, cy * S - (b - t) / 2 - t), s, font=fnt, fill=fill)

def dashed(d, pts, color, w, dash=9, gap=6):
    for i in range(len(pts) - 1):
        x1, y1 = pts[i]
        x2, y2 = pts[i + 1]
        seg = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
        if seg == 0:
            continue
        ux, uy = (x2 - x1) / seg, (y2 - y1) / seg
        pos = 0
        while pos < seg:
            a = pos
            b = min(pos + dash, seg)
            line(d, (x1 + ux * a, y1 + uy * a), (x1 + ux * b, y1 + uy * b), color, w)
            pos += dash + gap

def solid(d, pts, color, w):
    for i in range(len(pts) - 1):
        line(d, pts[i], pts[i + 1], color, w)

def diamond(d, base, direction, color):
    x, y = base
    if direction == 'D':
        p = [(x, y), (x - 7, y + 8), (x, y + 16), (x + 7, y + 8)]
    elif direction == 'U':
        p = [(x, y), (x - 7, y - 8), (x, y - 16), (x + 7, y - 8)]
    elif direction == 'L':
        p = [(x, y), (x - 8, y - 6), (x - 16, y), (x - 8, y + 6)]
    else:
        p = [(x, y), (x + 8, y - 6), (x + 16, y), (x + 8, y + 6)]
    d.polygon([(px * S, py * S) for px, py in p], fill=color, outline=color)

def open_arrow(d, tip, heading, color, w):
    x, y = tip
    if heading == 'R':
        a, b = (x - 13, y - 8), (x - 13, y + 8)
    elif heading == 'L':
        a, b = (x + 13, y - 8), (x + 13, y + 8)
    elif heading == 'D':
        a, b = (x - 8, y - 13), (x + 8, y - 13)
    else:
        a, b = (x - 8, y + 13), (x + 8, y + 13)
    line(d, a, tip, color, w)
    line(d, b, tip, color, w)

# ---------- clases UML ----------
CLASSES = {
    'VehicleTMR': {
        'attrs': ['- mode: str', '- running: bool'],
        'meths': ['+ run()', '+ set_mode(mode)', '+ run_mode(dt)']},
    'CameraStream': {
        'attrs': ['- fps: int', '- _frame: ndarray'],
        'meths': ['+ start()', '+ stop()', '+ get_frame(): ndarray']},
    'LanePipeline': {
        'attrs': ['- hsv_white_lo/hi'],
        'meths': ['+ process(frame): LaneResult', '+ calibrate_bev()']},
    'SignDetector': {
        'attrs': ['- model: YOLO', '- _detections: list'],
        'meths': ['+ start()', '+ get_detections(): list', '+ closest_sign(): Detection']},
    'TemporalFilter': {
        'attrs': ['- window: int', '- _history: deque'],
        'meths': ['+ update(dets)', '+ stable(): list']},
    'DistanceSensor': {
        'attrs': ['- _front_mm: float', '- _rear_mm: float'],
        'meths': ['+ start()', '+ front_mm(): float', '+ rear_mm(): float']},
    'AutonomousFSM': {
        'attrs': ['- _state: FSMState', '- pid: PIDController'],
        'meths': ['+ activate()', '+ update(dt)', '+ state(): FSMState']},
    'PIDController': {
        'attrs': ['- kp, ki, kd: float', '- last_output: float', '- integral: float'],
        'meths': ['+ compute(meas, dt): float', '+ reset()']},
    'MotorDriver': {
        'attrs': ['- current_duty: float'],
        'meths': ['+ set_speed(duty)', '+ brake()']},
    'SteeringDriver': {
        'attrs': ['- current_angle: float'],
        'meths': ['+ set_angle(deg)', '+ center()', '+ steer_from_error(err)']},
    'TurnSignals': {
        'attrs': ['- mode: SignalMode'],
        'meths': ['+ set_mode(mode)', '+ tick()']},
}

HDR_H, LINE_H, PAD = 38, 24, 9

def class_height(name):
    c = CLASSES[name]
    return HDR_H + (2 * PAD + len(c['attrs']) * LINE_H) + (2 * PAD + len(c['meths']) * LINE_H)

packages = [
    ('Percepcion', ['CameraStream', 'LanePipeline', 'SignDetector',
                    'TemporalFilter', 'DistanceSensor']),
    ('Control',    ['VehicleTMR', 'AutonomousFSM', 'PIDController']),
    ('Actuacion',  ['MotorDriver', 'SteeringDriver', 'TurnSignals']),
]

# ---------- geometria ----------
MARGIN = 66
PKG_GAP = 66
PKG_TOP = 104
TAB_H = 28
COL_W = (W - 2 * MARGIN - 2 * PKG_GAP) / 3
CLS_PAD = 22
CLS_W = COL_W - 2 * CLS_PAD
CLS_GAP = 32

boxes = {}
pkg_rects = []
px = MARGIN
for pname, members in packages:
    total = sum(class_height(m) for m in members) + CLS_GAP * (len(members) - 1)
    pkg_h = TAB_H + 2 * CLS_PAD + total
    pkg_rects.append((pname, px, PKG_TOP, COL_W, pkg_h))
    cy = PKG_TOP + TAB_H + CLS_PAD
    for m in members:
        ch = class_height(m)
        boxes[m] = (px + CLS_PAD, cy, CLS_W, ch)
        cy += ch + CLS_GAP
    px += COL_W + PKG_GAP

# ---------- sombras ----------
shadow = Image.new('RGBA', img.size, (0, 0, 0, 0))
sd = ImageDraw.Draw(shadow)
for (x, y, w, h) in boxes.values():
    rect(sd, [x + 4, y + 5, x + w + 4, y + h + 5], fill=(20, 20, 20, 55))
shadow = shadow.filter(ImageFilter.GaussianBlur(3 * S))
img = Image.alpha_composite(img, shadow)
d = ImageDraw.Draw(img)

# ---------- titulo ----------
text_c(d, W / 2, 44, 'TMR 2026  -  Diagrama de Clases (UML)', f_title, INK)

# ---------- paquetes ----------
for (pname, x, y, w, h) in pkg_rects:
    tab_w = 168
    rect(d, [x, y, x + tab_w, y + TAB_H], fill=PKG_FILL, outline=PKG_BD, width=1.4)
    text_c(d, x + tab_w / 2, y + TAB_H / 2, pname, f_pkg, INK)
    rect(d, [x, y + TAB_H, x + w, y + h], fill=PKG_FILL, outline=PKG_BD, width=1.4)

# ---------- clases ----------
for name, (x, y, w, h) in boxes.items():
    c = CLASSES[name]
    rect(d, [x, y, x + w, y + h], fill=BODY_FILL, outline=INK, width=1.6)
    rect(d, [x, y, x + w, y + HDR_H], fill=HDR_FILL, outline=INK, width=1.6)
    text_c(d, x + w / 2, y + HDR_H / 2, name, f_cls, INK)
    ya = y + HDR_H
    attr_h = 2 * PAD + len(c['attrs']) * LINE_H
    line(d, (x, ya + attr_h), (x + w, ya + attr_h), INK, 1.6)
    ty = ya + PAD + 3
    for a in c['attrs']:
        text_l(d, x + 14, ty, a, f_mem, INK)
        ty += LINE_H
    ty = ya + attr_h + PAD + 3
    for m in c['meths']:
        text_l(d, x + 14, ty, m, f_mem, INK)
        ty += LINE_H

# ---------- relaciones ----------
def edge(name, side, frac=0.5):
    x, y, w, h = boxes[name]
    if side == 'L':
        return (x, y + h * frac)
    if side == 'R':
        return (x + w, y + h * frac)
    if side == 'T':
        return (x + w * frac, y)
    return (x + w * frac, y + h)

# composiciones internas verticales
for (src, dst) in [('VehicleTMR', 'AutonomousFSM'), ('AutonomousFSM', 'PIDController'),
                   ('SignDetector', 'TemporalFilter')]:
    s = edge(src, 'B'); t = edge(dst, 'T')
    diamond(d, s, 'D', INK)
    solid(d, [(s[0], s[1] + 16), (s[0], t[1])], INK, 1.6)
    open_arrow(d, t, 'D', INK, 1.6)

# VehicleTMR -> paquete Percepcion (3 lineas hacia la izquierda)
fracs = [0.30, 0.5, 0.70]
for k, (tname, fr) in enumerate(zip(['CameraStream', 'SignDetector', 'DistanceSensor'], fracs)):
    s = edge('VehicleTMR', 'L', fr)
    t = edge(tname, 'R', 0.5)
    xm = t[0] + (s[0] - t[0]) * (0.38 + 0.13 * k)
    diamond(d, s, 'L', INK)
    solid(d, [(s[0] - 16, s[1]), (xm, s[1]), (xm, t[1]), t], INK, 1.6)
    open_arrow(d, t, 'L', INK, 1.6)

# AutonomousFSM -> paquete Actuacion (3 lineas hacia la derecha)
for k, (tname, fr) in enumerate(zip(['MotorDriver', 'SteeringDriver', 'TurnSignals'], fracs)):
    s = edge('AutonomousFSM', 'R', fr)
    t = edge(tname, 'L', 0.5)
    xm = s[0] + (t[0] - s[0]) * (0.38 + 0.13 * k)
    diamond(d, s, 'R', INK)
    solid(d, [(s[0] + 16, s[1]), (xm, s[1]), (xm, t[1]), t], INK, 1.6)
    open_arrow(d, t, 'R', INK, 1.6)

# dependencias: LanePipeline / SignDetector ..> CameraStream
for src, fr in [('LanePipeline', 0.72), ('SignDetector', 0.5)]:
    s = edge(src, 'L', 0.5)
    t = edge('CameraStream', 'L', fr)
    xm = min(s[0], t[0]) - 16
    dashed(d, [s, (xm, s[1]), (xm, t[1]), t], INK, 1.5)
    open_arrow(d, t, 'R', INK, 1.5)

# ---------- leyenda ----------
ly = H - 36
lx = MARGIN
diamond(d, (lx + 16, ly), 'L', INK)
solid(d, [(lx + 16, ly), (lx + 74, ly)], INK, 1.6)
open_arrow(d, (lx + 74, ly), 'R', INK, 1.6)
text_l(d, lx + 86, ly - 8, 'composicion', f_leg, INK)
lx += 250
dashed(d, [(lx, ly), (lx + 58, ly)], INK, 1.5)
open_arrow(d, (lx + 58, ly), 'R', INK, 1.5)
text_l(d, lx + 70, ly - 8, 'dependencia (usa)', f_leg, INK)

# ---------- exportar ----------
out = img.convert('RGB').resize((W, H), Image.LANCZOS)
out.save('TMR2026_ARCHITECTURE_SIMPLIFIED.png', 'PNG')
out.save('TMR2026_ARCHITECTURE_SIMPLIFIED.pdf', 'PDF', resolution=150)
print('OK -', os.getcwd())
