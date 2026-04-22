from ultralytics import YOLO
import cv2
import numpy as np

# =====================================
# MODELO
# =====================================
model = YOLO("weights/best.pt")

K = 8775

# =====================================
# CONFIG PERFORMANCE
# =====================================
REDUCCION = 0.5   # 🔥 escala imagen (0.5 = doble velocidad)
FRAME_SKIP = 2    # 🔥 procesa 1 de cada 2 frames

frame_count = 0

# =====================================
# COLOR SEMAFORO (OPTIMIZADO)
# =====================================
def detectar_color_semaforo(frame, box):

    x1, y1, x2, y2 = map(int, box)

    roi = frame[y1:y2, x1:x2]

    if roi.size == 0:
        return "none"

    # 🔥 reducir ROI
    roi = cv2.resize(roi, (0,0), fx=0.5, fy=0.5)

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    h = hsv.shape[0]
    tercio = h // 3

    zona_roja = hsv[0:tercio, :]
    zona_amarilla = hsv[tercio:2*tercio, :]
    zona_verde = hsv[2*tercio:h, :]

    rojo1 = cv2.inRange(zona_roja, (0, 120, 120), (10, 255, 255))
    rojo2 = cv2.inRange(zona_roja, (170, 120, 120), (180, 255, 255))
    mask_rojo = rojo1 + rojo2

    mask_amarillo = cv2.inRange(zona_amarilla, (20,120,120), (35,255,255))
    mask_verde = cv2.inRange(zona_verde, (40,80,80), (90,255,255))

    score_rojo = np.sum(mask_rojo) / 255
    score_amarillo = np.sum(mask_amarillo) / 255
    score_verde = np.sum(mask_verde) / 255

    scores = {
        "red": score_rojo,
        "yellow": score_amarillo,
        "green": score_verde,
    }

    color = max(scores, key=scores.get)

    if scores[color] < 20:   # 🔥 menor umbral (más rápido)
        return "none"

    return color

# =====================================
# DETECTOR PRINCIPAL
# =====================================
def obtener_distancia(frame):

    global frame_count
    frame_count += 1

    # 🔥 SKIP FRAMES
    if frame_count % FRAME_SKIP != 0:
        return None, "none", "none", None

    # =====================================
    # REDUCIR RESOLUCION
    # =====================================
    frame_small = cv2.resize(frame, (0,0), fx=REDUCCION, fy=REDUCCION)

    h_frame, w_frame = frame_small.shape[:2]

    distancia_stop = None
    semaforo = "none"
    zona_objeto = "none"
    cx_objeto = None

    # =====================================
    # YOLO (MAS RAPIDO)
    # =====================================
    results = model(frame_small, conf=0.25, verbose=False)

    for r in results:
        for box, cls in zip(r.boxes.xyxy, r.boxes.cls):

            x1, y1, x2, y2 = map(int, box)

            # 🔥 ESCALAR A IMAGEN ORIGINAL
            x1 = int(x1 / REDUCCION)
            y1 = int(y1 / REDUCCION)
            x2 = int(x2 / REDUCCION)
            y2 = int(y2 / REDUCCION)

            nombre = model.names[int(cls)]

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            tercio_w = frame.shape[1] // 3

            if cx < tercio_w:
                zona = "izquierda"
            elif cx < 2 * tercio_w:
                zona = "centro"
            else:
                zona = "derecha"

            # =====================================
            # STOP
            # =====================================
            if nombre == "stop":

                h_box = y2 - y1

                if h_box > 0:
                    distancia_stop = K / h_box

                zona_objeto = zona
                cx_objeto = cx

            # =====================================
            # SEMAFORO
            # =====================================
            elif nombre in ["red", "yellow", "green", "traffic light", "semaforo"]:

                color_detectado = detectar_color_semaforo(frame, (x1,y1,x2,y2))

                if color_detectado != "none":
                    semaforo = color_detectado
                    zona_objeto = zona
                    cx_objeto = cx

    return distancia_stop, semaforo, zona_objeto, cx_objeto
