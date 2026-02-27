from ultralytics import YOLO
import cv2
import numpy as np

# =====================================
# CARGAR MODELO (solo una vez)
# =====================================
model = YOLO("weights/best.pt")

K = 4775  # constante distancia

# =====================================
# DETECTOR DE COLOR POR HSV
# =====================================
def detectar_color_semaforo(frame, box):
    x1, y1, x2, y2 = map(int, box)
    roi = frame[y1:y2, x1:x2]

    if roi.size == 0:
        return "none"

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    h = hsv.shape[0]
    tercio = h // 3

    zona_roja = hsv[0:tercio, :]
    zona_amarilla = hsv[tercio:2*tercio, :]
    zona_verde = hsv[2*tercio:h, :]

    # ===== MASCARAS =====
    rojo1 = cv2.inRange(zona_roja, (0, 120, 120), (10, 255, 255))
    rojo2 = cv2.inRange(zona_roja, (170, 120, 120), (180, 255, 255))
    mask_rojo = rojo1 + rojo2

    mask_amarillo = cv2.inRange(
        zona_amarilla,
        (20, 120, 120),
        (35, 255, 255)
    )

    mask_verde = cv2.inRange(
        zona_verde,
        (40, 80, 80),
        (90, 255, 255)
    )

    # ===== SCORES =====
    score_rojo = np.sum(mask_rojo) / 255
    score_amarillo = np.sum(mask_amarillo) / 255
    score_verde = np.sum(mask_verde) / 255

    scores = {
        "red": score_rojo,
        "yellow": score_amarillo,
        "green": score_verde,
    }

    color = max(scores, key=scores.get)

    if scores[color] < 30:
        return "none"

    return color


# =====================================
# FUNCION PRINCIPAL DEL DETECTOR
# =====================================
def obtener_distancia(frame):

    distancia_stop = None
    semaforo = "none"
    zona_objeto = "none"
    cx_objeto = None

    h_frame, w_frame = frame.shape[:2]

    results = model(frame, conf=0.25, verbose=False)

    for r in results:
        for box, cls in zip(r.boxes.xyxy, r.boxes.cls):

            x1, y1, x2, y2 = map(int, box)
            nombre = model.names[int(cls)]

            # =====================================
            # CENTROIDE (NUEVO )
            # =====================================
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            tercio_w = w_frame // 3

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

                # debug visual
                cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
                cv2.putText(frame, zona, (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # =====================================
            # SEMAFORO
            # =====================================
            elif nombre in ["red", "yellow", "green",
                            "traffic light", "semaforo"]:

                color_detectado = detectar_color_semaforo(frame, box)

                if color_detectado != "none":
                    semaforo = color_detectado

                    zona_objeto = zona
                    cx_objeto = cx

                    # debug visual
                    cv2.circle(frame, (cx, cy), 6, (255, 0, 0), -1)
                    cv2.putText(frame, zona, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

    return distancia_stop, semaforo, zona_objeto, cx_objeto
