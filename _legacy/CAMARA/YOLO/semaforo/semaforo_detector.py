import cv2
import numpy as np

class SemaforoDetector:
    def __init__(self):
        pass

    def detectar_color_semaforo(self, frame, bbox):
        """
        Detecta el color de un semáforo en una región específica
        """
        x1, y1, x2, y2 = map(int, bbox)
        roi = frame[y1:y2, x1:x2]

        if roi.size == 0:
            return "none"

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        h = hsv.shape[0]
        tercio = h // 3

        zona_roja = hsv[0:tercio, :]
        zona_amarilla = hsv[tercio:2*tercio, :]
        zona_verde = hsv[2*tercio:h, :]

        # Máscaras de color
        rojo1 = cv2.inRange(zona_roja, (0, 120, 120), (10, 255, 255))
        rojo2 = cv2.inRange(zona_roja, (170, 120, 120), (180, 255, 255))
        mask_rojo = rojo1 + rojo2

        mask_amarillo = cv2.inRange(zona_amarilla, (20, 120, 120), (35, 255, 255))
        mask_verde = cv2.inRange(zona_verde, (40, 80, 80), (90, 255, 255))

        # Scores
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

    def procesar_semaforo(self, frame, bbox):
        """
        Procesa un semáforo detectado y retorna su estado
        """
        color = self.detectar_color_semaforo(frame, bbox)

        semaforo_info = {
            'estado': color,
            'bbox': bbox,
            'confianza': 0.0  # Podría calcularse basado en el score
        }

        return semaforo_info

    def dibujar_semaforo(self, frame, semaforo):
        """
        Dibuja información del semáforo en el frame
        """
        x1, y1, x2, y2 = map(int, semaforo['bbox'])
        estado = semaforo['estado']

        # Color del rectángulo según estado
        if estado == "red":
            color = (0, 0, 255)
        elif estado == "yellow":
            color = (0, 255, 255)
        elif estado == "green":
            color = (0, 255, 0)
        else:
            color = (128, 128, 128)

        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, f"Semaforo: {estado}", (x1, y1 - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        return frame