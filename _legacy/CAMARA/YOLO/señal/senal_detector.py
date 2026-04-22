from ultralytics import YOLO
import cv2

class SenalDetector:
    def __init__(self, model_path="weights/best.pt"):
        self.model = YOLO(model_path)
        self.K = 4775  # constante distancia

    def detectar_senales(self, frame):
        """
        Detecta señales de tráfico en el frame
        """
        h_frame, w_frame = frame.shape[:2]
        results = self.model(frame, conf=0.25, verbose=False)

        senales_detectadas = []

        for r in results:
            for box, cls in zip(r.boxes.xyxy, r.boxes.cls):
                x1, y1, x2, y2 = map(int, box)
                nombre = self.model.names[int(cls)]

                # Señales de interés
                senales_interes = [
                    "stop", "yield", "no_entry", "speed_limit_30", "speed_limit_50",
                    "speed_limit_80", "parking", "one_way", "turn_left", "turn_right"
                ]

                if nombre in senales_interes:
                    # Calcular distancia
                    h_box = y2 - y1
                    distancia = self.K / h_box if h_box > 0 else None

                    # Calcular centroide
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)

                    # Determinar zona
                    tercio_w = w_frame // 3
                    if cx < tercio_w:
                        zona = "izquierda"
                    elif cx < 2 * tercio_w:
                        zona = "centro"
                    else:
                        zona = "derecha"

                    senal = {
                        'tipo': nombre,
                        'distancia': distancia,
                        'zona': zona,
                        'centroide': (cx, cy),
                        'bbox': (x1, y1, x2, y2)
                    }
                    senales_detectadas.append(senal)

        return senales_detectadas

    def dibujar_senales(self, frame, senales):
        """
        Dibuja las señales detectadas en el frame
        """
        for senal in senales:
            x1, y1, x2, y2 = senal['bbox']
            cx, cy = senal['centroide']

            # Dibujar bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)

            # Dibujar centroide
            cv2.circle(frame, (cx, cy), 6, (255, 0, 255), -1)

            # Dibujar información
            info = f"{senal['tipo']} {senal['zona']}"
            cv2.putText(frame, info, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)

        return frame