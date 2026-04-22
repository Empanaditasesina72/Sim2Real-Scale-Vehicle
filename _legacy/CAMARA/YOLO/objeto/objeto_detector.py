from ultralytics import YOLO
import cv2
import numpy as np

class ObjetoDetector:
    def __init__(self, model_path="weights/best.pt"):
        self.model = YOLO(model_path)
        self.K = 4775  # constante distancia

    def detectar_objetos(self, frame):
        """
        Detecta objetos en el frame y retorna información de distancia y tipo
        """
        h_frame, w_frame = frame.shape[:2]
        results = self.model(frame, conf=0.25, verbose=False)

        objetos_detectados = []

        for r in results:
            for box, cls in zip(r.boxes.xyxy, r.boxes.cls):
                x1, y1, x2, y2 = map(int, box)
                nombre = self.model.names[int(cls)]

                # Solo procesar objetos de interés (stop, señales, etc.)
                if nombre in ["stop", "yield", "no_entry", "speed_limit"]:
                    # Calcular distancia basada en altura del bounding box
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

                    objeto = {
                        'tipo': nombre,
                        'distancia': distancia,
                        'zona': zona,
                        'centroide': (cx, cy),
                        'bbox': (x1, y1, x2, y2)
                    }
                    objetos_detectados.append(objeto)

        return objetos_detectados

    def dibujar_detecciones(self, frame, objetos):
        """
        Dibuja las detecciones en el frame para debugging
        """
        for obj in objetos:
            x1, y1, x2, y2 = obj['bbox']
            cx, cy = obj['centroide']

            # Dibujar bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Dibujar centroide
            cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)

            # Dibujar información
            info = f"{obj['tipo']}: {obj['distancia']:.1f}cm {obj['zona']}"
            cv2.putText(frame, info, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        return frame