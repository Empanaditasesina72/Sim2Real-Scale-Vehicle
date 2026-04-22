import cv2
import numpy as np

class LaneCenterDetector:
    def __init__(self):
        self.prev_center = None

    def detectar_lineas(self, frame):
        """
        Detecta líneas de carril usando procesamiento de imagen
        """
        # Convertir a escala de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Aplicar desenfoque gaussiano
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Detección de bordes con Canny
        edges = cv2.Canny(blur, 50, 150)

        # Definir región de interés (ROI)
        height, width = edges.shape
        mask = np.zeros_like(edges)

        # Polígono para ROI (parte inferior de la imagen)
        polygon = np.array([[
            (0, height),
            (width, height),
            (width, int(height * 0.6)),
            (0, int(height * 0.6))
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)

        # Aplicar máscara
        masked_edges = cv2.bitwise_and(edges, mask)

        # Detección de líneas con Hough Transform
        lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180,
                               threshold=50, minLineLength=100, maxLineGap=50)

        return lines

    def calcular_centro_carril(self, frame, lines):
        """
        Calcula el centro del carril basado en las líneas detectadas
        """
        if lines is None:
            return self.prev_center if self.prev_center else frame.shape[1] // 2

        height, width, _ = frame.shape
        center_x = width / 2

        left_lines = []
        right_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1 + 1e-6)  # Evitar división por cero

            if slope < -0.5:  # Línea izquierda
                left_lines.append(line)
            elif slope > 0.5:  # Línea derecha
                right_lines.append(line)

        if not left_lines or not right_lines:
            return self.prev_center if self.prev_center else center_x

        # Calcular punto medio de las líneas detectadas
        left_x = np.mean([line[0][0] for line in left_lines])
        right_x = np.mean([line[0][2] for line in right_lines])

        lane_center = (left_x + right_x) / 2
        self.prev_center = lane_center

        return lane_center

    def dibujar_centro_carril(self, frame, lane_center):
        """
        Dibuja el centro del carril en el frame
        """
        height, width = frame.shape[:2]

        # Dibujar línea vertical en el centro del carril
        cv2.line(frame, (int(lane_center), height//2),
                (int(lane_center), height), (255, 255, 0), 3)

        # Dibujar línea central del frame
        cv2.line(frame, (width//2, height//2),
                (width//2, height), (0, 255, 255), 2)

        return frame