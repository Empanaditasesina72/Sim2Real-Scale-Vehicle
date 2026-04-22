import cv2
import numpy as np

class LaneCurvatureDetector:
    def __init__(self):
        self.prev_curvature = 0

    def calcular_curvatura(self, frame, lines):
        """
        Calcula la curvatura de las líneas del carril
        """
        if lines is None or len(lines) < 2:
            return self.prev_curvature

        left_curves = []
        right_curves = []

        for line in lines:
            x1, y1, x2, y2 = line[0]

            # Calcular pendiente
            if x2 != x1:
                slope = (y2 - y1) / (x2 - x1)
            else:
                slope = float('inf')

            # Clasificar líneas
            if slope < -0.5:  # Línea izquierda
                # Ajustar a polinomio de segundo grado
                if abs(slope) > 0.1:
                    left_curves.append(self._fit_curve(x1, y1, x2, y2))
            elif slope > 0.5:  # Línea derecha
                if abs(slope) > 0.1:
                    right_curves.append(self._fit_curve(x1, y1, x2, y2))

        # Calcular curvatura promedio
        curvatures = []
        if left_curves:
            curvatures.extend(left_curves)
        if right_curves:
            curvatures.extend(right_curves)

        if curvatures:
            avg_curvature = np.mean(curvatures)
            self.prev_curvature = avg_curvature
            return avg_curvature

        return self.prev_curvature

    def _fit_curve(self, x1, y1, x2, y2):
        """
        Ajusta una curva polinomial a dos puntos
        """
        # Para dos puntos, la curvatura es infinita (línea recta)
        # Usamos una aproximación basada en la pendiente
        if x2 != x1:
            slope = (y2 - y1) / (x2 - x1)
            # Curvatura aproximada (derivada segunda)
            # Para una línea recta, la curvatura es 0
            return 0
        else:
            # Línea vertical - curvatura infinita
            return float('inf')

    def detectar_curvas_complejas(self, frame):
        """
        Detecta curvas más complejas usando contornos
        """
        # Convertir a escala de grises
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Aplicar threshold adaptativo
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                      cv2.THRESH_BINARY_INV, 11, 2)

        # Encontrar contornos
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filtrar contornos por área
        min_area = 1000
        lane_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > min_area]

        curvatures = []
        for contour in lane_contours:
            # Ajustar elipse al contorno
            if len(contour) >= 5:
                ellipse = cv2.fitEllipse(contour)
                center, axes, angle = ellipse

                # Calcular curvatura basada en la elipse
                a, b = axes
                if b > 0:
                    eccentricity = np.sqrt(1 - (b/a)**2)
                    curvature = eccentricity  # Aproximación
                    curvatures.append(curvature)

        if curvatures:
            return np.mean(curvatures)

        return 0

    def dibujar_curvatura(self, frame, curvature):
        """
        Dibuja información de curvatura en el frame
        """
        height, width = frame.shape[:2]

        # Color basado en curvatura
        if abs(curvature) < 0.1:
            color = (0, 255, 0)  # Verde - recto
            text = "Recto"
        elif abs(curvature) < 0.5:
            color = (0, 255, 255)  # Amarillo - curva suave
            text = "Curva suave"
        else:
            color = (0, 0, 255)  # Rojo - curva pronunciada
            text = "Curva pronunciada"

        cv2.putText(frame, f"Curvatura: {curvature:.3f}", (30, height - 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        cv2.putText(frame, text, (30, height - 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        return frame