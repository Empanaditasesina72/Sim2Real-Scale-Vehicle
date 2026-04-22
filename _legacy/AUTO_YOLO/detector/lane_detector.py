import cv2
import numpy as np





def dectector_lineas(frame):
    # Convertir a escala de grises
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Aplicar desenfoque gaussiano
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Detección de bordes con Canny
    edges = cv2.Canny(blur, 50, 150)
    
    # Definir una región de interés (ROI) para enfocarse en la parte inferior de la imagen
    height, width = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([[
        (0, height),
        (width, height),
        (width, int(height * 0.6)),
        (0, int(height * 0.6))
    ]], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    
    # Aplicar la máscara a los bordes detectados
    masked_edges = cv2.bitwise_and(edges, mask)
    
    # Detección de líneas con Hough Transform
    lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, threshold=50, minLineLength=100, maxLineGap=50)
    
    return lines

def calcular_desviacion(frame, lines):
    if lines is None:
        return 0  # No se detectaron líneas, asumir desviación cero
    
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
        return 0  # No se detectaron ambas líneas, asumir desviación cero
    
    # Calcular el punto medio de las líneas detectadas
    left_x = np.mean([line[0][0] for line in left_lines])
    right_x = np.mean([line[0][0] for line in right_lines])
    
    lane_center = (left_x + right_x) / 2
    desviacion = lane_center - center_x
    
    return desviacion

