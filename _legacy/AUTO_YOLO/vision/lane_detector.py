import cv2
import numpy as np

def detectar_carril(frame):

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    edges = cv2.Canny(blur, 50, 150)

    height, width = edges.shape
    mask = np.zeros_like(edges)

    polygon = np.array([[
        (0, height),
        (width, height),
        (width, int(height*0.6)),
        (0, int(height*0.6))
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    cropped = cv2.bitwise_and(edges, mask)

    lines = cv2.HoughLinesP(
        cropped,
        1,
        np.pi/180,
        50,
        minLineLength=50,
        maxLineGap=100
    )

    left_lines = []
    right_lines = []

    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            if x2 - x1 == 0:
                continue

            slope = (y2 - y1) / (x2 - x1)

            if slope < -0.5:
                left_lines.append(line[0])
            elif slope > 0.5:
                right_lines.append(line[0])

    def promedio_lineas(lines):
        if len(lines) == 0:
            return None

        x_coords = []
        y_coords = []

        for x1, y1, x2, y2 in lines:
            x_coords += [x1, x2]
            y_coords += [y1, y2]

        poly = np.polyfit(x_coords, y_coords, 1)
        return poly

    left_fit = promedio_lineas(left_lines)
    right_fit = promedio_lineas(right_lines)

    centro = None

    if left_fit is not None and right_fit is not None:
        y = height
        left_x = int((y - left_fit[1]) / left_fit[0])
        right_x = int((y - right_fit[1]) / right_fit[0])

        centro = (left_x + right_x) // 2

        # dibujo
        cv2.line(frame, (left_x, height), (left_x, int(height*0.6)), (255,0,0), 5)
        cv2.line(frame, (right_x, height), (right_x, int(height*0.6)), (0,0,255), 5)
        cv2.circle(frame, (centro, height), 10, (0,255,0), -1)

    return centro, frame
