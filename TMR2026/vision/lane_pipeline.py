# -*- coding: utf-8 -*-
"""
lane_pipeline.py — Pipeline de detección de carril BEV + HSV + Sliding Windows.

Pipeline completo:
  1. ROI: recortar mitad inferior del frame (ignorar cielo/ruido superior).
  2. Bird's-Eye View: transformación de perspectiva a vista cenital.
  3. Filtro HSV estricto: aislar blanco y rechazar reflejos del negro brillante.
  4. Morfología: eliminar ruido puntual (specular highlights del plástico negro).
  5. Sliding Windows: encontrar centros de carril izq y der de abajo hacia arriba.
  6. Calcular error_direccion respecto al centro del frame.
  7. Suavizado temporal (EMA) para reducir oscilaciones del servo.

Calibración BEV:
  Los puntos SRC deben calibrarse colocando el coche sobre el carril y
  ajustando hasta que las líneas blancas queden verticales en la vista BEV.
  Modificar BEV_SRC_RATIO en tu instancia o usar calibrar_bev().

Nota de rendimiento:
  A 640×480 este pipeline tarda ~8-12 ms en Pi 5 (sin aceleración GPU).
"""

from __future__ import annotations

import cv2
import numpy as np
from dataclasses import dataclass
from typing import Optional


@dataclass
class LaneResult:
    """Resultado del pipeline de detección de carril."""
    error_px:    float   # Error respecto al centro del BEV en píxeles
                         # Negativo = coche a la DERECHA del carril → girar izquierda
                         # Positivo = coche a la IZQUIERDA del carril → girar derecha
    confidence:  float   # [0.0 – 1.0]
    left_x:      Optional[int] = None   # Posición promedio línea izquierda (BEV px)
    right_x:     Optional[int] = None   # Posición promedio línea derecha  (BEV px)
    bev_frame:   Optional[np.ndarray] = None   # Vista BEV (debug)
    mask_frame:  Optional[np.ndarray] = None   # Máscara binaria (debug)


class LanePipeline:
    """
    Detector de carril para pista negra brillante con líneas blancas (~40 cm).

    Parámetros clave a calibrar en pista:
      bev_src_ratio: puntos de la trampa de perspectiva (fracción del frame)
      hsv_white_s_max: saturación máxima para aceptar blanco (rechaza reflejos grises)
      hsv_white_v_min: luminosidad mínima (rechaza sombras)
    """

    # ── Puntos de perspectiva BEV (fracción del ancho/alto del frame) ─────────
    # Trapecio en el frame original que se mapea a un rectángulo en BEV.
    # CALIBRADO para: cámara a 22 cm de altura, carro 20×35 cm.
    # Con esta altura la perspectiva ve más piso → trapecio más ancho abajo
    # y un poco más alto arriba para captar el carril más adelante.
    #   [bot-izq, bot-der, top-der, top-izq]
    BEV_SRC_RATIO = np.float32([
        [0.05, 1.00],   # abajo-izquierda (más ancho)
        [0.95, 1.00],   # abajo-derecha   (más ancho)
        [0.60, 0.60],   # arriba-derecha  (un poco más arriba y estrecho)
        [0.40, 0.60],   # arriba-izquierda
    ])
    BEV_DST_RATIO = np.float32([
        [0.20, 1.00],
        [0.80, 1.00],
        [0.80, 0.00],
        [0.20, 0.00],
    ])

    # ── Filtro HSV para blanco ────────────────────────────────────────────────
    # Pista negra brillante → reflejos tienen S alto (gris especular) o S muy bajo
    # pero V muy alto (sobreexposición puntual).
    # Blanco real: S bajo, V medio-alto.
    #
    # V_min=130 funciona desde luz tenue (linterna del celular ~100–200 lx)
    # hasta luz fuerte (lámpara directa). Si el plástico negro empieza a
    # contar como "blanco" en sombras profundas, sube V_min a 150–160.
    HSV_WHITE_LO = np.array([  0,  0, 130])   # H, S_min=0, V_min=130
    HSV_WHITE_HI = np.array([179, 60, 255])   # H, S_max=60 (rechaza grises brillantes)

    # ── Sliding Windows ───────────────────────────────────────────────────────
    N_WINDOWS  = 9     # Número de franjas horizontales en el BEV
    WIN_MARGIN = 70    # ±px alrededor del centro previo para buscar píxeles
    MIN_PIX    = 40    # Mínimo de píxeles blancos para aceptar una ventana

    # ── Suavizado temporal ────────────────────────────────────────────────────
    EMA_ALPHA  = 0.45  # Bajado de 0.65 → menos oscilación del servo
                       # (más alto = responde rápido pero más ruidoso)

    # ── Sesgo lateral dentro del carril ───────────────────────────────────────
    # 0.0 = pegado a la línea izquierda
    # 0.5 = centro exacto del carril (comportamiento histórico)
    # 1.0 = pegado a la línea derecha
    # Para TMR suele interesar 0.60–0.75 (seguir el carril derecho).
    RIGHT_BIAS = 0.5

    def __init__(
        self,
        frame_w: int = 640,
        frame_h: int = 480,
        debug: bool = False,
        right_bias: float = RIGHT_BIAS,
    ):
        self._w     = frame_w
        self._h     = frame_h
        self._debug = debug
        self._right_bias = max(0.0, min(1.0, float(right_bias)))

        # ROI: ignorar la mitad superior del frame
        self._roi_y = frame_h // 2

        # Calcular matrices de perspectiva
        src = self.BEV_SRC_RATIO.copy()
        dst = self.BEV_DST_RATIO.copy()
        src[:, 0] *= frame_w;  src[:, 1] *= frame_h
        dst[:, 0] *= frame_w;  dst[:, 1] *= frame_h

        # Ajustar src al ROI recortado
        src[:, 1] -= self._roi_y
        src[:, 1]  = np.clip(src[:, 1], 0, frame_h - self._roi_y - 1)

        self._M    = cv2.getPerspectiveTransform(src, dst)
        self._Minv = cv2.getPerspectiveTransform(dst, src)

        # BEV output size
        self._bev_w = frame_w
        self._bev_h = frame_h - self._roi_y

        # Estado EMA
        self._smooth_error = 0.0
        self._prev_conf    = 0.0

        # Kernel morfológico para limpiar ruido
        self._morph_k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    # ─── API pública ──────────────────────────────────────────────────────────

    def process(self, frame: np.ndarray) -> LaneResult:
        """
        Procesa un frame BGR y retorna el error de dirección.

        Parameters
        ----------
        frame : np.ndarray
            Frame BGR de la cámara (ya convertido con cv2.COLOR_RGB2BGR).

        Returns
        -------
        LaneResult con error_px y confidence.
        """
        # 1. ROI — descartar mitad superior (cielo, señales lejanas)
        roi = frame[self._roi_y:, :]

        # 2. Bird's-Eye View
        bev = cv2.warpPerspective(roi, self._M, (self._bev_w, self._bev_h))

        # 3. Filtro HSV estricto — aislar blanco, rechazar negro brillante
        hsv  = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.HSV_WHITE_LO, self.HSV_WHITE_HI)

        # 4. Morfología — quitar reflejos especulares puntales del plástico negro
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._morph_k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._morph_k)

        # 5. Sliding Windows
        result = self._sliding_windows(mask)

        # 6. Suavizado temporal (EMA)
        if result.confidence > 0.1:
            smoothed = (self.EMA_ALPHA * result.error_px
                        + (1 - self.EMA_ALPHA) * self._smooth_error)
            self._smooth_error = smoothed
            result.error_px    = smoothed

        # 7. Adjuntar imágenes de debug si se solicita
        if self._debug:
            result.bev_frame  = bev
            result.mask_frame = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        return result

    def calibrate_bev(self, src_points: np.ndarray) -> None:
        """
        Actualiza los puntos de perspectiva en caliente.

        Parameters
        ----------
        src_points : np.ndarray shape (4,2)
            Puntos en el frame original (píxeles absolutos).
        """
        dst = self.BEV_DST_RATIO.copy()
        dst[:, 0] *= self._w;  dst[:, 1] *= self._h
        # Ajustar al ROI
        src_roi = src_points.astype(np.float32)
        src_roi[:, 1] -= self._roi_y

        self._M    = cv2.getPerspectiveTransform(src_roi, dst)
        self._Minv = cv2.getPerspectiveTransform(dst, src_roi)

    # ─── Sliding Windows ──────────────────────────────────────────────────────

    def _sliding_windows(self, binary: np.ndarray) -> LaneResult:
        """
        Localiza las líneas blancas del carril usando ventanas deslizantes.

        Algoritmo:
          1. Histograma de la mitad inferior del BEV.
          2. Pico izquierdo y derecho como posición inicial de cada línea.
          3. N ventanas de abajo hacia arriba — recalibrar centro en cada ventana.
          4. Promediar posiciones encontradas → centro del carril.
          5. Error = centro_carril - centro_frame.
        """
        h, w = binary.shape
        mid  = w // 2

        # Histograma base
        hist     = np.sum(binary[h // 2:, :], axis=0).astype(np.int32)
        left_x   = int(np.argmax(hist[:mid]))
        right_x  = int(np.argmax(hist[mid:])) + mid

        has_left  = hist[left_x]  > 200
        has_right = hist[right_x] > 200

        if not has_left and not has_right:
            return LaneResult(error_px=self._smooth_error, confidence=0.0)

        win_h        = h // self.N_WINDOWS
        left_centers  = []
        right_centers = []

        cur_left  = left_x
        cur_right = right_x

        for i in range(self.N_WINDOWS):
            y_lo = h - (i + 1) * win_h
            y_hi = h - i * win_h

            # ── Ventana izquierda ─────────────────────────────
            if has_left:
                xl_lo = max(0, cur_left  - self.WIN_MARGIN)
                xl_hi = min(w, cur_left  + self.WIN_MARGIN)
                win_l = binary[y_lo:y_hi, xl_lo:xl_hi]
                nz_l  = np.count_nonzero(win_l)
                if nz_l >= self.MIN_PIX:
                    pts  = np.where(win_l > 0)[1]
                    cur_left = int(np.mean(pts)) + xl_lo
                    left_centers.append(cur_left)

            # ── Ventana derecha ───────────────────────────────
            if has_right:
                xr_lo = max(0, cur_right - self.WIN_MARGIN)
                xr_hi = min(w, cur_right + self.WIN_MARGIN)
                win_r = binary[y_lo:y_hi, xr_lo:xr_hi]
                nz_r  = np.count_nonzero(win_r)
                if nz_r >= self.MIN_PIX:
                    pts   = np.where(win_r > 0)[1]
                    cur_right = int(np.mean(pts)) + xr_lo
                    right_centers.append(cur_right)

        # ── Calcular centro y error ────────────────────────────
        frame_cx = w / 2.0
        bias     = self._right_bias   # 0=izq, 0.5=centro, 1=der

        if left_centers and right_centers:
            mean_l = float(np.mean(left_centers))
            mean_r = float(np.mean(right_centers))
            # Punto objetivo dentro del carril según sesgo:
            #   bias=0.5 → (mean_l + mean_r)/2   (centro)
            #   bias=1.0 → mean_r                (línea derecha)
            lane_cx    = mean_l + bias * (mean_r - mean_l)
            confidence = 1.0
            left_x_avg  = int(mean_l)
            right_x_avg = int(mean_r)
        elif left_centers:
            # Solo línea izquierda — estimar objetivo desplazado a la derecha
            # según el sesgo (más sesgo = más lejos de la izquierda).
            lane_cx    = np.mean(left_centers) + w * (0.20 + 0.16 * bias)
            confidence = 0.5
            left_x_avg  = int(np.mean(left_centers))
            right_x_avg = None
        elif right_centers:
            # Solo línea derecha — estimar objetivo desplazado a la izquierda
            # menos cuando el sesgo es derecho (queremos quedar cerca de ella).
            lane_cx    = np.mean(right_centers) - w * (0.36 - 0.16 * bias)
            confidence = 0.5
            left_x_avg  = None
            right_x_avg = int(np.mean(right_centers))
        else:
            return LaneResult(error_px=self._smooth_error, confidence=0.0)

        error = float(lane_cx - frame_cx)

        return LaneResult(
            error_px   = error,
            confidence = float(confidence),
            left_x     = left_x_avg if left_centers else None,
            right_x    = right_x_avg if right_centers else None,
        )

    # ─── Visualización de debug ───────────────────────────────────────────────

    def draw_debug(self, frame: np.ndarray, result: LaneResult) -> np.ndarray:
        """
        Dibuja la línea de carril detectada sobre el frame original.
        Retorna una copia anotada.
        """
        vis = frame.copy()
        H, W = vis.shape[:2]

        # Línea central del frame
        cv2.line(vis, (W // 2, H), (W // 2, H // 2), (0, 150, 150), 1)

        # Centro del carril detectado
        cx = W // 2 + int(result.error_px)
        cx = max(0, min(W - 1, cx))
        col = (0, 255, 0) if result.confidence >= 0.5 else (0, 80, 255)
        cv2.line(vis, (cx, H), (cx, H // 2), col, 3)

        # Info de texto
        cv2.putText(vis,
            f"err:{result.error_px:+.0f}px  conf:{result.confidence:.0%}",
            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2, cv2.LINE_AA)

        return vis
