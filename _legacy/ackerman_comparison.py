# -*- coding: utf-8 -*-
"""
Comparación: Ackerman vs Control Simple de Dirección

Este archivo demuestra las diferencias entre diferentes aproximaciones
de control de dirección para vehículos autónomos.
"""

import numpy as np
import matplotlib.pyplot as plt

class AckermanComparison:
    def __init__(self):
        # Parámetros del vehículo
        self.wheelbase = 0.25  # Distancia entre ejes (m)
        self.track_width = 0.18  # Ancho de vía (m)
        self.max_steering_angle = np.radians(30)

    def ackerman_steering(self, steering_angle_center):
        """
        Geometría de Ackerman tradicional
        Retorna ángulos diferentes para ruedas izquierda y derecha
        """
        steering_angle_center = np.clip(steering_angle_center,
                                      -self.max_steering_angle,
                                      self.max_steering_angle)

        if abs(steering_angle_center) < 1e-6:
            return 0, 0  # Recto

        # Ackerman: ruedas giran a ángulos diferentes
        tan_outer = self.wheelbase / (self.wheelbase / np.tan(steering_angle_center) + self.track_width/2)
        steering_angle_outer = np.arctan(tan_outer)

        tan_inner = self.wheelbase / (self.wheelbase / np.tan(steering_angle_center) - self.track_width/2)
        steering_angle_inner = np.arctan(tan_inner)

        return steering_angle_outer, steering_angle_inner

    def simple_steering(self, steering_angle):
        """
        Control simple: ambas ruedas giran al mismo ángulo
        """
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        return steering_angle, steering_angle  # Ambas ruedas igual

    def compare_turning_radii(self, steering_angle):
        """
        Compara radios de giro entre Ackerman y simple
        """
        # Ackerman
        outer, inner = self.ackerman_steering(steering_angle)
        ackerman_radius = self.wheelbase / np.tan((outer + inner) / 2)  # Radio promedio

        # Simple
        simple_radius = self.wheelbase / np.tan(steering_angle)

        return ackerman_radius, simple_radius

    def plot_comparison(self):
        """
        Grafica la comparación entre Ackerman y control simple
        """
        angles = np.linspace(-np.radians(25), np.radians(25), 50)

        ackerman_radii = []
        simple_radii = []
        ackerman_outer = []
        ackerman_inner = []
        simple_angles = []

        for angle in angles:
            # Radios
            ack_r, simple_r = self.compare_turning_radii(angle)
            ackerman_radii.append(ack_r)
            simple_radii.append(simple_r)

            # Ángulos Ackerman
            outer, inner = self.ackerman_steering(angle)
            ackerman_outer.append(np.degrees(outer))
            ackerman_inner.append(np.degrees(inner))

            # Ángulos simples
            simple_angles.append(np.degrees(angle))

        # Crear figura
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))

        # 1. Comparación de radios de giro
        ax1.plot(np.degrees(angles), ackerman_radii, 'b-', label='Ackerman', linewidth=2)
        ax1.plot(np.degrees(angles), simple_radii, 'r--', label='Simple', linewidth=2)
        ax1.set_xlabel('Ángulo de Dirección (grados)')
        ax1.set_ylabel('Radio de Giro (m)')
        ax1.set_title('Comparación de Radios de Giro')
        ax1.legend()
        ax1.grid(True)

        # 2. Ángulos de ruedas Ackerman
        ax2.plot(np.degrees(angles), ackerman_outer, 'b-', label='Rueda Exterior', linewidth=2)
        ax2.plot(np.degrees(angles), ackerman_inner, 'g-', label='Rueda Interior', linewidth=2)
        ax2.plot(np.degrees(angles), simple_angles, 'r--', label='Simple (igual)', linewidth=2)
        ax2.set_xlabel('Ángulo Central (grados)')
        ax2.set_ylabel('Ángulo de Rueda (grados)')
        ax2.set_title('Ángulos de las Ruedas')
        ax2.legend()
        ax2.grid(True)

        # 3. Diferencia de ángulos
        angle_diff = np.array(ackerman_outer) - np.array(ackerman_inner)
        ax3.plot(np.degrees(angles), angle_diff, 'purple', linewidth=2)
        ax3.set_xlabel('Ángulo Central (grados)')
        ax3.set_ylabel('Diferencia (grados)')
        ax3.set_title('Diferencia Ackerman: Exterior - Interior')
        ax3.grid(True)

        # 4. Error relativo
        relative_error = (np.array(simple_radii) - np.array(ackerman_radii)) / np.array(ackerman_radii) * 100
        ax4.plot(np.degrees(angles), relative_error, 'orange', linewidth=2)
        ax4.set_xlabel('Ángulo Central (grados)')
        ax4.set_ylabel('Error Relativo (%)')
        ax4.set_title('Error del Control Simple vs Ackerman')
        ax4.grid(True)

        plt.tight_layout()
        plt.show()

def main():
    """
    Función principal de comparación
    """
    print("=== COMPARACIÓN: ACKERMAN vs CONTROL SIMPLE ===\n")

    comparator = AckermanComparison()

    # Ángulos de ejemplo
    test_angles = [np.radians(10), np.radians(20), np.radians(30)]

    print("Comparación de ángulos de dirección:")
    print("Ángulo Central | Ackerman (Ext, Int) | Simple (Ambas)")
    print("-" * 55)

    for angle in test_angles:
        ack_outer, ack_inner = comparator.ackerman_steering(angle)
        simple_left, simple_right = comparator.simple_steering(angle)

        print(".1f")

    print("\n" + "="*60)
    print("CONCLUSIONES:")
    print("="*60)
    print("1. ACKERMAN es ideal para:")
    print("   - Coches con dirección independiente en cada rueda")
    print("   - Alta precisión en giros")
    print("   - Vehículos de tamaño real")
    print()
    print("2. CONTROL SIMPLE es adecuado para:")
    print("   - Ruedas delanteras que giran juntas")
    print("   - Sistemas RC y prototipos")
    print("   - Cuando la precisión no es crítica")
    print()
    print("3. PARA TU SISTEMA: Usa control simple + PID")
    print("   - Tu lane detector ya tiene control proporcional")
    print("   - Ambas ruedas giran igual → control simple")
    print("   - Para parking: algoritmos específicos de maniobra")
    print()
    print("4. ALTERNATIVAS AVANZADAS:")
    print("   - Model Predictive Control (MPC)")
    print("   - Control LQR (Linear Quadratic Regulator)")
    print("   - Pure Pursuit algorithm")
    print("   - Stanley controller")

    # Mostrar gráfica si matplotlib está disponible
    try:
        comparator.plot_comparison()
    except ImportError:
        print("\nInstala matplotlib para ver las gráficas: pip install matplotlib")

if __name__ == "__main__":
    main()