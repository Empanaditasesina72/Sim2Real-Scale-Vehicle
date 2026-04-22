import numpy as np

class PerceptionOutput:
    def __init__(self):
        self.reset()

    def reset(self):
        """
        Reinicia todos los valores de percepción
        """
        self.distancia_obj = None
        self.tipo_obj = None
        self.lane_error = 0.0
        self.semaforo = "none"
        self.interseccion = False

    def actualizar_objetos(self, objetos_detectados):
        """
        Actualiza información de objetos detectados
        """
        if objetos_detectados:
            # Tomar el objeto más cercano
            objeto_cercano = min(objetos_detectados,
                               key=lambda x: x['distancia'] if x['distancia'] else float('inf'))

            self.distancia_obj = objeto_cercano['distancia']
            self.tipo_obj = objeto_cercano['tipo']
        else:
            self.distancia_obj = None
            self.tipo_obj = None

    def actualizar_semaforo(self, semaforo_info):
        """
        Actualiza información del semáforo
        """
        if semaforo_info:
            self.semaforo = semaforo_info['estado']
        else:
            self.semaforo = "none"

    def actualizar_lane_error(self, lane_center, frame_width):
        """
        Actualiza el error de carril
        """
        if lane_center is not None:
            frame_center = frame_width // 2
            self.lane_error = lane_center - frame_center
        else:
            self.lane_error = 0.0

    def detectar_interseccion(self, senales_detectadas):
        """
        Detecta si hay una intersección basada en señales
        """
        senales_interseccion = ["stop", "yield", "traffic_light"]
        self.interseccion = any(senal['tipo'] in senales_interseccion
                              for senal in senales_detectadas)

    def obtener_estado_percepcion(self):
        """
        Retorna el estado completo de la percepción
        """
        return {
            'distancia_obj': self.distancia_obj,
            'tipo_obj': self.tipo_obj,
            'lane_error': self.lane_error,
            'semaforo': self.semaforo,
            'interseccion': self.interseccion
        }

    def debug_info(self):
        """
        Retorna información de debug
        """
        estado = self.obtener_estado_percepcion()
        return f"Obj: {estado['tipo_obj']}@{estado['distancia_obj']:.1f}cm | " \
               f"Lane Error: {estado['lane_error']:.1f} | " \
               f"Semaforo: {estado['semaforo']} | " \
               f"Interseccion: {estado['interseccion']}"