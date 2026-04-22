import RPi.GPIO as GPIO


class MotorDriver:

    def __init__(self, pin, frequency=1000):
        """
        pin: GPIO donde está conectado el control del motor
        frequency: frecuencia PWM (Hz)
        """

        self.pin = pin

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)

        # Crear PWM
        self.pwm = GPIO.PWM(self.pin, frequency)

        # iniciar apagado
        self.pwm.start(0)


    def set_speed(self, percent):
        """
        Cambia la velocidad del motor en porcentaje
        0 = apagado
        100 = máxima potencia
        """

        percent = max(0, min(100, percent))

        self.pwm.ChangeDutyCycle(percent)


    def stop(self):
        """
        Detiene el motor
        """

        self.pwm.ChangeDutyCycle(0)


    def cleanup(self):
        """
        Limpia los pines GPIO
        """

        self.pwm.stop()
        GPIO.cleanup()
