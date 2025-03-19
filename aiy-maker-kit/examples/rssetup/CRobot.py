from gpiozero import PWMOutputDevice, DigitalOutputDevice

class CRobot:
    def __init__(self, LMPins, RMPins, PWMPins):
        # Custom Robot Class for TB6612 Motor Driver

        # Left Motor Pins
        self.LMForward = DigitalOutputDevice(LMPins[0])  # AIN1
        self.LMBackwards = DigitalOutputDevice(LMPins[1])  # AIN2

        # Right motor pins
        self.RMForward = DigitalOutputDevice(RMPins[0])  # BIN1
        self.RMBackwards = DigitalOutputDevice(RMPins[1])  # BIN2

        # PWM control
        self.LPWM = PWMOutputDevice(PWMPins[0])  # PWMA
        self.RPWM = PWMOutputDevice(PWMPins[1])  # PWMB

    def backward(self, speed=1.0):
        self.LMForward.on()
        self.LMBackwards.off()
        self.RMForward.on()
        self.RMBackwards.off()
        self.LPWM.value = speed
        self.RPWM.value = speed

    def forward(self, speed=1.0):
        self.LMForward.off()
        self.LMBackwards.on()
        self.RMForward.off()
        self.RMBackwards.on()
        self.LPWM.value = speed
        self.RPWM.value = speed

    def left(self, speed=0.5):
        self.LMForward.off()
        self.LMBackwards.on()
        self.RMForward.on()
        self.RMBackwards.off()
        self.LPWM.value = speed
        self.RPWM.value= speed

    def right(self, speed=0.5):
        self.LMForward.on()
        self.LMBackwards.off()
        self.RMForward.off()
        self.RMBackwards.on()
        self.LPWM.value = speed
        self.RPWM.value = speed

    def stop(self):
        self.LMForward.off()
        self.LMBackwards.off()
        self.RMForward.off()
        self.RMBackwards.off()
        self.LPWM.value = 0
        self.RPWM.value = 0