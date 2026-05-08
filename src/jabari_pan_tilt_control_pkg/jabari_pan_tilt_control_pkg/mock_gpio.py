# mock_gpio.py

# Constants
BCM = 'BCM'
OUT = 'OUT'
IN = 'IN'
HIGH = 1
LOW = 0

# Functions
def setmode(mode):
    print(f"[MOCK] GPIO.setmode({mode})")

def setup(pin, mode):
    print(f"[MOCK] GPIO.setup(pin={pin}, mode={mode})")

def output(pin, state):
    print(f"[MOCK] GPIO.output(pin={pin}, state={state})")

def input(pin):
    print(f"[MOCK] GPIO.input(pin={pin})")
    return LOW

def cleanup():
    print("[MOCK] GPIO.cleanup()")

# PWM class and factory
class MockPWM:
    def __init__(self, pin, freq):
        print(f"[MOCK] PWM initialized on pin {pin} with freq {freq}Hz")

    def start(self, duty):
        print(f"[MOCK] PWM started with duty cycle: {duty}%")

    def ChangeDutyCycle(self, duty):
        print(f"[MOCK] PWM changed duty cycle to: {duty}%")

    def stop(self):
        print("[MOCK] PWM stopped")

def PWM(pin, freq):
    return MockPWM(pin, freq)

def main():
    print("Mock GPIO initialized")


if __name__ == "__main__":
    main()
