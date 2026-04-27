import pigpio
import time

class DHT11:
    def __init__(self, pi, gpio):
        self.pi = pi
        self.gpio = gpio

    def read(self):
        MAX_WAIT = 100
        self.pi.set_mode(self.gpio,  pigpio.OUTPUT)
        self.pi.write(self.gpio, 0)
        time.sleep(0.018)
        self.pi.set_mode(self.gpio, pigpio.INPUT)

        # Wait for sensor response
        t0 = time.time()
        count = 0
        while self.pi.read(self.gpio) == 1:
            count += 1
            if count > MAX_WAIT:
                return None

        # Read data
        data = []
        for i in range(40):
            while self.pi.read(self.gpio) == 0:
                pass
            t1 = time.time()
            while self.pi.read(self.gpio) == 1:
                pass
            t2 = time.time()
            data.append(t2 - t1)

        bits = [1 if bit > 0.00005 else 0 for bit in data]
        bytes_ = []
        for i in range(0, 40, 8):
            byte = 0
            for j in range(8):
                byte = byte << 1 | bits[i + j]
            bytes_.append(byte)

        if len(bytes_) != 5 or ((sum(bytes_[:4]) & 0xFF) != bytes_[4]):
            return None

        return {
            "temperature": bytes_[2],
            "humidity": bytes_[0]
        }
