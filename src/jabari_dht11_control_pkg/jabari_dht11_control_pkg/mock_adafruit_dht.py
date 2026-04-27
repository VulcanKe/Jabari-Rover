# mock_adafruit_dht.py

class DHT11:
    def __init__(self):
        pass  # No-op constructor


def read_retry(sensor, pin):
    """
    Mock read_retry function that simulates sensor readings.
    """
    # Simulate stable readings
    humidity = 55.0  # percent
    temperature = 40.0  # Celsius
    return humidity, temperature