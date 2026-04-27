# mock_rpi_global.py

import sys
import types
import os

# Ensure relative import works
import mock_gpio

rpi_module = types.ModuleType('RPi')
rpi_module.GPIO = mock_gpio

sys.modules['RPi'] = rpi_module
sys.modules['RPi.GPIO'] = mock_gpio

print("[INFO] mock_rpi_global: Injected mock GPIO into sys.modules")
