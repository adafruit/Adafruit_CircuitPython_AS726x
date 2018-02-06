Introduction
============

.. image:: https://readthedocs.org/projects/adafruit-circuitpython-as726x/badge/?version=latest
    :target: https://circuitpython.readthedocs.io/projects/as726x/en/latest/
    :alt: Documentation Status

.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://discord.gg/nBQh6qu
    :alt: Discord

.. image:: https://travis-ci.org/adafruit/adafruit_CircuitPython_AS726x.svg?branch=master
    :target: https://travis-ci.org/adafruit/adafruit_CircuitPython_AS726x
    :alt: Build Status

Driver for the AS726x spectral sensors

Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_
* `Register <https://github.com/adafruit/Adafruit_CircuitPython_Register>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://github.com/adafruit/Adafruit_CircuitPython_Bundle>`_.

Usage Example
=============
```
import time

import board
import busio

from adafruit_as726x import Adafruit_AS726x

#maximum value for sensor reading
max_val = 16000

#max number of characters in each graph
max_graph = 80

def graph_map(x):
    return min(int(x * max_graph / max_val), max_graph)

# Initialize I2C bus and sensor.
i2c = busio.I2C(board.SCL, board.SDA)
sensor = Adafruit_AS726x(i2c)

sensor.conversion_mode = sensor.MODE_2

while 1:
    #wait for data to be ready
    while not sensor.data_ready:
        time.sleep(.1)

    #plot plot the data
    print("\n")
    print("V: " + graph_map(sensor.violet_calibrated)*'=')
    print("B: " + graph_map(sensor.blue_calibrated)*'=')
    print("G: " + graph_map(sensor.green_calibrated)*'=')
    print("Y: " + graph_map(sensor.yellow_calibrated)*'=')
    print("O: " + graph_map(sensor.orange_calibrated)*'=')
    print("R: " + graph_map(sensor.red_calibrated)*'=')

    time.sleep(1)
```

Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/adafruit/adafruit_CircuitPython_AS726x/blob/master/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.

Building locally
================

Zip release files
-----------------

To build this library locally you'll need to install the
`circuitpython-build-tools <https://github.com/adafruit/circuitpython-build-tools>`_ package.

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install circuitpython-build-tools

Once installed, make sure you are in the virtual environment:

.. code-block:: shell

    source .env/bin/activate

Then run the build:

.. code-block:: shell

    circuitpython-build-bundles --filename_prefix adafruit-circuitpython-as726x --library_location .

Sphinx documentation
-----------------------

Sphinx is used to build the documentation based on rST files and comments in the code. First,
install dependencies (feel free to reuse the virtual environment from above):

.. code-block:: shell

    python3 -m venv .env
    source .env/bin/activate
    pip install Sphinx sphinx-rtd-theme

Now, once you have the virtual environment activated:

.. code-block:: shell

    cd docs
    sphinx-build -E -W -b html . _build/html

This will output the documentation to ``docs/_build/html``. Open the index.html in your browser to
view them. It will also (due to -W) error out on any warning like Travis will. This is a good way to
locally verify it will pass.
