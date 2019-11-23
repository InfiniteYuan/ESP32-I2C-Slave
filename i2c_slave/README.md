# I2C Slave Example

idf commit id: 90af68bd3c1941d9f5513bee22a12de7250208d7

## Overview

This example demonstrates basic usage of I2C driver by running two tasks on I2C bus:

1. Use one of ESP32’s I2C port (master mode) to read and write another I2C port (slave mode) in ESP32.

If you have a new I2C application to go (for example, read the temperature data from external sensor with I2C interface), try this as a basic template, then add your own code.
