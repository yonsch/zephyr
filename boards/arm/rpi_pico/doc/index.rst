.. _rpi_pico:

Raspberry Pi Pico
#################

Overview
********

The Raspberry Pi Pico is a small, low-cost, versatile board from
Raspberry Pi. It is equipped with an RP2040 SoC, an on-board LED,
a USB connector, and an SWD interface. The USB bootloader allows it
to be flashed without any adapter, in a drag-and-drop manner.
It is also possible to flash and debug the Pico with its SWD interface,
using an external adapter.

Hardware
********
- Dual core Arm Cortex-M0+ processor running up to 133MHz
- 264KB on-chip SRAM
- 2MB on-board QSPI flash with XIP capabilities
- 26 GPIO pins
- 3 Analog inputs
- 2 UART peripherals
- 2 SPI controllers
- 2 I2C controllers
- 16 PWM channels
- USB 1.1 controller (host/device)
- 8 Programmable I/O (PIO) for custom peripherals
- On-board LED


.. figure:: img/rpi_pico.png
     :width: 150px
     :align: center
     :alt: Raspberry Pi Pico

     Raspberry Pi Pico (Image courtesy of Raspberry Pi)

Supported Features
==================

The rpi_pico board configuration supports the following
hardware features:

.. list-table::
   :header-rows: 1

   * - Peripheral
     - Kconfig option
     - Devicetree compatible
   * - NVIC
     - N/A
     - :dtcompatible:`arm,v6m-nvic`
   * - UART
     - :kconfig:`CONFIG_SERIAL`
     - :dtcompatible:`rpi,pico-uart`
   * - GPIO
     - :kconfig:`CONFIG_GPIO`
     - :dtcompatible:`rpi,pico-gpio`
