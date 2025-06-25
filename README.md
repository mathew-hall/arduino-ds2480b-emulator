# Arduino DS2480B Emulator for owfs

This project allows an Arduino to implement just enough of the DS2480B behaviour to function with the owfs project. This may be useful to those migrating from a Raspberry Pi using the w1 hardware owfs driver to an x86 based Home Assistant server while reusing existing configurations and preserving sensor history.

This is a basic implementation built from the DS datasheet and owfs' DS9097U driver code. It doesn't implement the full command set but emulates everything apparently required in order to get owfs to read a network of DS18B20 sensors.

# Usage

Connect the onewire bus to pin 2 on the Arduino and USB to the host.

# owfs configuration

Set up owserver to use the tty in your `/etc/owfs.conf` file:

    # Serial port: DS9097
    server: device = /dev/ttyS1wire

# Missing functions:

* Most config operations do nothing, except baud rates
* Pulse commands (12v programming and 5v strong) do nothing
* Automatic pullup-after-byte
* Timing customisation
* High-speed modes