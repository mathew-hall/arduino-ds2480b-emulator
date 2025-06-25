# Arduino DS2480B Emulator for owfs

This project allows an Arduino to implement just enough of the DS2480B behaviour to function with the owfs project. This may be useful to those migrating from a Raspberry Pi using the w1 hardware owfs driver to an x86 based Home Assistant server while reusing existing configurations and preserving sensor history.

This is a basic implementation built from the DS datasheet and owfs' DS9097U driver code. It doesn't implement the full command set but emulates everything apparently required in order to get owfs to read a network of DS18B20 sensors.

# Usage

Connect the onewire bus to pin 2 on the Arduino and USB to the host.

# owfs configuration

Set up owserver to use the tty in your `/etc/owfs.conf` file:

    # Serial port: DS9097
    server: device = /dev/ttyS1wire

# Working features:

* Search acceleration (see note)
* Byte writes/reads (read the bus by sending FF)
* Bus reset
* Baud rate changes
* Bit writes/reads (read the bus by writing 1)

# Missing functions:

* Most config operations do nothing, except baud rates
* Pulse commands (12v programming and 5v strong) do nothing
* Automatic pullup-after-byte
* Timing customisation
* High-speed modes

While the search mode *does* work, the emulator doesn't correctly enter CHECK mode if it receives an E3 byte within the 16 byte search packet. This seems fine because the search accelerator requires the client to send padding bits, which are hopefully set to 0, but if a client chooses to send 1s and also escapes the E3 byte by doubling it, then this code will likely produce unexpected results.

# Hacking

There is a debug mode available by compiling with DEBUG defined. This adds a lot of explanatory output on the serial port, which will also break functionality. To develop on this code, it's easiest to run owserver with debug options:

    owhttpd -d /dev/tty.usbserial-1410 --8bit --baud=9600 --debug --traffic --port 1234

This logs the bytes written and read. Then, if a problem occurs (e.g. the device responds with an unexpected value or times out) the input sequence can be reconstructed from the log and replayed with debugging enabled