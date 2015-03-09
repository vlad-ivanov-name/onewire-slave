# one-slave
1-wire<sup>1</sup> slave device emulation library for MSP430 microcontrollers.

## Features

* Up to 4 emulated devices on a single microcontroller
* Interrupt-driven, uses low-power mode when idle
* Easily extensible

## Status & TODO

- [x] Basic protocol handling, reading/writing timeslots
- [x] Search ROM command
- [x] Match ROM command
- [x] 8-bit switch implementation (one-2408)
- [ ] Extension API documentation

## Hardware

This library version was tested on MSP430G2553; it sholud work on similiar microconrollers from the same family.

Refer to [this wiki page](https://github.com/resetnow/one-slave/wiki/Schematics) for schematics and further hardware-related info.

<sup>1</sup> — 1-Wire is a trademark of [Maxim Integrated](http://www.maxim-ic.com).
