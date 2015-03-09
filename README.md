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

Since 1-wire<sup>1</sup> requires an open-drain driver and uses 5v logic level, some additional hardware is needed.

<img src="https://rawgit.com/resetnow/one-slave/master/hardware/main.svg" alt="Hardware schematics" width="400">

Part | Description
------------ | -------------
IC1 | Any 3.3v LDO. Required if you plan to power the microcontroller from the 1-wire line
Q1 | Can be replaced too, check RDS(on) and operating voltage

<sup>1</sup> — 1-Wire is a trademark of [Maxim Integrated](http://www.maxim-ic.com).
