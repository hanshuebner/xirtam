## Synopsis

xirtam is a an interface device to connect a USB keyboard to a computer with a
matrix keyboard interface, i.e. an 8 bit home computer.

The repository contains the AT90USB1287 firmware source code and KiCAD schematic/PCB
design for the xirtam project.  It is a fork of the [SmallyMouse2](https://www.waitingforfriday.com/?p=827)
project that implements a similar device for realy computer mice.

## Motivation

8 bit home computers interface to their keyboard through a matrix style interface.
The main CPU periodically scans the keyboard matrix for key presses converts them to
characters or control information as required.  This is in contrast to more modern
machines, where a separate microcontroller performs the keyboard scanning task and
interfaces to the main CPU using a serial interface.

Being able to use an USB interface with an 8 bit home computer is often desireable
because of the bad quality that home computer keyboards typically have. Some don't
even have physical keys, but most use cheap mechanical switches.  Furthermore, the
keyboard is always integrated into the system unit, and a detachable keyboard is
much desireable.

## Implementation

Interfacing a modern keyboard to a computer with a matrix interface is slightly
tricky because of the way how the scanning is typically implemented.  The keyboard
is organized as a matrix of switches, with each switch connecting a particular pair
of a vertical and horizontal line of the matrix.  It does so by sending a high 
signal to one of the row lines of the matrix, and then reading back the
value sent by the keyboard on the column lines.  For each key pressed in the
particular row, the respective column bit will be high.

The scanning of the keyboard is a rapid process, with little time between when the
row was selected and the column being read.  Typically, there will only be a few
hundred nanoseconds between when the CPU selects a row and when it reads back the
column data.  Hence, software based solutions to emulate a keyboard matrix, while
they exist, require a very fast microcontroller or additional hardware or software
running on the micro to work.

xirtam approaches the timing problem by using a matrix switch chip that works
the same way as a physical keyboard.  The physical switches of the keyboard are
replaced by semiconductor switches inside of the chip which can be controlled
by a host interface.  When one of the switches is turned on by the host, it
connects the row and column at its intersection until the host turns if off again.
Hence, the scanning of the keyboard by the home computer can be done at arbitrary
rate, like with a physical keyboard.

Matrix switch chips are used in video and
audio crossbar applications and have been around for a long time.  They are
available in various configurations and sizes and are reasonably priced.  The
chip chosen for xirtam is the [MT093](https://www.microsemi.com/document-portal/doc_download/127014-mt093-datasheet-sept11).  
It was developed by Zarlink (now Microchip) and provides for 96 switches in an 
8x12 configuration.  It provides an easy to use interface for control, 
allowing the microcontroller to address one of the switches on a parallel address
bus and sending the desired switch state on a separate data bit.

xirtam supports both JTAG and USB bootloader programming.  The AT90USB1287 
is pre-programmed by Atmel with the (FLIP) DFU bootloader; this bootloader is 
recognised by Atmel Studio as a programming device and can be used to flash the 
firmware to the board.

## Installation

Note: This is an Atmel Studio 7 project that can be loaded and compiled by the IDE

Please see http://www.waitingforfriday.com/?p=827 for detailed documentation about SmallyMouse2

## Authors

SmallyMouse2 was written and is maintained by Simon Inns.  It was adapted to xirtam
by Hans Hübner.

## License (Software)

    SmallyMouse2 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SmallyMouse2 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SmallyMouse2. If not, see <http://www.gnu.org/licenses/>.

## License (Hardware)

Both the schematic and the PCB design of xirtam (i.e. the KiCAD project and files)
are covered by a Creative Commons license; as with the software you are welcome 
(and encouraged!) to extend, re-spin and otherwise use and modify the design as 
allowed by the license.  However; under the terms of the Attribution-ShareAlike 
4.0 International (CC BY-SA 4.0) license you are required to release your design 
5.(or redesign) under the same license.  For details of the licensing requirements 
6.please see <https://creativecommons.org/licenses/by-sa/4.0/>
