/************************************************************************
	main.c

	Main functions
    xirtam - USB keyboard to switch matrix converter
    Copyright (C) 2017-2020 Simon Inns
    Copyright (C) 2021 Hans Hübner

	This file is part of xirtam.

    xirtam is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

	Email: hans.huebner@gmail.com

************************************************************************/

// Important notice:  If you port this firmware to another hardware design
// it *will be* a derivative of the original and therefore you must pay attention
// to the CC hardware licensing and release your hardware design as per the share-
// alike license.  Keep it open! ...and yes, that includes you Commodore chaps.

// System includes
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <stdio.h>
#include <stdlib.h>

#include <util/delay.h>

// Note: xirtam makes extensive use of the LUFA libraries:
//
// LUFA Library
// Copyright (C) Dean Camera, 2015.
// dean [at] fourwalledcubicle [dot] com
// www.lufa-lib.org

// Include LUFA libraries
#include <LUFA/Drivers/Misc/TerminalCodes.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Drivers/Peripheral/Serial.h>
#include <LUFA/Platform/Platform.h>

#include "ConfigDescriptor.h"

#include "usb_hid_keys.h"
#include "bit_array.h"
#include "main.h"

// Main function
int main(void)
{
  // Initialise the hardware
  initialiseHardware();
	
  // Enable interrupts (required for USB support and quadrature output ISRs)
  sei();
	
  // Main processing loop
  while(1) {
    // Perform any pending keyboard actions
    processKeyboard();

    // Process the USB host interface
    USB_USBTask();
  }
}

// Initialise the hardware
void initialiseHardware(void)
{
  // Disable the watchdog timer (if set in fuses)
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  // Disable the clock divider (if set in fuses)
  clock_prescale_set(clock_div_1);

  // Set the quadrature output pins to output
  X1_DDR |= X1; // Output
  X2_DDR |= X2; // Output
  Y1_DDR |= Y1; // Output
  Y2_DDR |= Y2; // Output
	
  // Set quadrature output pins to zero
  X1_PORT &= ~X1; // Pin = 0
  X2_PORT &= ~X2; // Pin = 0
  Y1_PORT &= ~Y1; // Pin = 0
  Y2_PORT &= ~Y2; // Pin = 0
	
  // Set mouse button output pins open drain
  // (solves issues with some retro machines that
  // don't like 5V to be sources from the pins)
  LB_DDR &= ~LB; // 0 = input
  MB_DDR &= ~MB; // 0 = input
  RB_DDR &= ~RB; // 0 = input
  LB_PORT &= ~LB; // Pin = 0 (off)
  MB_PORT &= ~MB; // Pin = 0 (off)
  RB_PORT &= ~RB; // Pin = 0 (off)

  // Set the rate limit configuration header to input
  RATESW_DDR &= ~RATESW; // Input
  RATESW_PORT |= RATESW; // Turn on weak pull-up
	
  // Configure E7 on the expansion header to act as
  // DPISW (since it is easily jumpered to 0V)	
  DPISW_DDR &= ~DPISW; // Input
  DPISW_PORT |= DPISW; // Turn on weak pull-up
	
  // Initialise the expansion (Ian) header
  E0_DDR |= ~E0; // Output
  E1_DDR |= ~E1; // Output
  E2_DDR |= ~E2; // Output
  E3_DDR |= ~E3; // Output
  E4_DDR |= ~E4; // Output
  E5_DDR |= ~E5; // Output
  E6_DDR |= ~E6; // Output
	
  E0_PORT &= ~E0; // Pin = 0
  E1_PORT &= ~E1; // Pin = 0
  E2_PORT &= ~E2; // Pin = 0
  E3_PORT &= ~E3; // Pin = 0
  E4_PORT &= ~E4; // Pin = 0
  E5_PORT &= ~E5; // Pin = 0
  E6_PORT &= ~E6; // Pin = 0
	
  // Initialise the LUFA USB stack
  USB_Init();
	
  // By default, xirtam will output USB debug events on the
  // AVR's UART port.  You can monitor the debug console by
  // connecting a serial to USB adapter.  Only the Tx (D3 and 0V
  // pins are required).
	
  // Initialise the serial UART - 9600 baud 8N1
  Serial_Init(9600, false);

  // Create a serial debug stream on stdio
  // Note: The serial UART is available on the 
  // expansion (Ian) header
  Serial_CreateStream(NULL);

  // Output some debug header information to the serial console
  puts_P(PSTR(ESC_FG_YELLOW "xirtam V0.1 - Serial debug console\r\n" ESC_FG_WHITE));
  puts_P(PSTR(ESC_FG_YELLOW "(c)2021 Hans Huebner\r\n" ESC_FG_WHITE));
}

// Read the keyboard USB report and process the information
void processKeyboard(void)
{
  USB_KeyboardReport_Data_t KeyboardReport;
		
  // Only process the keyboard if it is attached to the USB port
  if (USB_HostState != HOST_STATE_Configured) {
    return;
  }
	
  // Select keyboard data pipe
  Pipe_SelectPipe(KEYBOARD_DATA_IN_PIPE);

  // Unfreeze keyboard data pipe
  Pipe_Unfreeze();

  // Has a packet arrived from the USB device?
  if (!(Pipe_IsINReceived())) {
    // Nothing received, exit...
    Pipe_Freeze();
    return;
  }

  // Ensure pipe is in the correct state before reading
  if (Pipe_IsReadWriteAllowed()) {
		
    // Read in keyboard report data
    Pipe_Read_Stream_LE(&KeyboardReport, sizeof(KeyboardReport), NULL);

    printf_P(PSTR("Keyboard report received - Modifier 0x%02X %d %d %d %d %d %d\r\n"),
             KeyboardReport.Modifier,
             KeyboardReport.KeyCode[0], KeyboardReport.KeyCode[1], KeyboardReport.KeyCode[2],
             KeyboardReport.KeyCode[3], KeyboardReport.KeyCode[4], KeyboardReport.KeyCode[5]);
  }

  // Clear the IN endpoint, ready for next data packet
  Pipe_ClearIN();

  // Refreeze keyboard data pipe
  Pipe_Freeze();
}

// LUFA event handlers ------------------------------------------------------------------------------------------------

// Event handler for the USB_DeviceAttached event. This indicates that a device has been attached to the host, and
// starts the library USB task to begin the enumeration and USB management process.
void EVENT_USB_Host_DeviceAttached(void)
{
  puts_P(PSTR(ESC_FG_GREEN "USB Device attached\r\n" ESC_FG_WHITE));
}

// Event handler for the USB_DeviceUnattached event. This indicates that a device has been removed from the host, and
// stops the library USB task management process.
void EVENT_USB_Host_DeviceUnattached(void)
{
  puts_P(PSTR(ESC_FG_GREEN "USB Device detached\r\n" ESC_FG_WHITE));
}

// Event handler for the USB_DeviceEnumerationComplete event. This indicates that a device has been successfully
// enumerated by the host and is now ready to be used by the application.
void EVENT_USB_Host_DeviceEnumerationComplete(void)
{
  puts_P(PSTR("Getting configuration data from device...\r\n"));

  uint8_t ErrorCode;

  /* Get and process the configuration descriptor data */
  if ((ErrorCode = ProcessConfigurationDescriptor()) != SuccessfulConfigRead) {
    if (ErrorCode == ControlError) {
      puts_P(PSTR(ESC_FG_RED "Control Error (Get configuration)!\r\n"));
    } else {
      puts_P(PSTR(ESC_FG_RED "Invalid Device!\r\n"));
    }

    printf_P(PSTR(" -- Error Code: %d\r\n" ESC_FG_WHITE), ErrorCode);
    return;
  }

  // Set the device configuration to the first configuration (rarely do devices use multiple configurations)
  if ((ErrorCode = USB_Host_SetDeviceConfiguration(1)) != HOST_SENDCONTROL_Successful) {
    printf_P(PSTR(ESC_FG_RED "Control Error (Set configuration)!\r\n"
                  " -- Error Code: %d\r\n" ESC_FG_WHITE), ErrorCode);
    return;
  }
	
  // HID class request to set the keyboard protocol to the Boot Protocol
  USB_ControlRequest = (USB_Request_Header_t) {
                                               .bmRequestType = (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE),
                                               .bRequest      = HID_REQ_SetProtocol,
                                               .wValue        = 0,
                                               .wIndex        = 0,
                                               .wLength       = 0,
  };

  // Select the control pipe for the request transfer
  Pipe_SelectPipe(PIPE_CONTROLPIPE);

  // Send the request, display error and wait for device detach if request fails
  if ((ErrorCode = USB_Host_SendControlRequest(NULL)) != HOST_SENDCONTROL_Successful) {
    printf_P(PSTR(ESC_FG_RED "Control Error (Set protocol)!\r\n"
                  " -- Error Code: %d\r\n" ESC_FG_WHITE), ErrorCode);

    USB_Host_SetDeviceConfiguration(0);
    return;
  }

  puts_P(PSTR("USB keyboard enumeration successful\r\n"));
}

// Event handler for the USB_HostError event. This indicates that a hardware error occurred while in host mode.
void EVENT_USB_Host_HostError(const uint8_t ErrorCode)
{
  USB_Disable();

  printf_P(PSTR(ESC_FG_RED "Host Mode Error!\r\n"
                " -- Error Code %d\r\n" ESC_FG_WHITE), ErrorCode);

  while(1);
}

// Event handler for the USB_DeviceEnumerationFailed event. This indicates that a problem occurred while
// enumerating an attached USB device.
void EVENT_USB_Host_DeviceEnumerationFailed(const uint8_t ErrorCode,
                                            const uint8_t SubErrorCode)
{
  printf_P(PSTR(ESC_FG_RED "Device Enumeration Error!\r\n"
                " -- Error Code %d\r\n"
                " -- Sub Error Code %d\r\n"
                " -- In State %d\r\n" ESC_FG_WHITE), ErrorCode, SubErrorCode, USB_HostState);
}


