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

#include "usb_hid_keys.h"
#include "bit_array.h"

// USB configuration

/** HID Report Descriptor Usage Page value for a desktop keyboard. */
#define USAGE_PAGE_KEYBOARD      0x07

USB_ClassInfo_HID_Host_t Keyboard_HID_Interface =
  {
   .Config = {
              .DataINPipe = {
                             .Address = (PIPE_DIR_IN  | 1),
                             .Banks = 1,
                             },
              .DataOUTPipe = {
                              .Address        = (PIPE_DIR_OUT | 2),
                              .Banks          = 1,
                              },
              .HIDInterfaceProtocol = HID_CSCP_KeyboardBootProtocol
              },
  };


// MT093 hardware interface

#define BIT_STROBE 0x01
#define BIT_DATA   0x02
#define BIT_TXD    0x08
#define BIT_RESET  0x10

// Matrix definition

#define MATRIX_ROWS 7
#define MATRIX_COLUMNS 8

#define MATRIX_COUNT (MATRIX_ROWS * MATRIX_COLUMNS)

// Mapping from USB HID keyboard codes to matrix addresses

typedef struct {
  uint8_t row;
  uint8_t column;
} matrix_address_t;

#define MATRIX(r, c) { .row = r, .column = c }

typedef struct {
  uint8_t input_code;
  matrix_address_t matrix_address1, matrix_address2;
} key_map_t;

#define MAP(code, row, column) { code, MATRIX(row, column), MATRIX(row, column) }
#define MAP2(code, row1, column1, row2, column2) { code, MATRIX(row1, column1), MATRIX(row2, column2) }

#define LAST_MAP_ENTRY { .input_code = 0 }

key_map_t modifier_map[] =
  {
   MAP(KEY_MOD_LCTRL, 3, 6),
   MAP(KEY_MOD_LSHIFT, 3, 2),
   MAP(KEY_MOD_LALT, 3, 5),
   MAP(KEY_MOD_RCTRL, 3, 6),
   MAP(KEY_MOD_RSHIFT, 3, 2),
   MAP(KEY_MOD_RALT, 3, 5),
   LAST_MAP_ENTRY
  };

key_map_t key_map[] =
  {
   MAP(KEY_A, 1, 2),
   MAP(KEY_B, 2, 7),
   MAP(KEY_C, 5, 7),
   MAP(KEY_D, 5, 2),
   MAP(KEY_E, 5, 6),
   MAP(KEY_F, 6, 2),
   MAP(KEY_G, 2, 2),
   MAP(KEY_H, 2, 3),
   MAP(KEY_I, 5, 0),
   MAP(KEY_J, 6, 3),
   MAP(KEY_K, 5, 3),
   MAP(KEY_L, 4, 3),
   MAP(KEY_M, 6, 4),
   MAP(KEY_N, 2, 4),
   MAP(KEY_O, 4, 0),
   MAP(KEY_P, 1, 0),
   MAP(KEY_Q, 1, 6),
   MAP(KEY_R, 6, 6),
   MAP(KEY_S, 4, 2),
   MAP(KEY_T, 2, 6),
   MAP(KEY_U, 6, 0),
   MAP(KEY_V, 6, 7),
   MAP(KEY_W, 4, 6),
   MAP(KEY_X, 4, 7),
   MAP(KEY_Y, 2, 0),
   MAP(KEY_Z, 1, 7),
   MAP(KEY_0, 1, 1),
   MAP(KEY_1, 1, 5),
   MAP(KEY_2, 4, 5),
   MAP(KEY_3, 5, 5),
   MAP(KEY_4, 6, 5),
   MAP(KEY_5, 2, 5),
   MAP(KEY_6, 2, 1),
   MAP(KEY_7, 6, 1),
   MAP(KEY_8, 5, 1),
   MAP(KEY_9, 4, 1),
   MAP(KEY_ENTER, 3, 0),
   MAP(KEY_KPENTER, 3, 0),
   MAP(KEY_EQUAL, 3, 4),
   MAP(KEY_SPACE, 3, 3),
   MAP(KEY_DOT, 4, 4),
   MAP(KEY_COMMA, 5, 4),
   MAP(KEY_SEMICOLON, 1, 3),
   MAP(KEY_SLASH, 1, 4),
   MAP2(KEY_LEFT, 3, 7, 1, 2),
   MAP2(KEY_BACKSPACE, 3, 7, 1, 2),
   MAP2(KEY_RIGHT, 3, 7, 5, 2),
   MAP2(KEY_UP, 3, 7, 5, 6),
   MAP2(KEY_DOWN, 3, 7, 4, 7),
   MAP2(KEY_BACKSLASH, 3, 7, 1, 7),
   MAP2(KEY_GRAVE, 3, 7, 5, 7),
   MAP2(KEY_LEFTBRACE, 3, 7, 6, 6),
   MAP2(KEY_RIGHTBRACE, 3, 7, 2, 6),
   MAP2(KEY_MINUS, 3, 7, 6, 0),
   MAP2(KEY_APOSTROPHE, 3, 7, 4, 0),
   MAP2(KEY_PAUSE, 3, 7, 3, 4),
   MAP2(KEY_F1, 3, 7, 1, 5),
   MAP2(KEY_F2, 3, 7, 4, 5),
   MAP2(KEY_F3, 3, 7, 5, 5),
   MAP2(KEY_F4, 3, 7, 6, 5),
   MAP2(KEY_F5, 3, 7, 2, 5),
   MAP2(KEY_F6, 3, 7, 2, 1),
   MAP2(KEY_F7, 3, 7, 6, 1),
   MAP2(KEY_F8, 3, 7, 5, 1),
   MAP2(KEY_F9, 3, 7, 4, 1),
   MAP2(KEY_F10, 3, 7, 1, 1),
   LAST_MAP_ENTRY
  };

key_map_t shift_key_map[] =
  {
   MAP2(KEY_BACKSLASH, 3, 7, 1, 2),
   MAP2(KEY_GRAVE, 3, 7, 4, 6),
   MAP2(KEY_LEFTBRACE, 3, 7, 6, 2),
   MAP2(KEY_RIGHTBRACE, 3, 7, 2, 2),
   MAP2(KEY_SLASH, 3, 7, 5, 0),
   MAP2(KEY_APOSTROPHE, 3, 7, 1, 0),
   MAP2(KEY_MINUS, 3, 2, 3, 4),
   LAST_MAP_ENTRY
  };

void
set_bit(BIT_ARRAY* bits, uint8_t row, uint8_t column) {
  bit_array_set(bits, row * MATRIX_COLUMNS + column);
}

key_map_t*
find_in_map(uint8_t input_code, key_map_t map[]) {
  for (int i = 0; map[i].input_code; i++) {
    if (map[i].input_code == input_code) {
      return &(map[i]);
    }
  }
  return 0;
}

void
map_code(BIT_ARRAY* bits, uint8_t code, key_map_t map[])
{
  key_map_t* map_entry = find_in_map(code, map);
  if (map_entry) {
    set_bit(bits, map_entry->matrix_address1.row, map_entry->matrix_address1.column);
    set_bit(bits, map_entry->matrix_address2.row, map_entry->matrix_address2.column);
  }
}

void
map_modifiers(BIT_ARRAY* bits, uint8_t modifier_mask)
{
  for (uint8_t i = 0; i < 8; i++) {
    uint8_t code = 1 << i;
    if (modifier_mask & code) {
      map_code(bits, code, modifier_map);
    }
  }
}

void
map_key(BIT_ARRAY* bits, uint8_t code, key_map_t map[]) {
  if (code > 1) {
    map_code(bits, code, map);
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

  // Define the output pins
  DDRC |= 0x7F; // Matrix address
  DDRD |= BIT_STROBE | BIT_DATA | BIT_TXD | BIT_RESET;

  // Reset the MT093 chip
  PORTD |= BIT_RESET;
  PORTD &= ~BIT_RESET;

  // Initialise the LUFA USB stack
  USB_Init();

  // By default, xirtam will output USB debug events on the
  // AVR's UART port.  You can monitor the debug console by
  // connecting a serial to USB adapter.  Only the Tx (D3 and 0V
  // pins are required).

  // Initialise the serial UART - 9600 baud 8N1
  Serial_Init(9600, false);

  // Create a serial debug stream on stdio
  Serial_CreateStream(NULL);

  // Output some debug header information to the serial console
  puts_P(PSTR(ESC_FG_YELLOW "xirtam V0.1 - Serial debug console\r\n" ESC_FG_WHITE));
  puts_P(PSTR(ESC_FG_YELLOW "(c)2021 Hans Huebner\r\n" ESC_FG_WHITE));
}

void
matrix_set(uint8_t row, uint8_t column, uint8_t value) {
  printf_P(PSTR("%d/%d => %d\r\n"), row, column, value);

  PORTC = (row << 4) | column;
  if (value) {
    PORTD |= BIT_DATA;
  } else {
    PORTD &= ~BIT_DATA;
  }
  PORTD |= BIT_STROBE;
  PORTD &= ~BIT_STROBE;

  // Reset output ports for easier debugging
  PORTD &= ~BIT_DATA;
  PORTC = 0;
}

void
processReport(USB_KeyboardReport_Data_t* keyboard_report, BIT_ARRAY* old_state) {
  BIT_ARRAY* new_state = bit_array_create(MATRIX_COUNT);
  bit_array_clear_all(new_state);
  bit_index_t next_bit_set = 0;

  // First attempt to map shifted keys - If one of those is found, we
  // don't consult the other maps.
  if (keyboard_report->Modifier & (KEY_MOD_LSHIFT | KEY_MOD_RSHIFT)) {
    for (int i = 0; i < 6; i++) {
      map_key(new_state, keyboard_report->KeyCode[i], shift_key_map);
    }
  }
  if (bit_array_num_bits_set(new_state) == 0) {
    map_modifiers(new_state, keyboard_report->Modifier);
    for (int i = 0; i < 6; i++) {
      map_key(new_state, keyboard_report->KeyCode[i], key_map);
    }
  }

  bit_array_or(old_state, old_state, new_state);

  while (bit_array_find_next_set_bit(old_state, next_bit_set, &next_bit_set)) {
    uint8_t row = next_bit_set / MATRIX_COLUMNS;
    uint8_t column = next_bit_set % MATRIX_COLUMNS;
    matrix_set(row, column, bit_array_get_bit(new_state, next_bit_set));
    if (++next_bit_set == MATRIX_COUNT) {
      break;
    }
  }

  bit_array_copy_all(old_state, new_state);

  bit_array_free(new_state);
}

// Read the keyboard USB report and process the information
void processKeyboard(BIT_ARRAY* old_state)
{
  if ((USB_HostState == HOST_STATE_Configured)
      && HID_Host_IsReportReceived(&Keyboard_HID_Interface)) {

    USB_KeyboardReport_Data_t keyboard_report;
    HID_Host_ReceiveReport(&Keyboard_HID_Interface, &keyboard_report);

    printf_P(PSTR("Mod 0x%02x Keys 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n"),
             keyboard_report.Modifier,
             keyboard_report.KeyCode[0], keyboard_report.KeyCode[1], keyboard_report.KeyCode[2],
             keyboard_report.KeyCode[3], keyboard_report.KeyCode[4], keyboard_report.KeyCode[5]);

    processReport(&keyboard_report, old_state);
  }
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
void
EVENT_USB_Host_DeviceEnumerationComplete(void)
{
  uint16_t ConfigDescriptorSize;
  uint8_t  ConfigDescriptorData[512];

  if (USB_Host_GetDeviceConfigDescriptor(1, &ConfigDescriptorSize, ConfigDescriptorData,
                                         sizeof(ConfigDescriptorData)) != HOST_GETCONFIG_Successful) {
    puts_P(PSTR("Error Retrieving Configuration Descriptor.\r\n"));
    return;
  }

  if (HID_Host_ConfigurePipes(&Keyboard_HID_Interface,
                              ConfigDescriptorSize, ConfigDescriptorData) != HID_ENUMERROR_NoError) {
    puts_P(PSTR("Attached Device Not a Valid Keyboard.\r\n"));
    return;
  }

  if (USB_Host_SetDeviceConfiguration(1) != HOST_SENDCONTROL_Successful) {
    puts_P(PSTR("Error Setting Device Configuration.\r\n"));
    return;
  }

  if (HID_Host_SetBootProtocol(&Keyboard_HID_Interface) != 0) {
    puts_P(PSTR("Could not Set Boot Protocol Mode.\r\n"));
    USB_Host_SetDeviceConfiguration(0);
    return;
  }

  puts_P(PSTR("Keyboard successfully.\r\n"));
}

// Event handler for the USB_HostError event. This indicates that a hardware error occurred while in host mode.
void
EVENT_USB_Host_HostError(const uint8_t ErrorCode)
{
  USB_Disable();

  printf_P(PSTR(ESC_FG_RED "Host Mode Error!\r\n"
                " -- Error Code %d\r\n" ESC_FG_WHITE), ErrorCode);

  while(1);
}

// Event handler for the USB_DeviceEnumerationFailed event. This indicates that a problem occurred while
// enumerating an attached USB device.
void
EVENT_USB_Host_DeviceEnumerationFailed(const uint8_t ErrorCode,
                                            const uint8_t SubErrorCode)
{
  printf_P(PSTR(ESC_FG_RED "Device Enumeration Error!\r\n"
                " -- Error Code %d\r\n"
                " -- Sub Error Code %d\r\n"
                " -- In State %d\r\n" ESC_FG_WHITE), ErrorCode, SubErrorCode, USB_HostState);
}

bool
CALLBACK_HIDParser_FilterHIDReportItem(HID_ReportItem_t* const CurrentItem)
{
	/* Check the attributes of the current item - see if we are interested in it or not;
	 * only store KEYBOARD usage page items into the Processed HID Report structure to
	 * save RAM and ignore the rest
	 */
	return (CurrentItem->Attributes.Usage.Page == USAGE_PAGE_KEYBOARD);
}

// Main function
int
main(void)
{
  BIT_ARRAY* keyboard_state = bit_array_create(MATRIX_COUNT);

  bit_array_clear_all(keyboard_state);

  // Initialise the hardware
  initialiseHardware();

  // Enable interrupts (required for USB support)
  sei();

  // Main processing loop
  while(1) {
    // Perform any pending keyboard actions
    processKeyboard(keyboard_state);

    // Process the USB host interface
    USB_USBTask();
  }
}

