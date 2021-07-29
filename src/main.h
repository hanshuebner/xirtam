/************************************************************************
	main.h

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

#ifndef _MAIN_H_
#define _MAIN_H_

#include "bit_array.h"

// Function prototypes
void initialiseHardware(void);
void initialiseTimers(void);
void processKeyboard(BIT_ARRAY* old_state);

// USB callback event handlers (LUFA)
void EVENT_USB_Host_HostError(const uint8_t ErrorCode);
void EVENT_USB_Host_DeviceAttached(void);
void EVENT_USB_Host_DeviceUnattached(void);
void EVENT_USB_Host_DeviceEnumerationFailed(const uint8_t ErrorCode, const uint8_t SubErrorCode);
void EVENT_USB_Host_DeviceEnumerationComplete(void);

void ReadNextReport(void);

#endif

