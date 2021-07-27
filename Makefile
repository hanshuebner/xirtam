CC=avr-gcc
OBJCOPY=avr-objcopy
OBJS=	src/main.o src/ConfigDescriptor.o src/bit_array.o		\
	lufa-master/LUFA/Drivers/USB/Core/USBTask.o			\
	lufa-master/LUFA/Drivers/USB/Core/AVR8/USBController_AVR8.o	\
	lufa-master/LUFA/Drivers/USB/Core/AVR8/USBInterrupt_AVR8.o	\
	lufa-master/LUFA/Drivers/USB/Core/HostStandardReq.o		\
	lufa-master/LUFA/Drivers/USB/Core/ConfigDescriptors.o		\
	lufa-master/LUFA/Drivers/USB/Core/AVR8/Pipe_AVR8.o		\
	lufa-master/LUFA/Drivers/USB/Core/AVR8/PipeStream_AVR8.o	\
	lufa-master/LUFA/Drivers/USB/Core/AVR8/Host_AVR8.o		\
	lufa-master/LUFA/Drivers/Peripheral/AVR8/Serial_AVR8.o		\
	lufa-master/LUFA/Drivers/USB/Core/AVR8/Device_AVR8.o		\
	lufa-master/LUFA/Drivers/USB/Core/AVR8/Endpoint_AVR8.o		\
	lufa-master/LUFA/Drivers/USB/Core/Events.o

CFLAGS=-Os -DUSE_LUFA_CONFIG_HEADER -DARCH=ARCH_AVR8 -DBOARD=BOARD_NONE -DF_CPU=16000000UL \
	-DF_USB=16000000UL -D__AVR_AT90USB1287__ -mmcu=at90usb1287 \
	-Isrc/ -Ilufa-master/

xirtam.hex: xirtam.elf
	${OBJCOPY} -j .text -j .data -O ihex xirtam.elf xirtam.hex

xirtam.elf: ${OBJS}
	${CC} -mmcu=at90usb1287 -o xirtam.elf ${OBJS}

install: xirtam.hex
	dfu-programmer at90usb1287 erase --force
	dfu-programmer at90usb1287 flash xirtam.hex
