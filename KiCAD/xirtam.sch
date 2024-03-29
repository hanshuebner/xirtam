EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A3 16535 11693
encoding utf-8
Sheet 1 1
Title "xirtam TI-99/4A"
Date "2017-02-16"
Rev "2.0"
Comp "https://netzhansa.com"
Comment1 "© 2021 Hans Hübner"
Comment2 "License: Attribution-ShareAlike 4.0 International (CC BY-SA 4.0)"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Device:R R2
U 1 1 58A4E483
P 2900 2950
F 0 "R2" V 2980 2950 50  0000 C CNN
F 1 "22R" V 3050 2950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 2830 2950 50  0001 C CNN
F 3 "" H 2900 2950 50  0000 C CNN
	1    2900 2950
	0    1    1    0   
$EndComp
$Comp
L Device:R R1
U 1 1 58A4E546
P 2900 2850
F 0 "R1" V 2700 2850 50  0000 C CNN
F 1 "22R" V 2800 2850 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 2830 2850 50  0001 C CNN
F 3 "" H 2900 2850 50  0000 C CNN
	1    2900 2850
	0    1    1    0   
$EndComp
Wire Wire Line
	4150 2850 3050 2850
Wire Wire Line
	2750 2850 2350 2850
Wire Wire Line
	4000 3150 4150 3150
Wire Wire Line
	5450 1750 5450 1650
Wire Wire Line
	4000 1650 5450 1650
Wire Wire Line
	5550 1650 5550 1750
Connection ~ 5450 1650
Wire Wire Line
	5650 1650 5650 1750
Connection ~ 5550 1650
Connection ~ 4000 3150
Wire Wire Line
	5450 5750 5450 5850
Wire Wire Line
	4000 5850 5450 5850
Connection ~ 4000 5850
Wire Wire Line
	5550 5850 5550 5750
Connection ~ 5450 5850
Wire Wire Line
	5650 5850 5650 5750
Connection ~ 5550 5850
Wire Wire Line
	1950 2550 1950 2500
Wire Wire Line
	1950 2500 2450 2500
Wire Wire Line
	2450 2500 2450 2750
Wire Wire Line
	2350 2750 2450 2750
Connection ~ 2450 2750
Wire Wire Line
	2350 3050 2550 3050
Wire Wire Line
	2550 3050 2550 2450
$Comp
L power:GND #PWR01
U 1 1 58A4EDDC
P 2450 3150
F 0 "#PWR01" H 2450 2900 50  0001 C CNN
F 1 "GND" H 2450 3000 50  0000 C CNN
F 2 "" H 2450 3150 50  0000 C CNN
F 3 "" H 2450 3150 50  0000 C CNN
	1    2450 3150
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR02
U 1 1 58A4EDF8
P 2550 2450
F 0 "#PWR02" H 2550 2300 50  0001 C CNN
F 1 "+5V" H 2550 2590 50  0000 C CNN
F 2 "" H 2550 2450 50  0000 C CNN
F 3 "" H 2550 2450 50  0000 C CNN
	1    2550 2450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR03
U 1 1 58A4EE14
P 4000 1450
F 0 "#PWR03" H 4000 1300 50  0001 C CNN
F 1 "+5V" H 4000 1590 50  0000 C CNN
F 2 "" H 4000 1450 50  0000 C CNN
F 3 "" H 4000 1450 50  0000 C CNN
	1    4000 1450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 58A4F188
P 4000 5900
F 0 "#PWR04" H 4000 5650 50  0001 C CNN
F 1 "GND" H 4000 5750 50  0000 C CNN
F 2 "" H 4000 5900 50  0000 C CNN
F 3 "" H 4000 5900 50  0000 C CNN
	1    4000 5900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4150 2650 3750 2650
Wire Wire Line
	3450 2650 3350 2650
Wire Wire Line
	3350 2650 3350 3250
Connection ~ 4000 3550
$Comp
L Device:C C4
U 1 1 58A4F76F
P 3600 3250
F 0 "C4" V 3450 3200 50  0000 L CNN
F 1 "1uF" V 3750 3200 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3638 3100 50  0001 C CNN
F 3 "" H 3600 3250 50  0000 C CNN
	1    3600 3250
	0    1    1    0   
$EndComp
Wire Wire Line
	3450 3250 3350 3250
Connection ~ 3350 3250
$Comp
L Device:Crystal Y1
U 1 1 58A4F945
P 2550 1800
F 0 "Y1" V 2600 1550 50  0000 C CNN
F 1 "16 MHz" V 2500 1500 50  0000 C CNN
F 2 "Crystals:Crystal_SMD_5032-2pin_5.0x3.2mm_HandSoldering" H 2550 1800 50  0001 C CNN
F 3 "" H 2550 1800 50  0000 C CNN
	1    2550 1800
	0    -1   -1   0   
$EndComp
$Comp
L Device:C C1
U 1 1 58A4F9FD
P 2050 1600
F 0 "C1" V 1900 1550 50  0000 L CNN
F 1 "22pF" V 2200 1500 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 2088 1450 50  0001 C CNN
F 3 "" H 2050 1600 50  0000 C CNN
	1    2050 1600
	0    1    1    0   
$EndComp
$Comp
L Device:C C2
U 1 1 58A4FA4D
P 2050 2000
F 0 "C2" V 1900 1950 50  0000 L CNN
F 1 "22pF" V 2200 1900 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 2088 1850 50  0001 C CNN
F 3 "" H 2050 2000 50  0000 C CNN
	1    2050 2000
	0    1    1    0   
$EndComp
Wire Wire Line
	4150 2350 3050 2350
Wire Wire Line
	3050 2350 3050 1600
Wire Wire Line
	3050 1600 2550 1600
Wire Wire Line
	4150 2450 2950 2450
Wire Wire Line
	2950 2450 2950 2000
Wire Wire Line
	2950 2000 2550 2000
Wire Wire Line
	2550 1950 2550 2000
Connection ~ 2550 2000
Wire Wire Line
	2550 1650 2550 1600
Connection ~ 2550 1600
Wire Wire Line
	1900 1600 1700 1600
Wire Wire Line
	1700 1600 1700 2000
Wire Wire Line
	1900 2000 1700 2000
Connection ~ 1700 2000
$Comp
L power:GND #PWR05
U 1 1 58A4FC16
P 1700 2200
F 0 "#PWR05" H 1700 1950 50  0001 C CNN
F 1 "GND" H 1700 2050 50  0000 C CNN
F 2 "" H 1700 2200 50  0000 C CNN
F 3 "" H 1700 2200 50  0000 C CNN
	1    1700 2200
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 58A4FD2F
P 2750 9750
F 0 "C5" H 2775 9850 50  0000 L CNN
F 1 "100nF" H 2775 9650 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 2788 9600 50  0001 C CNN
F 3 "" H 2750 9750 50  0000 C CNN
	1    2750 9750
	-1   0    0    1   
$EndComp
$Comp
L Device:C C6
U 1 1 58A4FDE1
P 3150 9750
F 0 "C6" H 3175 9850 50  0000 L CNN
F 1 "100nF" H 3175 9650 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3188 9600 50  0001 C CNN
F 3 "" H 3150 9750 50  0000 C CNN
	1    3150 9750
	-1   0    0    1   
$EndComp
Wire Wire Line
	2750 9400 2750 9500
Wire Wire Line
	2750 9900 2750 10000
Connection ~ 2750 9500
Connection ~ 2750 10000
$Comp
L power:GND #PWR06
U 1 1 58A50022
P 2750 10100
F 0 "#PWR06" H 2750 9850 50  0001 C CNN
F 1 "GND" H 2750 9950 50  0000 C CNN
F 2 "" H 2750 10100 50  0000 C CNN
F 3 "" H 2750 10100 50  0000 C CNN
	1    2750 10100
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR07
U 1 1 58A5004E
P 2750 9400
F 0 "#PWR07" H 2750 9250 50  0001 C CNN
F 1 "+5V" H 2750 9540 50  0000 C CNN
F 2 "" H 2750 9400 50  0000 C CNN
F 3 "" H 2750 9400 50  0000 C CNN
	1    2750 9400
	1    0    0    -1  
$EndComp
NoConn ~ 6850 2350
NoConn ~ 4150 5450
NoConn ~ 4150 5350
NoConn ~ 4150 5250
NoConn ~ 4150 5150
NoConn ~ 4150 5050
NoConn ~ 4150 4850
NoConn ~ 4150 4750
NoConn ~ 4150 4150
NoConn ~ 4150 4050
NoConn ~ 4150 3950
NoConn ~ 4150 3850
Wire Wire Line
	4150 3250 3750 3250
Wire Wire Line
	3350 3550 4000 3550
NoConn ~ 6850 2550
NoConn ~ 6850 2650
NoConn ~ 6850 2750
NoConn ~ 6850 3350
NoConn ~ 6850 3450
NoConn ~ 6850 3550
NoConn ~ 6850 3650
$Comp
L Device:C C7
U 1 1 590F2464
P 3550 9750
F 0 "C7" H 3575 9850 50  0000 L CNN
F 1 "100nF" H 3575 9650 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3588 9600 50  0001 C CNN
F 3 "" H 3550 9750 50  0000 C CNN
	1    3550 9750
	-1   0    0    1   
$EndComp
Wire Wire Line
	3250 4950 3400 4950
$Comp
L power:GND #PWR018
U 1 1 590EF68D
P 3400 5450
F 0 "#PWR018" H 3400 5200 50  0001 C CNN
F 1 "GND" H 3400 5300 50  0000 C CNN
F 2 "" H 3400 5450 50  0000 C CNN
F 3 "" H 3400 5450 50  0000 C CNN
	1    3400 5450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 590EF999
P 3100 4950
F 0 "R3" V 3180 4950 50  0000 C CNN
F 1 "10K" V 3100 4950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 3030 4950 50  0001 C CNN
F 3 "" H 3100 4950 50  0000 C CNN
	1    3100 4950
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR019
U 1 1 590EFA86
P 2550 4850
F 0 "#PWR019" H 2550 4700 50  0001 C CNN
F 1 "+5V" H 2550 4990 50  0000 C CNN
F 2 "" H 2550 4850 50  0000 C CNN
F 3 "" H 2550 4850 50  0000 C CNN
	1    2550 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 4950 2550 4950
Wire Wire Line
	2550 4950 2550 4850
$Comp
L Device:R R4
U 1 1 590EFF41
P 3800 1900
F 0 "R4" H 3950 1850 50  0000 C CNN
F 1 "10K" H 3950 1950 50  0000 C CNN
F 2 "Resistor_SMD:R_0603_1608Metric_Pad1.05x0.95mm_HandSolder" V 3730 1900 50  0001 C CNN
F 3 "" H 3800 1900 50  0000 C CNN
	1    3800 1900
	-1   0    0    1   
$EndComp
$Comp
L xirtam-rescue:CONN_01X02 J5
U 1 1 590F13C8
P 3100 5300
F 0 "J5" H 3100 5450 50  0000 C CNN
F 1 "Bootloader" H 3400 5300 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Horizontal" H 3100 5300 50  0001 C CNN
F 3 "" H 3100 5300 50  0001 C CNN
	1    3100 5300
	-1   0    0    1   
$EndComp
Wire Wire Line
	3300 5350 3400 5350
Wire Wire Line
	3400 5350 3400 5450
Wire Wire Line
	3300 5250 3400 5250
Wire Wire Line
	3400 5250 3400 4950
Connection ~ 3400 4950
Wire Wire Line
	4000 1650 4000 3150
Wire Wire Line
	5450 1650 5550 1650
Wire Wire Line
	5550 1650 5650 1650
Wire Wire Line
	4000 3150 4000 3450
Wire Wire Line
	4000 5850 4000 5900
Wire Wire Line
	5450 5850 5550 5850
Wire Wire Line
	5550 5850 5650 5850
Wire Wire Line
	2450 2750 2450 3150
Wire Wire Line
	4000 3550 4150 3550
Wire Wire Line
	3350 3250 3350 3550
Wire Wire Line
	2550 2000 2200 2000
Wire Wire Line
	2550 1600 2200 1600
Wire Wire Line
	1700 2000 1700 2200
Wire Wire Line
	2750 9500 2750 9600
Wire Wire Line
	2750 10000 2750 10100
Wire Wire Line
	4000 3450 4150 3450
Wire Wire Line
	3400 4950 4150 4950
Wire Wire Line
	6850 3850 8450 3850
Wire Wire Line
	6850 4150 8450 4150
Wire Wire Line
	8450 4450 6850 4450
$Comp
L power:+5V #PWR0101
U 1 1 60938B96
P 9150 2150
F 0 "#PWR0101" H 9150 2000 50  0001 C CNN
F 1 "+5V" H 9150 2290 50  0000 C CNN
F 2 "" H 9150 2150 50  0000 C CNN
F 3 "" H 9150 2150 50  0000 C CNN
	1    9150 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	500  4850 500  5100
Wire Wire Line
	9150 2300 9150 2150
$Comp
L xirtam-rescue:CONN_01X02 J3
U 1 1 60963B37
P 1200 9750
F 0 "J3" H 1200 9900 50  0000 C CNN
F 1 "Power" H 1400 9750 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Horizontal" H 1200 9750 50  0001 C CNN
F 3 "" H 1200 9750 50  0001 C CNN
	1    1200 9750
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR0103
U 1 1 609690A0
P 1550 9550
F 0 "#PWR0103" H 1550 9400 50  0001 C CNN
F 1 "+5V" H 1550 9690 50  0000 C CNN
F 2 "" H 1550 9550 50  0000 C CNN
F 3 "" H 1550 9550 50  0000 C CNN
	1    1550 9550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 6096E499
P 1550 9950
F 0 "#PWR0104" H 1550 9700 50  0001 C CNN
F 1 "GND" H 1550 9800 50  0000 C CNN
F 2 "" H 1550 9950 50  0000 C CNN
F 3 "" H 1550 9950 50  0000 C CNN
	1    1550 9950
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 9700 1550 9700
Wire Wire Line
	1550 9700 1550 9550
Wire Wire Line
	1400 9800 1550 9800
Wire Wire Line
	1550 9800 1550 9950
$Comp
L power:GND #PWR0105
U 1 1 6097E1C0
P 9150 4900
F 0 "#PWR0105" H 9150 4650 50  0001 C CNN
F 1 "GND" H 9150 4750 50  0000 C CNN
F 2 "" H 9150 4900 50  0000 C CNN
F 3 "" H 9150 4900 50  0000 C CNN
	1    9150 4900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 4700 9150 4900
Text GLabel 12750 8550 0    60   Input ~ 0
TCK
Text GLabel 12750 8650 0    60   Input ~ 0
TDO
Text GLabel 12750 8750 0    60   Input ~ 0
TMS
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J4
U 1 1 60A4C514
P 13200 8750
F 0 "J4" H 13250 9167 50  0000 C CNN
F 1 "JTAG" H 13250 9076 50  0000 C CNN
F 2 "Connector_IDC:IDC-Header_2x05_P2.54mm_Horizontal" H 13200 8750 50  0001 C CNN
F 3 "~" H 13200 8750 50  0001 C CNN
	1    13200 8750
	1    0    0    -1  
$EndComp
Text GLabel 12750 8950 0    60   Input ~ 0
TDI
Text GLabel 13800 8750 2    60   Input ~ 0
~RESET
NoConn ~ 13500 8850
$Comp
L power:GND #PWR09
U 1 1 60A5D11A
P 13650 9200
F 0 "#PWR09" H 13650 8950 50  0001 C CNN
F 1 "GND" H 13650 9050 50  0000 C CNN
F 2 "" H 13650 9200 50  0000 C CNN
F 3 "" H 13650 9200 50  0000 C CNN
	1    13650 9200
	1    0    0    -1  
$EndComp
Wire Wire Line
	13500 8950 13650 8950
Wire Wire Line
	13650 8950 13650 9200
Wire Wire Line
	13500 8550 13650 8550
Wire Wire Line
	13650 8550 13650 8950
Connection ~ 13650 8950
Wire Wire Line
	13500 8750 13800 8750
Wire Wire Line
	12750 8750 13000 8750
Wire Wire Line
	13000 8650 12750 8650
Wire Wire Line
	12750 8550 13000 8550
Wire Wire Line
	13000 8950 12750 8950
Wire Wire Line
	13550 8650 13500 8650
$Comp
L power:+5V #PWR08
U 1 1 60AAE168
P 13550 8400
F 0 "#PWR08" H 13550 8250 50  0001 C CNN
F 1 "+5V" H 13550 8540 50  0000 C CNN
F 2 "" H 13550 8400 50  0000 C CNN
F 3 "" H 13550 8400 50  0000 C CNN
	1    13550 8400
	1    0    0    -1  
$EndComp
Wire Wire Line
	13550 8400 13550 8650
Text GLabel 3600 2150 0    60   Input ~ 0
~RESET
Text GLabel 3850 4250 0    60   Input ~ 0
TCK
Text GLabel 3850 4350 0    60   Input ~ 0
TMS
Text GLabel 3850 4450 0    60   Input ~ 0
TDO
Text GLabel 3850 4550 0    60   Input ~ 0
TDI
Wire Wire Line
	3850 4250 4150 4250
Wire Wire Line
	4150 4350 3850 4350
Wire Wire Line
	3850 4450 4150 4450
Wire Wire Line
	4150 4550 3850 4550
Wire Wire Line
	4000 3550 4000 5850
Wire Wire Line
	3600 2150 3800 2150
Wire Wire Line
	3800 1750 3800 1650
Wire Wire Line
	3800 1650 4000 1650
Connection ~ 4000 1650
Wire Wire Line
	4000 1450 4000 1650
Wire Wire Line
	3800 2050 3800 2150
Connection ~ 3800 2150
Wire Wire Line
	3800 2150 4150 2150
Text GLabel 8250 9800 2    60   Input ~ 0
~RESET
$Comp
L power:GND #PWR012
U 1 1 60BA7BDC
P 7500 9950
F 0 "#PWR012" H 7500 9700 50  0001 C CNN
F 1 "GND" H 7500 9800 50  0000 C CNN
F 2 "" H 7500 9950 50  0000 C CNN
F 3 "" H 7500 9950 50  0000 C CNN
	1    7500 9950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7500 9950 7500 9800
Wire Wire Line
	7500 9800 7650 9800
NoConn ~ 6850 2450
NoConn ~ 6850 2250
NoConn ~ 6850 2150
NoConn ~ 6850 2050
NoConn ~ 6850 5250
NoConn ~ 6850 5350
NoConn ~ 6850 5450
NoConn ~ 6850 4550
Wire Wire Line
	7500 5150 7500 2550
Wire Wire Line
	7500 2550 8450 2550
Wire Wire Line
	6850 4750 7600 4750
Wire Wire Line
	7600 4750 7600 2650
Wire Wire Line
	7600 2650 8450 2650
Wire Wire Line
	7700 4850 7700 2750
Wire Wire Line
	7700 2750 8450 2750
$Comp
L Device:C C3
U 1 1 58A4F61A
P 3600 2650
F 0 "C3" V 3450 2600 50  0000 L CNN
F 1 "100nF" V 3750 2550 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3638 2500 50  0001 C CNN
F 3 "" H 3600 2650 50  0000 C CNN
	1    3600 2650
	0    1    1    0   
$EndComp
Wire Wire Line
	2350 2950 2750 2950
Wire Wire Line
	3050 2950 4150 2950
$Comp
L xirtam-rescue:USB_A-RESCUE-SmallyMouse2-xirtam-rescue P1
U 1 1 58A4E3C4
P 2050 2850
F 0 "P1" V 1750 2850 50  0000 C CNN
F 1 "USB_A" V 2000 3150 50  0000 C CNN
F 2 "SimonsFPLibrary2017:Amphenol_87583-2010BLF" V 2000 2750 50  0001 C CNN
F 3 "" V 2000 2750 50  0000 C CNN
	1    2050 2850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	8050 9800 8250 9800
$Comp
L Connector:Conn_01x04_Male J6
U 1 1 61126BA0
P 11350 8750
F 0 "J6" H 11322 8632 50  0000 R CNN
F 1 "Debug" H 11322 8723 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Horizontal" H 11350 8750 50  0001 C CNN
F 3 "~" H 11350 8750 50  0001 C CNN
	1    11350 8750
	-1   0    0    1   
$EndComp
$Comp
L power:+5V #PWR013
U 1 1 611295DF
P 11000 8450
F 0 "#PWR013" H 11000 8300 50  0001 C CNN
F 1 "+5V" H 11000 8590 50  0000 C CNN
F 2 "" H 11000 8450 50  0000 C CNN
F 3 "" H 11000 8450 50  0000 C CNN
	1    11000 8450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 61129D3E
P 11000 8950
F 0 "#PWR016" H 11000 8700 50  0001 C CNN
F 1 "GND" H 11000 8800 50  0000 C CNN
F 2 "" H 11000 8950 50  0000 C CNN
F 3 "" H 11000 8950 50  0000 C CNN
	1    11000 8950
	1    0    0    -1  
$EndComp
Wire Wire Line
	11000 8950 11000 8850
Wire Wire Line
	11000 8850 11150 8850
Wire Wire Line
	11150 8550 11000 8550
Wire Wire Line
	11000 8550 11000 8450
Wire Wire Line
	10250 8750 11150 8750
Text Label 10750 8650 0    60   ~ 0
RxD
Text Label 10750 8750 0    60   ~ 0
TxD
Wire Wire Line
	9850 2550 11300 2550
Wire Wire Line
	11300 2650 9850 2650
Wire Wire Line
	9850 2750 11300 2750
Wire Wire Line
	11300 2850 9850 2850
Wire Wire Line
	9850 2950 11300 2950
Wire Wire Line
	11300 3050 9850 3050
Wire Wire Line
	9850 3150 11300 3150
Wire Wire Line
	11300 3250 9850 3250
Wire Wire Line
	9850 3350 11300 3350
Wire Wire Line
	11300 3450 9850 3450
Wire Wire Line
	9850 3550 11300 3550
Wire Wire Line
	9850 3650 11300 3650
Wire Wire Line
	11300 3750 9850 3750
Wire Wire Line
	9850 3850 11300 3850
Wire Wire Line
	11300 3950 9850 3950
$Comp
L xirtam-rescue:MT093APR1 U1
U 1 1 608FEEB5
P 9150 3500
F 0 "U1" H 9150 3500 60  0000 C CNN
F 1 "MT093APR1" H 9150 3400 60  0000 C CNN
F 2 "Package_LCC:PLCC-44" H 9150 3500 60  0001 C CNN
F 3 "" H 9150 3500 60  0001 C CNN
	1    9150 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9850 4050 11300 4050
Wire Wire Line
	11300 4150 9850 4150
Wire Wire Line
	9850 4250 11300 4250
Wire Wire Line
	11300 4350 9850 4350
Wire Wire Line
	9850 4450 11300 4450
$Comp
L Connector:Conn_01x28_Female J2
U 1 1 611A43EE
P 12950 6050
F 0 "J2" V 13023 5980 50  0000 C CNN
F 1 "Conn_01x28_Female" V 13114 5980 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x28_P2.54mm_Horizontal" H 12950 6050 50  0001 C CNN
F 3 "~" H 12950 6050 50  0001 C CNN
	1    12950 6050
	0    -1   1    0   
$EndComp
$Comp
L Connector:Conn_01x28_Male J1
U 1 1 611AAC92
P 12950 1750
F 0 "J1" V 12785 1678 50  0000 C CNN
F 1 "Conn_01x28_Male" V 12876 1678 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x28_P2.54mm_Horizontal" H 12950 1750 50  0001 C CNN
F 3 "~" H 12950 1750 50  0001 C CNN
	1    12950 1750
	0    -1   1    0   
$EndComp
Wire Wire Line
	11650 5650 11650 5850
Wire Wire Line
	11750 5850 11750 5650
Wire Wire Line
	11850 5650 11850 5850
Wire Wire Line
	11950 5850 11950 5650
Wire Wire Line
	12050 5650 12050 5850
Wire Wire Line
	12150 5850 12150 5650
Wire Wire Line
	12250 5650 12250 5850
Wire Wire Line
	12350 5850 12350 5650
Wire Wire Line
	12450 5650 12450 5850
Wire Wire Line
	12550 5850 12550 5650
Wire Wire Line
	12650 5650 12650 5850
Wire Wire Line
	12750 5850 12750 5650
Wire Wire Line
	12850 5650 12850 5850
Wire Wire Line
	12950 5850 12950 5650
Wire Wire Line
	13050 5650 13050 5850
Wire Wire Line
	13150 5850 13150 5650
Wire Wire Line
	13250 5650 13250 5850
Wire Wire Line
	13350 5850 13350 5650
Wire Wire Line
	13450 5650 13450 5850
Wire Wire Line
	13550 5850 13550 5650
Wire Wire Line
	13650 5650 13650 5850
Wire Wire Line
	13750 5850 13750 5650
Wire Wire Line
	13850 5650 13850 5850
Wire Wire Line
	13950 5850 13950 5650
Wire Wire Line
	14050 5650 14050 5850
Wire Wire Line
	14150 5850 14150 5650
Wire Wire Line
	14250 5650 14250 5850
Wire Wire Line
	14350 5850 14350 5650
Wire Wire Line
	11650 1950 11650 2150
Wire Wire Line
	11750 2150 11750 1950
Wire Wire Line
	11850 1950 11850 2150
Wire Wire Line
	11950 2150 11950 1950
Wire Wire Line
	12050 1950 12050 2150
Wire Wire Line
	12150 2150 12150 1950
Wire Wire Line
	12250 1950 12250 2150
Wire Wire Line
	12350 2150 12350 1950
Wire Wire Line
	12450 1950 12450 2150
Wire Wire Line
	12550 2150 12550 1950
Wire Wire Line
	12650 1950 12650 2150
Wire Wire Line
	12750 2150 12750 1950
Wire Wire Line
	12850 1950 12850 2150
Wire Wire Line
	12950 2150 12950 1950
Wire Wire Line
	13050 1950 13050 2150
Wire Wire Line
	13150 2150 13150 1950
Wire Wire Line
	13250 1950 13250 2150
Wire Wire Line
	13350 2150 13350 1950
Wire Wire Line
	13450 1950 13450 2150
Wire Wire Line
	13550 2150 13550 1950
Wire Wire Line
	13650 1950 13650 2150
Wire Wire Line
	13750 2150 13750 1950
Wire Wire Line
	13850 1950 13850 2150
Wire Wire Line
	13950 2150 13950 1950
Wire Wire Line
	14050 1950 14050 2150
Wire Wire Line
	14150 2150 14150 1950
Wire Wire Line
	14250 1950 14250 2150
Wire Wire Line
	14350 2150 14350 1950
$Comp
L Switch:SW_SPST SW1
U 1 1 61472E68
P 7850 9800
F 0 "SW1" H 7850 10035 50  0000 C CNN
F 1 "Reset" H 7850 9944 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_SPST_FSMSM" H 7850 9800 50  0001 C CNN
F 3 "~" H 7850 9800 50  0001 C CNN
	1    7850 9800
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 60B20C79
P 3950 9750
F 0 "C8" H 3975 9850 50  0000 L CNN
F 1 "100nF" H 3975 9650 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric_Pad1.05x0.95mm_HandSolder" H 3988 9600 50  0001 C CNN
F 3 "" H 3950 9750 50  0000 C CNN
	1    3950 9750
	-1   0    0    1   
$EndComp
Wire Wire Line
	3150 10000 3150 9900
Wire Wire Line
	2750 10000 3150 10000
Wire Wire Line
	3550 10000 3550 9900
Wire Wire Line
	3550 10000 3950 10000
Wire Wire Line
	3950 10000 3950 9900
Wire Wire Line
	3150 9500 3150 9600
Wire Wire Line
	2750 9500 3150 9500
Wire Wire Line
	3550 9500 3550 9600
Wire Wire Line
	3550 9500 3950 9500
Wire Wire Line
	3950 9500 3950 9600
$Comp
L power:GND #PWR0106
U 1 1 6153F26C
P 11100 5350
F 0 "#PWR0106" H 11100 5100 50  0001 C CNN
F 1 "GND" H 11100 5200 50  0000 C CNN
F 2 "" H 11100 5350 50  0000 C CNN
F 3 "" H 11100 5350 50  0000 C CNN
	1    11100 5350
	1    0    0    -1  
$EndComp
Wire Wire Line
	11100 5350 11100 5250
Wire Wire Line
	11100 5250 11300 5250
$Comp
L power:+5V #PWR0107
U 1 1 6154ACF7
P 11100 2250
F 0 "#PWR0107" H 11100 2100 50  0001 C CNN
F 1 "+5V" H 11100 2390 50  0000 C CNN
F 2 "" H 11100 2250 50  0000 C CNN
F 3 "" H 11100 2250 50  0000 C CNN
	1    11100 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	11100 2250 11100 5150
Wire Wire Line
	11100 5150 11300 5150
Wire Wire Line
	3150 9500 3550 9500
Connection ~ 3150 9500
Connection ~ 3550 9500
Wire Wire Line
	3550 10000 3150 10000
Connection ~ 3550 10000
Connection ~ 3150 10000
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 61691A08
P 1950 9550
F 0 "#FLG0101" H 1950 9625 50  0001 C CNN
F 1 "PWR_FLAG" H 1950 9723 50  0000 C CNN
F 2 "" H 1950 9550 50  0001 C CNN
F 3 "~" H 1950 9550 50  0001 C CNN
	1    1950 9550
	1    0    0    -1  
$EndComp
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 61692945
P 1950 9950
F 0 "#FLG0102" H 1950 10025 50  0001 C CNN
F 1 "PWR_FLAG" H 1950 10123 50  0000 C CNN
F 2 "" H 1950 9950 50  0001 C CNN
F 3 "~" H 1950 9950 50  0001 C CNN
	1    1950 9950
	-1   0    0    1   
$EndComp
Wire Wire Line
	1550 9700 1950 9700
Wire Wire Line
	1950 9700 1950 9550
Connection ~ 1550 9700
Wire Wire Line
	1550 9800 1950 9800
Wire Wire Line
	1950 9800 1950 9950
Connection ~ 1550 9800
Wire Wire Line
	13000 8850 12900 8850
Wire Wire Line
	12900 8850 12900 9100
Wire Wire Line
	12900 9100 13550 9100
Wire Wire Line
	13550 9100 13550 8650
Connection ~ 13550 8650
$Comp
L 4xxx:4066 U3
U 1 1 61076C9C
P 8300 6900
F 0 "U3" V 8346 6772 50  0000 R CNN
F 1 "4066" V 8255 6772 50  0000 R CNN
F 2 "Package_SO:SO-14_5.3x10.2mm_P1.27mm" H 8300 6900 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/cd4066b.pdf" H 8300 6900 50  0001 C CNN
	1    8300 6900
	0    -1   -1   0   
$EndComp
$Comp
L 4xxx:4066 U3
U 2 1 6107807D
P 9750 6900
F 0 "U3" V 9796 6772 50  0000 R CNN
F 1 "4066" V 9705 6772 50  0000 R CNN
F 2 "Package_SO:SO-14_5.3x10.2mm_P1.27mm" H 9750 6900 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/cd4066b.pdf" H 9750 6900 50  0001 C CNN
	2    9750 6900
	0    -1   -1   0   
$EndComp
$Comp
L 4xxx:4066 U3
U 3 1 61079672
P 8300 6150
F 0 "U3" V 8346 6022 50  0000 R CNN
F 1 "4066" V 8255 6022 50  0000 R CNN
F 2 "Package_SO:SO-14_5.3x10.2mm_P1.27mm" H 8300 6150 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/cd4066b.pdf" H 8300 6150 50  0001 C CNN
	3    8300 6150
	0    -1   -1   0   
$EndComp
$Comp
L 4xxx:4066 U3
U 4 1 6107B18D
P 9750 6150
F 0 "U3" V 9796 6022 50  0000 R CNN
F 1 "4066" V 9705 6022 50  0000 R CNN
F 2 "Package_SO:SO-14_5.3x10.2mm_P1.27mm" H 9750 6150 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/cd4066b.pdf" H 9750 6150 50  0001 C CNN
	4    9750 6150
	0    -1   -1   0   
$EndComp
$Comp
L 4xxx:4066 U3
U 5 1 6107C7BA
P 4550 9750
F 0 "U3" H 4780 9796 50  0000 L CNN
F 1 "4066" H 4780 9705 50  0000 L CNN
F 2 "Package_SO:SO-14_5.3x10.2mm_P1.27mm" H 4550 9750 50  0001 C CNN
F 3 "http://www.ti.com/lit/ds/symlink/cd4066b.pdf" H 4550 9750 50  0001 C CNN
	5    4550 9750
	1    0    0    -1  
$EndComp
Text GLabel 10250 8650 0    60   Input ~ 0
RxD
Text GLabel 10250 8750 0    60   Output ~ 0
TxD
Wire Wire Line
	10250 8650 11150 8650
Wire Wire Line
	6850 4350 8450 4350
Wire Wire Line
	6850 4250 8450 4250
Wire Wire Line
	6850 4850 7700 4850
Wire Wire Line
	6850 3950 8450 3950
Wire Wire Line
	6850 4050 8450 4050
Wire Wire Line
	6850 5150 7500 5150
Text GLabel 7000 5050 2    60   Output ~ 0
TxD
Text GLabel 7000 4950 2    60   Input ~ 0
RxD
Wire Wire Line
	6850 4950 7000 4950
Wire Wire Line
	6850 5050 7000 5050
Wire Wire Line
	9750 7250 9750 7200
Wire Wire Line
	9750 7250 10150 7250
Wire Wire Line
	11000 7250 11000 5050
Wire Wire Line
	11000 5050 11300 5050
$Comp
L Jumper:MatrixInterconnectMatrix M1
U 1 1 610F295C
P 13000 3900
F 0 "M1" H 12950 3800 60  0000 L CNN
F 1 "MatrixInterconnectMatrix" H 12450 3950 60  0000 L CNN
F 2 "Jumper:matrixinterconnectmatrix" H 13200 4450 60  0001 C CNN
F 3 "" H 13200 4450 60  0001 C CNN
	1    13000 3900
	1    0    0    -1  
$EndComp
Wire Wire Line
	9750 6600 10900 6600
Wire Wire Line
	10900 6600 10900 4950
Wire Wire Line
	10900 4950 11300 4950
Wire Wire Line
	9750 5850 10800 5850
Wire Wire Line
	10800 5850 10800 4850
Wire Wire Line
	10800 4850 11300 4850
Wire Wire Line
	8300 5850 8300 5750
Wire Wire Line
	10700 5750 10700 4750
Wire Wire Line
	10700 4750 11300 4750
Wire Wire Line
	8650 6600 8650 5550
Wire Wire Line
	10500 5550 10500 4550
Wire Wire Line
	10500 4550 11300 4550
Wire Wire Line
	10600 5650 10600 4650
Wire Wire Line
	10600 4650 11300 4650
$Comp
L xirtam-rescue:AT90USB1287-RESCUE-SmallyMouse2-xirtam-rescue U2
U 1 1 58A4E37E
P 5550 3750
F 0 "U2" H 5600 3700 60  0000 C CNN
F 1 "AT90USB1287" H 5550 3600 60  0000 C CNN
F 2 "Housings_QFP:TQFP-64_14x14mm_Pitch0.8mm" H 5550 3350 60  0001 C CNN
F 3 "" H 5550 3350 60  0001 C CNN
	1    5550 3750
	1    0    0    -1  
$EndComp
Text GLabel 7000 2950 2    60   Output ~ 0
AUX1
Text GLabel 7000 3050 2    60   Output ~ 0
AUX2
Text GLabel 7000 3150 2    60   Output ~ 0
AUX3
Text GLabel 7000 3250 2    60   Output ~ 0
AUX4
Wire Wire Line
	6850 2950 7000 2950
Wire Wire Line
	6850 3050 7000 3050
Wire Wire Line
	6850 3150 7000 3150
Wire Wire Line
	6850 3250 7000 3250
Wire Wire Line
	9750 6450 10150 6450
Wire Wire Line
	10150 6450 10150 7250
Connection ~ 10150 7250
Wire Wire Line
	10150 7250 11000 7250
Wire Wire Line
	8300 6600 8650 6600
Wire Wire Line
	8750 5650 8750 7200
Wire Wire Line
	8750 7200 8300 7200
Wire Wire Line
	8300 6450 9750 6450
Connection ~ 9750 6450
Wire Wire Line
	8300 5750 10700 5750
Wire Wire Line
	8750 5650 10600 5650
Wire Wire Line
	8650 5550 10500 5550
Text GLabel 8000 6900 0    60   Input ~ 0
AUX1
Text GLabel 8000 6150 0    60   Input ~ 0
AUX2
Text GLabel 9450 6150 0    60   Input ~ 0
AUX3
Text GLabel 9450 6900 0    60   Input ~ 0
AUX4
Wire Wire Line
	3950 9500 3950 9250
Wire Wire Line
	3950 9250 4550 9250
Connection ~ 3950 9500
Wire Wire Line
	3950 10000 3950 10250
Wire Wire Line
	3950 10250 4550 10250
Connection ~ 3950 10000
$EndSCHEMATC
