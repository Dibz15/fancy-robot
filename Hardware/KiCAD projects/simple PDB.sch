EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:switches
LIBS:relays
LIBS:motors
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L USB_OTG J?
U 1 1 5A3C0983
P 3250 2050
F 0 "J?" H 3050 2500 50  0000 L CNN
F 1 "USB_OTG" H 3050 2400 50  0000 L CNN
F 2 "" H 3400 2000 50  0001 C CNN
F 3 "" H 3400 2000 50  0001 C CNN
	1    3250 2050
	1    0    0    -1  
$EndComp
$Comp
L USB_OTG J?
U 1 1 5A3C0AC4
P 3250 3100
F 0 "J?" H 3050 3550 50  0000 L CNN
F 1 "USB_OTG" H 3050 3450 50  0000 L CNN
F 2 "" H 3400 3050 50  0001 C CNN
F 3 "" H 3400 3050 50  0001 C CNN
	1    3250 3100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A3C0B41
P 3250 3500
F 0 "#PWR?" H 3250 3250 50  0001 C CNN
F 1 "GND" H 3250 3350 50  0000 C CNN
F 2 "" H 3250 3500 50  0001 C CNN
F 3 "" H 3250 3500 50  0001 C CNN
	1    3250 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 3500 3250 3500
$Comp
L GND #PWR?
U 1 1 5A3C0B64
P 3250 2450
F 0 "#PWR?" H 3250 2200 50  0001 C CNN
F 1 "GND" H 3250 2300 50  0000 C CNN
F 2 "" H 3250 2450 50  0001 C CNN
F 3 "" H 3250 2450 50  0001 C CNN
	1    3250 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	3150 2450 3250 2450
$Comp
L USB_A J?
U 1 1 5A3C0BC4
P 6050 2050
F 0 "J?" H 5850 2500 50  0000 L CNN
F 1 "USB_A" H 5850 2400 50  0000 L CNN
F 2 "" H 6200 2000 50  0001 C CNN
F 3 "" H 6200 2000 50  0001 C CNN
	1    6050 2050
	1    0    0    -1  
$EndComp
$Comp
L USB_A J?
U 1 1 5A3C0C19
P 6050 3100
F 0 "J?" H 5850 3550 50  0000 L CNN
F 1 "USB_A" H 5850 3450 50  0000 L CNN
F 2 "" H 6200 3050 50  0001 C CNN
F 3 "" H 6200 3050 50  0001 C CNN
	1    6050 3100
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A3C0CE2
P 6050 3500
F 0 "#PWR?" H 6050 3250 50  0001 C CNN
F 1 "GND" H 6050 3350 50  0000 C CNN
F 2 "" H 6050 3500 50  0001 C CNN
F 3 "" H 6050 3500 50  0001 C CNN
	1    6050 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 3500 6050 3500
$Comp
L GND #PWR?
U 1 1 5A3C0D0B
P 6050 2450
F 0 "#PWR?" H 6050 2200 50  0001 C CNN
F 1 "GND" H 6050 2300 50  0000 C CNN
F 2 "" H 6050 2450 50  0001 C CNN
F 3 "" H 6050 2450 50  0001 C CNN
	1    6050 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 2450 6050 2450
Wire Wire Line
	3550 1850 4700 1850
Wire Wire Line
	4700 1850 4700 1400
Wire Wire Line
	4700 1400 6450 1400
Wire Wire Line
	6450 1400 6450 1850
Wire Wire Line
	6450 1850 6350 1850
Wire Wire Line
	3550 2900 5650 2900
Wire Wire Line
	5650 2900 5650 2700
Wire Wire Line
	5650 2700 6450 2700
Wire Wire Line
	6450 2700 6450 2900
Wire Wire Line
	6450 2900 6350 2900
NoConn ~ 3550 3100
NoConn ~ 3550 3200
NoConn ~ 3550 3300
NoConn ~ 3550 2050
NoConn ~ 3550 2150
NoConn ~ 3550 2250
NoConn ~ 6350 2050
NoConn ~ 6350 2150
NoConn ~ 6350 3100
NoConn ~ 6350 3200
$Comp
L SW_DPDT_x2 SW?
U 1 1 5A3C0DA9
P 4500 2350
F 0 "SW?" H 4500 2520 50  0000 C CNN
F 1 "SW_DPDT_x2" H 4500 2150 50  0000 C CNN
F 2 "" H 4500 2350 50  0001 C CNN
F 3 "" H 4500 2350 50  0001 C CNN
	1    4500 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 2350 4300 1850
Connection ~ 4300 1850
Wire Wire Line
	4700 2450 4750 2450
Wire Wire Line
	4750 2450 4750 2900
Connection ~ 4750 2900
NoConn ~ 4700 2250
Text Notes 3050 3850 0    60   ~ 0
Switch/Jumper in the center switches the circuit from two separate supplies to being able to use a single shared supply.\n
$EndSCHEMATC