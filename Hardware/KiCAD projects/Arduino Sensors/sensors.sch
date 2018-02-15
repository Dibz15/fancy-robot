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
LIBS:ECE34xCompLib
LIBS:sensors-cache
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
L Arduino_Uno_R3 J?
U 1 1 5A7F8A66
P 5800 3150
F 0 "J?" H 5800 4150 60  0000 C CNN
F 1 "Arduino_Uno_R3" H 5800 2000 60  0000 C CNN
F 2 "" H 6000 3150 60  0001 C CNN
F 3 "" H 6000 3150 60  0001 C CNN
	1    5800 3150
	1    0    0    -1  
$EndComp
$Comp
L Battery BT?
U 1 1 5A7F8D36
P 4400 3800
F 0 "BT?" H 4500 3900 50  0000 L CNN
F 1 "Battery" H 4500 3800 50  0000 L CNN
F 2 "" V 4400 3860 50  0001 C CNN
F 3 "" V 4400 3860 50  0001 C CNN
	1    4400 3800
	1    0    0    -1  
$EndComp
$Comp
L HC-SR04 U?
U 1 1 5A7F8EE0
P 7500 2600
F 0 "U?" H 7500 2350 60  0000 C CNN
F 1 "HC-SR04" H 7500 2900 60  0000 C CNN
F 2 "" H 7500 2600 60  0001 C CNN
F 3 "" H 7500 2600 60  0001 C CNN
	1    7500 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7000 2550 6650 2550
Wire Wire Line
	6650 2550 6650 3500
Wire Wire Line
	6650 3500 6350 3500
Wire Wire Line
	7000 2750 6950 2750
Wire Wire Line
	6950 2750 6950 3100
Wire Wire Line
	7000 2450 6750 2450
Wire Wire Line
	6750 2450 6750 3400
Wire Wire Line
	6750 3400 6350 3400
Wire Wire Line
	6950 2650 7000 2650
Wire Wire Line
	6950 1950 6950 2650
Wire Wire Line
	6350 2600 6450 2600
Wire Wire Line
	6450 2600 6450 2700
$Comp
L GND #PWR?
U 1 1 5A7F94A0
P 6450 2700
F 0 "#PWR?" H 6450 2450 50  0001 C CNN
F 1 "GND" H 6450 2550 50  0000 C CNN
F 2 "" H 6450 2700 50  0001 C CNN
F 3 "" H 6450 2700 50  0001 C CNN
	1    6450 2700
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A7F94BA
P 6950 3100
F 0 "#PWR?" H 6950 2850 50  0001 C CNN
F 1 "GND" H 6950 2950 50  0000 C CNN
F 2 "" H 6950 3100 50  0001 C CNN
F 3 "" H 6950 3100 50  0001 C CNN
	1    6950 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 3600 5200 3600
Wire Wire Line
	4400 4000 4400 4100
$Comp
L GND #PWR?
U 1 1 5A7F956C
P 4400 4100
F 0 "#PWR?" H 4400 3850 50  0001 C CNN
F 1 "GND" H 4400 3950 50  0000 C CNN
F 2 "" H 4400 4100 50  0001 C CNN
F 3 "" H 4400 4100 50  0001 C CNN
	1    4400 4100
	1    0    0    -1  
$EndComp
Wire Wire Line
	6950 1950 4950 1950
Wire Wire Line
	4950 1950 4950 3100
Wire Wire Line
	4950 3100 5200 3100
Text Notes 7450 7500 0    60   ~ 0
Sensors Overview Schematic\n
Text Notes 8150 7650 0    60   ~ 0
2/11/2018\n
Text Notes 7050 7100 0    60   ~ 0
Author: Austin Dibble\n
$EndSCHEMATC
