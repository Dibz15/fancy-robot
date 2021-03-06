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
LIBS:hat-design-cache
LIBS:RPiShield-cache
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
L USB_OTG J1
U 1 1 5A78FE7E
P 3050 3100
F 0 "J1" H 2850 3550 50  0000 L CNN
F 1 "Batt_In" H 2850 3450 50  0000 L CNN
F 2 "" H 3200 3050 50  0001 C CNN
F 3 "" H 3200 3050 50  0001 C CNN
	1    3050 3100
	1    0    0    -1  
$EndComp
$Comp
L USB_A J3
U 1 1 5A790039
P 3050 2050
F 0 "J3" H 2850 2500 50  0000 L CNN
F 1 "SPKR_Out" H 2850 2400 50  0000 L CNN
F 2 "" H 3200 2000 50  0001 C CNN
F 3 "" H 3200 2000 50  0001 C CNN
	1    3050 2050
	1    0    0    -1  
$EndComp
$Comp
L Conn_02x20_Odd_Even J13
U 1 1 5A7900DA
P 7200 3450
F 0 "J13" H 7250 4450 50  0000 C CNN
F 1 "GPIO" H 7250 2350 50  0000 C CNN
F 2 "" H 7200 3450 50  0001 C CNN
F 3 "" H 7200 3450 50  0001 C CNN
	1    7200 3450
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x03 J9
U 1 1 5A790334
P 3550 5100
F 0 "J9" H 3550 5300 50  0000 C CNN
F 1 "TILT SRVO" H 3550 4900 50  0000 C CNN
F 2 "" H 3550 5100 50  0001 C CNN
F 3 "" H 3550 5100 50  0001 C CNN
	1    3550 5100
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x03 J10
U 1 1 5A79045B
P 3550 5800
F 0 "J10" H 3550 6000 50  0000 C CNN
F 1 "PAN SRVO" H 3550 5600 50  0000 C CNN
F 2 "" H 3550 5800 50  0001 C CNN
F 3 "" H 3550 5800 50  0001 C CNN
	1    3550 5800
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x03 J11
U 1 1 5A7904C5
P 4350 5100
F 0 "J11" H 4350 5300 50  0000 C CNN
F 1 "ENC R" H 4350 4900 50  0000 C CNN
F 2 "" H 4350 5100 50  0001 C CNN
F 3 "" H 4350 5100 50  0001 C CNN
	1    4350 5100
	-1   0    0    1   
$EndComp
$Comp
L Conn_01x03 J12
U 1 1 5A790535
P 4350 5800
F 0 "J12" H 4350 6000 50  0000 C CNN
F 1 "ENC L" H 4350 5600 50  0000 C CNN
F 2 "" H 4350 5800 50  0001 C CNN
F 3 "" H 4350 5800 50  0001 C CNN
	1    4350 5800
	-1   0    0    1   
$EndComp
$Comp
L Conn_01x04 J8
U 1 1 5A7905CC
P 5000 3300
F 0 "J8" H 5000 3500 50  0000 C CNN
F 1 "MTR CTRL" H 5000 3000 50  0000 C CNN
F 2 "" H 5000 3300 50  0001 C CNN
F 3 "" H 5000 3300 50  0001 C CNN
	1    5000 3300
	-1   0    0    1   
$EndComp
$Comp
L USB_A J4
U 1 1 5A81081F
P 3050 3950
F 0 "J4" H 2850 4400 50  0000 L CNN
F 1 "RPi_Out" H 2850 4300 50  0000 L CNN
F 2 "" H 3200 3900 50  0001 C CNN
F 3 "" H 3200 3900 50  0001 C CNN
	1    3050 3950
	1    0    0    -1  
$EndComp
$Comp
L Conn_02x03_Odd_Even J6
U 1 1 5A810B4E
P 5050 1950
F 0 "J6" H 5100 2150 50  0000 C CNN
F 1 "PWR 2" H 5100 1750 50  0000 C CNN
F 2 "" H 5050 1950 50  0001 C CNN
F 3 "" H 5050 1950 50  0001 C CNN
	1    5050 1950
	1    0    0    -1  
$EndComp
$Comp
L USB_OTG J2
U 1 1 5A810BFD
P 3050 1150
F 0 "J2" H 2850 1600 50  0000 L CNN
F 1 "Batt_In2" H 2850 1500 50  0000 L CNN
F 2 "" H 3200 1100 50  0001 C CNN
F 3 "" H 3200 1100 50  0001 C CNN
	1    3050 1150
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5A8113F0
P 3800 1100
F 0 "C1" H 3825 1200 50  0000 L CNN
F 1 "100uF" H 3825 1000 50  0000 L CNN
F 2 "" H 3838 950 50  0001 C CNN
F 3 "" H 3800 1100 50  0001 C CNN
	1    3800 1100
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5A81149D
P 3850 3050
F 0 "C2" H 3875 3150 50  0000 L CNN
F 1 "100uF" H 3875 2950 50  0000 L CNN
F 2 "" H 3888 2900 50  0001 C CNN
F 3 "" H 3850 3050 50  0001 C CNN
	1    3850 3050
	1    0    0    -1  
$EndComp
$Comp
L Conn_01x03 J7
U 1 1 5A833215
P 5100 1050
F 0 "J7" H 5100 1250 50  0000 C CNN
F 1 "MTR PWR" H 5100 850 50  0000 C CNN
F 2 "" H 5100 1050 50  0001 C CNN
F 3 "" H 5100 1050 50  0001 C CNN
	1    5100 1050
	1    0    0    -1  
$EndComp
$Comp
L Conn_02x03_Odd_Even J5
U 1 1 5A8332C2
P 4350 3850
F 0 "J5" H 4400 4050 50  0000 C CNN
F 1 "PWR 1" H 4400 3650 50  0000 C CNN
F 2 "" H 4350 3850 50  0001 C CNN
F 3 "" H 4350 3850 50  0001 C CNN
	1    4350 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	3850 3500 3850 3200
Wire Wire Line
	3350 950  4900 950 
Wire Wire Line
	3800 1250 3800 2450
Wire Wire Line
	2950 1550 4200 1550
Connection ~ 3800 950 
Wire Wire Line
	4200 1550 4200 1050
Wire Wire Line
	4200 1050 4900 1050
Connection ~ 3800 1550
Wire Wire Line
	4900 1150 4600 1150
Wire Wire Line
	4600 1150 4600 950 
Connection ~ 4600 950 
Wire Wire Line
	3350 1850 4850 1850
Wire Wire Line
	3500 1850 3500 950 
Connection ~ 3500 950 
Connection ~ 3500 1850
Wire Wire Line
	4850 1950 4750 1950
Wire Wire Line
	4750 1950 4750 1850
Connection ~ 4750 1850
Wire Wire Line
	4850 2050 4650 2050
Wire Wire Line
	4650 2050 4650 1850
Connection ~ 4650 1850
Connection ~ 3800 2450
Wire Wire Line
	3350 3750 4150 3750
Wire Wire Line
	4150 3850 4100 3850
Wire Wire Line
	4100 3850 4100 3750
Connection ~ 4100 3750
Wire Wire Line
	4150 3950 4050 3950
Wire Wire Line
	4050 3950 4050 3750
Connection ~ 4050 3750
Wire Wire Line
	4650 3850 4750 3850
Wire Wire Line
	4750 3750 4650 3750
Connection ~ 3850 2900
Connection ~ 3850 3500
Connection ~ 3050 3500
Connection ~ 3050 2450
Connection ~ 3050 4350
NoConn ~ 3350 2050
NoConn ~ 3350 2150
NoConn ~ 3350 3100
NoConn ~ 3350 3200
NoConn ~ 3350 3300
NoConn ~ 3350 3950
NoConn ~ 3350 4050
NoConn ~ 3350 1150
NoConn ~ 3350 1250
NoConn ~ 3350 1350
Connection ~ 3050 1550
Wire Wire Line
	4550 5100 4800 5100
Wire Wire Line
	4800 5100 4800 5250
Wire Wire Line
	4550 5000 4800 5000
Wire Wire Line
	4800 5000 4800 4900
$Comp
L VCC #PWR?
U 1 1 5A85D110
P 3850 2900
F 0 "#PWR?" H 3850 2750 50  0001 C CNN
F 1 "VCC" H 3850 3050 50  0000 C CNN
F 2 "" H 3850 2900 50  0001 C CNN
F 3 "" H 3850 2900 50  0001 C CNN
	1    3850 2900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A85D142
P 3850 3500
F 0 "#PWR?" H 3850 3250 50  0001 C CNN
F 1 "GND" H 3850 3350 50  0000 C CNN
F 2 "" H 3850 3500 50  0001 C CNN
F 3 "" H 3850 3500 50  0001 C CNN
	1    3850 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3600 2900 3600 3750
Connection ~ 3600 3750
Connection ~ 3600 2900
Wire Wire Line
	3750 3500 3750 4350
Connection ~ 3750 4350
Connection ~ 3750 3500
$Comp
L GND #PWR?
U 1 1 5A85D2B7
P 4800 6050
F 0 "#PWR?" H 4800 5800 50  0001 C CNN
F 1 "GND" H 4800 5900 50  0000 C CNN
F 2 "" H 4800 6050 50  0001 C CNN
F 3 "" H 4800 6050 50  0001 C CNN
	1    4800 6050
	1    0    0    -1  
$EndComp
Text Label 4550 5900 0    60   ~ 0
ENCL_SIG
Text Label 4550 5200 0    60   ~ 0
ENCR_SIG
$Comp
L VCC #PWR?
U 1 1 5A85D382
P 4800 4900
F 0 "#PWR?" H 4800 4750 50  0001 C CNN
F 1 "VCC" H 4800 5050 50  0000 C CNN
F 2 "" H 4800 4900 50  0001 C CNN
F 3 "" H 4800 4900 50  0001 C CNN
	1    4800 4900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A85D3B4
P 4800 5250
F 0 "#PWR?" H 4800 5000 50  0001 C CNN
F 1 "GND" H 4800 5100 50  0000 C CNN
F 2 "" H 4800 5250 50  0001 C CNN
F 3 "" H 4800 5250 50  0001 C CNN
	1    4800 5250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 5700 4550 5700
Wire Wire Line
	4550 5800 4800 5800
Wire Wire Line
	4800 5800 4800 6050
Wire Wire Line
	4800 5700 4800 5600
$Comp
L VCC #PWR?
U 1 1 5A85D632
P 4800 5600
F 0 "#PWR?" H 4800 5450 50  0001 C CNN
F 1 "VCC" H 4800 5750 50  0000 C CNN
F 2 "" H 4800 5600 50  0001 C CNN
F 3 "" H 4800 5600 50  0001 C CNN
	1    4800 5600
	1    0    0    -1  
$EndComp
Text Label 5200 3100 0    60   ~ 0
MOT_LF
Text Label 5200 3200 0    60   ~ 0
MOT_LR
Text Label 5200 3300 0    60   ~ 0
MOT_RF
Text Label 5200 3400 0    60   ~ 0
MOT_RR
Text Label 3350 5000 0    60   ~ 0
TSERVO_SIG
Text Label 3350 5700 0    60   ~ 0
PSERVO_SIG
Wire Wire Line
	2950 3500 3850 3500
Wire Wire Line
	3350 2900 3850 2900
Wire Wire Line
	3750 4350 2950 4350
Wire Wire Line
	4750 3750 4750 4050
Wire Wire Line
	4750 3950 4650 3950
Connection ~ 4750 3850
Connection ~ 4750 3950
$Comp
L GND #PWR?
U 1 1 5A85ED3E
P 4750 4050
F 0 "#PWR?" H 4750 3800 50  0001 C CNN
F 1 "GND" H 4750 3900 50  0000 C CNN
F 2 "" H 4750 4050 50  0001 C CNN
F 3 "" H 4750 4050 50  0001 C CNN
	1    4750 4050
	1    0    0    -1  
$EndComp
Wire Wire Line
	5450 2450 2950 2450
Wire Wire Line
	5350 1850 5450 1850
Wire Wire Line
	5450 1850 5450 2450
Wire Wire Line
	5450 1950 5350 1950
Wire Wire Line
	5450 2050 5350 2050
Connection ~ 5450 1950
Connection ~ 5450 2050
Wire Wire Line
	3350 5800 3200 5800
Wire Wire Line
	3200 5800 3200 5600
Wire Wire Line
	3350 5900 3200 5900
Wire Wire Line
	3200 5900 3200 6050
$Comp
L VCC #PWR?
U 1 1 5A85F4D8
P 3200 5600
F 0 "#PWR?" H 3200 5450 50  0001 C CNN
F 1 "VCC" H 3200 5750 50  0000 C CNN
F 2 "" H 3200 5600 50  0001 C CNN
F 3 "" H 3200 5600 50  0001 C CNN
	1    3200 5600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR?
U 1 1 5A85F50A
P 3200 6050
F 0 "#PWR?" H 3200 5800 50  0001 C CNN
F 1 "GND" H 3200 5900 50  0000 C CNN
F 2 "" H 3200 6050 50  0001 C CNN
F 3 "" H 3200 6050 50  0001 C CNN
	1    3200 6050
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 5100 3200 5100
Wire Wire Line
	3200 5100 3200 4900
Wire Wire Line
	3350 5200 3200 5200
$Comp
L GND #PWR?
U 1 1 5A85F5ED
P 3200 5200
F 0 "#PWR?" H 3200 4950 50  0001 C CNN
F 1 "GND" H 3200 5050 50  0000 C CNN
F 2 "" H 3200 5200 50  0001 C CNN
F 3 "" H 3200 5200 50  0001 C CNN
	1    3200 5200
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR?
U 1 1 5A85F61F
P 3200 4900
F 0 "#PWR?" H 3200 4750 50  0001 C CNN
F 1 "VCC" H 3200 5050 50  0000 C CNN
F 2 "" H 3200 4900 50  0001 C CNN
F 3 "" H 3200 4900 50  0001 C CNN
	1    3200 4900
	1    0    0    -1  
$EndComp
$EndSCHEMATC
