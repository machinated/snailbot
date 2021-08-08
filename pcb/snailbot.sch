EESchema Schematic File Version 4
LIBS:snailbot-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "Snailbot"
Date "2019-09-21"
Rev "2"
Comp "Michał Cieśnik"
Comment1 "Arduino compatible robot"
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Battery_Management:MCP73831-2-OT U1
U 1 1 5D5AF21A
P 3350 1750
F 0 "U1" H 3350 2228 50  0000 C CNN
F 1 "MCP73831-2-OT" H 3350 2137 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-5" H 3400 1500 50  0001 L CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20001984g.pdf" H 3200 1700 50  0001 C CNN
	1    3350 1750
	1    0    0    -1  
$EndComp
$Comp
L Device:Battery_Cell BT1
U 1 1 5D5AF3EE
P 4050 2300
F 0 "BT1" H 3932 2396 50  0000 R CNN
F 1 "Battery_Cell" H 3932 2305 50  0000 R CNN
F 2 "mc_mech:BatteryHolder_1x18650" V 4050 2360 50  0001 C CNN
F 3 "~" V 4050 2360 50  0001 C CNN
	1    4050 2300
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0101
U 1 1 5D5AF7FE
P 8650 1650
F 0 "#PWR0101" H 8650 1500 50  0001 C CNN
F 1 "VCC" H 8667 1823 50  0000 C CNN
F 2 "" H 8650 1650 50  0001 C CNN
F 3 "" H 8650 1650 50  0001 C CNN
	1    8650 1650
	1    0    0    -1  
$EndComp
$Comp
L power:+BATT #PWR0102
U 1 1 5D5AF82D
P 4050 1550
F 0 "#PWR0102" H 4050 1400 50  0001 C CNN
F 1 "+BATT" H 4065 1723 50  0000 C CNN
F 2 "" H 4050 1550 50  0001 C CNN
F 3 "" H 4050 1550 50  0001 C CNN
	1    4050 1550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5D5AFA3E
P 2350 1950
F 0 "#PWR0103" H 2350 1700 50  0001 C CNN
F 1 "GND" H 2355 1777 50  0000 C CNN
F 2 "" H 2350 1950 50  0001 C CNN
F 3 "" H 2350 1950 50  0001 C CNN
	1    2350 1950
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5D5AFA7B
P 2750 1850
F 0 "R2" V 2543 1850 50  0000 C CNN
F 1 "2k2" V 2634 1850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2680 1850 50  0001 C CNN
F 3 "~" H 2750 1850 50  0001 C CNN
	1    2750 1850
	0    1    1    0   
$EndComp
$Comp
L Device:Polyfuse F2
U 1 1 5D5AFB39
P 4050 1900
F 0 "F2" H 4138 1946 50  0000 L CNN
F 1 "1A" H 4138 1855 50  0000 L CNN
F 2 "Fuse:Fuse_1812_4532Metric_Pad1.30x3.40mm_HandSolder" H 4100 1700 50  0001 L CNN
F 3 "~" H 4050 1900 50  0001 C CNN
	1    4050 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C1
U 1 1 5D5AFD5D
P 2350 1650
F 0 "C1" H 2465 1696 50  0000 L CNN
F 1 "4u7" H 2465 1605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2388 1500 50  0001 C CNN
F 3 "~" H 2350 1650 50  0001 C CNN
	1    2350 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2950 1850 2900 1850
Wire Wire Line
	2600 1850 2350 1850
Wire Wire Line
	2350 1850 2350 1800
Wire Wire Line
	2350 1850 2350 1950
Connection ~ 2350 1850
Wire Wire Line
	3350 1450 3350 1400
Wire Wire Line
	3350 1400 2850 1400
Wire Wire Line
	2350 1400 2350 1500
$Comp
L power:GND #PWR0104
U 1 1 5D5B00AF
P 3350 2100
F 0 "#PWR0104" H 3350 1850 50  0001 C CNN
F 1 "GND" H 3355 1927 50  0000 C CNN
F 2 "" H 3350 2100 50  0001 C CNN
F 3 "" H 3350 2100 50  0001 C CNN
	1    3350 2100
	1    0    0    -1  
$EndComp
Wire Wire Line
	3350 2050 3350 2100
$Comp
L Device:LED D1
U 1 1 5D5B017E
P 3100 1150
F 0 "D1" H 3092 895 50  0000 C CNN
F 1 "RED" H 3092 986 50  0000 C CNN
F 2 "LED_SMD:LED_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 3100 1150 50  0001 C CNN
F 3 "~" H 3100 1150 50  0001 C CNN
	1    3100 1150
	-1   0    0    1   
$EndComp
$Comp
L Device:R R1
U 1 1 5D5B03E5
P 3550 1150
F 0 "R1" V 3343 1150 50  0000 C CNN
F 1 "1k" V 3434 1150 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3480 1150 50  0001 C CNN
F 3 "~" H 3550 1150 50  0001 C CNN
	1    3550 1150
	0    1    1    0   
$EndComp
$Comp
L MCU_Microchip_ATmega:ATmega328P-PU U4
U 1 1 5D5B0AD8
P 1850 5450
F 0 "U4" H 1209 5496 50  0000 R CNN
F 1 "ATmega328P-PU" H 1209 5405 50  0000 R CNN
F 2 "Package_DIP:DIP-28_W7.62mm" H 1850 5450 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/ATmega328_P%20AVR%20MCU%20with%20picoPower%20Technology%20Data%20Sheet%2040001984A.pdf" H 1850 5450 50  0001 C CNN
	1    1850 5450
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5D5B0BC6
P 4050 2500
F 0 "#PWR0105" H 4050 2250 50  0001 C CNN
F 1 "GND" H 4055 2327 50  0000 C CNN
F 2 "" H 4050 2500 50  0001 C CNN
F 3 "" H 4050 2500 50  0001 C CNN
	1    4050 2500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3750 1850 3850 1850
Wire Wire Line
	3850 1850 3850 1150
Wire Wire Line
	3850 1150 3700 1150
Wire Wire Line
	3400 1150 3250 1150
Wire Wire Line
	2950 1150 2850 1150
Wire Wire Line
	2850 1150 2850 1400
Connection ~ 2850 1400
Wire Wire Line
	2850 1400 2350 1400
Wire Wire Line
	3750 1650 4050 1650
Wire Wire Line
	4050 1650 4050 1750
Wire Wire Line
	4050 1650 4050 1550
Connection ~ 4050 1650
Wire Wire Line
	4050 1650 4400 1650
Wire Wire Line
	4400 1650 4400 1750
Connection ~ 4400 1650
Wire Wire Line
	4050 2050 4050 2100
Wire Wire Line
	4050 2400 4050 2450
Wire Wire Line
	4050 2450 4400 2450
Wire Wire Line
	4400 2450 4400 2050
Connection ~ 4050 2450
Wire Wire Line
	4050 2450 4050 2500
$Comp
L Device:L L1
U 1 1 5D5B22BD
P 6800 1250
F 0 "L1" V 6622 1250 50  0000 C CNN
F 1 "4u7" V 6713 1250 50  0000 C CNN
F 2 "Inductor_SMD:L_1210_3225Metric_Pad1.42x2.65mm_HandSolder" H 6800 1250 50  0001 C CNN
F 3 "~" H 6800 1250 50  0001 C CNN
	1    6800 1250
	0    1    1    0   
$EndComp
$Comp
L Transistor_Array:ULN2003 U3
U 1 1 5D5B2549
P 9400 3900
F 0 "U3" H 9400 4567 50  0000 C CNN
F 1 "ULN2003" H 9400 4476 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 9450 3350 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/uln2003a.pdf" H 9500 3700 50  0001 C CNN
	1    9400 3900
	1    0    0    -1  
$EndComp
$Comp
L mc_analog:MCP1642B U2
U 1 1 5D5B4EAE
P 6800 1850
F 0 "U2" H 6800 2317 50  0000 C CNN
F 1 "MCP1642B" H 6800 2226 50  0000 C CNN
F 2 "Package_SO:MSOP-8-1EP_3x3mm_P0.65mm_EP2.54x2.8mm" H 6850 1600 50  0001 L CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/20005253A.pdf" H 6550 2300 50  0001 C CNN
	1    6800 1850
	1    0    0    -1  
$EndComp
Wire Notes Line
	2200 3100 2200 750 
Wire Notes Line
	2200 750  4800 750 
Wire Notes Line
	4800 750  4800 3100
Wire Notes Line
	4800 3100 2200 3100
Text Notes 3050 750  0    98   ~ 0
CHARGING
Wire Wire Line
	6650 1250 6200 1250
Wire Wire Line
	6200 1250 6200 1650
Wire Wire Line
	6200 1650 6400 1650
$Comp
L Device:C C4
U 1 1 5D5B24C9
P 5250 1950
F 0 "C4" H 5135 1996 50  0000 R CNN
F 1 "10u" H 5135 1905 50  0000 R CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 5288 1800 50  0001 C CNN
F 3 "~" H 5250 1950 50  0001 C CNN
	1    5250 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 5D5B25B7
P 6800 2250
F 0 "#PWR0106" H 6800 2000 50  0001 C CNN
F 1 "GND" H 6805 2077 50  0000 C CNN
F 2 "" H 6800 2250 50  0001 C CNN
F 3 "" H 6800 2250 50  0001 C CNN
	1    6800 2250
	1    0    0    -1  
$EndComp
NoConn ~ 6400 1950
$Comp
L Device:R R6
U 1 1 5D5B42D6
P 7400 2350
F 0 "R6" H 7470 2396 50  0000 L CNN
F 1 "150k 1%" H 7470 2305 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7330 2350 50  0001 C CNN
F 3 "~" H 7400 2350 50  0001 C CNN
	1    7400 2350
	1    0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 5D5B433A
P 7400 1950
F 0 "R3" H 7470 1996 50  0000 L CNN
F 1 "470k 1%" H 7470 1905 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7330 1950 50  0001 C CNN
F 3 "~" H 7400 1950 50  0001 C CNN
	1    7400 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5D5B44F2
P 7400 2600
F 0 "#PWR0107" H 7400 2350 50  0001 C CNN
F 1 "GND" H 7405 2427 50  0000 C CNN
F 2 "" H 7400 2600 50  0001 C CNN
F 3 "" H 7400 2600 50  0001 C CNN
	1    7400 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	7400 2200 7400 2150
Wire Wire Line
	7400 2150 7300 2150
Connection ~ 7400 2150
Wire Wire Line
	7400 2150 7400 2100
Wire Wire Line
	7400 1800 7400 1750
$Comp
L Device:C C5
U 1 1 5D5B5EFD
P 7950 2050
F 0 "C5" H 8065 2096 50  0000 L CNN
F 1 "10u" H 8065 2005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 7988 1900 50  0001 C CNN
F 3 "~" H 7950 2050 50  0001 C CNN
	1    7950 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:C C6
U 1 1 5D5B5F95
P 8300 2050
F 0 "C6" H 8415 2096 50  0000 L CNN
F 1 "10u" H 8415 2005 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 8338 1900 50  0001 C CNN
F 3 "~" H 8300 2050 50  0001 C CNN
	1    8300 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D3
U 1 1 5D5B60B7
P 8650 2350
F 0 "D3" V 8688 2232 50  0000 R CNN
F 1 "YELLOW" V 8597 2232 50  0000 R CNN
F 2 "LED_SMD:LED_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 8650 2350 50  0001 C CNN
F 3 "~" H 8650 2350 50  0001 C CNN
	1    8650 2350
	0    -1   -1   0   
$EndComp
$Comp
L Device:R R4
U 1 1 5D5B62CE
P 8650 1950
F 0 "R4" H 8720 1996 50  0000 L CNN
F 1 "470" H 8720 1905 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 8580 1950 50  0001 C CNN
F 3 "~" H 8650 1950 50  0001 C CNN
	1    8650 1950
	1    0    0    1   
$EndComp
Wire Wire Line
	7400 2500 7400 2550
Wire Wire Line
	7400 2550 7950 2550
Wire Wire Line
	7950 2550 7950 2200
Connection ~ 7400 2550
Wire Wire Line
	7400 2550 7400 2600
Wire Wire Line
	7950 2550 8300 2550
Wire Wire Line
	8300 2550 8300 2200
Connection ~ 7950 2550
Wire Wire Line
	8300 2550 8650 2550
Connection ~ 8300 2550
Wire Wire Line
	7400 1750 7950 1750
Wire Wire Line
	7950 1750 7950 1900
Wire Wire Line
	7950 1750 8300 1750
Wire Wire Line
	8300 1750 8300 1900
Connection ~ 7950 1750
Wire Wire Line
	8650 2200 8650 2100
Wire Wire Line
	8650 2550 8650 2500
Wire Wire Line
	8300 1750 8650 1750
Wire Wire Line
	8650 1750 8650 1800
Connection ~ 8300 1750
Wire Wire Line
	8650 1750 8650 1650
Connection ~ 8650 1750
$Comp
L Switch:SW_Push SW1
U 1 1 5D5BD1EA
P 5750 2350
F 0 "SW1" H 5750 2257 50  0000 C CNN
F 1 "SW_Push" H 5750 2166 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_SPST_CK_RS282G05A3" H 5750 2550 50  0001 C CNN
F 3 "" H 5750 2550 50  0001 C CNN
	1    5750 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 1650 5250 1800
Connection ~ 6200 1650
Connection ~ 5250 1650
$Comp
L Device:R R5
U 1 1 5D5BFD70
P 6000 2050
F 0 "R5" H 6070 2096 50  0000 L CNN
F 1 "1M" H 6070 2005 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5930 2050 50  0001 C CNN
F 3 "~" H 6000 2050 50  0001 C CNN
	1    6000 2050
	1    0    0    1   
$EndComp
$Comp
L Device:R R8
U 1 1 5D5BFE24
P 6000 2600
F 0 "R8" H 6070 2646 50  0000 L CNN
F 1 "100k" H 6070 2555 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5930 2600 50  0001 C CNN
F 3 "~" H 6000 2600 50  0001 C CNN
	1    6000 2600
	1    0    0    1   
$EndComp
Wire Wire Line
	6400 1850 6000 1850
Wire Wire Line
	6000 1850 6000 1900
Wire Wire Line
	6000 1850 5750 1850
Connection ~ 6000 1850
$Comp
L power:GND #PWR0108
U 1 1 5D5C4F77
P 6000 2850
F 0 "#PWR0108" H 6000 2600 50  0001 C CNN
F 1 "GND" H 6005 2677 50  0000 C CNN
F 2 "" H 6000 2850 50  0001 C CNN
F 3 "" H 6000 2850 50  0001 C CNN
	1    6000 2850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 2750 6000 2800
Wire Wire Line
	6000 2450 6000 2350
Wire Wire Line
	5950 2350 6000 2350
Connection ~ 6000 2350
Wire Wire Line
	6000 2350 6000 2200
Wire Wire Line
	6000 2350 6300 2350
$Comp
L power:GND #PWR0109
U 1 1 5D5C9BE6
P 5250 2200
F 0 "#PWR0109" H 5250 1950 50  0001 C CNN
F 1 "GND" H 5255 2027 50  0000 C CNN
F 2 "" H 5250 2200 50  0001 C CNN
F 3 "" H 5250 2200 50  0001 C CNN
	1    5250 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5250 2100 5250 2200
$Comp
L Device:C C7
U 1 1 5D5CB322
P 6300 2600
F 0 "C7" H 6415 2646 50  0000 L CNN
F 1 "10u" H 6415 2555 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 6338 2450 50  0001 C CNN
F 3 "~" H 6300 2600 50  0001 C CNN
	1    6300 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	6000 2800 6300 2800
Wire Wire Line
	6300 2800 6300 2750
Connection ~ 6000 2800
Wire Wire Line
	6000 2800 6000 2850
Wire Wire Line
	6300 2450 6300 2350
Connection ~ 6300 2350
Wire Wire Line
	6300 2350 6450 2350
Wire Notes Line
	4900 750  9100 750 
Wire Notes Line
	9100 750  9100 3100
Wire Notes Line
	9100 3100 4900 3100
Wire Notes Line
	4900 3100 4900 750 
Text Notes 6700 750  0    98   ~ 0
POWER
$Comp
L power:GND #PWR0110
U 1 1 5D5D9221
P 1850 7000
F 0 "#PWR0110" H 1850 6750 50  0001 C CNN
F 1 "GND" H 1855 6827 50  0000 C CNN
F 2 "" H 1850 7000 50  0001 C CNN
F 3 "" H 1850 7000 50  0001 C CNN
	1    1850 7000
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0111
U 1 1 5D5D925C
P 1950 3700
F 0 "#PWR0111" H 1950 3550 50  0001 C CNN
F 1 "VCC" H 1967 3873 50  0000 C CNN
F 2 "" H 1950 3700 50  0001 C CNN
F 3 "" H 1950 3700 50  0001 C CNN
	1    1950 3700
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 5D5D9297
P 1050 4450
F 0 "C10" H 1165 4496 50  0000 L CNN
F 1 "100n" H 1165 4405 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1088 4300 50  0001 C CNN
F 3 "~" H 1050 4450 50  0001 C CNN
	1    1050 4450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R13
U 1 1 5D5D937F
P 2800 5550
F 0 "R13" H 2870 5596 50  0000 L CNN
F 1 "10k" H 2870 5505 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 2730 5550 50  0001 C CNN
F 3 "~" H 2800 5550 50  0001 C CNN
	1    2800 5550
	1    0    0    1   
$EndComp
$Comp
L Switch:SW_Push SW2
U 1 1 5D5D9451
P 3500 5750
F 0 "SW2" H 3500 6035 50  0000 C CNN
F 1 "SW_Push" H 3500 5944 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_SPST_CK_RS282G05A3" H 3500 5950 50  0001 C CNN
F 3 "" H 3500 5950 50  0001 C CNN
	1    3500 5750
	1    0    0    -1  
$EndComp
$Comp
L Connector:USB_B_Micro J1
U 1 1 5D5D9777
P 1000 1600
F 0 "J1" H 1055 2067 50  0000 C CNN
F 1 "USB_B_Micro" H 1055 1976 50  0000 C CNN
F 2 "mc_mech:USB_Micro-B_MOLEX_105017-0001" H 1150 1550 50  0001 C CNN
F 3 "~" H 1150 1550 50  0001 C CNN
	1    1000 1600
	1    0    0    -1  
$EndComp
$Comp
L Transistor_Array:ULN2003 U5
U 1 1 5D5D9B11
P 9400 5550
F 0 "U5" H 9400 6217 50  0000 C CNN
F 1 "ULN2003" H 9400 6126 50  0000 C CNN
F 2 "Package_SO:SOIC-16_3.9x9.9mm_P1.27mm" H 9450 5000 50  0001 L CNN
F 3 "http://www.ti.com/lit/ds/symlink/uln2003a.pdf" H 9500 5350 50  0001 C CNN
	1    9400 5550
	1    0    0    -1  
$EndComp
$Comp
L Device:D_TVS D2
U 1 1 5D5D9C66
P 1500 1600
F 0 "D2" V 1454 1679 50  0000 L CNN
F 1 "D_TVS" V 1545 1679 50  0000 L CNN
F 2 "Diode_SMD:D_SMB_Handsoldering" H 1500 1600 50  0001 C CNN
F 3 "~" H 1500 1600 50  0001 C CNN
	1    1500 1600
	0    1    1    0   
$EndComp
NoConn ~ 1300 1600
NoConn ~ 1300 1700
NoConn ~ 1300 1800
$Comp
L Device:R R7
U 1 1 5D5DE899
P 900 2500
F 0 "R7" H 970 2546 50  0000 L CNN
F 1 "1k" H 970 2455 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 830 2500 50  0001 C CNN
F 3 "~" H 900 2500 50  0001 C CNN
	1    900  2500
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5D5DEA79
P 900 2700
F 0 "#PWR0112" H 900 2450 50  0001 C CNN
F 1 "GND" H 905 2527 50  0000 C CNN
F 2 "" H 900 2700 50  0001 C CNN
F 3 "" H 900 2700 50  0001 C CNN
	1    900  2700
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR0113
U 1 1 5D5DEBD9
P 1950 1300
F 0 "#PWR0113" H 1950 1150 50  0001 C CNN
F 1 "VDD" H 1967 1473 50  0000 C CNN
F 2 "" H 1950 1300 50  0001 C CNN
F 3 "" H 1950 1300 50  0001 C CNN
	1    1950 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1300 1400 1500 1400
Wire Wire Line
	1500 1400 1500 1450
$Comp
L power:GND #PWR0114
U 1 1 5D5E246A
P 1000 2050
F 0 "#PWR0114" H 1000 1800 50  0001 C CNN
F 1 "GND" H 1005 1877 50  0000 C CNN
F 2 "" H 1000 2050 50  0001 C CNN
F 3 "" H 1000 2050 50  0001 C CNN
	1    1000 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1000 2000 1000 2050
Wire Wire Line
	900  2000 900  2300
Wire Wire Line
	900  2300 1500 2300
Wire Wire Line
	1500 2300 1500 1750
Wire Wire Line
	900  2650 900  2700
Wire Wire Line
	900  2350 900  2300
Connection ~ 900  2300
$Comp
L Device:Polyfuse F1
U 1 1 5D5EDA8F
P 1750 1400
F 0 "F1" V 1525 1400 50  0000 C CNN
F 1 "750mA" V 1616 1400 50  0000 C CNN
F 2 "Fuse:Fuse_1812_4532Metric_Pad1.30x3.40mm_HandSolder" H 1800 1200 50  0001 L CNN
F 3 "~" H 1750 1400 50  0001 C CNN
	1    1750 1400
	0    1    1    0   
$EndComp
Wire Wire Line
	1500 1400 1600 1400
Connection ~ 1500 1400
Wire Wire Line
	1900 1400 1950 1400
Wire Wire Line
	1950 1400 1950 1300
Wire Wire Line
	1850 6950 1850 7000
$Comp
L Device:C C8
U 1 1 5D5F60DD
P 1650 3800
F 0 "C8" V 1398 3800 50  0000 C CNN
F 1 "100n" V 1489 3800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 1688 3650 50  0001 C CNN
F 3 "~" H 1650 3800 50  0001 C CNN
	1    1650 3800
	0    1    1    0   
$EndComp
$Comp
L Device:C C9
U 1 1 5D5F617F
P 2150 3800
F 0 "C9" V 1898 3800 50  0000 C CNN
F 1 "100n" V 1989 3800 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2188 3650 50  0001 C CNN
F 3 "~" H 2150 3800 50  0001 C CNN
	1    2150 3800
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5D5F62ED
P 1050 4650
F 0 "#PWR0115" H 1050 4400 50  0001 C CNN
F 1 "GND" H 1055 4477 50  0000 C CNN
F 2 "" H 1050 4650 50  0001 C CNN
F 3 "" H 1050 4650 50  0001 C CNN
	1    1050 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 4650 1050 4600
Wire Wire Line
	1250 4250 1050 4250
Wire Wire Line
	1050 4250 1050 4300
$Comp
L power:VCC #PWR0116
U 1 1 5D5FA96D
P 1050 4200
F 0 "#PWR0116" H 1050 4050 50  0001 C CNN
F 1 "VCC" H 1067 4373 50  0000 C CNN
F 2 "" H 1050 4200 50  0001 C CNN
F 3 "" H 1050 4200 50  0001 C CNN
	1    1050 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	1050 4250 1050 4200
Connection ~ 1050 4250
$Comp
L power:GND #PWR0117
U 1 1 5D60B786
P 2350 3850
F 0 "#PWR0117" H 2350 3600 50  0001 C CNN
F 1 "GND" H 2438 3813 50  0000 L CNN
F 2 "" H 2350 3850 50  0001 C CNN
F 3 "" H 2350 3850 50  0001 C CNN
	1    2350 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2300 3800 2350 3800
Wire Wire Line
	2350 3800 2350 3850
$Comp
L power:GND #PWR0118
U 1 1 5D60DF6F
P 1450 3850
F 0 "#PWR0118" H 1450 3600 50  0001 C CNN
F 1 "GND" H 1372 3813 50  0000 R CNN
F 2 "" H 1450 3850 50  0001 C CNN
F 3 "" H 1450 3850 50  0001 C CNN
	1    1450 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 3950 1950 3800
Wire Wire Line
	1950 3800 2000 3800
Wire Wire Line
	1850 3950 1850 3800
Wire Wire Line
	1850 3800 1800 3800
Wire Wire Line
	1950 3800 1950 3700
Connection ~ 1950 3800
Wire Wire Line
	1950 3800 1850 3800
Connection ~ 1850 3800
Wire Wire Line
	1450 3850 1450 3800
Wire Wire Line
	1450 3800 1500 3800
$Comp
L power:VCC #PWR0119
U 1 1 5D5BFCBA
P 2800 5300
F 0 "#PWR0119" H 2800 5150 50  0001 C CNN
F 1 "VCC" H 2900 5350 50  0000 C CNN
F 2 "" H 2800 5300 50  0001 C CNN
F 3 "" H 2800 5300 50  0001 C CNN
	1    2800 5300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 5D5BFD09
P 3750 5850
F 0 "#PWR0120" H 3750 5600 50  0001 C CNN
F 1 "GND" H 3838 5813 50  0000 L CNN
F 2 "" H 3750 5850 50  0001 C CNN
F 3 "" H 3750 5850 50  0001 C CNN
	1    3750 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:C C13
U 1 1 5D5C0022
P 2800 5950
F 0 "C13" H 2915 5996 50  0000 L CNN
F 1 "100n" H 2915 5905 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 2838 5800 50  0001 C CNN
F 3 "~" H 2800 5950 50  0001 C CNN
	1    2800 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 5750 2800 5750
Wire Wire Line
	2800 5750 2800 5700
Connection ~ 2800 5750
Wire Wire Line
	2800 5800 2800 5750
Wire Wire Line
	3700 5750 3750 5750
Wire Wire Line
	3750 5750 3750 5850
Wire Wire Line
	2800 5400 2800 5300
$Comp
L power:GND #PWR0121
U 1 1 5D5D0DB5
P 2800 6150
F 0 "#PWR0121" H 2800 5900 50  0001 C CNN
F 1 "GND" H 2888 6113 50  0000 L CNN
F 2 "" H 2800 6150 50  0001 C CNN
F 3 "" H 2800 6150 50  0001 C CNN
	1    2800 6150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 6100 2800 6150
$Comp
L Device:R R14
U 1 1 5D5D8109
P 3100 5750
F 0 "R14" V 2893 5750 50  0000 C CNN
F 1 "100" V 2984 5750 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 3030 5750 50  0001 C CNN
F 3 "~" H 3100 5750 50  0001 C CNN
	1    3100 5750
	0    -1   1    0   
$EndComp
Wire Wire Line
	3300 5750 3250 5750
Wire Wire Line
	2950 5750 2800 5750
$Comp
L Device:Crystal Y1
U 1 1 5D5E31B2
P 3000 4750
F 0 "Y1" V 2954 4881 50  0000 L CNN
F 1 "16MHz" V 3045 4881 50  0000 L CNN
F 2 "Crystal:Crystal_HC49-U_Vertical" H 3000 4750 50  0001 C CNN
F 3 "~" H 3000 4750 50  0001 C CNN
	1    3000 4750
	0    1    1    0   
$EndComp
$Comp
L Device:C C12
U 1 1 5D5E321A
P 3250 4950
F 0 "C12" V 3410 4950 50  0000 C CNN
F 1 "22p" V 3501 4950 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3288 4800 50  0001 C CNN
F 3 "~" H 3250 4950 50  0001 C CNN
	1    3250 4950
	0    1    1    0   
$EndComp
$Comp
L Device:C C11
U 1 1 5D5E330D
P 3250 4550
F 0 "C11" V 2998 4550 50  0000 C CNN
F 1 "22p" V 3089 4550 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 3288 4400 50  0001 C CNN
F 3 "~" H 3250 4550 50  0001 C CNN
	1    3250 4550
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0122
U 1 1 5D5EB179
P 3450 5000
F 0 "#PWR0122" H 3450 4750 50  0001 C CNN
F 1 "GND" H 3538 4963 50  0000 L CNN
F 2 "" H 3450 5000 50  0001 C CNN
F 3 "" H 3450 5000 50  0001 C CNN
	1    3450 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3450 5000 3450 4950
Wire Wire Line
	3450 4950 3400 4950
Wire Wire Line
	3400 4550 3450 4550
Wire Wire Line
	3450 4550 3450 4950
Connection ~ 3450 4950
Wire Wire Line
	3100 4950 3000 4950
Wire Wire Line
	3000 4950 3000 4900
Wire Wire Line
	3100 4550 3000 4550
Wire Wire Line
	3000 4550 3000 4600
Wire Wire Line
	2450 4950 3000 4950
Connection ~ 3000 4950
Wire Wire Line
	3000 4550 2750 4550
Wire Wire Line
	2750 4550 2750 4850
Wire Wire Line
	2750 4850 2450 4850
Connection ~ 3000 4550
$Comp
L Connector:Conn_01x05_Male J5
U 1 1 5D60C0DF
P 10550 5450
F 0 "J5" H 10523 5473 50  0000 R CNN
F 1 "Conn_01x05_Male" H 10523 5382 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 10550 5450 50  0001 C CNN
F 3 "~" H 10550 5450 50  0001 C CNN
	1    10550 5450
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x05_Male J3
U 1 1 5D60C36D
P 10500 3900
F 0 "J3" H 10473 3923 50  0000 R CNN
F 1 "Conn_01x05_Male" H 10473 3832 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 10500 3900 50  0001 C CNN
F 3 "~" H 10500 3900 50  0001 C CNN
	1    10500 3900
	-1   0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Female J2
U 1 1 5D60C57C
P 4950 3900
F 0 "J2" H 4843 4185 50  0000 C CNN
F 1 "Conn_01x04_Female" H 4843 4094 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 4950 3900 50  0001 C CNN
F 3 "~" H 4950 3900 50  0001 C CNN
	1    4950 3900
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x06_Female J4
U 1 1 5D60C70A
P 7700 4050
F 0 "J4" H 7750 4500 50  0000 C CNN
F 1 "Conn_01x06_Female" H 7800 4400 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 7700 4050 50  0001 C CNN
F 3 "~" H 7700 4050 50  0001 C CNN
	1    7700 4050
	1    0    0    -1  
$EndComp
$Comp
L Device:LED D5
U 1 1 5D60C9BB
P 7350 5300
F 0 "D5" V 7388 5182 50  0000 R CNN
F 1 "BLUE" V 7297 5182 50  0000 R CNN
F 2 "LED_SMD:LED_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 7350 5300 50  0001 C CNN
F 3 "~" H 7350 5300 50  0001 C CNN
	1    7350 5300
	0    -1   -1   0   
$EndComp
$Comp
L Switch:SW_Push SW3
U 1 1 5D60D168
P 6500 5600
F 0 "SW3" H 6500 5885 50  0000 C CNN
F 1 "SW_Push" H 6500 5794 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_SPST_CK_RS282G05A3" H 6500 5800 50  0001 C CNN
F 3 "" H 6500 5800 50  0001 C CNN
	1    6500 5600
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0123
U 1 1 5D611F38
P 6150 5150
F 0 "#PWR0123" H 6150 5000 50  0001 C CNN
F 1 "VCC" H 6167 5323 50  0000 C CNN
F 2 "" H 6150 5150 50  0001 C CNN
F 3 "" H 6150 5150 50  0001 C CNN
	1    6150 5150
	1    0    0    -1  
$EndComp
Text GLabel 2450 6150 2    50   Input ~ 0
PD2
Text GLabel 2450 6250 2    50   Input ~ 0
PD3
$Comp
L Device:R R15
U 1 1 5D62098D
P 6150 5350
F 0 "R15" H 6081 5304 50  0000 R CNN
F 1 "10k" H 6081 5395 50  0000 R CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6080 5350 50  0001 C CNN
F 3 "~" H 6150 5350 50  0001 C CNN
	1    6150 5350
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR0124
U 1 1 5D620F0D
P 6750 5700
F 0 "#PWR0124" H 6750 5450 50  0001 C CNN
F 1 "GND" H 6838 5663 50  0000 L CNN
F 2 "" H 6750 5700 50  0001 C CNN
F 3 "" H 6750 5700 50  0001 C CNN
	1    6750 5700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 5200 6150 5150
Wire Wire Line
	6150 5500 6150 5600
Wire Wire Line
	6150 5600 6300 5600
Wire Wire Line
	6700 5600 6750 5600
Wire Wire Line
	6750 5600 6750 5700
Wire Wire Line
	6150 5600 6000 5600
Connection ~ 6150 5600
Text GLabel 6000 5600 0    50   Input ~ 0
PD2
Text GLabel 2450 5950 2    50   Input ~ 0
RXD
Text GLabel 2450 6050 2    50   Input ~ 0
TXD
$Comp
L power:VCC #PWR0125
U 1 1 5D634937
P 7400 4250
F 0 "#PWR0125" H 7400 4100 50  0001 C CNN
F 1 "VCC" V 7400 4450 50  0000 C CNN
F 2 "" H 7400 4250 50  0001 C CNN
F 3 "" H 7400 4250 50  0001 C CNN
	1    7400 4250
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0126
U 1 1 5D63499E
P 7450 4150
F 0 "#PWR0126" H 7450 3900 50  0001 C CNN
F 1 "GND" V 7455 4022 50  0000 R CNN
F 2 "" H 7450 4150 50  0001 C CNN
F 3 "" H 7450 4150 50  0001 C CNN
	1    7450 4150
	0    1    1    0   
$EndComp
Text Notes 7700 3850 0    50   ~ 0
<< STATE
Text Notes 7700 3950 0    50   ~ 0
>> RXD
Text Notes 7700 4050 0    50   ~ 0
<< TXD
Text Notes 7700 4150 0    50   ~ 0
>> GND
Text Notes 7700 4250 0    50   ~ 0
>> VCC
Text Notes 7700 4350 0    50   ~ 0
>> EN
Wire Wire Line
	7500 4150 7450 4150
Wire Wire Line
	7500 4250 7400 4250
Wire Wire Line
	7500 4050 7400 4050
Text GLabel 7400 4050 0    50   Input ~ 0
RXD
$Comp
L Device:R R10
U 1 1 5D64988B
P 6100 3750
F 0 "R10" H 6170 3796 50  0000 L CNN
F 1 "10k" H 6170 3705 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6030 3750 50  0001 C CNN
F 3 "~" H 6100 3750 50  0001 C CNN
	1    6100 3750
	1    0    0    1   
$EndComp
$Comp
L Device:R R11
U 1 1 5D6499A1
P 6100 4150
F 0 "R11" H 6170 4196 50  0000 L CNN
F 1 "15k" H 6170 4105 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6030 4150 50  0001 C CNN
F 3 "~" H 6100 4150 50  0001 C CNN
	1    6100 4150
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR0127
U 1 1 5D649A35
P 6100 4350
F 0 "#PWR0127" H 6100 4100 50  0001 C CNN
F 1 "GND" H 6105 4177 50  0000 C CNN
F 2 "" H 6100 4350 50  0001 C CNN
F 3 "" H 6100 4350 50  0001 C CNN
	1    6100 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4300 6100 4350
Wire Wire Line
	6100 3600 6100 3500
Wire Wire Line
	6100 3500 6000 3500
Text GLabel 6000 3500 0    50   Input ~ 0
TXD
Wire Wire Line
	7500 4350 7400 4350
Wire Wire Line
	7500 3850 7400 3850
Wire Notes Line
	5700 3300 8300 3300
Wire Notes Line
	8300 3300 8300 4600
Wire Notes Line
	8300 4600 5700 4600
Wire Notes Line
	5700 4600 5700 3300
Text Notes 6550 3300 0    98   ~ 0
BLUETOOTH
Text Notes 5000 3800 0    50   ~ 0
GND
Text Notes 5000 3900 0    50   ~ 0
ECHO
Text Notes 5000 4000 0    50   ~ 0
TRIG
Text Notes 5000 4100 0    50   ~ 0
VCC
$Comp
L power:GND #PWR0128
U 1 1 5D6D6967
P 4450 3800
F 0 "#PWR0128" H 4450 3550 50  0001 C CNN
F 1 "GND" V 4455 3672 50  0000 R CNN
F 2 "" H 4450 3800 50  0001 C CNN
F 3 "" H 4450 3800 50  0001 C CNN
	1    4450 3800
	0    1    1    0   
$EndComp
$Comp
L power:VCC #PWR0129
U 1 1 5D6DC461
P 4450 4100
F 0 "#PWR0129" H 4450 3950 50  0001 C CNN
F 1 "VCC" V 4450 4300 50  0000 C CNN
F 2 "" H 4450 4100 50  0001 C CNN
F 3 "" H 4450 4100 50  0001 C CNN
	1    4450 4100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4750 4100 4450 4100
Wire Wire Line
	4750 3800 4450 3800
$Comp
L power:GND #PWR0130
U 1 1 5D6E7E79
P 9400 4550
F 0 "#PWR0130" H 9400 4300 50  0001 C CNN
F 1 "GND" H 9405 4377 50  0000 C CNN
F 2 "" H 9400 4550 50  0001 C CNN
F 3 "" H 9400 4550 50  0001 C CNN
	1    9400 4550
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR0131
U 1 1 5D6E8046
P 9850 3400
F 0 "#PWR0131" H 9850 3250 50  0001 C CNN
F 1 "VCC" V 9850 3600 50  0000 C CNN
F 2 "" H 9850 3400 50  0001 C CNN
F 3 "" H 9850 3400 50  0001 C CNN
	1    9850 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 3500 9850 3500
Wire Wire Line
	9850 3500 9850 3400
Wire Wire Line
	9400 4500 9400 4550
$Comp
L power:VCC #PWR0132
U 1 1 5D7129EA
P 10200 4100
F 0 "#PWR0132" H 10200 3950 50  0001 C CNN
F 1 "VCC" V 10200 4300 50  0000 C CNN
F 2 "" H 10200 4100 50  0001 C CNN
F 3 "" H 10200 4100 50  0001 C CNN
	1    10200 4100
	0    -1   -1   0   
$EndComp
Wire Wire Line
	10200 4100 10300 4100
Wire Wire Line
	9800 4000 10300 4000
Wire Wire Line
	9800 3900 10300 3900
Wire Wire Line
	9800 3800 10300 3800
Wire Wire Line
	9800 3700 10300 3700
$Comp
L power:VCC #PWR0133
U 1 1 5D750204
P 10250 5250
F 0 "#PWR0133" H 10250 5100 50  0001 C CNN
F 1 "VCC" V 10250 5450 50  0000 C CNN
F 2 "" H 10250 5250 50  0001 C CNN
F 3 "" H 10250 5250 50  0001 C CNN
	1    10250 5250
	0    -1   1    0   
$EndComp
Wire Wire Line
	10250 5250 10350 5250
$Comp
L power:GND #PWR0134
U 1 1 5D7564DF
P 9400 6200
F 0 "#PWR0134" H 9400 5950 50  0001 C CNN
F 1 "GND" H 9405 6027 50  0000 C CNN
F 2 "" H 9400 6200 50  0001 C CNN
F 3 "" H 9400 6200 50  0001 C CNN
	1    9400 6200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 6200 9400 6150
$Comp
L power:VCC #PWR0135
U 1 1 5D7760F1
P 9850 5100
F 0 "#PWR0135" H 9850 4950 50  0001 C CNN
F 1 "VCC" V 9850 5300 50  0000 C CNN
F 2 "" H 9850 5100 50  0001 C CNN
F 3 "" H 9850 5100 50  0001 C CNN
	1    9850 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	9800 5150 9850 5150
Wire Wire Line
	9850 5150 9850 5100
$Comp
L power:+BATT #PWR0136
U 1 1 5D77CFAD
P 2700 5100
F 0 "#PWR0136" H 2700 4950 50  0001 C CNN
F 1 "+BATT" H 2850 5150 50  0000 C CNN
F 2 "" H 2700 5100 50  0001 C CNN
F 3 "" H 2700 5100 50  0001 C CNN
	1    2700 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2450 5150 2500 5150
Text GLabel 2450 5550 2    50   Input ~ 0
PC4
Text GLabel 2450 5650 2    50   Input ~ 0
PC5
Text GLabel 5750 1850 0    50   Input ~ 0
PC4
Text GLabel 6450 2350 2    50   Input ~ 0
PC5
Wire Wire Line
	4750 3900 4700 3900
Wire Wire Line
	4750 4000 4700 4000
Text Notes 1250 750  0    98   ~ 0
USB
Wire Notes Line
	2100 750  2100 3100
Wire Notes Line
	2100 3100 650  3100
Wire Notes Line
	650  3100 650  750 
Wire Notes Line
	650  750  2100 750 
Wire Wire Line
	1950 1400 2350 1400
Connection ~ 1950 1400
Connection ~ 2350 1400
Wire Wire Line
	9000 3700 8950 3700
Wire Wire Line
	9000 3800 8950 3800
Wire Wire Line
	9000 3900 8950 3900
Wire Wire Line
	9000 4000 8950 4000
Text GLabel 2450 4750 2    50   Input ~ 0
PB5
Text GLabel 2450 4650 2    50   Input ~ 0
PB4
Text GLabel 2450 4550 2    50   Input ~ 0
PB3
Text GLabel 2450 4450 2    50   Input ~ 0
PB2
Text GLabel 2450 4350 2    50   Input ~ 0
PB1
Text GLabel 4700 4000 0    50   Input ~ 0
PB1
Text GLabel 4700 3900 0    50   Input ~ 0
PD3
Text GLabel 2450 6350 2    50   Input ~ 0
PD4
Text GLabel 2450 6450 2    50   Input ~ 0
PD5
Text GLabel 2450 6550 2    50   Input ~ 0
PD6
Text GLabel 2450 6650 2    50   Input ~ 0
PD7
Wire Wire Line
	9000 5350 8950 5350
Wire Wire Line
	9000 5450 8950 5450
Wire Wire Line
	9000 5550 8950 5550
Wire Wire Line
	9000 5650 8950 5650
$Comp
L Device:R R12
U 1 1 5D61B479
P 7350 5700
F 0 "R12" H 7420 5746 50  0000 L CNN
F 1 "470" H 7420 5655 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 7280 5700 50  0001 C CNN
F 3 "~" H 7350 5700 50  0001 C CNN
	1    7350 5700
	1    0    0    1   
$EndComp
Text GLabel 2450 5450 2    50   Input ~ 0
PC3
Text GLabel 7400 4350 0    50   Input ~ 0
PC3
Text GLabel 2450 4250 2    50   Input ~ 0
PB0
$Comp
L power:GND #PWR0137
U 1 1 5D61CEB4
P 7350 5950
F 0 "#PWR0137" H 7350 5700 50  0001 C CNN
F 1 "GND" H 7355 5777 50  0000 C CNN
F 2 "" H 7350 5950 50  0001 C CNN
F 3 "" H 7350 5950 50  0001 C CNN
	1    7350 5950
	1    0    0    -1  
$EndComp
Wire Wire Line
	7350 5150 7350 5050
Wire Wire Line
	7350 5050 7250 5050
Wire Wire Line
	7350 5550 7350 5450
Wire Wire Line
	7350 5950 7350 5850
Text GLabel 7250 5050 0    50   Input ~ 0
PB0
NoConn ~ 9000 4100
NoConn ~ 9000 4200
NoConn ~ 9000 4300
NoConn ~ 9000 5750
NoConn ~ 9000 5850
NoConn ~ 9000 5950
NoConn ~ 2450 5250
Text GLabel 2450 5350 2    50   Input ~ 0
PC2
$Comp
L Device:LED D4
U 1 1 5D6CDD1B
P 7200 3600
F 0 "D4" H 7191 3816 50  0000 C CNN
F 1 "BLUE" H 7191 3725 50  0000 C CNN
F 2 "LED_SMD:LED_1206_3216Metric_Pad1.42x1.75mm_HandSolder" H 7200 3600 50  0001 C CNN
F 3 "~" H 7200 3600 50  0001 C CNN
	1    7200 3600
	1    0    0    -1  
$EndComp
$Comp
L Device:R R9
U 1 1 5D6CDE9F
P 6850 3600
F 0 "R9" V 6965 3600 50  0000 C CNN
F 1 "470" V 7056 3600 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 6780 3600 50  0001 C CNN
F 3 "~" H 6850 3600 50  0001 C CNN
	1    6850 3600
	0    -1   1    0   
$EndComp
Wire Wire Line
	6100 3900 6100 3950
Wire Wire Line
	6100 3950 7500 3950
Connection ~ 6100 3950
Wire Wire Line
	6100 3950 6100 4000
$Comp
L power:GND #PWR0140
U 1 1 5D7100B0
P 6650 3700
F 0 "#PWR0140" H 6650 3450 50  0001 C CNN
F 1 "GND" H 6655 3527 50  0000 C CNN
F 2 "" H 6650 3700 50  0001 C CNN
F 3 "" H 6650 3700 50  0001 C CNN
	1    6650 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 3600 6650 3600
Wire Wire Line
	6650 3600 6650 3700
Wire Wire Line
	7000 3600 7050 3600
Wire Wire Line
	7350 3600 7400 3600
Wire Wire Line
	7400 3600 7400 3850
Wire Wire Line
	7400 3850 7300 3850
Connection ~ 7400 3850
Text GLabel 7300 3850 0    50   Input ~ 0
PC2
NoConn ~ 9800 5750
NoConn ~ 9800 5850
NoConn ~ 9800 5950
NoConn ~ 9800 4100
NoConn ~ 9800 4200
NoConn ~ 9800 4300
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 5D7A0987
P 1850 2150
F 0 "#FLG0101" H 1850 2225 50  0001 C CNN
F 1 "PWR_FLAG" H 1850 2323 50  0000 C CNN
F 2 "" H 1850 2150 50  0001 C CNN
F 3 "~" H 1850 2150 50  0001 C CNN
	1    1850 2150
	-1   0    0    1   
$EndComp
$Comp
L power:VDD #PWR0138
U 1 1 5D7AA2DC
P 1850 2050
F 0 "#PWR0138" H 1850 1900 50  0001 C CNN
F 1 "VDD" H 1867 2223 50  0000 C CNN
F 2 "" H 1850 2050 50  0001 C CNN
F 3 "" H 1850 2050 50  0001 C CNN
	1    1850 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1850 2050 1850 2150
Wire Notes Line
	5700 4850 7850 4850
Wire Notes Line
	7850 4850 7850 6200
Wire Notes Line
	7850 6200 5700 6200
Wire Notes Line
	5700 6200 5700 4850
Text Notes 6350 4850 0    98   ~ 0
GENERIC I/O
Text GLabel 8950 5650 0    50   Input ~ 0
PD4
Text GLabel 8950 5350 0    50   Input ~ 0
PD7
Text GLabel 8950 5550 0    50   Input ~ 0
PD5
Text GLabel 8950 5450 0    50   Input ~ 0
PD6
Wire Wire Line
	9800 5350 10350 5350
Wire Wire Line
	9800 5450 10350 5450
Wire Wire Line
	9800 5550 10350 5550
Wire Wire Line
	9800 5650 10350 5650
Text GLabel 8950 4000 0    50   Input ~ 0
PB5
Text GLabel 8950 3700 0    50   Input ~ 0
PB2
Text GLabel 8950 3800 0    50   Input ~ 0
PB3
Text GLabel 8950 3900 0    50   Input ~ 0
PB4
$Comp
L Device:Net-Tie_2 NT1
U 1 1 5D7A919F
P 2600 5150
F 0 "NT1" H 2600 5250 50  0000 C CNN
F 1 "Net-Tie_2" H 2650 5200 50  0000 C CNN
F 2 "NetTie:NetTie-2_SMD_Pad0.5mm" H 2600 5150 50  0001 C CNN
F 3 "~" H 2600 5150 50  0001 C CNN
	1    2600 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	2700 5150 2700 5100
Wire Wire Line
	5450 2350 5450 2150
Wire Wire Line
	5450 2350 5550 2350
Wire Wire Line
	7300 1250 7300 1650
Wire Wire Line
	7300 1650 7250 1650
Wire Wire Line
	6950 1250 7300 1250
Wire Wire Line
	7250 1750 7400 1750
Connection ~ 7400 1750
Wire Wire Line
	7250 1950 7300 1950
Wire Wire Line
	7300 1950 7300 2150
Wire Wire Line
	6700 2150 6700 2200
Wire Wire Line
	6700 2200 6800 2200
Wire Wire Line
	6800 2200 6800 2150
Wire Wire Line
	6800 2200 6900 2200
Wire Wire Line
	6900 2200 6900 2150
Connection ~ 6800 2200
Wire Wire Line
	6800 2250 6800 2200
Wire Wire Line
	5250 1650 5450 1650
$Comp
L Device:R R16
U 1 1 5D86D4FB
P 5450 2000
F 0 "R16" H 5520 2046 50  0000 L CNN
F 1 "100" H 5520 1955 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric_Pad1.15x1.40mm_HandSolder" V 5380 2000 50  0001 C CNN
F 3 "~" H 5450 2000 50  0001 C CNN
	1    5450 2000
	1    0    0    1   
$EndComp
Wire Wire Line
	5450 1850 5450 1650
Wire Wire Line
	5450 1650 6200 1650
Connection ~ 5450 1650
$Comp
L Device:C C2
U 1 1 5D8788BD
P 4400 1900
F 0 "C2" H 4515 1946 50  0000 L CNN
F 1 "10u" H 4515 1855 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric_Pad1.15x1.40mm_HandSolder" H 4438 1750 50  0001 C CNN
F 3 "~" H 4400 1900 50  0001 C CNN
	1    4400 1900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4400 1650 5250 1650
Text Notes 2550 2000 0    50   ~ 0
~500mA
$EndSCHEMATC
