EESchema Schematic File Version 4
EELAYER 30 0
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
L Connector:TestPoint TP1
U 1 1 6149BD02
P 6400 5400
F 0 "TP1" H 6458 5518 50  0000 L CNN
F 1 "LoRa" H 6458 5427 50  0000 L CNN
F 2 "TestPoint:TestPoint_Plated_Hole_D2.0mm" H 6600 5400 50  0001 C CNN
F 3 "~" H 6600 5400 50  0001 C CNN
	1    6400 5400
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J1
U 1 1 6149D714
P 6900 5450
F 0 "J1" H 6928 5426 50  0000 L CNN
F 1 "SMA Ant" H 6928 5335 50  0000 L CNN
F 2 "dmcginnis427Footprints:SMA_pcb" H 6900 5450 50  0001 C CNN
F 3 "~" H 6900 5450 50  0001 C CNN
	1    6900 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 5400 6400 5450
Wire Wire Line
	6400 5450 6700 5450
$Comp
L power:GND #PWR09
U 1 1 614A1BD1
P 6600 5550
F 0 "#PWR09" H 6600 5300 50  0001 C CNN
F 1 "GND" H 6605 5377 50  0000 C CNN
F 2 "" H 6600 5550 50  0001 C CNN
F 3 "" H 6600 5550 50  0001 C CNN
	1    6600 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 5550 6600 5550
$Comp
L dmcginnis427:RPIHeader U1
U 1 1 61483681
P 4800 3700
F 0 "U1" H 4750 5865 50  0000 C CNN
F 1 "RPIHeader" H 4750 5774 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x20_P2.54mm_Vertical" H 4500 5200 50  0001 C CNN
F 3 "" H 4500 5200 50  0001 C CNN
	1    4800 3700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5500 2000 5700 2000
Wire Wire Line
	5700 2000 5700 1800
Wire Wire Line
	5700 1800 5500 1800
$Comp
L power:+5V #PWR08
U 1 1 61487914
P 5700 1800
F 0 "#PWR08" H 5700 1650 50  0001 C CNN
F 1 "+5V" H 5715 1973 50  0000 C CNN
F 2 "" H 5700 1800 50  0001 C CNN
F 3 "" H 5700 1800 50  0001 C CNN
	1    5700 1800
	0    1    1    0   
$EndComp
Connection ~ 5700 1800
NoConn ~ 5500 5600
NoConn ~ 5500 5400
NoConn ~ 5500 5200
NoConn ~ 5500 4800
NoConn ~ 5500 4400
NoConn ~ 5500 4200
NoConn ~ 5500 4000
NoConn ~ 5500 3800
NoConn ~ 5500 3400
NoConn ~ 5500 3200
NoConn ~ 5500 2800
NoConn ~ 4000 1800
NoConn ~ 4000 2000
NoConn ~ 4000 2200
NoConn ~ 4000 2400
NoConn ~ 4000 2800
NoConn ~ 4000 3000
NoConn ~ 4000 3200
NoConn ~ 4000 3400
NoConn ~ 4000 3600
NoConn ~ 4000 3800
NoConn ~ 4000 4000
NoConn ~ 4000 4400
NoConn ~ 4000 4600
NoConn ~ 4000 4800
NoConn ~ 4000 5000
NoConn ~ 4000 5200
NoConn ~ 4000 5400
NoConn ~ 5500 3000
$Comp
L power:+5V #PWR0101
U 1 1 61484500
P 8200 3000
F 0 "#PWR0101" H 8200 2850 50  0001 C CNN
F 1 "+5V" H 8215 3173 50  0000 C CNN
F 2 "" H 8200 3000 50  0001 C CNN
F 3 "" H 8200 3000 50  0001 C CNN
	1    8200 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	7950 3000 8200 3000
$Comp
L power:GND #PWR0102
U 1 1 61484E75
P 6900 2400
F 0 "#PWR0102" H 6900 2150 50  0001 C CNN
F 1 "GND" H 6905 2227 50  0000 C CNN
F 2 "" H 6900 2400 50  0001 C CNN
F 3 "" H 6900 2400 50  0001 C CNN
	1    6900 2400
	-1   0    0    -1  
$EndComp
Wire Wire Line
	7150 2400 6900 2400
$Comp
L power:PWR_FLAG #FLG0101
U 1 1 61485745
P 8650 3000
F 0 "#FLG0101" H 8650 3075 50  0001 C CNN
F 1 "PWR_FLAG" H 8650 3173 50  0000 C CNN
F 2 "" H 8650 3000 50  0001 C CNN
F 3 "~" H 8650 3000 50  0001 C CNN
	1    8650 3000
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 3000 8200 3000
Connection ~ 8200 3000
$Comp
L power:PWR_FLAG #FLG0102
U 1 1 614862B7
P 6900 2400
F 0 "#FLG0102" H 6900 2475 50  0001 C CNN
F 1 "PWR_FLAG" H 6900 2573 50  0000 C CNN
F 2 "" H 6900 2400 50  0001 C CNN
F 3 "~" H 6900 2400 50  0001 C CNN
	1    6900 2400
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 61486B25
P 5500 4600
F 0 "#PWR0103" H 5500 4350 50  0001 C CNN
F 1 "GND" V 5505 4472 50  0000 R CNN
F 2 "" H 5500 4600 50  0001 C CNN
F 3 "" H 5500 4600 50  0001 C CNN
	1    5500 4600
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 61487A0C
P 5500 3600
F 0 "#PWR0104" H 5500 3350 50  0001 C CNN
F 1 "GND" V 5505 3472 50  0000 R CNN
F 2 "" H 5500 3600 50  0001 C CNN
F 3 "" H 5500 3600 50  0001 C CNN
	1    5500 3600
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 614887CB
P 5500 2200
F 0 "#PWR0105" H 5500 1950 50  0001 C CNN
F 1 "GND" V 5505 2072 50  0000 R CNN
F 2 "" H 5500 2200 50  0001 C CNN
F 3 "" H 5500 2200 50  0001 C CNN
	1    5500 2200
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0106
U 1 1 61488C1E
P 5500 5000
F 0 "#PWR0106" H 5500 4750 50  0001 C CNN
F 1 "GND" V 5505 4872 50  0000 R CNN
F 2 "" H 5500 5000 50  0001 C CNN
F 3 "" H 5500 5000 50  0001 C CNN
	1    5500 5000
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 61488E2E
P 4000 5600
F 0 "#PWR0107" H 4000 5350 50  0001 C CNN
F 1 "GND" V 4005 5472 50  0000 R CNN
F 2 "" H 4000 5600 50  0001 C CNN
F 3 "" H 4000 5600 50  0001 C CNN
	1    4000 5600
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 614894CE
P 4000 4200
F 0 "#PWR0108" H 4000 3950 50  0001 C CNN
F 1 "GND" V 4005 4072 50  0000 R CNN
F 2 "" H 4000 4200 50  0001 C CNN
F 3 "" H 4000 4200 50  0001 C CNN
	1    4000 4200
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 61489BE8
P 4000 2600
F 0 "#PWR0109" H 4000 2350 50  0001 C CNN
F 1 "GND" V 4005 2472 50  0000 R CNN
F 2 "" H 4000 2600 50  0001 C CNN
F 3 "" H 4000 2600 50  0001 C CNN
	1    4000 2600
	0    1    1    0   
$EndComp
$Comp
L dmcginnis427:FeatherM0LoRa U2
U 1 1 61D627CA
P 7150 1800
F 0 "U2" H 7550 2065 50  0000 C CNN
F 1 "FeatherM0LoRa" H 7550 1974 50  0000 C CNN
F 2 "dmcginnis427Footprints:FeatherM0LoRa" H 7150 1800 50  0001 C CNN
F 3 "" H 7150 1800 50  0001 C CNN
	1    7150 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	7150 4400 6450 4400
Wire Wire Line
	6450 4400 6450 2400
Wire Wire Line
	6450 2400 5500 2400
Wire Wire Line
	7150 4600 6300 4600
Wire Wire Line
	6300 4600 6300 2600
Wire Wire Line
	6300 2600 5500 2600
Connection ~ 6900 2400
NoConn ~ 7150 1800
NoConn ~ 7150 2000
NoConn ~ 7150 2200
NoConn ~ 7150 2600
NoConn ~ 7150 2800
NoConn ~ 7150 3000
NoConn ~ 7150 3200
NoConn ~ 7150 3400
NoConn ~ 7150 3600
NoConn ~ 7150 3800
NoConn ~ 7150 4000
NoConn ~ 7150 4200
NoConn ~ 7150 4800
NoConn ~ 7950 4800
NoConn ~ 7950 4600
NoConn ~ 7950 4400
NoConn ~ 7950 4200
NoConn ~ 7950 4000
NoConn ~ 7950 3800
NoConn ~ 7950 3600
NoConn ~ 7950 3400
NoConn ~ 7950 3200
NoConn ~ 7950 2800
NoConn ~ 7950 2600
$EndSCHEMATC
