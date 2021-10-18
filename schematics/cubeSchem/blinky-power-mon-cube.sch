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
L IRM-20-5:IRM-20-5 PS1
U 1 1 5E3C8266
P 5600 2150
F 0 "PS1" H 5600 2515 50  0000 C CNN
F 1 "IRM-20-5" H 5600 2424 50  0000 C CNN
F 2 "IRM-20-5:CONV_IRM-20-5" H 5600 2150 50  0001 L BNN
F 3 "" H 5600 2150 50  0001 L BNN
F 4 "Meanwell" H 5600 2150 50  0001 L BNN "Field4"
F 5 "24mm" H 5600 2150 50  0001 L BNN "Field5"
F 6 "IPC-7351B" H 5600 2150 50  0001 L BNN "Field6"
	1    5600 2150
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x03 J2
U 1 1 5E3D1E6B
P 4450 2100
F 0 "J2" H 4368 2417 50  0000 C CNN
F 1 "VACin" H 4368 2326 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_bornier-3_P5.08mm" H 4450 2100 50  0001 C CNN
F 3 "~" H 4450 2100 50  0001 C CNN
	1    4450 2100
	-1   0    0    -1  
$EndComp
$Comp
L power:GND #PWR02
U 1 1 5E3DC2CB
P 4650 1700
F 0 "#PWR02" H 4650 1450 50  0001 C CNN
F 1 "GND" H 4655 1527 50  0000 C CNN
F 2 "" H 4650 1700 50  0001 C CNN
F 3 "" H 4650 1700 50  0001 C CNN
	1    4650 1700
	-1   0    0    1   
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5E3DC6AB
P 6200 2350
F 0 "#PWR04" H 6200 2100 50  0001 C CNN
F 1 "GND" H 6205 2177 50  0000 C CNN
F 2 "" H 6200 2350 50  0001 C CNN
F 3 "" H 6200 2350 50  0001 C CNN
	1    6200 2350
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 2250 6200 2350
$Comp
L power:+5V #PWR03
U 1 1 5E3DCEA9
P 6200 1950
F 0 "#PWR03" H 6200 1800 50  0001 C CNN
F 1 "+5V" H 6215 2123 50  0000 C CNN
F 2 "" H 6200 1950 50  0001 C CNN
F 3 "" H 6200 1950 50  0001 C CNN
	1    6200 1950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6200 2050 6200 1950
$Comp
L power:GND #PWR08
U 1 1 5E3ED3E8
P 3450 3350
F 0 "#PWR08" H 3450 3100 50  0001 C CNN
F 1 "GND" V 3455 3222 50  0000 R CNN
F 2 "" H 3450 3350 50  0001 C CNN
F 3 "" H 3450 3350 50  0001 C CNN
	1    3450 3350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 5E44005B
P 3300 1700
F 0 "#PWR01" H 3300 1450 50  0001 C CNN
F 1 "GND" H 3305 1527 50  0000 C CNN
F 2 "" H 3300 1700 50  0001 C CNN
F 3 "" H 3300 1700 50  0001 C CNN
	1    3300 1700
	-1   0    0    1   
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J3
U 1 1 5E4984F6
P 2650 4050
F 0 "J3" H 2568 3825 50  0000 C CNN
F 1 "CTin" H 2568 3916 50  0000 C CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1.5mm" H 2650 4050 50  0001 C CNN
F 3 "~" H 2650 4050 50  0001 C CNN
	1    2650 4050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J4
U 1 1 5E498FE0
P 2650 3650
F 0 "J4" H 2568 3425 50  0000 C CNN
F 1 "CTout" H 2568 3516 50  0000 C CNN
F 2 "Connector_Wire:SolderWirePad_1x01_Drill1.5mm" H 2650 3650 50  0001 C CNN
F 3 "~" H 2650 3650 50  0001 C CNN
	1    2650 3650
	1    0    0    -1  
$EndComp
$Comp
L FEATHER_M0_BASIC_PROTO:FEATHER_M0_BASIC_PROTO U3
U 1 1 6147908E
P 5850 4250
F 0 "U3" H 5850 5417 50  0000 C CNN
F 1 "FEATHER_M0_BASIC_PROTO" H 5850 5326 50  0000 C CNN
F 2 "FEATHER_M0_BASIC_PROTO:MODULE_FEATHER_M0_BASIC_PROTO" H 5850 4250 50  0001 L BNN
F 3 "" H 5850 4250 50  0001 L BNN
F 4 "Adafruit" H 5850 4250 50  0001 L BNN "MANUFACTURER"
F 5 "Manufacturer Recommendation" H 5850 4250 50  0001 L BNN "STANDARD"
	1    5850 4250
	1    0    0    -1  
$EndComp
$Comp
L Connector:Screw_Terminal_01x03 J6
U 1 1 6147A9BF
P 8450 4050
F 0 "J6" H 8368 3725 50  0000 C CNN
F 1 "TripStatus" H 8700 4050 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_bornier-3_P5.08mm" H 8450 4050 50  0001 C CNN
F 3 "~" H 8450 4050 50  0001 C CNN
	1    8450 4050
	1    0    0    1   
$EndComp
$Comp
L dmcginnis427:ACS712_20 U2
U 1 1 5E3C936E
P 3250 3150
F 0 "U2" H 3478 3196 50  0000 L CNN
F 1 "ACS712_20" H 3478 3105 50  0000 L CNN
F 2 "dmcginnis427:ACS712_20A" H 3250 3450 50  0001 C CNN
F 3 "" V 3050 3150 50  0001 C CNN
	1    3250 3150
	-1   0    0    -1  
$EndComp
Wire Wire Line
	3450 3050 3450 2900
Wire Wire Line
	3450 3350 3450 3250
$Comp
L power:+5V #PWR011
U 1 1 614A8F4C
P 7400 3550
F 0 "#PWR011" H 7400 3400 50  0001 C CNN
F 1 "+5V" H 7415 3723 50  0000 C CNN
F 2 "" H 7400 3550 50  0001 C CNN
F 3 "" H 7400 3550 50  0001 C CNN
	1    7400 3550
	0    1    1    0   
$EndComp
Wire Wire Line
	7400 3550 7050 3550
Wire Wire Line
	8050 4050 8250 4050
$Comp
L power:GND #PWR014
U 1 1 614ADFB2
P 8250 4700
F 0 "#PWR014" H 8250 4450 50  0001 C CNN
F 1 "GND" V 8255 4572 50  0000 R CNN
F 2 "" H 8250 4700 50  0001 C CNN
F 3 "" H 8250 4700 50  0001 C CNN
	1    8250 4700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR013
U 1 1 614AE60E
P 8250 3300
F 0 "#PWR013" H 8250 3050 50  0001 C CNN
F 1 "GND" V 8255 3172 50  0000 R CNN
F 2 "" H 8250 3300 50  0001 C CNN
F 3 "" H 8250 3300 50  0001 C CNN
	1    8250 3300
	-1   0    0    1   
$EndComp
Wire Wire Line
	8250 4700 8250 4600
Wire Wire Line
	8250 4300 8250 4250
Wire Wire Line
	8250 3950 8250 3700
Wire Wire Line
	8250 3400 8250 3300
$Comp
L power:GND #PWR010
U 1 1 614B7B4C
P 7050 5150
F 0 "#PWR010" H 7050 4900 50  0001 C CNN
F 1 "GND" V 7055 5022 50  0000 R CNN
F 2 "" H 7050 5150 50  0001 C CNN
F 3 "" H 7050 5150 50  0001 C CNN
	1    7050 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 5150 7050 5050
$Comp
L Connector:Screw_Terminal_01x02 J5
U 1 1 614BA906
P 2250 4800
F 0 "J5" H 2168 5017 50  0000 C CNN
F 1 "4-20mA" H 2168 4926 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_bornier-2_P5.08mm" H 2250 4800 50  0001 C CNN
F 3 "~" H 2250 4800 50  0001 C CNN
	1    2250 4800
	-1   0    0    -1  
$EndComp
$Comp
L Device:R R3
U 1 1 614BDC4E
P 8250 3550
F 0 "R3" H 8320 3596 50  0000 L CNN
F 1 "10k" H 8320 3505 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8180 3550 50  0001 C CNN
F 3 "~" H 8250 3550 50  0001 C CNN
	1    8250 3550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR09
U 1 1 614BF5D2
P 2800 5100
F 0 "#PWR09" H 2800 4850 50  0001 C CNN
F 1 "GND" V 2805 4972 50  0000 R CNN
F 2 "" H 2800 5100 50  0001 C CNN
F 3 "" H 2800 5100 50  0001 C CNN
	1    2800 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 5000 2800 5100
$Comp
L Device:R R4
U 1 1 614C3ECC
P 8250 4450
F 0 "R4" H 8320 4496 50  0000 L CNN
F 1 "10k" H 8320 4405 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 8180 4450 50  0001 C CNN
F 3 "~" H 8250 4450 50  0001 C CNN
	1    8250 4450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 614C459B
P 2800 4850
F 0 "R1" H 2870 4896 50  0000 L CNN
F 1 "100" H 2870 4805 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 2730 4850 50  0001 C CNN
F 3 "~" H 2800 4850 50  0001 C CNN
	1    2800 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 4700 2450 4700
Wire Wire Line
	2450 4700 2450 4800
Wire Wire Line
	2450 4900 2450 5000
Wire Wire Line
	2450 5000 2800 5000
Connection ~ 2800 5000
$Comp
L Device:D_Zener D3.3V1
U 1 1 614C7F1D
P 3400 4850
F 0 "D3.3V1" V 3354 4930 50  0000 L CNN
F 1 "D_Zener" V 3445 4930 50  0000 L CNN
F 2 "Diode_THT:D_DO-35_SOD27_P7.62mm_Horizontal" H 3400 4850 50  0001 C CNN
F 3 "~" H 3400 4850 50  0001 C CNN
	1    3400 4850
	0    1    1    0   
$EndComp
$Comp
L Device:R R2
U 1 1 614CEADF
P 3150 4700
F 0 "R2" V 2943 4700 50  0000 C CNN
F 1 "1k" V 3034 4700 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3080 4700 50  0001 C CNN
F 3 "~" H 3150 4700 50  0001 C CNN
	1    3150 4700
	0    1    1    0   
$EndComp
Wire Wire Line
	3400 5000 2800 5000
Wire Wire Line
	3000 4700 2800 4700
Connection ~ 2800 4700
Wire Wire Line
	3300 4700 3400 4700
Wire Wire Line
	4650 3750 3900 3750
Wire Wire Line
	3900 3750 3900 4700
Wire Wire Line
	3900 4700 3400 4700
Connection ~ 3400 4700
Wire Wire Line
	7050 4150 7300 4150
Wire Wire Line
	7300 4150 7300 3950
Wire Wire Line
	7300 3950 8250 3950
Connection ~ 8250 3950
Wire Wire Line
	7050 4250 8250 4250
Connection ~ 8250 4250
Wire Wire Line
	8250 4250 8250 4150
NoConn ~ 7050 3450
NoConn ~ 7050 4050
NoConn ~ 7050 4350
NoConn ~ 7050 4450
NoConn ~ 7050 4550
NoConn ~ 7050 4650
NoConn ~ 7050 4750
NoConn ~ 7050 4850
NoConn ~ 7050 4950
NoConn ~ 4650 4850
NoConn ~ 4650 4750
NoConn ~ 4650 4650
NoConn ~ 4650 4550
NoConn ~ 4650 4450
NoConn ~ 4650 4350
NoConn ~ 4650 4250
NoConn ~ 4650 4150
NoConn ~ 4650 4050
NoConn ~ 4650 3950
NoConn ~ 4650 3550
NoConn ~ 4650 3450
$Comp
L Connector:Screw_Terminal_01x03 J1
U 1 1 615078CC
P 3500 2100
F 0 "J1" H 3418 2417 50  0000 C CNN
F 1 "VACout" H 3418 2326 50  0000 C CNN
F 2 "TerminalBlock:TerminalBlock_bornier-3_P5.08mm" H 3500 2100 50  0001 C CNN
F 3 "~" H 3500 2100 50  0001 C CNN
	1    3500 2100
	1    0    0    1   
$EndComp
Wire Wire Line
	4650 2100 4850 2100
Wire Wire Line
	5000 2100 5000 2050
Wire Wire Line
	4650 2200 4700 2200
Wire Wire Line
	5000 2200 5000 2250
Wire Wire Line
	4650 2000 4650 1700
Wire Wire Line
	3300 2000 3300 1700
Wire Wire Line
	3300 2200 3250 2200
Wire Wire Line
	3250 2200 3250 2500
Wire Wire Line
	3250 2500 4700 2500
Wire Wire Line
	4700 2500 4700 2200
Connection ~ 4700 2200
Wire Wire Line
	4700 2200 5000 2200
Wire Wire Line
	3300 2100 2450 2100
Wire Wire Line
	2450 2100 2450 3650
Wire Wire Line
	2450 4050 2350 4050
Wire Wire Line
	2350 4050 2350 1450
Wire Wire Line
	2350 1450 4850 1450
Wire Wire Line
	4850 1450 4850 2100
Connection ~ 4850 2100
Wire Wire Line
	4850 2100 5000 2100
$Comp
L power:+5V #PWR05
U 1 1 6147C57F
P 3450 2900
F 0 "#PWR05" H 3450 2750 50  0001 C CNN
F 1 "+5V" H 3465 3073 50  0000 C CNN
F 2 "" H 3450 2900 50  0001 C CNN
F 3 "" H 3450 2900 50  0001 C CNN
	1    3450 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R5
U 1 1 61480B7E
P 3750 3150
F 0 "R5" V 3543 3150 50  0000 C CNN
F 1 "1k" V 3634 3150 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3680 3150 50  0001 C CNN
F 3 "~" H 3750 3150 50  0001 C CNN
	1    3750 3150
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 614811A8
P 4000 3300
F 0 "R6" V 3793 3300 50  0000 C CNN
F 1 "2k" V 3884 3300 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal" V 3930 3300 50  0001 C CNN
F 3 "~" H 4000 3300 50  0001 C CNN
	1    4000 3300
	-1   0    0    1   
$EndComp
Wire Wire Line
	3450 3150 3600 3150
Wire Wire Line
	3900 3150 4000 3150
$Comp
L power:GND #PWR06
U 1 1 6148367F
P 4000 3450
F 0 "#PWR06" H 4000 3200 50  0001 C CNN
F 1 "GND" V 4005 3322 50  0000 R CNN
F 2 "" H 4000 3450 50  0001 C CNN
F 3 "" H 4000 3450 50  0001 C CNN
	1    4000 3450
	1    0    0    -1  
$EndComp
Wire Wire Line
	4000 3150 4350 3150
Wire Wire Line
	4350 3150 4350 3850
Wire Wire Line
	4350 3850 4650 3850
Connection ~ 4000 3150
Wire Wire Line
	8050 4050 8050 3350
Wire Wire Line
	8050 3350 7050 3350
$Comp
L Connector:TestPoint TP1
U 1 1 6149BD02
P 5300 5500
F 0 "TP1" H 5358 5618 50  0000 L CNN
F 1 "LoRa" H 5358 5527 50  0000 L CNN
F 2 "TestPoint:TestPoint_Plated_Hole_D2.0mm" H 5500 5500 50  0001 C CNN
F 3 "~" H 5500 5500 50  0001 C CNN
	1    5300 5500
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J7
U 1 1 6149D714
P 5800 5550
F 0 "J7" H 5828 5526 50  0000 L CNN
F 1 "SMA Ant" H 5828 5435 50  0000 L CNN
F 2 "dmcginnis427Footprints:SMA_pcb" H 5800 5550 50  0001 C CNN
F 3 "~" H 5800 5550 50  0001 C CNN
	1    5800 5550
	1    0    0    -1  
$EndComp
Wire Wire Line
	5300 5500 5300 5550
Wire Wire Line
	5300 5550 5600 5550
$Comp
L power:GND #PWR07
U 1 1 614A1BD1
P 5500 5650
F 0 "#PWR07" H 5500 5400 50  0001 C CNN
F 1 "GND" H 5505 5477 50  0000 C CNN
F 2 "" H 5500 5650 50  0001 C CNN
F 3 "" H 5500 5650 50  0001 C CNN
	1    5500 5650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5600 5650 5500 5650
$EndSCHEMATC