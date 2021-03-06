EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
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
L Connector:Screw_Terminal_01x04 J38
U 1 1 60A0AEEA
P 9600 1300
F 0 "J38" H 9680 1292 50  0000 L CNN
F 1 "ODO_Left" H 9680 1201 50  0000 L CNN
F 2 "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical" H 9600 1300 50  0001 C CNN
F 3 "~" H 9600 1300 50  0001 C CNN
	1    9600 1300
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR086
U 1 1 60A0B19C
P 9400 1500
F 0 "#PWR086" H 9400 1350 50  0001 C CNN
F 1 "+3.3V" V 9415 1628 50  0000 L CNN
F 2 "" H 9400 1500 50  0001 C CNN
F 3 "" H 9400 1500 50  0001 C CNN
	1    9400 1500
	0    -1   -1   0   
$EndComp
Text GLabel 8750 1000 0    50   Input ~ 0
pinOdometryRight
$Comp
L Connector:Screw_Terminal_01x04 J39
U 1 1 60A0D0C7
P 9600 2050
F 0 "J39" H 9680 2042 50  0000 L CNN
F 1 "ODO_Right" H 9680 1951 50  0000 L CNN
F 2 "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical" H 9600 2050 50  0001 C CNN
F 3 "~" H 9600 2050 50  0001 C CNN
	1    9600 2050
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR088
U 1 1 60A0D3D3
P 9400 2250
F 0 "#PWR088" H 9400 2100 50  0001 C CNN
F 1 "+3.3V" V 9415 2378 50  0000 L CNN
F 2 "" H 9400 2250 50  0001 C CNN
F 3 "" H 9400 2250 50  0001 C CNN
	1    9400 2250
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR087
U 1 1 60A0D3DD
P 9400 2150
F 0 "#PWR087" H 9400 2000 50  0001 C CNN
F 1 "+5V" V 9415 2278 50  0000 L CNN
F 2 "" H 9400 2150 50  0001 C CNN
F 3 "" H 9400 2150 50  0001 C CNN
	1    9400 2150
	0    -1   -1   0   
$EndComp
Text GLabel 8750 1750 0    50   Input ~ 0
pinOdometryLeft
$Comp
L power:+5V #PWR085
U 1 1 60A0B1A6
P 9400 1400
F 0 "#PWR085" H 9400 1250 50  0001 C CNN
F 1 "+5V" V 9415 1528 50  0000 L CNN
F 2 "" H 9400 1400 50  0001 C CNN
F 3 "" H 9400 1400 50  0001 C CNN
	1    9400 1400
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR082
U 1 1 60A40444
P 8950 2050
F 0 "#PWR082" H 8950 1800 50  0001 C CNN
F 1 "GND" V 8955 1922 50  0000 R CNN
F 2 "" H 8950 2050 50  0001 C CNN
F 3 "" H 8950 2050 50  0001 C CNN
	1    8950 2050
	0    1    1    0   
$EndComp
$Comp
L Device:R R17
U 1 1 60A4079E
P 9250 1900
F 0 "R17" H 9320 1946 50  0000 L CNN
F 1 "1,5KO" V 9250 1800 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 9180 1900 50  0001 C CNN
F 3 "~" H 9250 1900 50  0001 C CNN
	1    9250 1900
	1    0    0    -1  
$EndComp
$Comp
L Device:R R14
U 1 1 60A407A8
P 9000 1750
F 0 "R14" V 8900 1750 50  0000 C CNN
F 1 "1KO" V 9000 1750 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 8930 1750 50  0001 C CNN
F 3 "~" H 9000 1750 50  0001 C CNN
	1    9000 1750
	0    1    1    0   
$EndComp
Wire Wire Line
	8750 1750 8850 1750
Wire Wire Line
	9150 1750 9250 1750
Wire Wire Line
	9250 1750 9400 1750
Connection ~ 9250 1750
Wire Wire Line
	9400 2050 9250 2050
Wire Wire Line
	9250 2050 8950 2050
Connection ~ 9250 2050
$Comp
L power:GND #PWR067
U 1 1 60A447B6
P 8950 1300
F 0 "#PWR067" H 8950 1050 50  0001 C CNN
F 1 "GND" V 8955 1172 50  0000 R CNN
F 2 "" H 8950 1300 50  0001 C CNN
F 3 "" H 8950 1300 50  0001 C CNN
	1    8950 1300
	0    1    1    0   
$EndComp
$Comp
L Device:R R16
U 1 1 60A447C8
P 9250 1150
F 0 "R16" H 9320 1196 50  0000 L CNN
F 1 "1,5KO" V 9250 1050 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 9180 1150 50  0001 C CNN
F 3 "~" H 9250 1150 50  0001 C CNN
	1    9250 1150
	1    0    0    -1  
$EndComp
$Comp
L Device:R R6
U 1 1 60A447D2
P 9000 1000
F 0 "R6" V 8900 1000 50  0000 C CNN
F 1 "1KO" V 9000 1000 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 8930 1000 50  0001 C CNN
F 3 "~" H 9000 1000 50  0001 C CNN
	1    9000 1000
	0    1    1    0   
$EndComp
Wire Wire Line
	8750 1000 8850 1000
Wire Wire Line
	9150 1000 9250 1000
Wire Wire Line
	9250 1000 9400 1000
Connection ~ 9250 1000
Wire Wire Line
	9400 1300 9250 1300
Wire Wire Line
	9250 1300 8950 1300
Connection ~ 9250 1300
Wire Wire Line
	9400 1000 9400 1200
Wire Wire Line
	9400 1750 9400 1950
$Comp
L Connector:Screw_Terminal_01x05 J40
U 1 1 60A17AF1
P 2300 1450
F 0 "J40" H 2380 1492 50  0000 L CNN
F 1 "Motor_Right1" H 2380 1401 50  0000 L CNN
F 2 "Connector_JST:JST_XH_B5B-XH-A_1x05_P2.50mm_Vertical" H 2300 1450 50  0001 C CNN
F 3 "~" H 2300 1450 50  0001 C CNN
	1    2300 1450
	1    0    0    -1  
$EndComp
$Comp
L Device:Jumper_NC_Dual JP4
U 1 1 60A196AB
P 1150 1000
F 0 "JP4" H 1150 1239 50  0000 C CNN
F 1 "MotorPower" H 1150 1148 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 1150 1000 50  0001 C CNN
F 3 "~" H 1150 1000 50  0001 C CNN
	1    1150 1000
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR092
U 1 1 60A1A220
P 1400 1000
F 0 "#PWR092" H 1400 850 50  0001 C CNN
F 1 "+3.3V" V 1415 1128 50  0000 L CNN
F 2 "" H 1400 1000 50  0001 C CNN
F 3 "" H 1400 1000 50  0001 C CNN
	1    1400 1000
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR089
U 1 1 60A1A61A
P 900 1000
F 0 "#PWR089" H 900 850 50  0001 C CNN
F 1 "+5V" V 915 1128 50  0000 L CNN
F 2 "" H 900 1000 50  0001 C CNN
F 3 "" H 900 1000 50  0001 C CNN
	1    900  1000
	0    -1   -1   0   
$EndComp
Text GLabel 2100 1450 0    50   Input ~ 0
pinMotorRightPWM
Text GLabel 2100 1550 0    50   Input ~ 0
pinMotorRightDir
Text GLabel 2100 1650 0    50   Input ~ 0
pinMotorRightEnable
$Comp
L Connector:Screw_Terminal_01x05 J41
U 1 1 60A2B4C3
P 2300 2850
F 0 "J41" H 2380 2892 50  0000 L CNN
F 1 "Motor_Left1" H 2380 2801 50  0000 L CNN
F 2 "Connector_JST:JST_XH_B5B-XH-A_1x05_P2.50mm_Vertical" H 2300 2850 50  0001 C CNN
F 3 "~" H 2300 2850 50  0001 C CNN
	1    2300 2850
	1    0    0    -1  
$EndComp
Text GLabel 2100 2850 0    50   Input ~ 0
pinMotorLeftPWM
Text GLabel 2100 2950 0    50   Input ~ 0
pinMotorLeftDir
Text GLabel 2100 3050 0    50   Input ~ 0
pinMotorLeftEnable
$Comp
L Connector:Screw_Terminal_01x05 J42
U 1 1 60A3BF2F
P 2350 5350
F 0 "J42" H 2430 5392 50  0000 L CNN
F 1 "Cutter" H 2430 5301 50  0000 L CNN
F 2 "Connector_JST:JST_XH_B5B-XH-A_1x05_P2.50mm_Vertical" H 2350 5350 50  0001 C CNN
F 3 "~" H 2350 5350 50  0001 C CNN
	1    2350 5350
	1    0    0    -1  
$EndComp
Text GLabel 2150 5350 0    50   Input ~ 0
pinMotorMowPWM
Text GLabel 2150 5450 0    50   Input ~ 0
pinMotorMowDir
Text GLabel 2150 5550 0    50   Input ~ 0
pinMotorMowEnable
$Comp
L Device:Jumper_NC_Dual JP5
U 1 1 60A40CF2
P 1200 4800
F 0 "JP5" H 1200 5039 50  0000 C CNN
F 1 "CutterPower" H 1200 4948 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x03_P2.54mm_Vertical" H 1200 4800 50  0001 C CNN
F 3 "~" H 1200 4800 50  0001 C CNN
	1    1200 4800
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR093
U 1 1 60A4118A
P 1450 4800
F 0 "#PWR093" H 1450 4650 50  0001 C CNN
F 1 "+3.3V" V 1465 4928 50  0000 L CNN
F 2 "" H 1450 4800 50  0001 C CNN
F 3 "" H 1450 4800 50  0001 C CNN
	1    1450 4800
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR090
U 1 1 60A41194
P 950 4800
F 0 "#PWR090" H 950 4650 50  0001 C CNN
F 1 "+5V" V 965 4928 50  0000 L CNN
F 2 "" H 950 4800 50  0001 C CNN
F 3 "" H 950 4800 50  0001 C CNN
	1    950  4800
	0    -1   -1   0   
$EndComp
Wire Wire Line
	1200 4900 1200 5150
$Comp
L power:GND #PWR094
U 1 1 60A4A597
P 2150 5250
F 0 "#PWR094" H 2150 5000 50  0001 C CNN
F 1 "GND" V 2155 5122 50  0000 R CNN
F 2 "" H 2150 5250 50  0001 C CNN
F 3 "" H 2150 5250 50  0001 C CNN
	1    2150 5250
	0    1    1    0   
$EndComp
Wire Wire Line
	1200 5150 2150 5150
Wire Wire Line
	2100 1250 1150 1250
Wire Wire Line
	2100 2650 1150 2650
Wire Wire Line
	2100 1350 1000 1350
Wire Wire Line
	1000 2750 2100 2750
$Comp
L power:GND #PWR091
U 1 1 60A27474
P 800 2750
F 0 "#PWR091" H 800 2500 50  0001 C CNN
F 1 "GND" V 805 2622 50  0000 R CNN
F 2 "" H 800 2750 50  0001 C CNN
F 3 "" H 800 2750 50  0001 C CNN
	1    800  2750
	0    1    1    0   
$EndComp
Connection ~ 1000 2750
Wire Wire Line
	1150 1100 1150 1250
Connection ~ 1150 1250
Wire Wire Line
	800  2750 1000 2750
Wire Wire Line
	1000 1350 1000 2750
Wire Wire Line
	1150 1250 1150 2650
$EndSCHEMATC
