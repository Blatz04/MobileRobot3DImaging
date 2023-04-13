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
L Raspy:RASPBERRY_PI_3_MODEL_B+ RPi3B+1
U 1 1 637C14DC
P 5800 3600
F 0 "RPi3B+1" H 5750 4767 50  0000 C CNN
F 1 "RASPBERRY_PI_3_MODEL_B+" H 5750 4676 50  0000 C CNN
F 2 "MODULE_RASPBERRY_PI_3_MODEL_B+" H 5800 3600 50  0001 L BNN
F 3 "" H 5800 3600 50  0001 L BNN
F 4 "Raspberry Pi" H 5800 3600 50  0001 L BNN "MANUFACTURER"
F 5 "1.0" H 5800 3600 50  0001 L BNN "PARTREV"
F 6 "Manufacturer Recommendations" H 5800 3600 50  0001 L BNN "STANDARD"
F 7 "18mm" H 5800 3600 50  0001 L BNN "MAXIMUM_PACKAGE_HIEGHT"
	1    5800 3600
	1    0    0    -1  
$EndComp
$Comp
L DRV8825:DRV8825_STEPPER_MOTOR_DRIVER_CARRIER DRV8825
U 1 1 637C5347
P 8900 3800
F 0 "DRV8825" H 8900 4665 50  0000 C CNN
F 1 "DRV8825_STEPPER_MOTOR_DRIVER_CARRIER" H 8900 4574 50  0000 C CNN
F 2 "IC_DRV8825_STEPPER_MOTOR_DRIVER_CARRIER" H 8900 3800 50  0001 L BNN
F 3 "" H 8900 3800 50  0001 L BNN
F 4 "None" H 8900 3800 50  0001 L BNN "PRICE"
F 5 "None" H 8900 3800 50  0001 L BNN "PACKAGE"
F 6 "DRV8825 STEPPER MOTOR DRIVER CARRIER" H 8900 3800 50  0001 L BNN "MP"
F 7 "Unavailable" H 8900 3800 50  0001 L BNN "AVAILABILITY"
F 8 "Pololu" H 8900 3800 50  0001 L BNN "MF"
F 9 "Stepper motor controler; IC: DRV8825; 1.5A; Uin mot: 8.2÷45V" H 8900 3800 50  0001 L BNN "DESCRIPTION"
	1    8900 3800
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male SW2
U 1 1 637C493D
P 7850 2050
F 0 "SW2" H 7958 2231 50  0000 C CNN
F 1 "LimitSwitch" H 7958 2140 50  0000 C CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-02A_1x02_P2.54mm_Vertical" H 7850 2050 50  0001 C CNN
F 3 "~" H 7850 2050 50  0001 C CNN
	1    7850 2050
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male MSt1
U 1 1 637C624E
P 10300 3900
F 0 "MSt1" H 10272 3782 50  0000 R CNN
F 1 "MotorStepper" H 10272 3873 50  0000 R CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-04A_1x04_P2.54mm_Vertical" H 10300 3900 50  0001 C CNN
F 3 "~" H 10300 3900 50  0001 C CNN
	1    10300 3900
	-1   0    0    1   
$EndComp
$Comp
L XL4005:XL4005_DC-DC PS1
U 1 1 637CB003
P 3200 4200
F 0 "PS1" H 3200 4565 50  0000 C CNN
F 1 "XL4005_DC-DC" H 3200 4474 50  0000 C CNN
F 2 "XL4005:XL4005_DC-DC" H 3200 4200 50  0001 C CNN
F 3 "" H 3200 4200 50  0001 C CNN
	1    3200 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 4000 9650 4000
Wire Wire Line
	9650 4000 9650 3900
Wire Wire Line
	9650 3900 10100 3900
Wire Wire Line
	10100 4000 9700 4000
Wire Wire Line
	9700 4100 9600 4100
Text GLabel 8050 2150 2    50   Input ~ 0
GND
Text GLabel 6800 4900 2    50   Input ~ 0
GND
Text GLabel 3600 4300 2    50   Input ~ 0
GND
Text GLabel 8200 3800 0    50   Input ~ 0
STEP
Text GLabel 8200 3700 0    50   Input ~ 0
DIR
Text GLabel 8050 2050 2    50   Input ~ 0
limitSwitch
Text GLabel 4700 3600 0    50   Input ~ 0
STEP
Text GLabel 4700 3700 0    50   Input ~ 0
DIR
Text GLabel 4700 3800 0    50   Input ~ 0
limitSwitch
Text GLabel 3600 4100 2    50   Input ~ 0
5V
$Comp
L Connector:Conn_01x02_Female Emg1
U 1 1 637D3423
P 2200 4500
F 0 "Emg1" V 2046 4548 50  0000 L CNN
F 1 "Emergency" V 2137 4548 50  0000 L CNN
F 2 "Connector_Wire:SolderWire-1.5sqmm_1x02_P6mm_D1.7mm_OD3mm" H 2200 4500 50  0001 C CNN
F 3 "~" H 2200 4500 50  0001 C CNN
	1    2200 4500
	0    1    1    0   
$EndComp
$Comp
L Connector:Conn_01x02_Female Bt12V1
U 1 1 637D4938
P 1700 4200
F 0 "Bt12V1" H 1592 3875 50  0000 C CNN
F 1 "Bt12VInput" H 1592 3966 50  0000 C CNN
F 2 "Amass:AMASS_XT30U-F_1x02_P5.0mm_Vertical" H 1700 4200 50  0001 C CNN
F 3 "~" H 1700 4200 50  0001 C CNN
	1    1700 4200
	-1   0    0    1   
$EndComp
Text GLabel 2650 3850 1    50   Input ~ 0
12V
Text GLabel 9750 3200 2    50   Input ~ 0
12V
Text GLabel 6800 2700 2    50   Input ~ 0
5V
Text GLabel 8200 3500 0    50   Input ~ 0
5V
Text GLabel 8200 3600 0    50   Input ~ 0
5V
Text GLabel 9750 4400 2    50   Input ~ 0
GND
$Comp
L Device:CP C1
U 1 1 637DA241
P 9700 3500
F 0 "C1" H 9818 3546 50  0000 L CNN
F 1 "100uF" H 9818 3455 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D5.0mm_P2.50mm" H 9738 3350 50  0001 C CNN
F 3 "~" H 9700 3500 50  0001 C CNN
	1    9700 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	9600 3200 9700 3200
Wire Wire Line
	9700 3350 9700 3200
Connection ~ 9700 3200
Wire Wire Line
	9700 3200 9750 3200
Wire Wire Line
	9700 4000 9700 4100
Wire Wire Line
	9700 3650 9700 3850
Wire Wire Line
	9700 3850 9800 3850
Wire Wire Line
	9800 3850 9800 4250
Wire Wire Line
	9800 4250 9650 4250
Wire Wire Line
	9650 4250 9650 4400
Wire Wire Line
	9650 4400 9600 4400
Wire Wire Line
	9750 4400 9650 4400
Connection ~ 9650 4400
$Comp
L Connector:Conn_01x02_Female SW1
U 1 1 637DCEF0
P 2100 3900
F 0 "SW1" V 2038 3712 50  0000 R CNN
F 1 "Switch1" V 1947 3712 50  0000 R CNN
F 2 "Connector_Wire:SolderWire-1.5sqmm_1x02_P6mm_D1.7mm_OD3mm" H 2100 3900 50  0001 C CNN
F 3 "~" H 2100 3900 50  0001 C CNN
	1    2100 3900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2100 4100 1950 4100
Wire Wire Line
	1900 4200 1950 4200
Wire Wire Line
	2100 4200 2100 4300
Wire Wire Line
	2200 4300 2800 4300
Wire Wire Line
	2800 4100 2650 4100
Wire Wire Line
	2650 3850 2650 4100
Connection ~ 2650 4100
Wire Wire Line
	2650 4100 2200 4100
$Comp
L Connector:Conn_01x02_Female chrg12
U 1 1 637E693C
P 1200 4200
F 0 "chrg12" H 1092 3875 50  0000 C CNN
F 1 "12VtoCharger" H 1092 3966 50  0000 C CNN
F 2 "Amass:AMASS_XT30U-F_1x02_P5.0mm_Vertical" H 1200 4200 50  0001 C CNN
F 3 "~" H 1200 4200 50  0001 C CNN
	1    1200 4200
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x04_Male Chrg3S1
U 1 1 637E7AA0
P 1650 2750
F 0 "Chrg3S1" H 1758 3031 50  0000 C CNN
F 1 "toCharger3S" H 1758 2940 50  0000 C CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-04A_1x04_P2.54mm_Vertical" H 1650 2750 50  0001 C CNN
F 3 "~" H 1650 2750 50  0001 C CNN
	1    1650 2750
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male Bt3S1
U 1 1 637E9434
P 2500 2850
F 0 "Bt3S1" H 2472 2732 50  0000 R CNN
F 1 "toBattery3S" H 2472 2823 50  0000 R CNN
F 2 "Connector_Molex:Molex_KK-254_AE-6410-04A_1x04_P2.54mm_Vertical" H 2500 2850 50  0001 C CNN
F 3 "~" H 2500 2850 50  0001 C CNN
	1    2500 2850
	-1   0    0    1   
$EndComp
Wire Wire Line
	1400 4100 1400 3850
Wire Wire Line
	1400 3850 1950 3850
Wire Wire Line
	1950 3850 1950 4100
Connection ~ 1950 4100
Wire Wire Line
	1950 4100 1900 4100
Wire Wire Line
	1400 4200 1400 4450
Wire Wire Line
	1400 4450 1950 4450
Wire Wire Line
	1950 4450 1950 4200
Connection ~ 1950 4200
Wire Wire Line
	1950 4200 2100 4200
Text Label 5450 5450 0    79   ~ 16
Master
Text Label 8750 4900 0    79   ~ 16
StepperUnit
Text Label 7800 2400 0    79   ~ 16
LimitSwitch
Text Label 2000 5150 0    79   ~ 16
PowerUnit
Wire Wire Line
	1850 2650 2300 2650
Wire Wire Line
	1850 2750 2300 2750
Wire Wire Line
	1850 2850 2300 2850
Wire Wire Line
	1850 2950 2300 2950
Wire Wire Line
	9850 3700 9850 3800
Wire Wire Line
	9850 3800 10100 3800
Wire Wire Line
	9600 3700 9850 3700
Wire Wire Line
	9600 3800 9800 3800
Wire Wire Line
	9800 3800 9800 3750
Wire Wire Line
	9800 3750 10000 3750
Wire Wire Line
	10000 3750 10000 3700
Wire Wire Line
	10000 3700 10100 3700
$EndSCHEMATC
