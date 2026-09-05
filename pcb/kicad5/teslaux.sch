EESchema Schematic File Version 4
LIBS:teslaux-cache
EELAYER 30 0
EELAYER END
$Descr A2 22047 15591
encoding utf-8
Sheet 1 1
Title "TeslAux single-board bridge"
Date ""
Rev "A"
Comp ""
Comment1 "OTG_FS (PA11/PA12) to the car, OTG_HS (PB14/PB15) to the phone"
Comment2 "Connectivity is by global label - see pcb/netlist.csv"
Comment3 "Layout is NOT generated - see HARDWARE-PCB.md"
Comment4 ""
$EndDescr
$Comp
L teslaux:STM32F407VETx U1
U 1 1 00000001
P 6000 5000
F 0 "U1" H 6000 4950 50  0000 C CNN
F 1 "STM32F407VET6" H 6000 5050 50  0000 C CNN
F 2 "Package_QFP:LQFP-100_14x14mm_P0.5mm" H 6000 5000 50  0001 C CNN
F 3 "" H 6000 5000 50  0001 C CNN
	1    6000 5000
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin R1
U 1 1 00000002
P 1200 1200
F 0 "R1" H 1200 1150 50  0000 C CNN
F 1 "22" H 1200 1250 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 1200 1200 50  0001 C CNN
F 3 "" H 1200 1200 50  0001 C CNN
	1    1200 1200
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin R2
U 1 1 00000003
P 1900 1200
F 0 "R2" H 1900 1150 50  0000 C CNN
F 1 "22" H 1900 1250 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 1900 1200 50  0001 C CNN
F 3 "" H 1900 1200 50  0001 C CNN
	1    1900 1200
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin R3
U 1 1 00000004
P 2600 1200
F 0 "R3" H 2600 1150 50  0000 C CNN
F 1 "22" H 2600 1250 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 2600 1200 50  0001 C CNN
F 3 "" H 2600 1200 50  0001 C CNN
	1    2600 1200
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin R4
U 1 1 00000005
P 3300 1200
F 0 "R4" H 3300 1150 50  0000 C CNN
F 1 "22" H 3300 1250 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 3300 1200 50  0001 C CNN
F 3 "" H 3300 1200 50  0001 C CNN
	1    3300 1200
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin R5
U 1 1 00000006
P 4000 1200
F 0 "R5" H 4000 1150 50  0000 C CNN
F 1 "5.1k" H 4000 1250 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 4000 1200 50  0001 C CNN
F 3 "" H 4000 1200 50  0001 C CNN
	1    4000 1200
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin R6
U 1 1 00000007
P 4700 1200
F 0 "R6" H 4700 1150 50  0000 C CNN
F 1 "5.1k" H 4700 1250 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 4700 1200 50  0001 C CNN
F 3 "" H 4700 1200 50  0001 C CNN
	1    4700 1200
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin R7
U 1 1 00000008
P 5400 1200
F 0 "R7" H 5400 1150 50  0000 C CNN
F 1 "5.1k" H 5400 1250 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 5400 1200 50  0001 C CNN
F 3 "" H 5400 1200 50  0001 C CNN
	1    5400 1200
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin R8
U 1 1 00000009
P 6100 1200
F 0 "R8" H 6100 1150 50  0000 C CNN
F 1 "5.1k" H 6100 1250 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 6100 1200 50  0001 C CNN
F 3 "" H 6100 1200 50  0001 C CNN
	1    6100 1200
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin R9
U 1 1 0000000A
P 6800 1200
F 0 "R9" H 6800 1150 50  0000 C CNN
F 1 "10k" H 6800 1250 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 6800 1200 50  0001 C CNN
F 3 "" H 6800 1200 50  0001 C CNN
	1    6800 1200
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin R10
U 1 1 0000000B
P 7500 1200
F 0 "R10" H 7500 1150 50  0000 C CNN
F 1 "1k" H 7500 1250 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 7500 1200 50  0001 C CNN
F 3 "" H 7500 1200 50  0001 C CNN
	1    7500 1200
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin C1
U 1 1 0000000C
P 8200 1200
F 0 "C1" H 8200 1150 50  0000 C CNN
F 1 "2.2u" H 8200 1250 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8200 1200 50  0001 C CNN
F 3 "" H 8200 1200 50  0001 C CNN
	1    8200 1200
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin C2
U 1 1 0000000D
P 8900 1200
F 0 "C2" H 8900 1150 50  0000 C CNN
F 1 "2.2u" H 8900 1250 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 8900 1200 50  0001 C CNN
F 3 "" H 8900 1200 50  0001 C CNN
	1    8900 1200
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin C3
U 1 1 0000000E
P 9600 1200
F 0 "C3" H 9600 1150 50  0000 C CNN
F 1 "100n" H 9600 1250 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 9600 1200 50  0001 C CNN
F 3 "" H 9600 1200 50  0001 C CNN
	1    9600 1200
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin C4
U 1 1 0000000F
P 10300 1200
F 0 "C4" H 10300 1150 50  0000 C CNN
F 1 "100n" H 10300 1250 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 10300 1200 50  0001 C CNN
F 3 "" H 10300 1200 50  0001 C CNN
	1    10300 1200
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin C5
U 1 1 00000010
P 11000 1200
F 0 "C5" H 11000 1150 50  0000 C CNN
F 1 "100n" H 11000 1250 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 11000 1200 50  0001 C CNN
F 3 "" H 11000 1200 50  0001 C CNN
	1    11000 1200
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin C6
U 1 1 00000011
P 1200 2100
F 0 "C6" H 1200 2050 50  0000 C CNN
F 1 "100n" H 1200 2150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1200 2100 50  0001 C CNN
F 3 "" H 1200 2100 50  0001 C CNN
	1    1200 2100
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin C7
U 1 1 00000012
P 1900 2100
F 0 "C7" H 1900 2050 50  0000 C CNN
F 1 "100n" H 1900 2150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 1900 2100 50  0001 C CNN
F 3 "" H 1900 2100 50  0001 C CNN
	1    1900 2100
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin C8
U 1 1 00000013
P 2600 2100
F 0 "C8" H 2600 2050 50  0000 C CNN
F 1 "100n" H 2600 2150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 2600 2100 50  0001 C CNN
F 3 "" H 2600 2100 50  0001 C CNN
	1    2600 2100
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin C13
U 1 1 00000014
P 3300 2100
F 0 "C13" H 3300 2050 50  0000 C CNN
F 1 "10u" H 3300 2150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 3300 2100 50  0001 C CNN
F 3 "" H 3300 2100 50  0001 C CNN
	1    3300 2100
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin C14
U 1 1 00000015
P 4000 2100
F 0 "C14" H 4000 2050 50  0000 C CNN
F 1 "10u" H 4000 2150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4000 2100 50  0001 C CNN
F 3 "" H 4000 2100 50  0001 C CNN
	1    4000 2100
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin C15
U 1 1 00000016
P 4700 2100
F 0 "C15" H 4700 2050 50  0000 C CNN
F 1 "1u" H 4700 2150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4700 2100 50  0001 C CNN
F 3 "" H 4700 2100 50  0001 C CNN
	1    4700 2100
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin C16
U 1 1 00000017
P 5400 2100
F 0 "C16" H 5400 2050 50  0000 C CNN
F 1 "100n" H 5400 2150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5400 2100 50  0001 C CNN
F 3 "" H 5400 2100 50  0001 C CNN
	1    5400 2100
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin C17
U 1 1 00000018
P 6100 2100
F 0 "C17" H 6100 2050 50  0000 C CNN
F 1 "100n" H 6100 2150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6100 2100 50  0001 C CNN
F 3 "" H 6100 2100 50  0001 C CNN
	1    6100 2100
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin C18
U 1 1 00000019
P 6800 2100
F 0 "C18" H 6800 2050 50  0000 C CNN
F 1 "18p" H 6800 2150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6800 2100 50  0001 C CNN
F 3 "" H 6800 2100 50  0001 C CNN
	1    6800 2100
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin C19
U 1 1 0000001A
P 7500 2100
F 0 "C19" H 7500 2050 50  0000 C CNN
F 1 "18p" H 7500 2150 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 7500 2100 50  0001 C CNN
F 3 "" H 7500 2100 50  0001 C CNN
	1    7500 2100
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin FB1
U 1 1 0000001B
P 8200 2100
F 0 "FB1" H 8200 2050 50  0000 C CNN
F 1 "600R" H 8200 2150 50  0000 C CNN
F 2 "Inductor_SMD:L_0603_1608Metric" H 8200 2100 50  0001 C CNN
F 3 "" H 8200 2100 50  0001 C CNN
	1    8200 2100
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin Y1
U 1 1 0000001C
P 8900 2100
F 0 "Y1" H 8900 2050 50  0000 C CNN
F 1 "8MHz" H 8900 2150 50  0000 C CNN
F 2 "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm" H 8900 2100 50  0001 C CNN
F 3 "" H 8900 2100 50  0001 C CNN
	1    8900 2100
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin D1
U 1 1 0000001D
P 9600 2100
F 0 "D1" H 9600 2050 50  0000 C CNN
F 1 "LED" H 9600 2150 50  0000 C CNN
F 2 "LED_SMD:LED_0603_1608Metric" H 9600 2100 50  0001 C CNN
F 3 "" H 9600 2100 50  0001 C CNN
	1    9600 2100
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Device_2pin SW1
U 1 1 0000001E
P 10300 2100
F 0 "SW1" H 10300 2050 50  0000 C CNN
F 1 "RESET" H 10300 2150 50  0000 C CNN
F 2 "Button_Switch_SMD:SW_SPST_TL3342" H 10300 2100 50  0001 C CNN
F 3 "" H 10300 2100 50  0001 C CNN
	1    10300 2100
	1    0    0    -1  
$EndComp
$Comp
L teslaux:USB_C_Receptacle J1
U 1 1 0000001F
P 2500 3300
F 0 "J1" H 2500 3250 50  0000 C CNN
F 1 "USB-C CAR" H 2500 3350 50  0000 C CNN
F 2 "" H 2500 3300 50  0001 C CNN
F 3 "" H 2500 3300 50  0001 C CNN
	1    2500 3300
	1    0    0    -1  
$EndComp
$Comp
L teslaux:USB_C_Receptacle J2
U 1 1 00000020
P 5000 3300
F 0 "J2" H 5000 3250 50  0000 C CNN
F 1 "USB-C PHONE" H 5000 3350 50  0000 C CNN
F 2 "" H 5000 3300 50  0001 C CNN
F 3 "" H 5000 3300 50  0001 C CNN
	1    5000 3300
	1    0    0    -1  
$EndComp
$Comp
L teslaux:AMS1117-3.3 U2
U 1 1 00000021
P 7500 3300
F 0 "U2" H 7500 3250 50  0000 C CNN
F 1 "AMS1117-3.3" H 7500 3350 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 7500 3300 50  0001 C CNN
F 3 "" H 7500 3300 50  0001 C CNN
	1    7500 3300
	1    0    0    -1  
$EndComp
$Comp
L teslaux:USBLC6-2SC6 U3
U 1 1 00000022
P 9500 3300
F 0 "U3" H 9500 3250 50  0000 C CNN
F 1 "USBLC6-2SC6" H 9500 3350 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 9500 3300 50  0001 C CNN
F 3 "" H 9500 3300 50  0001 C CNN
	1    9500 3300
	1    0    0    -1  
$EndComp
$Comp
L teslaux:USBLC6-2SC6 U4
U 1 1 00000023
P 2500 4700
F 0 "U4" H 2500 4650 50  0000 C CNN
F 1 "USBLC6-2SC6" H 2500 4750 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 2500 4700 50  0001 C CNN
F 3 "" H 2500 4700 50  0001 C CNN
	1    2500 4700
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Conn_01x05 J3
U 1 1 00000024
P 5000 4700
F 0 "J3" H 5000 4650 50  0000 C CNN
F 1 "SWD" H 5000 4750 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical" H 5000 4700 50  0001 C CNN
F 3 "" H 5000 4700 50  0001 C CNN
	1    5000 4700
	1    0    0    -1  
$EndComp
$Comp
L teslaux:Conn_01x02 J4
U 1 1 00000025
P 7500 4700
F 0 "J4" H 7500 4650 50  0000 C CNN
F 1 "BOOT0" H 7500 4750 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical" H 7500 4700 50  0001 C CNN
F 3 "" H 7500 4700 50  0001 C CNN
	1    7500 4700
	1    0    0    -1  
$EndComp
Text GLabel 5000 3000 2    50   BiDi ~ 0
+3V3
Text GLabel 5000 3400 2    50   BiDi ~ 0
GND
Text GLabel 5000 3500 2    50   BiDi ~ 0
+3V3
Text GLabel 5000 3600 2    50   BiDi ~ 0
OSC_IN
Text GLabel 5000 3700 2    50   BiDi ~ 0
OSC_OUT
Text GLabel 5000 3800 2    50   BiDi ~ 0
NRST
Text GLabel 5000 4300 2    50   BiDi ~ 0
+3V3
Text GLabel 5000 4400 2    50   BiDi ~ 0
GND
Text GLabel 5000 4500 2    50   BiDi ~ 0
VDDA
Text GLabel 5000 4600 2    50   BiDi ~ 0
VDDA
Text GLabel 5000 4800 2    50   BiDi ~ 0
LED_A
Text GLabel 5000 5100 2    50   BiDi ~ 0
GND
Text GLabel 5000 5200 2    50   BiDi ~ 0
+3V3
Text GLabel 5000 7300 2    50   BiDi ~ 0
VCAP_1
Text GLabel 5000 7400 2    50   BiDi ~ 0
+3V3
Text GLabel 7000 2700 0    50   BiDi ~ 0
USB2_DM_R
Text GLabel 7000 2800 0    50   BiDi ~ 0
USB2_DP_R
Text GLabel 7000 4400 0    50   BiDi ~ 0
USB1_DM_R
Text GLabel 7000 4500 0    50   BiDi ~ 0
USB1_DP_R
Text GLabel 7000 4600 0    50   BiDi ~ 0
SWDIO
Text GLabel 7000 4700 0    50   BiDi ~ 0
VCAP_2
Text GLabel 7000 4800 0    50   BiDi ~ 0
GND
Text GLabel 7000 4900 0    50   BiDi ~ 0
+3V3
Text GLabel 7000 5000 0    50   BiDi ~ 0
SWCLK
Text GLabel 7000 6800 0    50   BiDi ~ 0
BOOT0
Text GLabel 7000 7300 0    50   BiDi ~ 0
GND
Text GLabel 7000 7400 0    50   BiDi ~ 0
+3V3
Text GLabel 1200 1050 1    50   BiDi ~ 0
USB1_DP
Text GLabel 1200 1350 3    50   BiDi ~ 0
USB1_DP_R
Text GLabel 1900 1050 1    50   BiDi ~ 0
USB1_DM
Text GLabel 1900 1350 3    50   BiDi ~ 0
USB1_DM_R
Text GLabel 2600 1050 1    50   BiDi ~ 0
USB2_DP
Text GLabel 2600 1350 3    50   BiDi ~ 0
USB2_DP_R
Text GLabel 3300 1050 1    50   BiDi ~ 0
USB2_DM
Text GLabel 3300 1350 3    50   BiDi ~ 0
USB2_DM_R
Text GLabel 4000 1050 1    50   BiDi ~ 0
USB1_CC1
Text GLabel 4000 1350 3    50   BiDi ~ 0
GND
Text GLabel 4700 1050 1    50   BiDi ~ 0
USB1_CC2
Text GLabel 4700 1350 3    50   BiDi ~ 0
GND
Text GLabel 5400 1050 1    50   BiDi ~ 0
USB2_CC1
Text GLabel 5400 1350 3    50   BiDi ~ 0
GND
Text GLabel 6100 1050 1    50   BiDi ~ 0
USB2_CC2
Text GLabel 6100 1350 3    50   BiDi ~ 0
GND
Text GLabel 6800 1050 1    50   BiDi ~ 0
BOOT0
Text GLabel 6800 1350 3    50   BiDi ~ 0
GND
Text GLabel 7500 1050 1    50   BiDi ~ 0
LED_A
Text GLabel 7500 1350 3    50   BiDi ~ 0
LED_K
Text GLabel 8200 1050 1    50   BiDi ~ 0
VCAP_1
Text GLabel 8200 1350 3    50   BiDi ~ 0
GND
Text GLabel 8900 1050 1    50   BiDi ~ 0
VCAP_2
Text GLabel 8900 1350 3    50   BiDi ~ 0
GND
Text GLabel 9600 1050 1    50   BiDi ~ 0
+3V3
Text GLabel 9600 1350 3    50   BiDi ~ 0
GND
Text GLabel 10300 1050 1    50   BiDi ~ 0
+3V3
Text GLabel 10300 1350 3    50   BiDi ~ 0
GND
Text GLabel 11000 1050 1    50   BiDi ~ 0
+3V3
Text GLabel 11000 1350 3    50   BiDi ~ 0
GND
Text GLabel 1200 1950 1    50   BiDi ~ 0
+3V3
Text GLabel 1200 2250 3    50   BiDi ~ 0
GND
Text GLabel 1900 1950 1    50   BiDi ~ 0
+3V3
Text GLabel 1900 2250 3    50   BiDi ~ 0
GND
Text GLabel 2600 1950 1    50   BiDi ~ 0
+3V3
Text GLabel 2600 2250 3    50   BiDi ~ 0
GND
Text GLabel 3300 1950 1    50   BiDi ~ 0
+3V3
Text GLabel 3300 2250 3    50   BiDi ~ 0
GND
Text GLabel 4000 1950 1    50   BiDi ~ 0
+5V
Text GLabel 4000 2250 3    50   BiDi ~ 0
GND
Text GLabel 4700 1950 1    50   BiDi ~ 0
VDDA
Text GLabel 4700 2250 3    50   BiDi ~ 0
GND
Text GLabel 5400 1950 1    50   BiDi ~ 0
VDDA
Text GLabel 5400 2250 3    50   BiDi ~ 0
GND
Text GLabel 6100 1950 1    50   BiDi ~ 0
NRST
Text GLabel 6100 2250 3    50   BiDi ~ 0
GND
Text GLabel 6800 1950 1    50   BiDi ~ 0
OSC_IN
Text GLabel 6800 2250 3    50   BiDi ~ 0
GND
Text GLabel 7500 1950 1    50   BiDi ~ 0
OSC_OUT
Text GLabel 7500 2250 3    50   BiDi ~ 0
GND
Text GLabel 8200 1950 1    50   BiDi ~ 0
+3V3
Text GLabel 8200 2250 3    50   BiDi ~ 0
VDDA
Text GLabel 8900 1950 1    50   BiDi ~ 0
OSC_IN
Text GLabel 8900 2250 3    50   BiDi ~ 0
OSC_OUT
Text GLabel 9600 1950 1    50   BiDi ~ 0
LED_K
Text GLabel 9600 2250 3    50   BiDi ~ 0
GND
Text GLabel 10300 1950 1    50   BiDi ~ 0
NRST
Text GLabel 10300 2250 3    50   BiDi ~ 0
GND
Text GLabel 2000 2900 2    50   BiDi ~ 0
+5V
Text GLabel 2000 3000 2    50   BiDi ~ 0
USB1_DP
Text GLabel 2000 3100 2    50   BiDi ~ 0
USB1_DM
Text GLabel 2000 3200 2    50   BiDi ~ 0
USB1_CC1
Text GLabel 2000 3300 2    50   BiDi ~ 0
USB1_CC2
Text GLabel 2000 3400 2    50   BiDi ~ 0
USB1_DP
Text GLabel 2000 3500 2    50   BiDi ~ 0
USB1_DM
Text GLabel 2000 3600 2    50   BiDi ~ 0
GND
Text GLabel 4500 2900 2    50   BiDi ~ 0
VBUS_PHONE
Text GLabel 4500 3000 2    50   BiDi ~ 0
USB2_DP
Text GLabel 4500 3100 2    50   BiDi ~ 0
USB2_DM
Text GLabel 4500 3200 2    50   BiDi ~ 0
USB2_CC1
Text GLabel 4500 3300 2    50   BiDi ~ 0
USB2_CC2
Text GLabel 4500 3400 2    50   BiDi ~ 0
USB2_DP
Text GLabel 4500 3500 2    50   BiDi ~ 0
USB2_DM
Text GLabel 4500 3600 2    50   BiDi ~ 0
GND
Text GLabel 7000 3150 2    50   BiDi ~ 0
GND
Text GLabel 7000 3250 2    50   BiDi ~ 0
+3V3
Text GLabel 7000 3350 2    50   BiDi ~ 0
+5V
Text GLabel 9000 3000 2    50   BiDi ~ 0
USB1_DP
Text GLabel 9000 3100 2    50   BiDi ~ 0
GND
Text GLabel 9000 3200 2    50   BiDi ~ 0
USB1_DM
Text GLabel 9000 3300 2    50   BiDi ~ 0
USB1_DM
Text GLabel 9000 3400 2    50   BiDi ~ 0
+5V
Text GLabel 9000 3500 2    50   BiDi ~ 0
USB1_DP
Text GLabel 2000 4400 2    50   BiDi ~ 0
USB2_DP
Text GLabel 2000 4500 2    50   BiDi ~ 0
GND
Text GLabel 2000 4600 2    50   BiDi ~ 0
USB2_DM
Text GLabel 2000 4700 2    50   BiDi ~ 0
USB2_DM
Text GLabel 2000 4800 2    50   BiDi ~ 0
VBUS_PHONE
Text GLabel 2000 4900 2    50   BiDi ~ 0
USB2_DP
Text GLabel 4600 4450 2    50   BiDi ~ 0
+3V3
Text GLabel 4600 4550 2    50   BiDi ~ 0
SWDIO
Text GLabel 4600 4650 2    50   BiDi ~ 0
GND
Text GLabel 4600 4750 2    50   BiDi ~ 0
SWCLK
Text GLabel 4600 4850 2    50   BiDi ~ 0
NRST
Text GLabel 7100 4600 2    50   BiDi ~ 0
BOOT0
Text GLabel 7100 4700 2    50   BiDi ~ 0
+3V3
$EndSCHEMATC
