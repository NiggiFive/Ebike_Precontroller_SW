/*Hier kommen alle Parameter ein, die man �ndern k�nnen soll, je nach Anwendung
 * config.h
 *
 *  Created on: 14.02.2020
 *      Author: Niggi
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define DISPLAY_CONNECTED
#define DISPLAY_MODI	7

//#define SPEED_LIMIT_BY_VESC

//#define UNDERVOLTAGE_LIMIT_BY_VESC

#define TORQUE_CTRL     1     //Current-Control (=fixed Torque-Steps)
#define POWER_CTRL      2     // Power-Control     (= fixed Power-Steps, torque descreases with speed)
// 3: Voltage-Control (only recommended for cheap-china-controllers )
#define CTRL_MODE   POWER_CTRL

// Reverse Up/Down-Buttons
//#define REVERSE_BUTTONS

//UART-Speed for VESC-Communication:
//#define VESC_BAUDRATE 19200
//#define VESC_BAUDRATE 28800
//#define VESC_BAUDRATE 115200
#define VESC_BAUDRATE 	57600

// 1 for first Version
// 2 for 2nd version
#define HW_VERSION 	1

// Motor Data
//Anzahl Polpaare
#define MOTOR_POLE_PAIRS 	10
#define MOTOR_POLES			20
//Uebersetzungsverhaeltnis des Planetengetriebes (falls vorhanden, sonst 1)
#define MOTOR_GEAR_RATIO 	4.42
#define MOTOR_FLUX_LINKAGE  0.01276     //Voltseconds
//#define PI                  3.14

// Assume linear PSM with Ld=Lq
#define TORQUE_PER_AMP              1.5*MOTOR_FLUX_LINKAGE      // Caution: "Electrical Torque" because Pole-Pairs are missing in formula!

#define AMPS_PER_WATTS_AND_ERPM     60/(TWO_PI*TORQUE_PER_AMP)

#define ABS_MAX_CURRENT             30.0

//Radumfang in Metern
//fuer 28 Zoll (Schwalbe Marathon)
//#define RADUMFANG 			2.155

//fuer 26 Zoll (Big Apple 2,15)
#define RADUMFANG			2.130

// Maximale Geschwindigkeit fuer Unterstuetzung
#define MAX_SPEED_KMH 		27
//Geschwindigkeit ab der der Strom zur�ckgenommen wird
#define VGRENZ				23
#define MAX_RPM 			(MAX_SPEED_KMH*1000)/(RADUMFANG*60)
#define MAX_ERPM			MAX_RPM*MOTOR_POLE_PAIRS*MOTOR_GEAR_RATIO
#define RPM_GRENZ			(VGRENZ*1000)/(RADUMFANG*60)
#define ERPM_GRENZ			RPM_GRENZ*MOTOR_POLE_PAIRS*MOTOR_GEAR_RATIO

//Max ERPM (f�r VESC Konfiguration):
//f�r VMAX = 27km/h
//26 Zoll : ERPM_max = 9340
//28 Zoll : ERPM_max = 9230

// Undervoltage-Grenzen fuer Batterie:
#define UNDERVOLTAGE_12S 3.3*12.0
#define UNDERVOLTAGE_10S 3.25*10.0
#define UNDERVOLTAGE_9S 3.3*9.0
#define UNDERVOLTAGE_6S 3.3*6.0
#define UNDERVOLTAGE_3S	3.3*3.0

// Grenze zur Erkennung 6s vs. 9s vs. 12s Batterie
#define BAT10S12S_GRENZE 42.0
#define BAT9S10S_GRENZE 37.0
#define BAT6S9S_GRENZE 26.0
#define BAT3S6S_GRENZE 15.0

// Werte fuer Pedalsensierung:
#define DOUBLE_HALL	0			// 1 for Double-Hall-Sensor, 0 for non-double-Hall
#define PAS_MAGNETS 10			// Anzahl an Magneten in Scheibe -> bei mamas Rad sinds 10, bei meinem Reise-MTB 12
#define MIN_CADENCE	25			// Minimale Kadenz (Kurbelumdrehungen pro Minute) fuer Unterstuetzung

#define CONV_PAS_TIME_TO_CADENCE 60000/PAS_MAGNETS

//Haltezeit fuer Pedalsensierung in ms -> Berechnet aus Anzahl an Magneten und minimaler Kadenz
#define PAS_TIMEOUT CONV_PAS_TIME_TO_CADENCE/MIN_CADENCE

#define PAS_FACTOR_MIN	110		// Wichtig um Vorwaerts und Rueckwaerts-Pedalieren unterscheiden zu k�nnen bei non-double-Hall-Sensoren

// Zeit (in 100ms-Schritten), nach der beim aufh�ren zu Pedalieren die Utnerstuetzungsstufe wieder auf den default-Wert zur�ckgesetzt wird
#define TIME_TO_RESET_AFTER_PEDAL_STOP	100

// Zeitfenster fuer Tastereingabe in ms
#define TASTER_TIME_WINDOW 500

#define ANZAHL_STUFEN	4

// Strom-Stufen fuer VESC
//#define STUFE5_I 30
#define STUFE4_I 30		// -> 30A schiebt schon krass
#define STUFE3_I 20
#define STUFE2_I 12
#define STUFE1_I 6 		// 4A war ziemlich wenig

// Power-Modes for VESC:
#define STUFE4_P 500
#define STUFE3_P 350
#define STUFE2_P 200
#define STUFE1_P 100


// Geschwindigkeits-Stufen Ausgangsspannungen fuer Ku-63
//#define STUFE5_V 3.75
//#define STUFE4_V 3.25
//#define STUFE3_V 2.75
//#define STUFE5_V 3.0
#define STUFE4_V 3.0
#define STUFE3_V 2.7
#define STUFE2_V 2.5
#define STUFE1_V 2.0
#define STUFE0 0

#define SCHIEBEHILFE STUFE1_V

#define DEFAULT_STUFE 1

//  Spannungsteiler Spannungssensierung:
// 	Nominalwerte:
//  Rupper=47k
//  Rlower=4k7

// bei HW-Version 1 waren es 22k und 2k2! 
#if HW_VERSION == 1
    #define RUPPER_BATSENSE 22.0
    #define RLOWER_BATSENSE 2.2

#elif HW_VERSION == 2 
    #define RUPPER_BATSENSE 47.0
    #define RLOWER_BATSENSE 4.7
#endif

#define REFVOLT 5.00

// Spannungsteiler zur sensierung der Arduino-Versorgungsspannung
#define RUPPER_VARDU     22.0
#define RLOWER_VARDU    12.0

#endif /* CONFIG_H_ */
