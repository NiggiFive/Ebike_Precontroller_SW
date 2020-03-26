/*Hier kommen alle Parameter ein, die man �ndern k�nnen soll, je nach Anwendung
 * config.h
 *
 *  Created on: 14.02.2020
 *      Author: Niggi
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define DISPLAY_CONNECTED

//UART-Geschwindigkeit f�r VESC einstellen:
//#define VESC_BAUDRATE 19200
//#define VESC_BAUDRATE 28800
//#define VESC_BAUDRATE 115200
#define VESC_BAUDRATE 57600

//Anzahl Polpaare
#define MOTOR_POLE_PAIRS 	10
//Uebersetzungsverh�ltnis des Planetengetriebes (falls vorhanden, sonst 1)
#define MOTOR_GEAR_RATIO 	4.42
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
#define UNDERVOLTAGE_9S 3.3*9.0
#define UNDERVOLTAGE_6S 3.3*6.0
#define UNDERVOLTAGE_10S 3.25*10.0

// Grenze zur Erkennung 6s vs. 9s Batterie
#define BAT6S9S_GRENZE 26.0

// Werte fuer Pedalsensierung:
#define DOUBLE_HALL	0			// 1 f�r Double-Hall-Sensor, 0 f�r non-double-Hall
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

// Geschwindigkeits-Stufen Ausgangsspannungen f�r Ku-63
//#define STUFE5_V 3.75
//#define STUFE4_V 3.25
//#define STUFE3_V 2.75
#define STUFE5_V 3.0
#define STUFE4_V 2.7
#define STUFE3_V 2.6
#define STUFE2_V 2.5
#define STUFE1_V 1.7
#define STUFE0 0

#define SCHIEBEHILFE STUFE1_V

#define DEFAULT_STUFE 1

#endif /* CONFIG_H_ */
