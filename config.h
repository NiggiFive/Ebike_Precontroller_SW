/*Hier kommen alle Parameter ein, die man ändern können soll, je nach Anwendung
 * config.h
 *
 *  Created on: 14.02.2020
 *      Author: Niggi
 */

#ifndef CONFIG_H_
#define CONFIG_H_

//#define DISPLAY_CONNECTED

//UART-Geschwindigkeit für VESC einstellen:
//#define VESC_BAUDRATE 19200
#define VESC_BAUDRATE 115200

//Anzahl Polpaare
#define MOTOR_POLE_PAIRS 	10
//Uebersetzungsverhältnis des Planetengetriebes (falls vorhanden, sonst 1)
#define MOTOR_GEAR_RATIO 	4.42
//Radumfang in Metern
//fuer 28 Zoll (Schwalbe Marathon)
#define RADUMFANG 			2.155

//fuer 26 Zoll (Big Apple 2,15)
//#define RADUMFANG			2.130

// Maximale Geschwindigkeit fuer Unterstuetzung
#define MAX_SPEED_KMH 		27
#define MAX_RPM 			(MAX_SPEED_KMH*1000)/(RADUMFANG*60)
#define MAX_ERPM			MAX_RPM*MOTOR_POLE_PAIRS*MOTOR_GEAR_RATIO

//Geschwindigkeit ab der der Strom zurückgenommen wird
#define VGRENZ				23

//Max ERPM (für VESC Konfiguration):
//für VMAX = 27km/h
//26 Zoll : ERPM_max = 9340
//28 Zoll : ERPM_max = 9230

// Undervoltage-Grenzen fuer Batterie:
#define UNDERVOLTAGE_9S 3.3*9.0
#define UNDERVOLTAGE_6S 3.3*6.0
#define UNDERVOLTAGE_10S 3.25*10.0


// Werte fuer Pedalsensierung:
#define DOUBLE_HALL	0			// 1 für Double-Hall-Sensor, 0 für non-double-Hall
#define PAS_MAGNETS 10			// Anzahl an Magneten in Scheibe -> bei mamas Rad sinds 10, bei meinem Reise-MTB 12
#define MIN_CADENCE	25			// Minimale Kadenz (Kurbelumdrehungen pro Minute) fuer Unterstuetzung

#define PAS_FACTOR_MIN	120		// Wichtig um Vorwaerts und Rueckwaerts-Pedalieren unterscheiden zu können bei non-double-Hall-Sensoren

// Zeit, nach der beim aufhören zu Pedalieren die Utnerstuetzungsstufe wieder auf den default-Wert zurückgesetzt wird in Millisekunden
#define TIME_TO_RESET_AFTER_PEDAL_STOP	10000

#define ANZAHL_STUFEN	4

// Strom-Stufen fuer VESC
//#define STUFE5_I 30
#define STUFE4_I 30
#define STUFE3_I 20
#define STUFE2_I 10
#define STUFE1_I 6 		// 4A war ziemlich wenig

#define DEFAULT_STUFE 1

#endif /* CONFIG_H_ */
