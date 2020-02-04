/* letzte Aenderung: 22.01.2020: Pedal-Erkennung funktioniert
 * 21.01.2020: Display-Ansteuerung Funktioniert
   V1.00: Neue Version f�r neue Hardware
	nur Pin 2 und 3 k�nnen flanken-interrupts */

#include "Arduino.h"
#include <Wire.h>
//#include "Adafruit_GFX.h"
//#include "Adafruit_SSD1306.h"

//#include <TimerOne.h>

#include "VescUart.h"

#include <U8g2lib.h>

/** Initiate VescUart class */
VescUart UART;

// Fuer Display
//#define OLED_RESET 4 // not used / nicht genutzt bei diesem Display
//Adafruit_SSD1306 display(OLED_RESET);

//Display Constructor
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(A5, A4);


// Einstellung, ob VESC oder KU63 angeschlossen
#define CTRLMODE VESC
#define VESC 1
#define KU63 0

//  Spannungsteiler Spannungssensierung:
// 	Nominalwerte:
//  Rupper=22k
//  Rlower=2k2
#define RUPPER 22.0
#define RLOWER 2.2
#define REFVOLT 5.00

// Undervoltage-Grenzen f�r Batterie:
#define UNDERVOLTAGE_9S 3.4*9.0
#define UNDERVOLTAGE_6S 3.4*6.0
#define UNDERVOLTAGE_10S 3.4*10.0

// Grenze zur Erkennung 6s vs. 9s Batterie
#define BAT6S9S_GRENZE 27.0

//Pins f�r Ein- und Ausg�nge
#define INPUT_PAS 3

#define INPUT_TASTER1 2
#define INPUT_TASTER2 4		// -> Taster2 �blicherweise f�r Licht?
#define INPUT_3W_SW_RED 6
#define INPUT_3W_SW_GREEN 7
#define OUTPUT_THROTTLE 5
#define OUTPUT_DISPLAY_SUPPLY 8
#define OUTPUT_LIGHT_SWITCH 9
#define INPUT_BATSENSE 6
#define LED 13

#define VESC_BAUDRATE 19200

#define SERIAL_MONITOR_BAUDRATE 9600

//Anzahl Polpaare
#define MOTOR_POLE_PAIRS 	10
#define MOTOR_GEAR_RATIO 	4.42
//Radumfang in Metern
//f�r 28 Zoll (Schwalbe Marathon)
#define RADUMFANG 			2.155

//f�r 26 Zoll
//#define RADUMFANG			2.130

// Maximale Geschwindigkeit f�r Unterst�tzung
#define MAX_SPEED_KMH 		15
#define MAX_RPM 			(MAX_SPEED_KMH*1000)/(RADUMFANG*60)
#define MAX_ERPM			MAX_RPM*MOTOR_POLE_PAIRS*MOTOR_GEAR_RATIO


// Geschwindigkeits-Stufen Ausgangsspannungen
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

// Strom-Stufen f�r VESC
#define STUFE5_I 20
#define STUFE4_I 15
#define STUFE3_I 10
#define STUFE2_I 6
#define STUFE1_I 3

#define DEFAULT_STUFE 1

// Zeitfenster f�r Tastereingabe in ms
#define TASTER_TIME_WINDOW 500

// Werte f�r Pedalsensierung:
#define PAS_MAGNETS 12			// Anzahl an Magneten in Scheibe

#define MIN_CADENCE	20				// Minimale Kadenz (Kurbelumdrehungen pro Minute) f�r Unterst�tzung

#define CONV_PAS_TIME_TO_CADENCE 60000/PAS_MAGNETS

//Haltezeit f�r Pedalsensierung in ms -> Berechnet aus Anzahl an Magneten und minimaler Kadenz
#define PAS_TIMEOUT CONV_PAS_TIME_TO_CADENCE/MIN_CADENCE

// 10ms Timer
#define FAST_TIMER 10

//100ms Timer
#define SLOW_TIMER 100

#define ULTRA_SLOW_TIMER 1000

// Taster-Entprellzeit in ms
#define ENTPRELLZEIT 5

//f�r Display
//#define DRAW_DELAY 118
//#define D_NUM 47


//const float stufenV [6] = {0.0, STUFE1_V, STUFE2_V, STUFE3_V, STUFE4_V, STUFE5_V};
int stufenI [6] = {0, STUFE1_I, STUFE2_I, STUFE3_I, STUFE4_I, STUFE5_I};

unsigned long milliseconds;

int test1;
int test2;
int test3;

bool temp_bool;

bool vesc_connected = false;
bool vesc_connection_error = false;

bool led_state;

bool pipapo = 0;

uint8_t numberOfCells = 9;
float voltageArdu = 0.0;
float voltageVesc = 0.0;
float avgCellVolt = 0.0;
uint8_t SOC = 0;


/*struct vescDataStruct
{
	float avgMotorCurrent;
	float avgInputCurrent;
	float dutyCycleNow;
	long rpm;
	float inpVoltage;
	float ampHours;
	float ampHoursCharged;
	long tachometer;
	long tachometerAbs;
};*/

struct throttleControlStruct
{
  int aktStufe = DEFAULT_STUFE;
  float current = 0.0;
  float throttleVoltage = 0.0;
};


	int taster1_state = 0;
	bool taster1_ready = false;
	int taster1_edges = 0;
	long int taster1_lastEvent = 0;

	int taster2_state = false;
	bool taster2_ready = false;
	int taster2_edges = 0;
	long int taster2lastEvent = 0;

	int switchRed_state = false;
	bool switchRed_ready = false;
	int switchRed_edges = 0;
	long int switchRed_lastEvent = 0;

	int switchGreen_state = false;
	bool switchGreen_ready = false;
	int switchGreen_edges = 0;
	long int switchGreen_lastEvent = 0;


//Variables for Timer
	int slowTimerVariable = 0;
	unsigned long lastSlowLoop;
	unsigned long lastUltraSlowLoop;
	unsigned long lastFastLoop;
	bool slowTimerFlag = false;
	int fastTimerVariable = 0;
	bool fastTimerFlag = false;
	int ultraslowTimerVariable = 0;
	bool ultraslowTimerFlag = false;


//Variables for PAS
	uint16_t pasTimeHigh = 0;
	uint16_t pasTimeLow = 0;
	unsigned long last_pas_event = 0;
	bool pas_status = false;
	uint16_t pas_factor = 0;
	uint16_t pas_failtime = 0;
	uint16_t cadence = 0;
    bool pedaling = false;

// Anlegen der ben�tigten Structs
throttleControlStruct throttleControl;


//ISR f�r PAS
void pas_ISR()
{
	if(last_pas_event > (millis()-ENTPRELLZEIT)){
		return;
	}
	pas_status = digitalRead(INPUT_PAS);
	if(pas_status == true){
		pasTimeLow = millis()-last_pas_event;
	}
	else{
		pasTimeHigh = millis()-last_pas_event;
	}
	last_pas_event = millis();
	cadence = CONV_PAS_TIME_TO_CADENCE/(pasTimeLow+pasTimeHigh);

	pas_factor = 100*pasTimeHigh/pasTimeLow;
	if(cadence>MIN_CADENCE){
		pedaling = true;
	}
}

/*void taster_ISR()
{

}*/

void checkTaster()
{

	//Taster1 abfragen
 /*temp_bool = digitalRead(INPUT_TASTER1);
  if(temp_bool!=taster1.state && temp_bool == false)
  {		//fallende Flanke -> wurde gerade gedr�ckt
	  taster1.edges++;
	  taster1.lastEvent = milliseconds;
  }
  taster1.state = temp_bool;
  if(milliseconds - taster1.lastEvent >= TASTER_TIME_WINDOW)
  {
	  taster1.ready = true;
  }*/

  // Licht-Taster abfragen
	//test2 = digitalRead(INPUT_TASTER2);
	  temp_bool = digitalRead(INPUT_TASTER2);
	  if(temp_bool==false)
	  {
		  if(taster2_state == true)
		  {//fallende Flanke -> wurde gerade gedr�ckt
			  taster2_edges++;
		  }
		  taster2_state = temp_bool;
	  }

  // 3W Rot-Taster abfragen
test1 = digitalRead(INPUT_3W_SW_RED);

  // 3W-Gr�n-Taster abfragen
  temp_bool = digitalRead(INPUT_3W_SW_GREEN);
  if(temp_bool == false)
  {
	  if(switchGreen_state == true)
	  {//fallende Flanke -> wurde gerade gedr�ckt
		  switchGreen_edges++;
		  switchGreen_state = temp_bool;
	  }
  }
}

// Batteriespannung auslesen und umrechnen
void readBattVolt()
{
  int voltHex = analogRead(INPUT_BATSENSE);
  voltageArdu = (float)((voltHex*REFVOLT/1024.0)*(RUPPER+RLOWER)/RLOWER);
  avgCellVolt = voltageArdu/numberOfCells;
}


// 1ms-Timer-ISR
/*void timer1_ISR()
{
	// Variable f�r zweiten und dritten Timer hochz�hlen
	fastTimerVariable++;
	slowTimerVariable++;
	ultraslowTimerVariable++;

	//10ms-Timer:
  if (fastTimerVariable >= FAST_TIMER)
  {
	  //Flag setzen
	  fastTimerFlag = true;
      // Timer zur�cksetzen
      fastTimerVariable = 0;
  }

  //100ms-Timer
  if(slowTimerVariable >= SLOW_TIMER)
	{
		slowTimerFlag = true;
		slowTimerVariable = 0;
		if(digitalRead(LED))
		{
			digitalWrite(LED, LOW);
		}
		else
		{
			digitalWrite(LED, HIGH);
		}
	}

  //1s-Timer
  if(ultraslowTimerVariable >= ULTRA_SLOW_TIMER)
	{
		ultraslowTimerFlag = true;
		ultraslowTimerVariable = 0;
	}
}*/

void calculateSOC()
{
	/*Wertetabelle f�r SOC Berechnung
	 * Spannung		Prozent
	 * >4.08V		100
	 * >4.06		95
	 * >4.00		90
	 * >3.97		85
	 * >3.94		80
	 * >3.92		75
	 * >3.88		70
	 * >3.85		65
	 * >3.83		60
	 * >3.82		55
	 * >3.80		50
	 * >3.79		45
	 * >3.78		40
	 * >3.77		35
	 * >3.76		30
	 * >3.75		25
	 * >3.74		20
	 * >3.72		15
	 * >3.69		10
	 * >3.68		5
	 * <3.68		0
	 */
	int temp = (int)(avgCellVolt*1000.0);
	if(temp > 4080)
	{
		SOC = 100;
	}
	else if (temp >4060)
	{
		SOC = 95;
	}
	else if (temp >4000)
	{
		SOC = 90;
	}
	else if (temp >3970)
	{
		SOC = 85;
	}
	else if (temp >3940)
	{
		SOC = 80;
	}
	else if (temp >3920)
	{
		SOC = 75;
	}
	else if (temp >3880)
	{
		SOC = 70;
	}
	else if (temp >3850)
	{
		SOC = 65;
	}
	else if (temp >3830)
	{
		SOC = 60;
	}
	else if (temp >3820)
	{
		SOC = 55;
	}
	else if (temp >3800)
	{
		SOC = 50;
	}
	else if (temp >3790)
	{
		SOC = 45;
	}
	else if (temp >3780)
	{
		SOC = 40;
	}
	else if (temp >3770)
	{
		SOC = 35;
	}
	else if (temp >3760)
	{
		SOC = 30;
	}
	else if (temp >3750)
	{
		SOC = 25;
	}
	else if (temp >3740)
	{
		SOC = 20;
	}
	else if (temp >3720)
	{
		SOC = 15;
	}
	else if (temp >3690)
	{
		SOC = 10;
	}
	else if (temp >3680)
	{
		SOC = 5;
	}
	else
	{
		SOC = 0;
	}

}

void setThrottlePWM()
{
  analogWrite(OUTPUT_THROTTLE, throttleControl.throttleVoltage/5*255.0);
  if(vesc_connected){
  }

}

/*void refreshDisplay()
{
	  display.clearDisplay();

	  // set text color / Textfarbe setzen
	  display.setTextColor(WHITE);
	  // set text size / Textgroesse setzen
	  display.setTextSize(1);
	  // set text cursor position / Textstartposition einstellen
	  display.setCursor(1,1);

	  if(vesc_connected){
		  //display.println("vesc");
		  //Zeile 1:
		  //Batteriespannung ausgeben
		  display.print(UART.data.inpVoltage);
		  display.print("V ");
		  //SOC Ausgeben
		  display.print(SOC);
		  display.println("%");

		  if(pedaling)
		  {
			  display.println("pedaling");
		  }
		  else
		  {
			  display.println("not pedaling");
		  }

		  //Zeile 2:
		  //MOTOR Strom ausgeben
		  //display.print("Im=");
		  display.print(UART.data.avgMotorCurrent);
		  display.print("A ");
		  //Akkustrom ausgeben:
		  //display.print(" Ibat=");
		  display.print(UART.data.avgInputCurrent);
		  display.println("A");

		  //Zeile 3:
		  //ERPM ausgeben:
		  display.print(UART.data.rpm);
		  display.print("erpm");
		  //Geschwindigkeit Ausgeben:
		  /*display.print(" 15.0");
		  display.println("km/h");

		  //Zeile 4:
		  //Leistung ausgeben:
		  /*display.print("P=");
		  display.print("100");
		  display.print("W");
		  //Unterst�tzungsstufe ausgeben:
		  display.print("St ");
		  display.print(throttleControl.aktStufe);


	  }

	  else{
		  //Zeile 1:
		  //Batteriespannung ausgeben
		  display.print(voltage_ardu);
		  display.print("V  ");
		  //SOC Ausgeben
		  display.print(SOC);
		  display.println("%");

		  //Pedaling?
		  if(pedaling)
		  {
			  display.println("pedaling");
		  }
		  else
		  {
			  display.println("not pedaling");
		  }

		  //Unterst�tzungsstufe ausgeben:
		  display.print("Stufe ");
		  display.println(throttleControl.aktStufe);

		  //scheinbar kann ich hier max. 5 buchstaben schreiben?!?! wtf

	  }
	  display.display();
}*/

void refreshu8x8Display()
{
	  u8x8.clearDisplay();
	  //delay(1);

	  if(vesc_connected){

		  u8x8.home();
		  //display.println("vesc");
		  //Zeile 1:
		  //Batteriespannung ausgeben
		  u8x8.print(UART.data.inpVoltage);
		  u8x8.print("V ");
		  //SOC Ausgeben
		  u8x8.print(SOC);
		  u8x8.println("%");

		  if(pedaling)
		  {
			  u8x8.println("pedaling");
		  }
		  else
		  {
			  u8x8.println("not pedaling");
		  }

		  //Unterst�tzungsstufe ausgeben:
		  u8x8.print("Stufe ");
		  u8x8.println(throttleControl.aktStufe);

		  //Zeile 2:
		  //MOTOR Strom ausgeben
		  //display.print("Im=");
		  u8x8.print(UART.data.avgMotorCurrent);
		  u8x8.print("A ");
		  //Akkustrom ausgeben:
		  //display.print(" Ibat=");
		  u8x8.print(UART.data.avgInputCurrent);
		  u8x8.println("A");

		  //Zeile 3:
		  //ERPM ausgeben:
		  u8x8.print(UART.data.rpm);
		  u8x8.print("rpm ");
		  //Geschwindigkeit Ausgeben:
		  u8x8.print((UART.data.rpm*RADUMFANG/MOTOR_POLE_PAIRS/MOTOR_GEAR_RATIO/60000));
		  u8x8.println("km/h");

		  //Zeile 4:
		  //Leistung ausgeben:
		  u8x8.print("P=");
		  u8x8.print("100");
		  u8x8.print("W");



	  }

	  else{
		  //u8x8.println("blabliblub");
		  u8x8.home();
		  //Zeile 1:
		  //Batteriespannung ausgeben
		  u8x8.print(voltageArdu);
		  u8x8.print("V  ");
		  //SOC Ausgeben
		  u8x8.print(SOC);
		  u8x8.println("%");

		  //Pedaling?
		  if(pedaling)
		  {
			  u8x8.println("pedaling");
		  }
		  else
		  {
			  u8x8.println("not pedaling");
		  }

		  //Unterst�tzungsstufe ausgeben:
		  u8x8.print("Stufe ");
		  u8x8.println(throttleControl.aktStufe);

	  }
	  u8x8.display();
}

void setup() {
  // put your setup code here, to run once:

	// Pin A4 als Ausgang einstellen (f�r I2C)
	pinMode(PIN_A4, OUTPUT);
	digitalWrite(PIN_A4, 0);

	//PAS-Sensor-Eingang:
	pinMode(INPUT_PAS, INPUT_PULLUP);

	// Gas-Taster-Eingang:
	pinMode(INPUT_TASTER1, INPUT_PULLUP);

	// Licht-Taster-Eingang:
	pinMode(INPUT_TASTER2, INPUT_PULLUP);

	//3W-Switch-Rot-Eingang:
	pinMode(INPUT_3W_SW_RED, INPUT_PULLUP);
	//3W-Switch-Gr�n-Eingang:
	pinMode(INPUT_3W_SW_GREEN, INPUT_PULLUP);

	//Throttle PWM-Output:
	pinMode(OUTPUT_THROTTLE, OUTPUT);

	//f�r Display:
	//Display-Versorgung einschalten:
	pinMode(OUTPUT_DISPLAY_SUPPLY, OUTPUT);
	digitalWrite(OUTPUT_DISPLAY_SUPPLY, LOW);

	//LED auf ArduinoNano Pin 13
	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);


	// initialize with the I2C addr 0x3C / mit I2C-Adresse 0x3c initialisieren
	//display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

    u8x8.begin();

    u8x8.setFont(u8x8_font_chroma48medium8_r);

	// Zellspannung einmal abfragen f�r 6s-9s-Erkennung:
	readBattVolt();
	if (voltageArdu > BAT6S9S_GRENZE)
	{
		numberOfCells = 9;
	}
	else
	{
		numberOfCells = 6;
	}

	/* millis() Returns the number of milliseconds passed since the Arduino board
	began running the current program. This number will overflow (go back to zero), after approximately 50 days. */
	milliseconds = millis();

	// Interrupt f�r PAS konfigurieren
	attachInterrupt(digitalPinToInterrupt(INPUT_PAS), pas_ISR, CHANGE);

	// falling-edge interrupt f�r Taster konfigurieren	-> TODO
	//attachInterrupt(digitalPinToInterrupt(INPUT_TASTER1), taster_ISR, FALLING);

	//Timer initialisieren auf 1ms (1000us) um zyklisch abzufragen etc:
	//Timer1.initialize(1000);
	//Timer1.attachInterrupt(timer1_ISR);

	//Delay bis VESC ready ist:
	delay(2000);

	// zusammen mit VESC kann man die serielle Schnittstelle nicht mehr nehmen
	//Serial.begin(9600);
	Serial.begin(VESC_BAUDRATE);

	// aus VESC-ARduino Beispiel, ich vermute zum abwarten bis serielle Schnittstelle bereit
	while (!Serial) {;}
	UART.setSerialPort(&Serial);

	//3mal Verbindung zum VESC checken
	for(int i = 0; i<3; i++)
	{
		if(UART.getVescValues())
		{
			//LED einschalten und weiter zur loop
			vesc_connected = true;
			digitalWrite(LED, HIGH);
			break;
		}
	}

	//wenn kein Vesc dran h�ngt serielle Schnittstelle beenden
	if(vesc_connected == false)
	{
		Serial.end();
	}
}

void loop() {
  // put your main code here, to run repeatedly:

	if(millis() > milliseconds)
	{
		milliseconds = millis();

		//ISR_1MS();
	}

	// PAS-Timeout?
	if ((millis() - last_pas_event) > PAS_TIMEOUT)
	{
		pedaling = false;
	}

	// Schnelle Routine
	if((millis()-lastFastLoop) > FAST_TIMER)
	//if(fastTimerFlag)
	{
		lastFastLoop = millis();
		fastTimerFlag = false;


		//taster1_state = digitalRead(INPUT_TASTER1);
		//taster2_state = digitalRead(INPUT_TASTER2);
		test1 = digitalRead(INPUT_3W_SW_RED);
		if(test1 == 0 && switchRed_state == 1)
		{
			switchRed_edges++;
		}
		switchRed_state = test1;

		test1 = digitalRead(INPUT_3W_SW_GREEN);
		if(test1 == 0 && switchGreen_state == 1)
		{
			switchGreen_edges++;
		}
		switchGreen_state = test1;
		//Taster auslesen:
		//Taster1 abfragen
	  //taster1_state = digitalRead(INPUT_TASTER1);


	  // Licht-Taster abfragen
		//test2 = digitalRead(INPUT_TASTER2);
		  //taster2_state = digitalRead(INPUT_TASTER2);

	  // 3W Rot-Taster abfragen
	//switchRed.state = digitalRead(INPUT_3W_SW_RED);

	  // 3W-Gr�n-Taster abfragen
	  //switchGreen_state = digitalRead(INPUT_3W_SW_GREEN);
	}


	// langsame Routine
  if((milliseconds - lastSlowLoop) > SLOW_TIMER)
	//if(slowTimerFlag)
  {
		slowTimerFlag = false;
	  lastSlowLoop = millis();


		if(digitalRead(LED)==HIGH)
		{
			digitalWrite(LED, LOW);
		}
		else
		{
			digitalWrite(LED, HIGH);
		}

	  //Batteriespannung auslesen
	  readBattVolt();

	  //Taster auswerten:
	  if(switchRed_edges>0)
	  {
		  //Unterst�tzung hochschalten
		  if(throttleControl.aktStufe < 5)
		  {
			  throttleControl.aktStufe++;
		  }
		  else
		  {
			  throttleControl.aktStufe = 5;
		  }
		  switchRed_edges = 0;
	  }
	  if(switchGreen_edges>0){
		  //Unterst�tzung runterschalten
		  if(throttleControl.aktStufe > 0)
		  {
			  throttleControl.aktStufe--;
		  }
		  else
		  {
			  throttleControl.aktStufe = 0;
		  }
		  switchGreen_edges = 0;
	  }



	  if(vesc_connected)
	  {
		  //3mal versuchen auszulesen
		for(int i = 0; i<3; i++)
		{
			if(UART.getVescValues())
			{
				break;
			}
			//wenns beim dritten Mal nicht geklappt hat stimmt wohl was nicht
			else if(i>=2)
			{
				vesc_connected = false;
				Serial.end();
				digitalWrite(LED, LOW);
			}
		}
	  }

	  if(vesc_connected)
	  {
		  if(pedaling == true)
		  {
			  // Stromsollwert bestimmen
			  throttleControl.current = (float)stufenI[throttleControl.aktStufe];

			//Geschwindigkeitsgrenze:
			 if(UART.data.rpm > 0.9*MAX_ERPM)
			{
			 //throttleControl.current = (1.0-(UART.data.rpm-0.9*MAX_ERPM)/(0.2*MAX_ERPM))*throttleControl.current;
			 //throttleControl.current = 0.0;
			 }
	  	  }
		  else
		  {
			  throttleControl.current = 0.0;
		  }
		  UART.setCurrent(throttleControl.current);
	  }

	  //refreshDisplay();*/
  }

	// ultra-langsame Routine
	if((milliseconds - lastUltraSlowLoop) > ULTRA_SLOW_TIMER)
	//if(ultraslowTimerFlag)
	{

		ultraslowTimerFlag = false;
		refreshu8x8Display();
		lastUltraSlowLoop = millis();
	}
}
