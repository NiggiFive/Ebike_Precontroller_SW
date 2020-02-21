/* letzte Aenderung: 22.01.2020: Pedal-Erkennung funktioniert
 * 21.01.2020: Display-Ansteuerung Funktioniert
   V1.00: Neue Version fuer neue Hardware
	nur Pin 2 und 3 koennen flanken-interrupts */

#include "config.h"
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



//#define DISPLAY_CONNECTED		//uncomment if no Display Connected
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

// Grenze zur Erkennung 6s vs. 9s Batterie
#define BAT6S9S_GRENZE 27.0

//Pins fuer Ein- und Ausgaenge
#define INPUT_PAS 3

#define INPUT_TASTER1 2
#define INPUT_TASTER2 4		// -> Taster2 ueblicherweise fuer Licht?
#define INPUT_3W_SW_RED 6
#define INPUT_3W_SW_GREEN 7
#define OUTPUT_THROTTLE 5
#define OUTPUT_DISPLAY_SUPPLY 8
#define OUTPUT_LIGHT 9
#define INPUT_BATSENSE 6

// Pin-Nummer der LED auf Arduino Nano
#define LED 13

#define SERIAL_MONITOR_BAUDRATE 9600

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

#define DEFAULT_STUFE 2

// Zeitfenster fuer Tastereingabe in ms
#define TASTER_TIME_WINDOW 500

#define CONV_PAS_TIME_TO_CADENCE 60000/PAS_MAGNETS

//Haltezeit fuer Pedalsensierung in ms -> Berechnet aus Anzahl an Magneten und minimaler Kadenz
#define PAS_TIMEOUT CONV_PAS_TIME_TO_CADENCE/MIN_CADENCE

// 10ms Timer
#define FAST_TIMER 10

//100ms Timer
#define SLOW_TIMER 100

#define ULTRA_SLOW_TIMER 250

// Entprellzeit für PAS (und evtl. Taster) in ms
#define ENTPRELLZEIT 5

//fuer Display
//#define DRAW_DELAY 118
//#define D_NUM 47

//Display Constructor
#ifdef DISPLAY_CONNECTED
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(A5, A4);
#endif

//const float stufenV [6] = {0.0, STUFE1_V, STUFE2_V, STUFE3_V, STUFE4_V, STUFE5_V};
int stufenI [6] = {0, STUFE1_I, STUFE2_I, STUFE3_I, STUFE4_I, STUFE5_I};

unsigned long milliseconds;

int temp_int;

uint8_t displayCounter;

bool temp_bool;

float temp_float;

bool vesc_connected = false;
bool vesc_connection_error = false;

bool led_state;

bool pipapo = false;

uint8_t numberOfCells = 9;
float voltageArdu = 0.0;
float voltageVesc = 0.0;
float avgCellVolt = 0.0;
uint8_t SOC = 0;

//fuer Undervoltage-Regler
/*float undervoltage_reg_out;
float undervoltage_reg_prop;
float undervoltage_reg_int;
float undervoltage_reg_diff;

const int undervoltage_reg_pterm = 3;
const float undervoltage_reg_iterm = 0.1;*/


int velocity = 0;
float undervoltageThreshold;


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

//Variables for User-Inputs
	int taster1_state = 0;
	bool taster1_ready = false;
	int taster1_edges = 0;
	long int taster1_lastEvent = 0;

	int taster2_state = false;
	bool taster2_ready = false;
	int taster2_edges = 0;
	long int taster2lastEvent = 0;

	int switchRed_state = false;
	bool switchRedGreen_ready = false;
	int switchRed_edges = 0;
	long int switchRedGreen_lastEvent = 0;

	int switchGreen_state = false;
	//bool switchGreen_ready = false;
	int switchGreen_edges = 0;
	//long int switchGreen_lastEvent = 0;


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
	const bool doubleHall = DOUBLE_HALL;
	uint16_t pasTimeHigh = 0;
	uint16_t pasTimeLow = 0;
	unsigned long last_pas_event = 0;
	bool pas_status = false;
	uint16_t pas_factor = 0;
	uint16_t pas_failtime = 0;
	uint16_t cadence = 0;
    bool pedaling = false;

    unsigned long lastTimePedaling;

// Anlegen der benuetigten Structs
throttleControlStruct throttleControl;

//ISR fuer PAS
void pas_ISR()
{
	if(last_pas_event > (millis()-ENTPRELLZEIT)){
		//Entprellung
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
	if(pas_factor > 900)
	{
		pas_factor = 900;
	}
	if(doubleHall == false)
	{	// wenn man keinen Double-Hall-Sensor hat muss man den PAS-Faktor mit einbeziehen
		if(cadence>=MIN_CADENCE && pas_factor > PAS_FACTOR_MIN)
		{
			pedaling = true;
		}
		else
		{
			if(pedaling == true)
			{
				lastTimePedaling = millis();
			}
			pedaling = false;
			//pas_factor = 0;
		}
	}
	else
	{
		// mit Double-Hall-Sensor muss man nur die Cadence überprüfen
		if(cadence>=MIN_CADENCE)
		{
			pedaling = true;
		}
		else
		{
			if(pedaling == true)
			{
				lastTimePedaling = millis();
			}
			pedaling = false;
			//pas_factor = 0;
		}
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
  {		//fallende Flanke -> wurde gerade gedrueckt
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
		  {//fallende Flanke -> wurde gerade gedrueckt
			  taster2_edges++;
		  }
		  taster2_state = temp_bool;
	  }

  // 3W Rot-Taster abfragen
temp_int = digitalRead(INPUT_3W_SW_RED);

  // 3W-Green-Taster abfragen
  temp_bool = digitalRead(INPUT_3W_SW_GREEN);
  if(temp_bool == false)
  {
	  if(switchGreen_state == true)
	  {//fallende Flanke -> wurde gerade gedrueckt
		  switchGreen_edges++;
		  switchGreen_state = temp_bool;
	  }
  }
}

// Batteriespannung auslesen und umrechnen
void readBattVolt()
{
  temp_int = analogRead(INPUT_BATSENSE);
  voltageArdu = (float)((temp_int*REFVOLT/1024.0)*(RUPPER+RLOWER)/RLOWER);
}

// 1ms-Timer-ISR
/*void timer1_ISR()
{
	// Variable fuer zweiten und dritten Timer hochzaehlen
	fastTimerVariable++;
	slowTimerVariable++;
	ultraslowTimerVariable++;

	//10ms-Timer:
  if (fastTimerVariable >= FAST_TIMER)
  {

  }

  //100ms-Timer
  if(slowTimerVariable >= SLOW_TIMER)
	{

	}

  //1s-Timer
  if(ultraslowTimerVariable >= ULTRA_SLOW_TIMER)
	{

	}
}*/

void calculateSOC()
{
	/*Wertetabelle fuer SOC Berechnung
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
	avgCellVolt = voltageVesc/numberOfCells;
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

// Das ist der Quellcode aus der Adafruit Library
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
		  //display.print(" 15.0");
		  display.println("km/h");

		  //Zeile 4:
		  //Leistung ausgeben:
		  //display.print("P=");
		  display.print("100");
		  display.print("W");
		  //Unterstuetzungsstufe ausgeben:
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

		  //Unterstuetzungsstufe ausgeben:
		  display.print("Stufe ");
		  display.println(throttleControl.aktStufe);

		  //scheinbar kann ich hier max. 5 buchstaben schreiben?!?! wtf

	  }
	  display.display();
}*/

#ifdef DISPLAY_CONNECTED
void refreshu8x8Display()
{
	  //u8x8.clearDisplay();

	  if(vesc_connected)
	  {

		  if(displayCounter == 0)
			  {
			  u8x8.home();
			  //Zeile 1:
			  //Batteriespannung ausgeben
			  u8x8.clearLine(0);
			  u8x8.clearLine(1);
			  u8x8.print(UART.data.inpVoltage);
			  u8x8.print("V ");
			  //SOC Ausgeben
			  u8x8.print(SOC);
			  u8x8.println("%");
			  }

		  if(displayCounter == 1)
		  {
			  u8x8.clearLine(2);
			  u8x8.clearLine(3);
			  //Pedaling?
			  if(pedaling)
			  {
				  u8x8.drawString(0,2,"pedaling");
			  }
			  else
			  {
				  u8x8.drawString(0,2,"not pedaling");
			  }
		  }

		  else if (displayCounter == 2)
		  {
			  u8x8.clearLine(4);
			  u8x8.clearLine(5);
			  u8x8.setCursor(0,4);
			  u8x8.print(pas_factor);
			  u8x8.print(" ");
			  u8x8.print(velocity);
			  u8x8.println("km/h");

		  }

		  else if (displayCounter == 3)
		  {
			  u8x8.clearLine(6);
			  u8x8.clearLine(7);
			  //Unterstuetzungsstufe ausgeben:
			  u8x8.print("Stufe ");
			  u8x8.println(throttleControl.aktStufe);
		  }

		  //Zeile 2:
		  //MOTOR Strom ausgeben
		  //display.print("Im=");
		  /*u8x8.print(UART.data.avgMotorCurrent);
		  u8x8.print("A ");^
		  //Akkustrom ausgeben:
		  //display.print(" Ibat=");
		  u8x8.print(UART.data.avgInputCurrent);
		  u8x8.println("A");*/

		  //Zeile 3:
		  //ERPM ausgeben:
		  /*u8x8.print((int)UART.data.rpm);
		  u8x8.println(" RPM ");
		  //Geschwindigkeit Ausgeben:
		  u8x8.print(velocity);
		  u8x8.println("km/h");*/
	  }

	  else{
		  if(displayCounter == 0)
		  {
			  u8x8.home();
			  //Zeile 1:
			  //Batteriespannung ausgeben
			  u8x8.print(voltageArdu);
			  u8x8.print("V  ");
			  //SOC Ausgeben
			  u8x8.print(SOC);
			  u8x8.print("%");
		  }

		  else if (displayCounter == 1)
		  {
			  //Pedaling?
			  if(pedaling)
			  {
				  u8x8.clearLine(2);
				  u8x8.clearLine(3);
				  u8x8.drawString(0,2,"pedaling");
			  }
			  else
			  {
				  u8x8.drawString(0,2,"not pedaling");
			  }
		  }

		  else if (displayCounter == 2)
		  {
			  u8x8.setCursor(0,4);
			  u8x8.println(pas_factor);
		  }

		  else if (displayCounter == 3)
		  {
			  //Unterstuetzungsstufe ausgeben:
			  u8x8.print("Stufe ");
			  u8x8.println(throttleControl.aktStufe);
		  }

	  }
	  u8x8.display();
	  if(displayCounter < 3)
	  {
		  displayCounter++;

	  }
	  else
	  {
		  displayCounter = 0;
	  }
}
#endif

void setup() {
  // put your setup code here, to run once:

	// Pin A4 als Ausgang einstellen (fuer I2C)
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
	//3W-Switch-Green-Eingang:
	pinMode(INPUT_3W_SW_GREEN, INPUT_PULLUP);

	//Throttle PWM-Output:
	pinMode(OUTPUT_THROTTLE, OUTPUT);

	//fuer Display:
	//Display-Versorgung einschalten:
	pinMode(OUTPUT_DISPLAY_SUPPLY, OUTPUT);
	digitalWrite(OUTPUT_DISPLAY_SUPPLY, LOW);

	//Licht-Ausgang konfigurieren (und gleich anschalten)
	pinMode(OUTPUT_LIGHT, OUTPUT);
	digitalWrite(OUTPUT_LIGHT, HIGH);

	//LED auf ArduinoNano Pin 13
	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);

	// initialize with the I2C addr 0x3C / mit I2C-Adresse 0x3c initialisieren
	//display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

#ifdef DISPLAY_CONNECTED
    u8x8.begin();

    //u8x8.setFont(u8x8_font_chroma48medium8_r);
    u8x8.setFont(u8x8_font_8x13_1x2_r);
#endif


	/* millis() Returns the number of milliseconds passed since the Arduino board
	began running the current program. This number will overflow (go back to zero), after approximately 50 days. */
	milliseconds = millis();

	// Interrupt fuer PAS konfigurieren
	attachInterrupt(digitalPinToInterrupt(INPUT_PAS), pas_ISR, CHANGE);

	// falling-edge interrupt fuer Taster konfigurieren	-> TODO
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

	//wenn kein Vesc dran huengt serielle Schnittstelle beenden
	if(vesc_connected == false)
	{
		Serial.end();
	}
	else
	{	//VESC ist dran
		// Zellspannung einmal abfragen fuer 6s-9s-Erkennung:
		//readBattVolt();
		if (UART.data.inpVoltage > BAT6S9S_GRENZE)
		{
			numberOfCells = 9;
			undervoltageThreshold = UNDERVOLTAGE_9S;
		}
		else
		{
			numberOfCells = 6;
			undervoltageThreshold = UNDERVOLTAGE_6S;
		}
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
		if(pedaling == true)
		{
			lastTimePedaling = millis();
		}
		pedaling = false;
		pas_factor = 0;
	}

	// Schnelle Routine
	if((millis()-lastFastLoop) > FAST_TIMER)
	//if(fastTimerFlag)
	{
		lastFastLoop = millis();
		fastTimerFlag = false;


		//taster1_state = digitalRead(INPUT_TASTER1);
		//taster2_state = digitalRead(INPUT_TASTER2);
		temp_int = digitalRead(INPUT_3W_SW_RED);
		if(temp_int == 0 && switchRed_state == 1)		//fallende Flanke erkannt
		{
			if(switchRedGreen_ready == false)		//Tastereingaben nur auswerten wenn das Zeitfenster noch nicht rum ist
			{
				switchRed_edges++;
			}
			switchRedGreen_lastEvent = millis();
		}
		switchRed_state = temp_int;


		temp_int = digitalRead(INPUT_3W_SW_GREEN);
		if(temp_int == 0 && switchGreen_state == 1)		//fallende Flanke erkannt
		{
			if(switchRedGreen_ready == false)
			{
				switchGreen_edges++;
			}
			switchRedGreen_lastEvent = millis();
		}
		switchGreen_state = temp_int;

		if(millis()-switchRedGreen_lastEvent > TASTER_TIME_WINDOW && (switchRed_edges>0 || switchGreen_edges >0))
		{
			switchRedGreen_ready = true;
		}
		else
		{
			switchRedGreen_ready = false;
		}

		//Taster auslesen:
		//Taster1 abfragen
	  //taster1_state = digitalRead(INPUT_TASTER1);


	  // Licht-Taster abfragen
		//test2 = digitalRead(INPUT_TASTER2);
		  //taster2_state = digitalRead(INPUT_TASTER2);
	}


	// langsame Routine
  if((milliseconds - lastSlowLoop) > SLOW_TIMER)
	//if(slowTimerFlag)
  {
	  slowTimerFlag = false;
	  lastSlowLoop = millis();

	  	  //LED blinken lassen zum Anzeigen ob noch aktiv
		if(digitalRead(LED)==HIGH)
		{
			digitalWrite(LED, LOW);
		}
		else
		{
			digitalWrite(LED, HIGH);
		}


	  //Taster auswerten:
		if(switchRedGreen_ready == true)
		{
			if(switchRed_edges == 1 && switchGreen_edges == 4)
			{
				pipapo = true;
			}
			else
			{
				  if(switchRed_edges>0)
				  {
					  //Unterstuetzung hochschalten
					  throttleControl.aktStufe = throttleControl.aktStufe+switchRed_edges;
					  if(throttleControl.aktStufe>5)
					  {
						  throttleControl.aktStufe = 5;
					  }
				  }
				  if(switchGreen_edges>0)
				  {
					  //Unterstuetzung runterschalten
					  throttleControl.aktStufe = throttleControl.aktStufe-switchGreen_edges;

					  if(throttleControl.aktStufe <=0)
					  {
						  throttleControl.aktStufe = 0;
						  pipapo = false;
					  }
				  }
			}
			switchRed_edges=0;
			switchGreen_edges=0;
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
		  voltageVesc = UART.data.inpVoltage;
		  //wenn die Eingangsspannung unter die Grenze sinkt -> Unterstützungsstufe zurückschalten

		  //Undervoltage-Regulator

		  if(voltageVesc < undervoltageThreshold)
		  {
			  if(throttleControl.aktStufe > 0)
			  {
				  throttleControl.aktStufe--;
			  }
		  }
		  velocity = UART.data.rpm*RADUMFANG*60/MOTOR_POLE_PAIRS/MOTOR_GEAR_RATIO/1000;
		  if(pedaling == true)
		  {
			  temp_float = (float)stufenI[throttleControl.aktStufe];

			//Geschwindigkeitsgrenze:
			 if(UART.data.rpm > 0.9*MAX_ERPM && pipapo ==false)
			 {
				 temp_float = (int)((1.0-(UART.data.rpm-0.9*MAX_ERPM)/(0.2*MAX_ERPM))*temp_float);
				 /*if(temp_float < 0.0)
				 {
					 temp_float = 0.0;
				 }*/
			 }
			 if(temp_float <= 0.0)
			 {
				 throttleControl.current = 0.0;
			 }
			 else if(temp_float <= throttleControl.current)
			 {
				 throttleControl.current = throttleControl.current - 1.0;
			 }
			 else
			 {
				 // Rampe fahren beim Strom erhoehen
				 throttleControl.current = throttleControl.current + 1.0;
			 }
	  	  }
		  else
		  {
			  throttleControl.current = 0.0;
		  }
		  //Stromsollwert an VESC geben
		  UART.setCurrent(throttleControl.current);
	  }
  }

	// ultra-langsame Routine
	if((milliseconds - lastUltraSlowLoop) > ULTRA_SLOW_TIMER)
	//if(ultraslowTimerFlag)
	{
		ultraslowTimerFlag = false;

#ifdef DISPLAY_CONNECTED
		refreshu8x8Display();
#endif

		lastUltraSlowLoop = millis();
		calculateSOC();

		// bei bedarf Unterstuetzung auf default zurückstellen
		/*if(pedaling == false && (millis()-lastTimePedaling) > TIME_TO_RESET_AFTER_PEDAL_STOP)
		{
			throttleControl.aktStufe = DEFAULT_STUFE;
		}*/
	}
}
