/* letzte Aenderung: 22.01.2020: Pedal-Erkennung funktioniert
 * 21.01.2020: Display-Ansteuerung Funktioniert
   V1.00: Neue Version fuer neue Hardware
	nur Pin 2 und 3 koennen flanken-interrupts */

#include "config.h"
#include "Arduino.h"

#include "MemoryFree.h"
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
//#define CTRLMODE VESC
//#define VESC 1
//#define KU63 0

//  Spannungsteiler Spannungssensierung:
// 	Nominalwerte:
//  Rupper=22k
//  Rlower=2k2
#define RUPPER 22.0
#define RLOWER 2.2
#define REFVOLT 5.00

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

// 10ms Timer
#define FAST_TIMER 10

//100ms Timer
#define SLOW_TIMER 100

#define ULTRA_SLOW_TIMER 250

// Entprellzeit für PAS (und evtl. Taster) in ms
#define ENTPRELLZEIT 5

#define MAX_STROMSTEIGUNG	1.0

//fuer Display
//#define DRAW_DELAY 118
//#define D_NUM 47

//Display Constructor
#ifdef DISPLAY_CONNECTED
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(A5, A4);
#endif

//const float stufenV [6] = {0.0, STUFE1_V, STUFE2_V, STUFE3_V, STUFE4_V, STUFE5_V};
uint8_t stufenI [ANZAHL_STUFEN+1] = {0, STUFE1_I, STUFE2_I, STUFE3_I, STUFE4_I};

unsigned long milliseconds;

uint16_t notPedalingCounter;

uint8_t resetFlagRegister = 0;

struct batteryDataStruct
{
	uint8_t numberOfCells = 9;
	float voltageArdu = 0.0;
	//float voltageVesc = 0.0;
	float avgCellVolt = 0.0;
	uint8_t SOC = 0;
	int16_t batteryPower = 0.0;
};


struct undervoltageRegStruct
{
	//fuer Undervoltage-Regler
	float reg_out=0;
	float reg_out_int=0;
	float reg_out_filtered=0;
	float reg_diff;
};

struct speedRegStruct
{
	//fuer Geschwindigkeits-Regler
	//const uint8_t vmax = MAX_SPEED_KMH;
	//const uint8_t vgrenz = VGRENZ;
	//const uint16_t erpm_max = MAX_ERPM;
	//const uint16_t erpm_grenz = ERPM_GRENZ;
	//uint16_t vreg_diff = 0;
	float vreg_out = 0.0;
	float vreg_out_int = 0.0;
	float vreg_out_filtered = 0.0;
	float velocity = 0.0;
};

speedRegStruct speedReg;

batteryDataStruct batteryData;
undervoltageRegStruct undervoltageReg;

//int temp_int;

uint8_t displayCounter;

uint8_t vescConnectionErrors = 0;

bool vesc_connected = false;

bool headlight_on = true;

bool pipapo = false;

float undervoltageThreshold;

struct throttleControlStruct
{
  int8_t aktStufe = DEFAULT_STUFE;
  float current_next = 0.0;
  float current_now = 0.0;
  float throttleVoltage = 0.0;
};

//Variables for User-Inputs

struct tasterStruct
{
	uint8_t state = 0;
	bool ready = false;
	bool edge_detected = false;
	uint8_t edges = 0;
	uint32_t lastEvent = 0;
};

tasterStruct taster1;
tasterStruct taster2;

//tasterStruct tasterUp;
//tasterStruct tasterDown;

	uint8_t switchRed_state = false;
	bool switchRedGreen_ready = false;
	bool seitchRed_edge_detected = false;
	uint8_t switchRed_edges = 0;
	uint32_t switchRedGreen_lastEvent = 0;

	uint8_t switchGreen_state = false;
	//bool switchGreen_ready = false;
	bool switchGreen_edge_detected = false;
	uint8_t switchGreen_edges = 0;
	//long int switchGreen_lastEvent = 0;


//Variables for Timer
	//int slowTimerVariable = 0;
	uint32_t lastSlowLoop;
	uint32_t lastUltraSlowLoop;
	uint32_t lastFastLoop;
	bool slowTimerFlag = false;
	//int fastTimerVariable = 0;
	bool fastTimerFlag = false;
	//int ultraslowTimerVariable = 0;
	bool ultraslowTimerFlag = false;

	struct pasStruct
	{
		//Variables for PAS
		//const uint8_t pas_allowed_fails = 1;
		const bool doubleHall = DOUBLE_HALL;
		uint16_t pasTimeHigh = 0;
		uint16_t pasTimeLow = 0;
		uint16_t pasTimeGesamt = 0;
		uint32_t last_pas_event = 0;
		bool pas_status = false;
		uint16_t pas_factor = 0;
		uint16_t pas_factor_filtered = 0;
		uint16_t pas_factor_int = 0;
		//uint16_t cadence = 0;
		bool pedaling = false;

	};

	pasStruct pasData;

// Anlegen der benuetigten Structs
throttleControlStruct throttleControl;

//ISR fuer PAS
void pas_ISR()
{
	if(pasData.last_pas_event > (millis()-ENTPRELLZEIT))
	{
		//Entprellung
		return;
	}
	pasData.pas_status = digitalRead(INPUT_PAS);
	if(pasData.pas_status == true)
	{
		pasData.pasTimeLow = millis()-pasData.last_pas_event;
	}
	else
	{
		pasData.pasTimeHigh = millis()-pasData.last_pas_event;
	}
	pasData.last_pas_event = millis();

	pasData.pasTimeGesamt = pasData.pasTimeLow+pasData.pasTimeHigh;
	//TODO: Diese Division könnte man eigentlich vermeiden
	//pasData.cadence = CONV_PAS_TIME_TO_CADENCE/(pasData.pasTimeLow+pasData.pasTimeHigh);

	if(pasData.doubleHall == false)
	{	// wenn man keinen Double-Hall-Sensor hat muss man den PAS-Faktor mit einbeziehen
		if(pasData.pasTimeLow == 0)
		{
			// um keine division durch 0 zu machen!
			pasData.pasTimeLow = 1;
		}
		pasData.pas_factor = 100*pasData.pasTimeHigh/pasData.pasTimeLow;
			if(pasData.pas_factor > 200)
			{
				pasData.pas_factor = 200;
			}

		// PT1-Filterung des PAS-Faktors:
		pasData.pas_factor_int +=(pasData.pas_factor-pasData.pas_factor_filtered);
		pasData.pas_factor_filtered = pasData.pas_factor_int >>1;

		if(pasData.pasTimeGesamt <= PAS_TIMEOUT && pasData.pas_factor_filtered > PAS_FACTOR_MIN)
		{
			pasData.pedaling = true;
		}
		else
		{
			pasData.pedaling = false;
		}
	}
	else
	{
		// mit Double-Hall-Sensor muss man nur die Cadence überprüfen
		//if(pasData.cadence>=MIN_CADENCE)
		//statt der kadenz kann man auch die PAS-Time überprüfen, dadurch spart man sich die Berechnung der Kadenz (division)
		if(pasData.pasTimeGesamt <= PAS_TIMEOUT)
		{
			pasData.pedaling = true;
		}
		else
		{
			pasData.pedaling = false;
		}
	}
}


void checkTaster()
{
	//taster1_state = digitalRead(INPUT_TASTER1);
	//taster2_state = digitalRead(INPUT_TASTER2);

	uint8_t temp = digitalRead(INPUT_3W_SW_RED);
	if(temp == 0 && switchRed_state == 1)		//fallende Flanke erkannt
	{
		if(switchRedGreen_ready == false)		//Tastereingaben nur auswerten wenn das Zeitfenster noch nicht rum ist
		{
			switchRed_edges++;
		}
		switchRedGreen_lastEvent = millis();
	}
	switchRed_state = temp;


	//Pin auslesen
	temp = digitalRead(INPUT_3W_SW_GREEN);
	if(temp == 0 && switchGreen_state == 1)		//fallende Flanke erkannt
	{
		switchGreen_edge_detected = true;
		/*if(switchRedGreen_ready == false)
		{
			switchGreen_edges++;
		}
		switchRedGreen_lastEvent = millis();*/
	}

	//keine Flanke erkannt aber noch auf low-level -> flanke bestätigen (Entprellung)
	else if (temp == 0 && switchGreen_state == 0)
	{
		if(switchGreen_edge_detected == true && switchRedGreen_ready == false)
		{
			switchGreen_edges++;
			switchRedGreen_lastEvent = millis();
			switchGreen_edge_detected = false;
		}
	}
	else if(switchGreen_edge_detected == true)
	{
		switchGreen_edge_detected = false;
	}
	switchGreen_state = temp;

	if(millis()-switchRedGreen_lastEvent > TASTER_TIME_WINDOW && (switchRed_edges>0 || switchGreen_edges >0))
	{
		switchRedGreen_ready = true;
	}
	else
	{
		switchRedGreen_ready = false;
	}
}

void interpretInputs()
{
  //Taster auswerten:
	if(switchRedGreen_ready == true)
	{
		if(switchRed_edges == 1 && switchGreen_edges == 4)
		{
			pipapo = !pipapo;
		}
		else if (switchRed_edges==2 && switchGreen_edges == 1)
		{
			if(headlight_on == true)
			{
				headlight_on = false;
				digitalWrite(OUTPUT_LIGHT, LOW);
			}
			else
			{
				headlight_on = true;
				digitalWrite(OUTPUT_LIGHT, HIGH);
			}
		}
		else
		{
			  if(switchRed_edges>0)
			  {
				  //Unterstuetzung hochschalten
				  throttleControl.aktStufe = throttleControl.aktStufe+switchRed_edges;
				  if(throttleControl.aktStufe>ANZAHL_STUFEN)
				  {
					  throttleControl.aktStufe = ANZAHL_STUFEN;
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
}

// Batteriespannung auslesen und umrechnen
void readBattVoltArdu()
{
  int temp = analogRead(INPUT_BATSENSE);
  batteryData.voltageArdu = (float)((temp*REFVOLT/1024.0)*(RUPPER+RLOWER)/RLOWER);
}


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
	if(vesc_connected)
	{
		//batteryData.avgCellVolt = batteryData.voltageVesc/batteryData.numberOfCells;
		batteryData.avgCellVolt = UART.data.inpVoltage/batteryData.numberOfCells;
	}
	else
	{
		batteryData.avgCellVolt = batteryData.voltageArdu/batteryData.numberOfCells;
	}
	int temp = (int)(batteryData.avgCellVolt*1000.0);
	if(temp > 4080)
	{
		batteryData.SOC = 100;
	}
	else if (temp >4060)
	{
		batteryData.SOC = 95;
	}
	else if (temp >4000)
	{
		batteryData.SOC = 90;
	}
	else if (temp >3970)
	{
		batteryData.SOC = 85;
	}
	else if (temp >3940)
	{
		batteryData.SOC = 80;
	}
	else if (temp >3920)
	{
		batteryData.SOC = 75;
	}
	else if (temp >3880)
	{
		batteryData.SOC = 70;
	}
	else if (temp >3850)
	{
		batteryData.SOC = 65;
	}
	else if (temp >3830)
	{
		batteryData.SOC = 60;
	}
	else if (temp >3820)
	{
		batteryData.SOC = 55;
	}
	else if (temp >3800)
	{
		batteryData.SOC = 50;
	}
	else if (temp >3790)
	{
		batteryData.SOC = 45;
	}
	else if (temp >3780)
	{
		batteryData.SOC = 40;
	}
	else if (temp >3770)
	{
		batteryData.SOC = 35;
	}
	else if (temp >3760)
	{
		batteryData.SOC = 30;
	}
	else if (temp >3750)
	{
		batteryData.SOC = 25;
	}
	else if (temp >3740)
	{
		batteryData.SOC = 20;
	}
	else if (temp >3720)
	{
		batteryData.SOC = 15;
	}
	else if (temp >3690)
	{
		batteryData.SOC = 10;
	}
	else if (temp >3680)
	{
		batteryData.SOC = 5;
	}
	else
	{
		batteryData.SOC = 0;
	}
}

void setThrottlePWM()
{
  analogWrite(OUTPUT_THROTTLE, throttleControl.throttleVoltage/5*255.0);
}


#ifdef DISPLAY_CONNECTED
void refreshu8x8Display()
{
	  if(displayCounter == 0)
	  {
		  u8x8.home();
		  u8x8.clearLine(0);
		  u8x8.clearLine(1);
		  //Zeile 1:
		  u8x8.print(F("FreeRAM: "));
		  u8x8.print(freeMemory());
	  }

	  else if (displayCounter == 1)
	  {
		  // Zeile 2:
		  u8x8.clearLine(2);
		  u8x8.clearLine(3);
		  u8x8.setCursor(0,2);
		  if(vesc_connected)
		  {
			  u8x8.print((int)UART.data.inpVoltage);
		  }
		  else
		  {
			  u8x8.print((int)batteryData.voltageArdu);
		  }

		  u8x8.print(F("V  "));
		  //SOC Ausgeben
		  u8x8.print(batteryData.SOC);
		  u8x8.print(F("% "));

		  u8x8.print((int)(UART.data.ampHours*1000));
		  u8x8.print(F("mAh"));

		  }

	  else if (displayCounter == 2)
	  {
		  //Zeile 3:
		  u8x8.clearLine(4);
		  u8x8.clearLine(5);
		  u8x8.setCursor(0,4);
		  //Unterstuetzungsstufe ausgeben:
		  u8x8.print(F("Stufe "));
		  u8x8.print(throttleControl.aktStufe);
		  //u8x8.print(" ");
		  if(pasData.pedaling == true)
		  {
			  u8x8.print(F(" ON"));
		  }
		  else
		  {
			  u8x8.print(F(" OFF"));
		  }
	  }

	  else if (displayCounter == 3)
	  {
		  //Zeile 4:
		  u8x8.clearLine(6);
		  u8x8.clearLine(7);
		  u8x8.setCursor(0,6);
		  u8x8.print(vescConnectionErrors);

		  /*u8x8.print((int)speedReg.velocity);
		  u8x8.print(F("km/h "));

		  u8x8.print((int)UART.data.avgInputCurrent);
		  u8x8.print(("A"));*/

		  /*if (resetFlagRegister & _BV(EXTRF))
		  {
		      // Reset button or otherwise some software reset
		      u8x8.print(F("Reset button"));
		  }
		  if (resetFlagRegister & (_BV(BORF) | _BV(PORF)))
		  {
		       // Brownout or Power On
		       u8x8.print(F("Power loss"));
		  }
		  if (resetFlagRegister & _BV(WDRF)){
		       //Watchdog Reset
		       u8x8.print(F("Watchdog"));
		  }
		  else
		  {
			  u8x8.print(resetFlagRegister);
		  }*/


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

void setup()
{
  // put your setup code here, to run once:

	//resetFlagRegister = MCUSR;
	 // Clear all MCUSR registers immediately for 'next use'
	MCUSR = 0;

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

	//Licht-Ausgang konfigurieren
	pinMode(OUTPUT_LIGHT, OUTPUT);
	//Licht anschalten
	digitalWrite(OUTPUT_LIGHT, HIGH);
	headlight_on = true;

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
	delay(1000);

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

	//wenn kein Vesc dran haengt serielle Schnittstelle beenden
	if(vesc_connected == false)
	{
		Serial.end();
	}
	else
	{	//VESC ist dran
		// Zellspannung einmal abfragen fuer 6s-9s-Erkennung:
		if (UART.data.inpVoltage > BAT6S9S_GRENZE)
		{
			batteryData.numberOfCells = 9;
			undervoltageThreshold = UNDERVOLTAGE_9S;
		}
		else
		{
			batteryData.numberOfCells = 6;
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
	if ((millis() - pasData.last_pas_event) > PAS_TIMEOUT)
	{
		pasData.pedaling = false;
		pasData.pas_factor = 0;
		pasData.pas_factor_filtered = 0;
	}
	//pasData.cadence = CONV_PAS_TIME_TO_CADENCE/(pasData.pasTimeLow+pasData.pasTimeHigh);

	// Schnelle Routine -> hier werden nur die Taster ausgelesen
	if((millis()-lastFastLoop) > FAST_TIMER)
	//if(fastTimerFlag)
	{
		lastFastLoop = millis();
		fastTimerFlag = false;

		checkTaster();
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

		interpretInputs();

	  if(vesc_connected)
	  {
		  uint8_t vescErrorsTemp = vescConnectionErrors;
		  //3mal versuchen auszulesen
		//for(int i = 0; i<3; i++)
		//{
			if(UART.getVescValues())
			{
				batteryData.batteryPower = UART.data.inpVoltage * UART.data.avgInputCurrent;

				//wenn die Eingangsspannung unter die Grenze sinkt -> Unterstützungsstufe zurückschalten
				/*if(batteryData.voltageVesc < undervoltageThreshold)
				{
					if(throttleControl.aktStufe > 0)
					{
						throttleControl.aktStufe--;
					}
				}*/

				 //Geschwindigkeit berechnen aus ausgelesener RPM
				speedReg.velocity = UART.data.rpm*RADUMFANG*60/MOTOR_POLE_PAIRS/MOTOR_GEAR_RATIO/1000;
				//speedReg.rpm = UART.data.rpm;

				// Geschwindigkeitsregler
				speedReg.vreg_out = 1.0-(float)((speedReg.velocity-VGRENZ)/(MAX_SPEED_KMH - VGRENZ));
				//Regeldifferenz:
				//speedReg.vreg_out = speedReg.erpm_max - speedReg.rpm;
				//speedReg.vreg_out = speedReg.vreg_out/(float)((speedReg.erpm_max-speedReg.erpm_grenz));
				  if(speedReg.vreg_out >1.0)
				  {
					  speedReg.vreg_out = 1.0;
				  }
				  else if (speedReg.vreg_out < 0.0)
				  {
					  speedReg.vreg_out = 0.0;
				  }

				  //PT1-Filterung des Reglerausgangs:
				  speedReg.vreg_out_int += (speedReg.vreg_out - speedReg.vreg_out_filtered);
				  speedReg.vreg_out_filtered = speedReg.vreg_out_int / 8;


				  //Unterspannungs-regler:
				  undervoltageReg.reg_diff = UART.data.inpVoltage - undervoltageThreshold;
				  undervoltageReg.reg_out = undervoltageReg.reg_diff;
				  if(undervoltageReg.reg_out > 1.0)
				  {
					  undervoltageReg.reg_out = 1.0;
				  }
				  else if (undervoltageReg.reg_out < 0.0)
				  {
					  undervoltageReg.reg_out = 0.0;
				  }

				  // Filterung des Reglerausgangs
				  undervoltageReg.reg_out_int = undervoltageReg.reg_out_int + undervoltageReg.reg_out - undervoltageReg.reg_out_filtered;
				  undervoltageReg.reg_out_filtered = undervoltageReg.reg_out_int / 20;

				//break;
			}
			//wenns beim dritten Mal nicht geklappt hat stimmt wohl was nicht
			//else if(i>=2)
				  else
			{
				vescConnectionErrors++;
				//vesc_connected = false;
				//Serial.end();
				//digitalWrite(LED, LOW);
			}
		//}
			// wenn ein Fehler aufgetreten ist bei der Übertragung starte ich einfach die Schnittstelle neu
		if(vescErrorsTemp != vescConnectionErrors)
		{
			Serial.end();
			Serial.begin(VESC_BAUDRATE);

			// aus VESC-ARduino Beispiel, ich vermute zum abwarten bis serielle Schnittstelle bereit
			while (!Serial) {;}
			UART.setSerialPort(&Serial);
		}

		  if(pasData.pedaling == true)
		  {
			  notPedalingCounter = 0;
			  throttleControl.current_next = (float)stufenI[throttleControl.aktStufe];

			//Geschwindigkeitsgrenze:
			  if(pipapo == false)
			  {
				  throttleControl.current_next = throttleControl.current_next*speedReg.vreg_out_filtered;
			  }

			  // Unterspannungsgrenze
			  throttleControl.current_next = throttleControl.current_next * undervoltageReg.reg_out_filtered;

			 if(throttleControl.current_next <= 0.0)
			 {
				 throttleControl.current_next = 0.0;
			 }

			 //Maximale Stromsteigung beachten
			 else if((throttleControl.current_next - throttleControl.current_now) > MAX_STROMSTEIGUNG )
			 {
				 throttleControl.current_next = throttleControl.current_now + MAX_STROMSTEIGUNG;
			 }
			 //Maximale negative Stromsteigung beachten
			 else if ((throttleControl.current_now - throttleControl.current_next) > MAX_STROMSTEIGUNG)
			 {
				 throttleControl.current_next = throttleControl.current_now - 10;
			 }
	  	  }

		  //not pedaling
		  else
		  {
			  throttleControl.current_next = 0.0;
			  notPedalingCounter++;
		  }

		  UART.setCurrent(throttleControl.current_next);
		  throttleControl.current_now = throttleControl.current_next;

	  }
	  else
	  {
		  readBattVoltArdu();
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

		// bei bedarf Unterstuetzung auf default zurückstellen nach gewisser Zeit
		/*if(notPedalingCounter >= TIME_TO_RESET_AFTER_PEDAL_STOP && throttleControl.aktStufe > DEFAULT_STUFE)
		{
			throttleControl.aktStufe = DEFAULT_STUFE;
		}*/
	}
}
