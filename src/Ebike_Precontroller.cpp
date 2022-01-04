
#include <Arduino.h>
#include "config.h"

#include "MemoryFree.h"

#include <avr/wdt.h>

//  Spannungsteiler Spannungssensierung:
// 	Nominalwerte:
//  Rupper=12k
//  Rlower=1.2k
#define RUPPER 12000.0
#define RLOWER 1215.0
#define REFVOLT 4.94

//Pins fuer Ein- und Ausgaenge
#define INPUT_PAS 2
#define INPUT_TASTER1 3
//#define INPUT_TASTER2 4		// -> Taster2 ueblicherweise fuer Licht?
#define INPUT_3W_SW_RED 6
#define INPUT_3W_SW_GREEN 7
#define OUTPUT_THROTTLE 5
#define INPUT_BATSENSE 4

// Pin-Nummer der LED auf Arduino Nano
#define LED 13

#define SERIAL_MONITOR_BAUDRATE 9600

// 10ms Timer
#define FAST_TIMER 10

//100ms Timer
#define SLOW_TIMER 100

#define ULTRA_SLOW_TIMER 1000

// Entprellzeit f�r PAS (und evtl. Taster) in ms
#define ENTPRELLZEIT 5

unsigned long milliseconds;

uint16_t notPedalingCounter;

uint8_t resetFlagRegister = 0;

struct batteryDataStruct
{
	uint8_t numberOfCells = 9;
	float voltageArdu = 0.0;
	float avgCellVolt = 0.0;
	uint8_t SOC = 0;
	int16_t batteryPower = 0.0;
	float undervoltageThreshold;
};


struct undervoltageRegStruct
{
	float reg_out_int=0;
	float reg_out_filtered=0;
};

struct speedRegStruct
{
	float vreg_out_int = 0.0;
	float vreg_out_filtered = 0.0;
	float velocity = 0.0;
};


speedRegStruct speedReg;
batteryDataStruct batteryData;
undervoltageRegStruct undervoltageReg;

uint8_t displayRowCounter = 0;

int16_t minFreeRAM = 2000;		// initialisiert mit 2000. wird verringert wenn der echte freie RAM weniger ist

bool pipapo = false;


struct throttleControlStruct
{
  int8_t aktStufe = DEFAULT_STUFE;
  float throttleVoltage = 0.0;
  bool throttleON = false;
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
//tasterStruct taster2;

//tasterStruct tasterUp;
//tasterStruct tasterDown;

	uint8_t switchRed_state = false;
	bool switchRedGreen_ready = false;
	bool switchRed_edge_detected = false;
	uint8_t switchRed_edges = 0;
	uint32_t switchRedGreen_lastEvent = 0;

	uint8_t switchGreen_state = false;

	bool switchGreen_edge_detected = false;
	uint8_t switchGreen_edges = 0;



//Variables for Timer
	uint32_t lastSlowLoop;
	uint32_t lastUltraSlowLoop;
	uint32_t lastFastLoop;


	struct pasStruct
	{
		//Variables for PAS
		uint16_t pasTimeHigh = 0;
		uint16_t pasTimeLow = 0;
		uint16_t pasTimeGesamt = 0;
		uint32_t last_pas_event = 0;
		bool pas_status = false;
		uint16_t pas_factor = 0;
		uint16_t pas_factor_filtered = 0;
		uint16_t pas_factor_int = 0;
		bool pedaling = false;
	};

	pasStruct pasData;

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

	if(DOUBLE_HALL == false)
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

		/*if(pasData.pasTimeGesamt <= PAS_TIMEOUT && pasData.pas_factor_filtered > PAS_FACTOR_MIN)
		{
			pasData.pedaling = true;
		}*/
		if(pasData.pasTimeGesamt <= PAS_TIMEOUT && pasData.pas_factor_filtered < PAS_FACTOR_MAX)
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
		// mit Double-Hall-Sensor muss man nur die Cadence �berpr�fen
		//if(pasData.cadence>=MIN_CADENCE)
		//statt der kadenz kann man auch die PAS-Time �berpr�fen, dadurch spart man sich die Berechnung der Kadenz (division)
		if(pasData.pasTimeGesamt <= PAS_TIMEOUT)
		{
			pasData.pedaling = true;
		}
		else
		{
			pasData.pedaling = false;
		}
	}
	if(freeMemory() < minFreeRAM)
	{
		minFreeRAM = freeMemory();
	}
}


void checkTaster()
{
	// UP/DOWN Schalter auswerten:
	uint8_t temp = digitalRead(INPUT_3W_SW_RED);
	if(temp == 0 && switchRed_state == 1)		//fallende Flanke erkannt
	{
		switchRed_edge_detected = true;
	}

	//keine Flanke erkannt aber noch auf low-level -> flanke best�tigen (Entprellung)
	else if (temp == 0 && switchRed_state == 0)
	{
		if(switchRed_edge_detected == true && switchRedGreen_ready == false)
		{
			switchRed_edges++;
			switchRedGreen_lastEvent = millis();
			switchRed_edge_detected = false;
		}
	}
	else if(switchRed_edge_detected == true)
	{
		switchRed_edge_detected = false;
	}
	switchRed_state = temp;


	//Pin auslesen
	temp = digitalRead(INPUT_3W_SW_GREEN);
	if(temp == 0 && switchGreen_state == 1)		//fallende Flanke erkannt
	{
		switchGreen_edge_detected = true;
	}

	//keine Flanke erkannt aber noch auf low-level -> flanke best�tigen (Entprellung)
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

	// Taster 1 auswerten:
	temp = digitalRead(INPUT_TASTER1);
	if(temp == 0 && taster1.state == 1)	//fallende Flanke erkannt
	{
		taster1.edge_detected = true;
	}

	//keine Flanke erkannt aber noch auf low-level -> flanke best�tigen (Entprellung)
	else if (temp == 0 && taster1.state == 0)
	{
		if(taster1.edge_detected == true && taster1.ready == false)
		{
			taster1.edges++;
			taster1.lastEvent = millis();
			taster1.edge_detected = false;
		}
	}
	else if(taster1.edge_detected == true)
	{
		taster1.edge_detected = false;
	}
	taster1.state = temp;

	if(millis()-taster1.lastEvent > TASTER_TIME_WINDOW && taster1.edges>0)
	{
		taster1.ready = true;
	}
	else
	{
		taster1.ready = false;
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
		else if (switchRed_edges==1 && switchGreen_edges == 2)
		{
		}
		else if (switchRed_edges==1 && switchGreen_edges ==1)
		{

		}
		else if (switchRed_edges==1 && switchGreen_edges ==9)
		{

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

	if(taster1.ready == true)
	{
		if(taster1.edges == 1)
		{
			// Unterst�tzung an/ausschalten
			throttleControl.throttleON = !throttleControl.throttleON;
		}
		else if (taster1.edges == 2)
		{
		  //Unterstuetzung runterschalten
		  throttleControl.aktStufe--;

		  if(throttleControl.aktStufe <=0)
		  {
			  throttleControl.aktStufe = 0;
			  pipapo = false;
		  }
		}
		else if (taster1.edges == 3)
		{
		  //Unterstuetzung hochschalten
		  throttleControl.aktStufe++;
		  if(throttleControl.aktStufe>ANZAHL_STUFEN)
		  {
			  throttleControl.aktStufe = ANZAHL_STUFEN;
		  }
		}
		taster1.edges = 0;
	}
}

// Batteriespannung auslesen und umrechnen
void readBattVoltArdu()
{
  int temp = analogRead(INPUT_BATSENSE);	// analogRead dauert ca. 100�s
  batteryData.voltageArdu = (float)((temp*REFVOLT/1024.0)*(RUPPER+RLOWER)/RLOWER);
}

void writeSerialData()
{
	//Serial.print("Vbat: ");
	//Serial.print(batteryData.voltageArdu);
	//Serial.print("V  SOC: ");
	//Serial.print(batteryData.SOC);
	//Serial.println("%");

	Serial.print("PAS-Faktor: ");
	Serial.println(pasData.pas_factor);

	Serial.print("pedaling: ");
	Serial.println(pasData.pedaling);

	Serial.print("Stufe: ");
	Serial.print(throttleControl.aktStufe);

	Serial.print("  Throttle ON: ");
	Serial.println(throttleControl.throttleON);
}

void calculateSOC()
{
	/*Wertetabelle fuer SOC Berechnung
	 * Spannung		Prozent
	 * >4.08V		100
	 * >4.00		90
	 * >3.94		80
	 * >3.88		70
	 * >3.83		60
	 * >3.80		50
	 * >3.78		40
	 * >3.76		30
	 * >3.74		20
	 * >3.69		10
	 * <3.68		0
	 */

	/*Wertetabelle neu:
	 * 	4,181	100,00%
		4,084	90%
		3,988	80%
		3,934	70%
		3,863	60%
		3,814	50%
		3,784	40%
		3,761	30%
		3,732	20%
		3,685	10%
	 */
	batteryData.avgCellVolt = batteryData.voltageArdu/batteryData.numberOfCells;

	if(batteryData.avgCellVolt > 4.18)
	{
		batteryData.SOC = 100;
	}
	else if (batteryData.avgCellVolt >4.08)
	{
		batteryData.SOC = 90;
	}
	else if (batteryData.avgCellVolt >3.98)
	{
		batteryData.SOC = 80;
	}
	else if (batteryData.avgCellVolt >3.93)
	{
		batteryData.SOC = 70;
	}
	else if (batteryData.avgCellVolt >3.86)
	{
		batteryData.SOC = 60;
	}
	else if (batteryData.avgCellVolt >3.81)
	{
		batteryData.SOC = 50;
	}
	else if (batteryData.avgCellVolt >3.78)
	{
		batteryData.SOC = 40;
	}
	else if (batteryData.avgCellVolt >3.76)
	{
		batteryData.SOC = 30;
	}
	else if (batteryData.avgCellVolt >3.73)
	{
		batteryData.SOC = 20;
	}
	else if (batteryData.avgCellVolt >3.68)
	{
		batteryData.SOC = 10;
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

void undervoltageRegulator()
{
	float regOut = batteryData.voltageArdu - batteryData.undervoltageThreshold;

	if(regOut > 1.0)
	{
	  regOut = 1.0;
	}
	else if (regOut < 0.0)
	{
	  regOut = 0.0;
	}

	// Filterung des Reglerausgangs
	undervoltageReg.reg_out_int = undervoltageReg.reg_out_int + regOut - undervoltageReg.reg_out_filtered;
	undervoltageReg.reg_out_filtered = undervoltageReg.reg_out_int / 20;
}


void setup()
{
  // put your setup code here, to run once:

	resetFlagRegister = MCUSR;
	 // Clear all MCUSR registers immediately for 'next use'
	//MCUSR = 0;

	//PAS-Sensor-Eingang:
	pinMode(INPUT_PAS, INPUT_PULLUP);

	// Gas-Taster-Eingang:
	pinMode(INPUT_TASTER1, INPUT_PULLUP);

	// Licht-Taster-Eingang:
	//pinMode(INPUT_TASTER2, INPUT_PULLUP);

	//3W-Switch-Rot-Eingang:
	pinMode(INPUT_3W_SW_RED, INPUT_PULLUP);
	//3W-Switch-Green-Eingang:
	pinMode(INPUT_3W_SW_GREEN, INPUT_PULLUP);

	//Throttle PWM-Output:
	pinMode(OUTPUT_THROTTLE, OUTPUT);

	//LED auf ArduinoNano Pin 13
	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);

    delay(1000);


	/* millis() Returns the number of milliseconds passed since the Arduino board
	began running the current program. This number will overflow (go back to zero), after approximately 50 days. */
	milliseconds = millis();

	// Interrupt fuer PAS konfigurieren
	attachInterrupt(digitalPinToInterrupt(INPUT_PAS), pas_ISR, CHANGE);

	// falling-edge interrupt fuer Taster konfigurieren	-> TODO
	//attachInterrupt(digitalPinToInterrupt(INPUT_TASTER1), taster_ISR, FALLING);

	Serial.begin(SERIAL_MONITOR_BAUDRATE);

	// Zellspannung einmal abfragen fuer 6s-9s-Erkennung:
    readBattVoltArdu();
	if (batteryData.voltageArdu > BAT6S9S_GRENZE)
	{
		batteryData.numberOfCells = 9;
		batteryData.undervoltageThreshold = UNDERVOLTAGE_9S;
	}
	else if (batteryData.voltageArdu > BAT3S6S_GRENZE)
	{
		batteryData.numberOfCells = 6;
		batteryData.undervoltageThreshold = UNDERVOLTAGE_6S;
	}
	else
	{
		batteryData.numberOfCells = 3;
		batteryData.undervoltageThreshold = UNDERVOLTAGE_3S;

	}
	if(freeMemory() < minFreeRAM)
	{
		minFreeRAM = freeMemory();
	}

	//wdt_enable(WDTO_8S);
}

void loop() {
  // put your main code here, to run repeatedly:

	if(millis() > milliseconds)
	{
		milliseconds = millis();
	}

	// PAS-Timeout?
	if ((millis() - pasData.last_pas_event) > PAS_TIMEOUT)
	{
		pasData.pedaling = false;
		pasData.pas_factor = 0;
		pasData.pas_factor_filtered = 0;
	}

	// Schnelle Routine -> hier werden nur die Taster ausgelesen
	if((millis()-lastFastLoop) > FAST_TIMER)
	{
		lastFastLoop = millis();
		checkTaster();
		readBattVoltArdu();
	}

	// langsame Routine
  if((milliseconds - lastSlowLoop) > SLOW_TIMER)
  {
	  lastSlowLoop = millis();

	  	  //LED blinken lassen zum Anzeigen ob noch aktiv
	  bool temp = digitalRead(LED);
	  digitalWrite(LED, !temp);

	  interpretInputs();

	  if(pasData.pedaling == true && throttleControl.throttleON == true)
	  {
		  notPedalingCounter = 0;
		  switch (throttleControl.aktStufe)
		  {
		  case 0: throttleControl.throttleVoltage = STUFE0;
		  break;
		  case 1: throttleControl.throttleVoltage = STUFE1_V;
		  break;
		  case 2: throttleControl.throttleVoltage = STUFE2_V;
		  break;
		  case 3: throttleControl.throttleVoltage = STUFE3_V;
		  break;
		  case 4: throttleControl.throttleVoltage = STUFE4_V;
		  break;
		  case 5: throttleControl.throttleVoltage = STUFE5_V;
		  break;
		  default: throttleControl.throttleVoltage = STUFE0;
		  break;
		  }
	  }
	  else		// not pedaling
	  {
		  throttleControl.throttleVoltage = 0.0;
		  notPedalingCounter++;
	  }
	  undervoltageRegulator();
	  throttleControl.throttleVoltage = throttleControl.throttleVoltage * undervoltageReg.reg_out_filtered;
	  setThrottlePWM();
  }


	// ultra-langsame Routine
	if((milliseconds - lastUltraSlowLoop) > ULTRA_SLOW_TIMER)
	{
		lastUltraSlowLoop = millis();
		calculateSOC();
		writeSerialData();

		// bei bedarf Unterstuetzung auf default zur�ckstellen nach gewisser Zeit
		if(notPedalingCounter >= TIME_TO_RESET_AFTER_PEDAL_STOP && throttleControl.aktStufe > DEFAULT_STUFE)
		{
			throttleControl.aktStufe = DEFAULT_STUFE;
		}
	}

	if(freeMemory() < minFreeRAM)
	{
		minFreeRAM = freeMemory();

	}
	//wdt_reset();
}
