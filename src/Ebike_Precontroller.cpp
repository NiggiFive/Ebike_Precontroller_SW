
#include "config.h"
#include "Arduino.h"

#include "MemoryFree.h"
#include <Wire.h>
#include <avr/wdt.h>
#include <EEPROM.h>

#include "VescUart.h"

#include <U8g2lib.h>

bldcMeasure vescValues;	// RolingGeckos Version

//Pins fuer Ein- und Ausgaenge
#define INPUT_PAS 3
#define INPUT_TASTER1 2
#define INPUT_TASTER2 4		// -> Taster2 ueblicherweise fuer Licht?

#ifdef REVERSE_BUTTONS
	#define INPUT_3W_SW_RED 6
	#define INPUT_3W_SW_GREEN 7
#else
	#define INPUT_3W_SW_RED 7
	#define INPUT_3W_SW_GREEN 6
#endif


#define OUTPUT_THROTTLE 5
#define OUTPUT_DISPLAY_SUPPLY 8
#define OUTPUT_LIGHT 9
#define INPUT_BATSENSE PIN_A6
#define INPUT_VARDUSENSE PIN_A7

#define INPUT_THROTTLE PIN_A0

// Pin-Nummer der LED auf Arduino Nano
#define LED 13

#define SERIAL_MONITOR_BAUDRATE 9600

// 10ms Timer
#define FAST_TIMER 10

//100ms Timer
#define SLOW_TIMER 100

#define ULTRA_SLOW_TIMER 250

// Debouncing-time for PAS (und evtl. Taster) in ms
#define ENTPRELLZEIT 5

#define MAX_CURRENT_RAMP_POS	1.0
#define MAX_CURRENT_RAMP_NEG	10.0

//Display Constructor
#ifdef DISPLAY_CONNECTED
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(A5, A4);
#endif

unsigned long milliseconds;

uint16_t notPedalingCounter;

long temprpm = 0;

uint8_t resetFlagRegister = 0;

uint16_t looptime = 0;
uint16_t dispWrtTime = 0;

struct batteryDataStruct
{
	uint8_t numberOfCells = 9;
	float vBatArdu = 0.0;
	float avgCellVolt = 0.0;
	uint8_t SOC = 0;
	int16_t batteryPower = 0.0;
	float undervoltageThreshold;
};

struct controllerDataStruct
{
	float vInArdu = 0.0;
	uint8_t controlMode = POWER_CTRL;
	bool reverseDirection = false;
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

struct odoStruct
{
	float kmTripMotor;
	float kmTripWheel;
	float kmOverallMotor;
	float kmOverallWheel;
	uint16_t minutesTrip;
	uint16_t minutesOverall;
	float wattHoursOverall;
	float ampHoursOverall;
};

struct displayStruct
{
	uint8_t RowCounter = 0;
	uint8_t displayMode = 0;
};

speedRegStruct speedReg;
batteryDataStruct batteryData;
controllerDataStruct controllerData;
undervoltageRegStruct undervoltageReg;
odoStruct odometry;
displayStruct display;



int16_t minFreeRAM = 2000;		// initialisiert mit 2000. wird verringert wenn der echte freie RAM weniger ist



uint8_t vescConnectionErrors = 0;

bool vesc_connected = false;

bool pipapo = false;


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

//tasterStruct taster1;
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
		//const bool doubleHall = DOUBLE_HALL;
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

// get ODO-Values from EEPROM
void getODO()
{
	uint8_t temp = 0;
	EEPROM.get(temp, odometry.kmOverallMotor);
	temp += sizeof(odometry.kmOverallMotor);
	EEPROM.get(temp, odometry.minutesOverall);
	temp+= sizeof(odometry.minutesOverall);
	EEPROM.get(temp, odometry.wattHoursOverall);
	temp+= sizeof(odometry.wattHoursOverall);
	EEPROM.get(temp, odometry.ampHoursOverall);
}

// Write ODO Values to EEPROM
void writeODO()
{
	int temp = 0;
	EEPROM.put(temp, odometry.kmOverallMotor + odometry.kmTripMotor);
	temp += sizeof(odometry.kmOverallMotor);
	EEPROM.put(temp, odometry.minutesOverall + odometry.minutesTrip);
	temp+= sizeof(odometry.minutesOverall);
	EEPROM.put(temp, odometry.wattHoursOverall + vescValues.watt_hours);
	temp+= sizeof(odometry.wattHoursOverall);
	EEPROM.put(temp, odometry.ampHoursOverall + vescValues.ampHours);
}

// reset ODO-Values
void resetODO()
{
	//Clear EEPROM:
	for (uint16_t i = 0 ; i < EEPROM.length() ; i++)
	{
	    EEPROM.write(i, 0);
	}
	getODO();
}

void checkTaster()
{
	//taster1_state = digitalRead(INPUT_TASTER1);
	//taster2_state = digitalRead(INPUT_TASTER2);

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
}

void interpretInputs()
{
  //Taster auswerten:
	if(switchRedGreen_ready == true)
	{
		if (switchRed_edges == 1 && switchGreen_edges == 5)
		{
			controllerData.reverseDirection = !controllerData.reverseDirection;
		}

		else if(switchRed_edges == 1 && switchGreen_edges == 4)
		{
			pipapo = !pipapo;
		}

		// Switch between Power and Torque-Control-Mode
		else if (switchRed_edges ==1 && switchGreen_edges == 3)
		{
			if (controllerData.controlMode == POWER_CTRL)
			{
				controllerData.controlMode = TORQUE_CTRL;
			}
			else
			{
				controllerData.controlMode = POWER_CTRL;
			}
		}
		else if (switchRed_edges==1 && switchGreen_edges == 2)
		{
			// Toggle Headlight
			digitalWrite(OUTPUT_LIGHT, !digitalRead(OUTPUT_LIGHT));
		}

		// Change Display-Mode
		else if (switchRed_edges==1 && switchGreen_edges ==1)
		{
			display.displayMode++;
			if(display.displayMode >=DISPLAY_MODI)
			{
				display.displayMode = 0;
			}
		}

		// Reset ODO-Data
		else if (switchRed_edges==1 && switchGreen_edges ==9)
		{
			resetODO();
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
void readVoltagesArdu()
{
  int temp = analogRead(INPUT_BATSENSE);	// analogRead dauert ca. 100�s
  batteryData.vBatArdu = (float)((temp*REFVOLT/1024.0)*(RUPPER_BATSENSE+RLOWER_BATSENSE)/RLOWER_BATSENSE);
  temp = analogRead(INPUT_VARDUSENSE);
  controllerData.vInArdu = (float)((temp*REFVOLT/1024.0)*(RUPPER_VARDU+RLOWER_VARDU)/RLOWER_VARDU);
}


void calculateSOC()
{
	/*Wertetabelle fuer SOC Berechnung Lipos
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

	/*Wertetabelle neu (aktuell genutzt):
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

	/* Wertetabelle für 18650-Zellen (Samsung INR18650-35E)
	4,1	100%
	4,02	90%
	3,94	80%
	3,86	70%
	3,78	60%
	3,7		50%
	3,62	40%
	3,54	30%
	3,46	20%
	3,38	10%
	3,3		0% */

	if(vesc_connected)
	{
		batteryData.avgCellVolt = vescValues.inpVoltage/batteryData.numberOfCells;
	}
	else
	{
		batteryData.avgCellVolt = batteryData.vBatArdu/batteryData.numberOfCells;
	}
	if(batteryData.numberOfCells == 10)
	{
		if(batteryData.avgCellVolt > 4.10)
		{
			batteryData.SOC = 100;
		}
		else if (batteryData.avgCellVolt >4.02)
		{
			batteryData.SOC = 90;
		}
		else if (batteryData.avgCellVolt >3.94)
		{
			batteryData.SOC = 80;
		}
		else if (batteryData.avgCellVolt >3.86)
		{
			batteryData.SOC = 70;
		}
		else if (batteryData.avgCellVolt >3.78)
		{
			batteryData.SOC = 60;
		}
		else if (batteryData.avgCellVolt >3.70)
		{
			batteryData.SOC = 50;
		}
		else if (batteryData.avgCellVolt >3.62)
		{
			batteryData.SOC = 40;
		}
		else if (batteryData.avgCellVolt >3.54)
		{
			batteryData.SOC = 30;
		}
		else if (batteryData.avgCellVolt >3.46)
		{
			batteryData.SOC = 20;
		}
		else if (batteryData.avgCellVolt >3.38)
		{
			batteryData.SOC = 10;
		}
		else
		{
			batteryData.SOC = 0;
		}
	}
	else
	{
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
}

void setThrottlePWM()
{
  analogWrite(OUTPUT_THROTTLE, throttleControl.throttleVoltage/5*255.0);
}

void undervoltageRegulator()
{
	float regOut;
	if(vesc_connected)
	{
		  regOut = vescValues.inpVoltage - batteryData.undervoltageThreshold;
	}
	else
	{
		regOut = batteryData.vBatArdu - batteryData.undervoltageThreshold;
	}

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


#ifdef DISPLAY_CONNECTED
void refreshu8x8Display()
{
	uint32_t tempTime = millis();
	switch (display.displayMode)
	{
	// Home-Display
	case 0:
		if(display.RowCounter == 0)
		{
			  u8x8.home();
			  u8x8.clearLine(0);
			  u8x8.clearLine(1);
			  if(vesc_connected)
			  {
				  u8x8.print((int)vescValues.inpVoltage);
			  }
			  else
			  {
				  u8x8.print((int)batteryData.vBatArdu);
			  }
			  u8x8.print(F(" V  "));
			  u8x8.print(batteryData.numberOfCells);
			  u8x8.print(F("s "));
			  //SOC Ausgeben
			  u8x8.print(batteryData.SOC);
			  u8x8.print(F(" % "));
		}
		else if (display.RowCounter == 1)
		{
			  // Zeile 2:
			  u8x8.clearLine(2);
			  u8x8.clearLine(3);
			  u8x8.setCursor(0,2);
			  u8x8.print((int)speedReg.velocity);
			  u8x8.print(F(" km/h "));
			  if(controllerData.controlMode == POWER_CTRL)
			  {
				  u8x8.print(F("PWR"));
			  }
			  else
			  {
				  u8x8.print(F("TRQ"));
			  }
		}
		else if (display.RowCounter == 2)
		{
			  //Zeile 3:
			  u8x8.clearLine(4);
			  u8x8.clearLine(5);
			  u8x8.setCursor(0,4);
			  u8x8.print(batteryData.batteryPower);
			  u8x8.print(F(" W   "));
			  u8x8.print(controllerData.vInArdu);
			  u8x8.print(F("V"));
		}
		else if (display.RowCounter == 3)
		{
			  //Zeile 4:
			  u8x8.clearLine(6);
			  u8x8.clearLine(7);
			  u8x8.setCursor(0,6);
			  u8x8.print(F("Stufe "));
			  u8x8.print(throttleControl.aktStufe);
			  if(pasData.pedaling)
			  {
				  u8x8.print(F("   P"));
			  }
			  else
			  {
				  u8x8.print(F("      "));
			  }
		}
		break;

	case 1:
		if(display.RowCounter == 0)
		{
			  u8x8.home();
			  u8x8.clearLine(0);
			  u8x8.clearLine(1);
			  if(vesc_connected == true)
			  {
			  	u8x8.print(F("VESC: "));
				u8x8.print((int)vescValues.temp_mos);
			    u8x8.print(F(" deg"));
			  }
		}
		else if (display.RowCounter == 1)
		{
			  // Zeile 2:
			  u8x8.clearLine(2);
			  u8x8.clearLine(3);
			  u8x8.setCursor(0,2);
			  u8x8.print(F("Imot = "));
			  u8x8.print((int)vescValues.avgMotorCurrent);
			  u8x8.print(F(" A "));
		}
		else if (display.RowCounter == 2)
		{
			  //Zeile 3:
			  u8x8.clearLine(4);
			  u8x8.clearLine(5);
			  u8x8.setCursor(0,4);
			  u8x8.print(F("Ibat = "));
			  u8x8.print((int)vescValues.avgInputCurrent);
			  u8x8.print(F(" A"));


		}
		else if (display.RowCounter == 3)
		{
			  //Zeile 4:
			  u8x8.clearLine(6);
			  u8x8.clearLine(7);
			  u8x8.setCursor(0,6);
//			  u8x8.print(vescValues.rpm);
//			  u8x8.print(F(" ERPM "));
			  u8x8.print(batteryData.batteryPower);
			  u8x8.print(F(" W  "));

			if(controllerData.reverseDirection == true)
			{
				u8x8.print(F("Rev."));
			}
			else
			{
				u8x8.print(F("Fwd."));
			}

		}
		break;


	// DEBUG Display 1
	case 2:
		if(display.RowCounter == 0)
		{
			  u8x8.home();
			  u8x8.clearLine(0);
			  u8x8.clearLine(1);
			  //u8x8.print(F("RST: "));
			  if (resetFlagRegister & _BV(EXTRF))
				{
				     // Reset button or otherwise some software reset
				     u8x8.print(F("EXT "));
				 }
			   if (resetFlagRegister & (_BV(BORF)))
			   {
				   u8x8.print(F("BOR "));
			   }
				if (resetFlagRegister &_BV(PORF))
				{
				      // Brownout or Power On
				      u8x8.println(F("POR "));
				 }
				 if (resetFlagRegister & _BV(WDRF))
				 {
				      //Watchdog Reset
					 u8x8.println(F("WD"));
				 }
			  //Bit 0 � PORF: Power-on Reset Flag
			  //Bit 1 � EXTRF: External Reset Flag
			  //Bit 2 � BORF: Brown-out Reset Flag
			  //Bit 3 � WDRF: Watchdog System Reset Flag

			  //u8x8.print(resetFlagRegister);
		}

		else if (display.RowCounter == 1)
		{
			  // Zeile 2:
			  u8x8.clearLine(2);
			  u8x8.clearLine(3);
			  u8x8.setCursor(0,2);
			  u8x8.print(F("Con-Errs: "));
			  u8x8.print(vescConnectionErrors);

		}
		else if (display.RowCounter == 2)
		{
			  //Zeile 3:
			  u8x8.clearLine(4);
			  u8x8.clearLine(5);
			  u8x8.setCursor(0,4);

			  u8x8.print(F("RAM: "));
			  //u8x8.print(freeMemory());
			  u8x8.print(minFreeRAM);
			  u8x8.print(F(" Bytes"));
		}
		else if (display.RowCounter == 3)
		{
			  //Zeile 4:
			  u8x8.clearLine(6);
			  u8x8.clearLine(7);
			  u8x8.setCursor(0,6);
			  u8x8.print(F("PAS: "));
			  u8x8.print(pasData.pas_factor);
			  if(pasData.pedaling)
			  {
				  u8x8.print(F(" P "));
			  }
		}
		break;

	// DEBUT-Display 2
	case 3:
		if(display.RowCounter == 0)
		{
			  u8x8.home();
			  u8x8.clearLine(0);
			  u8x8.clearLine(1);
			  u8x8.print(F("Loop: "));
			  u8x8.print(looptime);
			  u8x8.print(F("ms"));
		}

		else if (display.RowCounter == 1)
		{
			  // Zeile 2:
			  u8x8.clearLine(2);
			  u8x8.clearLine(3);
			  u8x8.setCursor(0,2);
			  u8x8.print(F("Disp: "));
			  u8x8.print(dispWrtTime);
			  u8x8.print(F("ms"));

		}
		else if (display.RowCounter == 2)
		{
			  //Zeile 3:
			  u8x8.clearLine(4);
			  u8x8.clearLine(5);
			  u8x8.setCursor(0,4);
		}
		else if (display.RowCounter == 3)
		{
			  //Zeile 4:
			  u8x8.clearLine(6);
			  u8x8.clearLine(7);
			  u8x8.setCursor(0,6);
		}
		break;

		// Trip
	case 4:
		if(display.RowCounter == 0)
		{
			  u8x8.home();
			  u8x8.clearLine(0);
			  u8x8.clearLine(1);
			  u8x8.print(F("Trip: "));
			  u8x8.print(odometry.kmTripMotor);
			  u8x8.print(F(" km "));
		}

		else if (display.RowCounter == 1)
		{
			  // Zeile 2:
			  u8x8.clearLine(2);
			  u8x8.clearLine(3);
			  u8x8.setCursor(0,2);
			  uint16_t temp_time = odometry.minutesTrip/60;
			  temp_time = temp_time % 24;
			  u8x8.print(temp_time);
			  u8x8.print(F("h "));
			  temp_time = (odometry.minutesTrip) % (60);
			  if(temp_time < 10)
			  {
				  u8x8.print(0);
			  }
			  u8x8.print(temp_time);
			  u8x8.print(F("m "));
		}

		else if (display.RowCounter == 2)
		{
			  //Zeile 3:
			  u8x8.clearLine(4);
			  u8x8.clearLine(5);
			  u8x8.setCursor(0,4);
			  float temp = vescValues.watt_hours;
			  if(temp <= 100.0)
			  {
				  u8x8.print(temp);
				  u8x8.print(F(" Wh "));
			  }
			  else
			  {
				  u8x8.print(temp/1000);
				  u8x8.print(F(" kWh "));
			  }

		}
		else if (display.RowCounter == 3)
		{
			  //Zeile 4:
			  u8x8.clearLine(6);
			  u8x8.clearLine(7);
			  u8x8.setCursor(0,6);
			  u8x8.print(vescValues.ampHours);
			  u8x8.print(F(" Ah"));
		}
		break;

	//ODO
	case 5:
		if(display.RowCounter == 0)
		{
			  u8x8.home();
			  u8x8.clearLine(0);
			  u8x8.clearLine(1);
			  u8x8.print(F("ODO: "));
			  u8x8.print(odometry.kmOverallMotor + odometry.kmTripMotor);
			  u8x8.print(F(" km "));
		}

		else if (display.RowCounter == 1)
		{
			  // Zeile 2:
			  u8x8.clearLine(2);
			  u8x8.clearLine(3);
			  u8x8.setCursor(0,2);
			  uint16_t temp_time = (odometry.minutesOverall+odometry.minutesTrip)/(60*24);
			  if(temp_time > 0)		// only Display days if min 1 passed
			  {
				  u8x8.print(temp_time);
				  u8x8.print(F("d "));
			  }
			  temp_time = (odometry.minutesOverall+odometry.minutesTrip)/60;
			  temp_time = temp_time % 24;
			  if(temp_time < 10)
			  {
				  u8x8.print(0);
			  }
			  u8x8.print(temp_time);
			  u8x8.print(F("h "));
			  temp_time = (odometry.minutesOverall+odometry.minutesTrip) % (60);
			  if(temp_time < 10)
			  {
				  u8x8.print(0);
			  }
			  u8x8.print(temp_time);
			  u8x8.print(F("m "));
		}

		else if (display.RowCounter == 2)
		{
			  //Zeile 3:
			  u8x8.clearLine(4);
			  u8x8.clearLine(5);
			  u8x8.setCursor(0,4);
			  float temp = odometry.wattHoursOverall + vescValues.watt_hours;
			  if(temp <= 100.0)
			  {
				  u8x8.print(temp);
				  u8x8.print(F(" Wh "));
			  }
			  else
			  {
				  u8x8.print(temp/1000);
				  u8x8.print(F(" kWh "));
			  }

		}
		else if (display.RowCounter == 3)
		{
			  //Zeile 4:
			  u8x8.clearLine(6);
			  u8x8.clearLine(7);
			  u8x8.setCursor(0,6);
			  if(odometry.ampHoursOverall < 100)
			  {
				  u8x8.print(odometry.ampHoursOverall + vescValues.ampHours);
			  }
			  else
			  {
				  u8x8.print((uint16_t)(odometry.ampHoursOverall + vescValues.ampHours));
			  }
			  u8x8.print(F(" Ah"));
		}
		break;

	case 6: u8x8.clearDisplay();
	break;

	default: break;
	}

	  if(display.RowCounter < 3)
	  {
		  display.RowCounter++;

	  }
	  else
	  {
		  display.RowCounter = 0;
	  }
	  dispWrtTime = millis()-tempTime;
}
#endif

void setup()
{
  // put your setup code here, to run once:

	resetFlagRegister = MCUSR;
	 // Clear all MCUSR registers immediately for 'next use'
	//MCUSR = 0;

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
	if (HW_VERSION == 2)
	{
		// Display Versorgung h�ngt direkt an IO von Ardu
		digitalWrite(OUTPUT_DISPLAY_SUPPLY, HIGH);
	}
	else if (HW_VERSION == 1)
	{
		// Display Versorgung wird mit PNP Highside geschaltet
		digitalWrite(OUTPUT_DISPLAY_SUPPLY, LOW);
	}


	//Licht-Ausgang konfigurieren
	pinMode(OUTPUT_LIGHT, OUTPUT);
	//Licht anschalten
	digitalWrite(OUTPUT_LIGHT, HIGH);

	//LED auf ArduinoNano Pin 13
	pinMode(LED, OUTPUT);
	digitalWrite(LED, LOW);


	// Gesamtkilometer etc aus EEPROM auslesen
	getODO();

	// Tageskilometer auf 0 Stellen
	odometry.kmTripMotor = 0;
	odometry.minutesTrip = 0;

#ifdef DISPLAY_CONNECTED
    u8x8.begin();

    u8x8.setFont(u8x8_font_courB18_2x3_r);
    u8x8.home();
    u8x8.setCursor(0,2);
    u8x8.setFont(u8x8_font_8x13_1x2_r);
    u8x8.print(F("Warte auf VESC"));
#endif


	/* millis() Returns the number of milliseconds passed since the Arduino board
	began running the current program. This number will overflow (go back to zero), after approximately 50 days. */
	milliseconds = millis();

	// Interrupt fuer PAS konfigurieren
	attachInterrupt(digitalPinToInterrupt(INPUT_PAS), pas_ISR, CHANGE);

	// falling-edge interrupt fuer Taster konfigurieren	-> TODO
	//attachInterrupt(digitalPinToInterrupt(INPUT_TASTER1), taster_ISR, FALLING);

	//Delay bis VESC ready ist:
	//Da muss man recht lange warten da der neue Bootloader deutlich schneller ist
	//TODO: evtl. kann man den Delay dann abh�ngig von der resetquelle machen
	// siehe auch hier: https://www.arduino.cc/reference/en/language/functions/communication/serial/ifserial/
	//delay(2500);

	// zusammen mit VESC kann man die serielle Schnittstelle nicht mehr nehmen
	//Serial.begin(9600);
	Serial.begin(VESC_BAUDRATE);

	// get Battery-Voltage to detect Number of Cells
    readVoltagesArdu();
    if (batteryData.vBatArdu > BAT10S12S_GRENZE)
    {
    	batteryData.numberOfCells = 12;
    	batteryData.undervoltageThreshold = UNDERVOLTAGE_12S;
    }
	else if (batteryData.vBatArdu >BAT9S10S_GRENZE)
	{
		batteryData.numberOfCells = 10;
		batteryData.undervoltageThreshold = UNDERVOLTAGE_10S;
	}
    else if (batteryData.vBatArdu > BAT6S9S_GRENZE)
	{
		batteryData.numberOfCells = 9;
		batteryData.undervoltageThreshold = UNDERVOLTAGE_9S;
	}
	else if (batteryData.vBatArdu > BAT3S6S_GRENZE)
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

	// check connection to VESC until timout reached
	uint32_t tempTime = millis();
	while (millis()-tempTime < VESC_TIMEOUT)
	{
		if(VescUartGetValue(vescValues))
		{
			//LED einschalten und weiter zur loop
			vesc_connected = true;
			digitalWrite(LED, HIGH);
			break;
		}
	}

	//3mal Verbindung zum VESC checken
	// for(int i = 0; i<3; i++)
	// {
	// 	//if(UART.getVescValues())
	// 	if(VescUartGetValue(vescValues))
	// 	{
	// 		//LED einschalten und weiter zur loop
	// 		vesc_connected = true;
	// 		digitalWrite(LED, HIGH);
	// 		break;
	// 	}
	// }

	//wenn kein Vesc dran haengt serielle Schnittstelle beenden
	if(vesc_connected == false)
	{
		Serial.end();
		#ifdef DISPLAY_CONNECTED
			u8x8.clearDisplay();
			u8x8.home();
			u8x8.println(F("VESC not found"));
	    	u8x8.print(F("PWM-Modus aktiv"));
		#endif
	    delay(1000);
	}

	wdt_enable(WDTO_2S);
}

void slowLoop()
{
	  	  //LED blinken lassen zum Anzeigen ob noch aktiv
		bool temp = digitalRead(LED);
		digitalWrite(LED, !temp);

		interpretInputs();

	  if(vesc_connected)
	  {
		  uint8_t vescErrorsTemp = vescConnectionErrors;
		if(VescUartGetValue(vescValues))
		{
			// TODO: possible to read out power directly from vesc?
			batteryData.batteryPower = vescValues.inpVoltage * vescValues.avgInputCurrent;

			 //Geschwindigkeit berechnen aus ausgelesener RPM
			 
			speedReg.velocity = vescValues.rpm*RADUMFANG*60/MOTOR_POLE_PAIRS/MOTOR_GEAR_RATIO/1000;
			if(controllerData.reverseDirection == true)
			{
				speedReg.velocity = -1 * speedReg.velocity;
			}

			if(speedReg.velocity < 0.0)
			{
				speedReg.velocity = 0.0;
			}

			// Speed-Limiting/Control (only necessary if VESC has no speed-limit)
			#ifndef SPEED_LIMIT_BY_VESC
				float regOut = 1.0-(float)((speedReg.velocity-VGRENZ)/(MAX_SPEED_KMH - VGRENZ));

				if(regOut >1.0)
				{
					regOut = 1.0;
				}
				else if (regOut < 0.0)
				{
					regOut = 0.0;
				}

				//PT1-Filterung des Reglerausgangs:
				speedReg.vreg_out_int += (regOut - speedReg.vreg_out_filtered);
				speedReg.vreg_out_filtered = speedReg.vreg_out_int / 8;
			#else
			  speedReg.vreg_out_filtered = 1.0;
			#endif
			  //Unterspannungs-regler:
			  undervoltageRegulator();
		}

		else
		{
			vescConnectionErrors++;
		}

		// restart Interface if Error occurred
		if(vescErrorsTemp != vescConnectionErrors)
		{
			Serial.end();
			Serial.begin(VESC_BAUDRATE);
		}

		  if(pasData.pedaling == true)
		  {
			  notPedalingCounter = 0;

			if (controllerData.controlMode == TORQUE_CTRL)
			{
				switch (throttleControl.aktStufe)
				{
					case 0: throttleControl.current_next = 0.0;
					break;
					case 1: throttleControl.current_next = STUFE1_I;
					break;
					case 2: throttleControl.current_next = STUFE2_I;
					break;
					case 3: throttleControl.current_next = STUFE3_I;
					break;
					case 4: throttleControl.current_next = STUFE4_I;
					break;
					default: throttleControl.current_next = 0.0;
					break;
				}
			}
			else if (controllerData.controlMode == POWER_CTRL)
			{
					temprpm = vescValues.rpm;
					if(controllerData.reverseDirection == true)
					{
						temprpm = -1 * temprpm;
					}

					// set to one if zero or (small) negative values occur
					if(temprpm <= 0 && temprpm > -500)
					{
						temprpm = 1;
					}

				switch(throttleControl.aktStufe)
				{
					case 0: throttleControl.current_next = 0.0;
					break;
					case 1: throttleControl.current_next = AMPS_PER_WATTS_AND_ERPM*STUFE1_P/temprpm;
					break;
					case 2: throttleControl.current_next = AMPS_PER_WATTS_AND_ERPM*STUFE2_P/temprpm;
					break;
					case 3: throttleControl.current_next = AMPS_PER_WATTS_AND_ERPM*STUFE3_P/temprpm;
					break;
					case 4: throttleControl.current_next = AMPS_PER_WATTS_AND_ERPM*STUFE4_P/temprpm;
					break;
				}
			}

			//Speed-Limit:
			if(pipapo == false)
			{
				throttleControl.current_next = throttleControl.current_next*speedReg.vreg_out_filtered;
			}

			  // Unterspannungsgrenze
			#ifndef UNDERVOLTAGE_LIMIT_BY_VESC
			  throttleControl.current_next = throttleControl.current_next * undervoltageReg.reg_out_filtered;
			#endif

			// prevent negative current
			 if(throttleControl.current_next < 0.0)
			 {
				 throttleControl.current_next = 0.0;
			 }
			 // upper abs-current-limit:
			 if(throttleControl.current_next > ABS_MAX_CURRENT)
			 {
				 throttleControl.current_next = ABS_MAX_CURRENT;
			 }

			 //Maximale Stromsteigung beachten
			 if((throttleControl.current_next - throttleControl.current_now) > MAX_CURRENT_RAMP_POS )
			 {
				 throttleControl.current_next = throttleControl.current_now + MAX_CURRENT_RAMP_POS;
			 }
			 //Maximale negative Stromsteigung beachten
			 else if ((throttleControl.current_now - throttleControl.current_next) > MAX_CURRENT_RAMP_NEG)
			 {
				 throttleControl.current_next = throttleControl.current_now - MAX_CURRENT_RAMP_NEG;
			 }
	  	  }

		  //not pedaling
		  else
		  {
			throttleControl.current_next = 0.0;
			notPedalingCounter++;
		  }
		  if(controllerData.reverseDirection)
		  {
			VescUartSetCurrent(-1*throttleControl.current_next);

		  }
		  else
		  {
		  	VescUartSetCurrent(throttleControl.current_next);
		  }

		  throttleControl.current_now = throttleControl.current_next;

	  }
	  else
	  {
		  //VESC not connected -> PWM-Mode
		  if(pasData.pedaling == true)
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
			  default: throttleControl.throttleVoltage = STUFE0;
			  break;
			  }
		  }
		  else
		  {
			  throttleControl.throttleVoltage = 0.0;
		  }
		  undervoltageRegulator();
		  throttleControl.throttleVoltage = throttleControl.throttleVoltage * undervoltageReg.reg_out_filtered;
		  setThrottlePWM();
	  }
	  looptime = millis()-lastSlowLoop;
	#ifdef DISPLAY_CONNECTED
	refreshu8x8Display();
	#endif
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
		readVoltagesArdu();

		//wenn Spannung stark abgesunken -> TachoWerte auf EEPROM schreiben
		if(batteryData.vBatArdu < (0.7 * batteryData.undervoltageThreshold) && batteryData.vBatArdu > 5.0)
		{
			writeODO();
			delay(5000);
		}
	}

	// langsame Routine
  if((millis() - lastSlowLoop) > SLOW_TIMER)
  {
		lastSlowLoop = millis();
	  	slowLoop();

  }

	// ultra-langsame Routine
	if((millis() - lastUltraSlowLoop) > ULTRA_SLOW_TIMER)
	{
		lastUltraSlowLoop = millis();

		// siehe auch in mcpwm_foc.c aus bldc project von Benjamin Vedder:
		// * The tachometer value in motor steps. The number of motor revolutions will
		// * be this number divided by (3 * MOTOR_POLE_NUMBER).
		odometry.kmTripMotor = vescValues.tachometer/(3* MOTOR_POLES * MOTOR_GEAR_RATIO);
		odometry.kmTripMotor = odometry.kmTripMotor*RADUMFANG;
		odometry.kmTripMotor = odometry.kmTripMotor/1000;

		odometry.minutesTrip = millis()/60000;
		calculateSOC();

		// bei bedarf Unterstuetzung auf default zur�ckstellen nach gewisser Zeit
		/*if(notPedalingCounter >= TIME_TO_RESET_AFTER_PEDAL_STOP && throttleControl.aktStufe > DEFAULT_STUFE)
		{
			throttleControl.aktStufe = DEFAULT_STUFE;
		}*/
	}

	if(freeMemory() < minFreeRAM)
	{
		minFreeRAM = freeMemory();
	}
	wdt_reset();
}
