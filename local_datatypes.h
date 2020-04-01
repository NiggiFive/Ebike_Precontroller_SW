#ifndef LOCAL_DATATYPES_H_
#define LOCAL_DATATYPES_H_

// Added by AC to store measured values
struct bldcMeasure {
	// Values  read:
	float temp_mos;
	//float temp_motor;
	float avgMotorCurrent;
	float avgInputCurrent;
	//float dutyCycleNow;
	long rpm;
	float inpVoltage;
	float ampHours;
	//float ampHoursCharged;
	float watt_hours;
	//float watt_hours_charged;
	long tachometer;
	//long tachometerAbs;
	mc_fault_code fault_code;
};
//Define remote Package

struct remotePackage {

	int		valXJoy;
	int		valYJoy;
	boolean	valUpperButton;
	boolean	valLowerButton;

};

#endif
