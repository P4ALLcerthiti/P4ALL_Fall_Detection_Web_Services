#pragma once
#define WIN32_LEAN_AND_MEAN
#include "phidget21.h"

//#include <timestamp.hpp>

#include <vector>

class FallDetect;
//class CFallDetectionDialogDlg;
class EventDetector;

class PhidgetSensorEventDetection
{
public:
	//PhidgetSensorEventDetection(CFallDetectionDialogDlg* inputCentralUnit);
	PhidgetSensorEventDetection();
	~PhidgetSensorEventDetection();

	bool initializePhidget(double sensitivityVal);
	void releasePhidget();
	
	void setTriggerLevel(double sensitivityVal);

	void setCentralUnitDlg(void *Param);
	void setIsVecicleEnvironment(bool bpilotEnvironment);

	//CFallDetectionDialogDlg* m_pCentralUnit;
	EventDetector* m_pEventDetector;
	void setEventDetector(void* Param);

	//Declare an InterfaceKit handle
	//CPhidgetInterfaceKitHandle ifKit;

	//Declare an accelerometer handle
	CPhidgetAccelerometerHandle m_accel;
	int accelerometer_serialNo;

private:

	int m_iAccelSensorTriggerLevel;
	
	int result, numAxes;
	const char *err;

public:
	double past_sampleX;
	double past_sampleY;
	double past_sampleZ;
	//Timestamp tmpstmp;


	bool m_bUpdatedValues1;
	bool m_bUpdatedValues2;
	bool m_bUpdatedValues3;

	std::vector<double> getAccelFeaturesInstant();
};