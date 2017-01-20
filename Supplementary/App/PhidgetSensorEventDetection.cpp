#include "stdafx.h"

#include "PhidgetSensorEventDetection.h"

//#include "api_definitions.h"
//#include "FallDetect.h"


PhidgetSensorEventDetection::PhidgetSensorEventDetection()
{
	//m_pCentralUnit = inputCentralUnit;
	accelerometer_serialNo=158425;
}


PhidgetSensorEventDetection::~PhidgetSensorEventDetection()
{
}

//////////////////////////////////////////////////////////////////////////
//							START - EVENT HANDLERS						//
//////////////////////////////////////////////////////////////////////////

//Sets an attach handler callback function. This is called when this Phidget is plugged into the system, and is ready for use.
int __stdcall AttachHandler(CPhidgetHandle IFK, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName(IFK, &name);
	CPhidget_getSerialNumber(IFK, &serialNo);
	
	//TRACE("%s %10d attached!\n", name, serialNo);
	return 0;
}

//Sets a detach handler callback function. This is called when this Phidget is unplugged from the system. 
int __stdcall DetachHandler(CPhidgetHandle IFK, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (IFK, &name);
	CPhidget_getSerialNumber(IFK, &serialNo);

	//TRACE("%s %10d detached!\n", name, serialNo);
	return 0;
}

//Sets the error handler callback function. This is called when an asynchronous error occurs. 
int __stdcall ErrorHandler(CPhidgetHandle IFK, void *userptr, int ErrorCode, const char *unknown)
{
	//TRACE("Error handled. %d - %s", ErrorCode, unknown);
	//TRACE("Phidget Error handled.");
	return 0;
}

//callback that will run if an input changes.
int __stdcall InputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State)
{
	//TRACE("Digital Input: %d > State: %d\n", Index, State);
	return 0;
}


//callback that will run if the sensor value changes by more than the OnSensorChange trigger.
//Index - Index of the sensor that generated the event, Value - the sensor read value
//int __stdcall SensorChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int Value)
//{
//	//TRACE("Sensor: %d > Value: %d\n", Index, Value);
//
//	// explicitly cast a pointer to Phidget class
//	PhidgetSensorEventDetection* m_pCapturePhidget=NULL;
//	m_pCapturePhidget = (PhidgetSensorEventDetection *)usrptr;
//
//	if(m_pCapturePhidget != NULL)
//	{
//		if((Index == (int)SENSOR_PHONE) && (Value <= (int)THRESHOLD_PHONE))
//		{
//			if(m_pCapturePhidget->m_pCentralUnit)
//			{
//				m_pCapturePhidget->m_pCentralUnit->AddUIString(_T("Accelerometer is moving."));
//				m_pCapturePhidget->m_pCentralUnit->m_iAccelerometerActivity = ACTIVITY_ACCELEROMETER_ON;
//			}
//			if(m_pCapturePhidget->m_pCentralUnit)
//			{
//				m_pCapturePhidget->m_pCentralUnit->m_iAccelerometerActivity = ACTIVITY_ACCELEROMETER_ON;
//			}
//		}
//		if((Index == (int)SENSOR_PHONE) && (Value > (int)THRESHOLD_PHONE))
//		{
//			if(m_pCapturePhidget->m_pCentralUnit)
//			{
//				m_pCapturePhidget->m_pCentralUnit->AddUIString(_T("Accelerometer is stable."));
//				m_pCapturePhidget->m_pCentralUnit->m_iAccelerometerActivity = ACTIVITY_ACCELEROMETER_OFF;
//			}
//			if(m_pCapturePhidget->m_pCentralUnit)
//			{
//				m_pCapturePhidget->m_pCentralUnit->m_iAccelerometerActivity = ACTIVITY_ACCELEROMETER_OFF;
//			}
//		}	
//	}
//
//	return 0;
//}


//callback that will run if an output changes.
int __stdcall OutputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State)
{
	//TRACE("Digital Output: %d > State: %d\n", Index, State);
	return 0;
}


//callback that will run if the acceleration changes by more than the Acceleration trigger.
//Index - Index of the axis that is generating the event, Value - the value read by the accelerometer axis
int __stdcall accel_AccelChangeHandler(CPhidgetAccelerometerHandle WGT, void *userptr, int Index, double Value)
{
	PhidgetSensorEventDetection* m_pCapturePhidget = NULL;
	m_pCapturePhidget = (PhidgetSensorEventDetection *)userptr;

	if(Index==0)
	{
		m_pCapturePhidget->past_sampleX = (m_pCapturePhidget->past_sampleX * 0.90) + (Value * 0.10);
		//TRACE("Axis: %i -- %6f\n", Index, m_pCapturePhidget->past_sampleX);
	}
	else if(Index==1)
	{
		m_pCapturePhidget->past_sampleY = (m_pCapturePhidget->past_sampleY * 0.90) + (Value * 0.10);
		//TRACE("Axis: %i -- %6f\n", Index, m_pCapturePhidget->past_sampleY);
	}
	else if(Index==2)
	{
		m_pCapturePhidget->past_sampleZ = (m_pCapturePhidget->past_sampleZ * 0.90) + (Value * 0.10);
		//TRACE("Axis: %i -- %6f\n", Index, m_pCapturePhidget->past_sampleZ);
	}

	if( !(m_pCapturePhidget->past_sampleX==0 && m_pCapturePhidget->past_sampleY==0 && m_pCapturePhidget->past_sampleZ==0))
	{
		//m_pCapturePhidget->tmpstmp = systemTime();

		m_pCapturePhidget->m_bUpdatedValues1 = true;
		m_pCapturePhidget->m_bUpdatedValues2 = true;
		m_pCapturePhidget->m_bUpdatedValues3 = true;
	}

	return 0;
}

//////////////////////////////////////////////////////////////////////////
//							END - EVENT HANDLERS						//
//////////////////////////////////////////////////////////////////////////


bool PhidgetSensorEventDetection::initializePhidget(double sensitivityVal)
{
	int result;
	const char *err;

	//ifKit = 0;
	//create the InterfaceKit object
	//CPhidgetInterfaceKit_create(&ifKit);
	
	m_bUpdatedValues1 = false;
	m_bUpdatedValues2 = false;
	m_bUpdatedValues3 = false;

	past_sampleX=0;
	past_sampleY=0;
	past_sampleZ=0;

	m_accel = 0;

	//create the accelerometer object
	CPhidgetAccelerometer_create(&m_accel);


	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)m_accel, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)m_accel, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)m_accel, ErrorHandler, NULL);


	//Registers a callback that will run if the acceleration changes by more than the Acceleration trigger.
	//Requires the handle for the Accelerometer, the function that will be called, 
	//and an arbitrary pointer that will be supplied to the callback function (may be NULL)
	CPhidgetAccelerometer_set_OnAccelerationChange_Handler(m_accel, accel_AccelChangeHandler, this);

	//open the acclerometer for device connections
	CPhidget_open((CPhidgetHandle)m_accel, accelerometer_serialNo);

	//get the program to wait for an accelerometer device to be attached
	//TRACE("Waiting for accelerometer to be attached.... \n");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)m_accel, 3000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}


	/*
	int connectedPhidgetSerialNr;
	CPhidget_getSerialNumber((CPhidgetHandle)m_accel, &connectedPhidgetSerialNr);

	if(connectedPhidgetSerialNr!=normal_accelerometer1_serialNO_R){
		//den exei syndethei to accel_1, exei syndethei to accel_2 sto m_accel
		int aa=1;
	}*/

	//get the number of available axes on the attached accelerometer
	CPhidgetAccelerometer_getAxisCount(m_accel, &numAxes);


	//set the sensor's trigger levels
	setTriggerLevel(sensitivityVal);

	return 1;
}

void PhidgetSensorEventDetection::setTriggerLevel(double sensitivityVal)
{
	// Specify the amount of change that has to occur after the last event before another event will be fired. 
	// Setting the ChangeTrigger to zero will cause all events to fire.

	/*
	* TODO: read sensor trigger from file...
	*/
	//Most accelerometers have 2 axes so we'll pre-set their sensitivity to make the data more readable
	CPhidgetAccelerometer_setAccelerationChangeTrigger(m_accel, 0, sensitivityVal);
	CPhidgetAccelerometer_setAccelerationChangeTrigger(m_accel, 1, sensitivityVal);
	CPhidgetAccelerometer_setAccelerationChangeTrigger(m_accel, 2, sensitivityVal);
}


void PhidgetSensorEventDetection::releasePhidget()
{
	//CPhidget_close((CPhidgetHandle)ifKit);
	//CPhidget_delete((CPhidgetHandle)ifKit);

	CPhidget_close((CPhidgetHandle)m_accel);
	CPhidget_delete((CPhidgetHandle)m_accel);
}



std::vector<double> PhidgetSensorEventDetection::getAccelFeaturesInstant()
{
	std::vector<double> accelFeatures;

	accelFeatures.push_back(past_sampleX);
	accelFeatures.push_back(past_sampleY);
	accelFeatures.push_back(past_sampleZ);
	
	return accelFeatures;
}