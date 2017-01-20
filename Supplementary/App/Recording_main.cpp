#include "stdafx.h"
#include "api_definitions.h"
#include "PhidgetSensorEventDetection.h"
#include "FallDetect.h"
#include "MLPClass.h"
#include <deque>
#include <timestamp.hpp>
#include <boost\filesystem.hpp>

const double SENSITIVITY_VAL = 0.1;

int _tmain(int argc, _TCHAR* argv[])
{
	FallDetect* m_fDetection = new FallDetect();
	PhidgetSensorEventDetection* m_pPhidgetSensor2 = new PhidgetSensorEventDetection();
	bool bFirstPass = false;
	Timestamp tmpstmp;
	Timestamp initialTmpstmp;
	int counter = 0;
	//m_fDetection->m_counter = 0;
	bool accelerometerRecorderStopped = false;

	std::string accelerometer_StoringPath_cstr = ".\\..\\";
	accelerometer_StoringPath_cstr.append(FOLDERNAME_ACCELERATION_RECORDINGS);

	boost::filesystem::path dir(accelerometer_StoringPath_cstr);
	boost::filesystem::create_directory(dir);

	accelerometer_StoringPath_cstr.append("\\");


	std::string accelerometer_TimestampStoringPath_cstr = accelerometer_StoringPath_cstr;

	accelerometer_StoringPath_cstr.append(FILENAME_ACCELEROMETER_ANGLES);
	accelerometer_TimestampStoringPath_cstr.append(FILENAME_TIMESTAMP);

	m_fDetection->m_ofs_data.open(accelerometer_StoringPath_cstr);
	m_fDetection->m_ofs_timestamp.open(accelerometer_TimestampStoringPath_cstr);

	bool retVal = m_pPhidgetSensor2->initializePhidget(SENSITIVITY_VAL);


	if(retVal)
	{
		cout << "Accelerometer Sensor initialized" << "\n";	

		while(true)
		{
			::Sleep(ACCELEROMETER_TIMER_PERIOD);
			if( bFirstPass && (tmpstmp.secondsTotal()-initialTmpstmp.secondsTotal() > OBSERVATION_FALL_TIME_IN_SECS + WAITING_TIME_BEFORE_RECORDING_IN_SECS) )
			{
				accelerometerRecorderStopped = true;
			}
			if(accelerometerRecorderStopped || counter>ACCELEREOMETER_MAXIMUM_RECORDING_CYCLES_NUMBER)
			{
				break;
			}

			bool bHasFallen = m_fDetection->checkUsersStatus(m_pPhidgetSensor2->past_sampleX,m_pPhidgetSensor2->past_sampleY,m_pPhidgetSensor2->past_sampleZ, systemTime());

			if(m_fDetection->m_bVectorsAreUpdated)
			{
				tmpstmp = m_fDetection->getTimestamp();

				if(!bFirstPass)
				{
					::Beep(7000, 200);
					Sleep(WAITING_TIME_BEFORE_RECORDING_IN_SECS*1000);

					initialTmpstmp = m_fDetection->getTimestamp();
					bFirstPass = true;

					::Beep(5000, 200);
				}
				
				m_fDetection->m_ofs_timestamp << std::setfill('0') << std::setw(12) 
					<< tmpstmp.secondsTotal() << "." << std::setfill('0') << std::setw(3) 
					<< tmpstmp.timeMilliSeconds() << "\t" << std::setfill(' ') << "\n";


				m_fDetection->m_ofs_data << m_fDetection->getR() << "\t" 
						<< m_fDetection->getDeltaTheta() << "\t" 
						<< m_fDetection->getDeltaPhi() << "\n";

				m_fDetection->m_bVectorsAreUpdated = false;
			}
			counter++;
			//m_fDetection->m_counter++;
		}

		m_fDetection->m_ofs_timestamp.close();
		m_fDetection->m_ofs_timestamp.clear();

		m_fDetection->m_ofs_data.close();
		m_fDetection->m_ofs_data.clear();

		::Beep(3500, 200);
		
	}
	else
	{
		cout << "WARNING: Initialization of Accelerometer Sensor failed..." << "\n";
	}

	//Release the phidget
	m_pPhidgetSensor2->releasePhidget();
	if(m_fDetection!=NULL)
	{
		delete m_fDetection;
		m_fDetection = NULL;
	}

	return 0;

}