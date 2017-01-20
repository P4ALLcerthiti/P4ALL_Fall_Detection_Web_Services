#include "stdafx.h"

#include "FallDetect.h"

#include <iomanip>
#include <math.h>

#include <iostream>

//extern "C" {
//#include "fftw3.h"
//};
#include "fasttransforms.h"

//#include "Vector.h"
#include "gmr.h"




FallDetect::FallDetect()
{
	m_bAccelerometerTrackerStopped = false;
	m_bAccelerometerRecorderStopped = false;
	//m_hThreadAccelerometerTracker = NULL;
	//m_hThreadAccelerometerRecorder = NULL;

	//m_bPlotFlag = false;
	m_bVectorsAreUpdated = false;

	m_current_sma = 0.0;
	m_current_deltaTheta = 0.0;
	m_current_deltaPhi = 0.0;
	m_current_fis = 0.0;

	m_counter = 0;
	m_accelVals_deq.resize(ACCELEROMETER_DIMENSIONS);
	for(int i=0; i<ACCELEROMETER_DIMENSIONS; i++)
	{
		m_accelVals_deq[i].resize(ACCELEROMETER_SAMPLES_FOR_SMA);
	}	

	m_accel_x = 0;
	m_accel_y = 0;
	m_accel_z = 0;

	m_bUpdatedValues1 = false;
	m_bUpdatedValues2 = false;
	m_bUpdatedValues3 = false;

	m_r = 0;
	m_previousR = 0;
	m_r_deq.resize(ACCELEROMETER_SAMPLES_FOR_SMA);
	
	m_theta = 0;
	m_previousTheta = 0;
	m_deltaThetaDer = 0;
	m_theta_deq.resize(ACCELEROMETER_SAMPLES_FOR_SMA);
	m_deltaTheta_deq.resize(ACCELEROMETER_SAMPLES_FOR_SMA);
	m_deltaThetaBuffer_deq.resize(ACCELEROMETERS_SMOOTHING_WINDOW);

	m_phi = 0;
	m_previousPhi = 0;
	m_deltaPhiDer = 0;
	m_phi_deq.resize(ACCELEROMETER_SAMPLES_FOR_SMA);
	m_deltaPhi_deq.resize(ACCELEROMETER_SAMPLES_FOR_SMA);
	m_deltaPhiBuffer_deq.resize(ACCELEROMETERS_SMOOTHING_WINDOW);

	m_sma = 0;
	m_sma_deq.resize(ACCELEROMETER_SAMPLES_FOR_SMA);
	m_smaBuffer_deq.resize(0.8*ACCELEROMETERS_SMOOTHING_WINDOW);
	m_movement_deq_smoother.resize(ceil( (double) (ACCELEROMETERS_SMOOTHING_WINDOW/2) ) );

	m_internalCounter = 0;
	m_intCounter = 0;

	m_tmpStmp = 0;
	m_previousTmpStmp = 0;
	m_tmpstmp_diff = 0;
	m_bNewValues = false;

	bIsMovementUpdated = false;

	//----------------------------------------------------------------------------------------------------------------------
	//Create Fuzzy Sets
	m_fuzzyEngine = new fl::FuzzyEngine("Fall-Detection");

	m_fuzzyEngine->hedgeSet().add(new fl::HedgeNot);
	m_fuzzyEngine->hedgeSet().add(new fl::HedgeSomewhat);
	m_fuzzyEngine->hedgeSet().add(new fl::HedgeVery);

	m_fuzzy_sma_var = new fl::InputLVar("Sma");
	m_fuzzy_sma_var->addTerm(new fl::ShoulderTerm("LOW", ACCEL_SMA_LOW_LB, ACCEL_SMA_LOW_UB, true));
	m_fuzzy_sma_var->addTerm(new fl::ShoulderTerm("HIGH", ACCEL_SMA_HIGH_LB, ACCEL_SMA_HIGH_UB, false));
	//m_fuzzy_sma_var->addTerm(new fl::TriangularTerm("OK", ACCEL_SMA_LOW_LB, ACCEL_SMA_LOW_UB));
	//m_fuzzy_sma_var->addTerm(new fl::TriangularTerm("HIGH", ACCEL_SMA_HIGH_LB, ACCEL_SMA_HIGH_UB));
	m_fuzzyEngine->addInputLVar(m_fuzzy_sma_var);

	m_fuzzy_theta_var = new fl::InputLVar("DeltaTheta");
	m_fuzzy_theta_var->addTerm(new fl::ShoulderTerm("REST", ACCEL_DELTA_THETA_REST_LB, ACCEL_DELTA_THETA_REST_UB, true));
	m_fuzzy_theta_var->addTerm(new fl::ShoulderTerm("HIGH", ACCEL_DELTA_THETA_HEAVY_LB, ACCEL_DELTA_THETA_HEAVY_UB, false));
	//m_fuzzy_theta_var->addTerm(new fl::TriangularTerm("REST", ACCEL_DELTA_THETA_REST_LB, ACCEL_DELTA_THETA_REST_UB));
	//m_fuzzy_theta_var->addTerm(new fl::TriangularTerm("HIGH", ACCEL_DELTA_THETA_HEAVY_LB, ACCEL_DELTA_THETA_HEAVY_UB));
	m_fuzzyEngine->addInputLVar(m_fuzzy_theta_var);

	m_fuzzy_phi_var = new fl::InputLVar("DeltaPhi");
	m_fuzzy_phi_var->addTerm(new fl::ShoulderTerm("REST", ACCEL_DELTA_PHI_REST_LB, ACCEL_DELTA_PHI_REST_UB, true));
	m_fuzzy_phi_var->addTerm(new fl::ShoulderTerm("HIGH", ACCEL_DELTA_PHI_HEAVY_LB, ACCEL_DELTA_PHI_HEAVY_UB, false));
	//m_fuzzy_phi_var->addTerm(new fl::TriangularTerm("REST", ACCEL_DELTA_PHI_REST_LB, ACCEL_DELTA_PHI_REST_UB));
	//m_fuzzy_phi_var->addTerm(new fl::TriangularTerm("HIGH", ACCEL_DELTA_PHI_HEAVY_LB, ACCEL_DELTA_PHI_HEAVY_UB));
	m_fuzzyEngine->addInputLVar(m_fuzzy_phi_var);

	m_fuzzy_movement_var = new fl::OutputLVar("Movement");
	m_fuzzy_movement_var->addTerm(new fl::ShoulderTerm("REST", MOV_REST_LB, MOV_REST_UB, true));
	m_fuzzy_movement_var->addTerm(new fl::TriangularTerm("AVERAGE", MOV_AVERAGE_LB, MOV_AVERAGE_UB));
	m_fuzzy_movement_var->addTerm(new fl::ShoulderTerm("HEAVY", MOV_HEAVY_LB, MOV_HEAVY_UB, false));
	//m_fuzzy_movement_var->addTerm(new fl::TriangularTerm("REST", MOV_REST_LB, MOV_REST_UB));
	//m_fuzzy_movement_var->addTerm(new fl::TriangularTerm("AVERAGE", MOV_AVERAGE_LB, MOV_AVERAGE_UB));
	//m_fuzzy_movement_var->addTerm(new fl::TriangularTerm("HEAVY", MOV_HEAVY_LB, MOV_HEAVY_UB));
	m_fuzzyEngine->addOutputLVar(m_fuzzy_movement_var);

	m_block = new fl::RuleBlock();
	//INDICATED SET OF RULES
	//m_block->addRule(new fl::MamdaniRule("if Sma is OK and DeltaTheta is REST and DeltaPhi is REST then Movement is REST", *m_fuzzyEngine));
	//m_block->addRule(new fl::MamdaniRule("if Sma is HIGH and DeltaTheta is HIGH then Movement is HEAVY", *m_fuzzyEngine));
	//m_block->addRule(new fl::MamdaniRule("if Sma is HIGH and DeltaPhi is HIGH then Movement is HEAVY", *m_fuzzyEngine));

	//FULL SET OF RULES
	m_block->addRule(new fl::MamdaniRule("if Sma is LOW and DeltaTheta is REST and DeltaPhi is REST then Movement is REST", *m_fuzzyEngine));
	m_block->addRule(new fl::MamdaniRule("if Sma is HIGH and DeltaTheta is REST and DeltaPhi is REST then Movement is REST", *m_fuzzyEngine));
	m_block->addRule(new fl::MamdaniRule("if Sma is LOW and DeltaTheta is REST and DeltaPhi is HIGH then Movement is REST", *m_fuzzyEngine));
	m_block->addRule(new fl::MamdaniRule("if Sma is HIGH and DeltaTheta is REST and DeltaPhi is HIGH then Movement is HEAVY", *m_fuzzyEngine));
	m_block->addRule(new fl::MamdaniRule("if Sma is LOW and DeltaTheta is HIGH and DeltaPhi is REST then Movement is REST", *m_fuzzyEngine));
	m_block->addRule(new fl::MamdaniRule("if Sma is HIGH and DeltaTheta is HIGH and DeltaPhi is REST then Movement is HEAVY", *m_fuzzyEngine));
	m_block->addRule(new fl::MamdaniRule("if Sma is LOW and DeltaTheta is HIGH and DeltaPhi is HIGH then Movement is HEAVY", *m_fuzzyEngine));
	m_block->addRule(new fl::MamdaniRule("if Sma is HIGH and DeltaTheta is HIGH and DeltaPhi is HIGH then Movement is HEAVY", *m_fuzzyEngine));
	
	m_block->addRule(new fl::MamdaniRule("if Sma is LOW and DeltaTheta is REST then Movement is REST", *m_fuzzyEngine));
	m_block->addRule(new fl::MamdaniRule("if Sma is HIGH and DeltaTheta is REST then Movement is AVERAGE", *m_fuzzyEngine));
	m_block->addRule(new fl::MamdaniRule("if Sma is LOW and DeltaTheta is HIGH then Movement is AVERAGE", *m_fuzzyEngine));
	m_block->addRule(new fl::MamdaniRule("if Sma is HIGH and DeltaTheta is HIGH then Movement is HEAVY", *m_fuzzyEngine));
	
	m_block->addRule(new fl::MamdaniRule("if Sma is LOW and DeltaPhi is REST then Movement is REST", *m_fuzzyEngine));
	m_block->addRule(new fl::MamdaniRule("if Sma is HIGH and DeltaPhi is REST then Movement is AVERAGE", *m_fuzzyEngine));
	m_block->addRule(new fl::MamdaniRule("if Sma is LOW and DeltaPhi is HIGH then Movement is AVERAGE", *m_fuzzyEngine));
	m_block->addRule(new fl::MamdaniRule("if Sma is HIGH and DeltaPhi is HIGH then Movement is HEAVY", *m_fuzzyEngine));
	
	m_block->addRule(new fl::MamdaniRule("if DeltaTheta is REST and DeltaPhi is REST then Movement is REST", *m_fuzzyEngine));
	m_block->addRule(new fl::MamdaniRule("if DeltaTheta is REST and DeltaPhi is HIGH then Movement is AVERAGE", *m_fuzzyEngine));
	m_block->addRule(new fl::MamdaniRule("if DeltaTheta is HIGH and DeltaPhi is REST then Movement is AVERAGE", *m_fuzzyEngine));
	m_block->addRule(new fl::MamdaniRule("if DeltaTheta is HIGH and DeltaPhi is HIGH then Movement is HEAVY", *m_fuzzyEngine));

	m_fuzzyEngine->addRuleBlock(m_block);

	m_movement_deq.clear();
	m_movement_deq.resize(OBSERVATION_FALL_TIME_IN_SECS*ACCELEROMETER_FREQUENCY);

	
	m_finalMLP_decision_deq.clear();
	m_finalMLP_decision_deq.resize(MLP_DECISIONS_SIZE);
	
}


FallDetect::~FallDetect()
{
	for(int i=0; i<m_accelVals_deq.size(); i++)
	{
		m_accelVals_deq[i].clear();
	}
	m_accelVals_deq.clear();

	m_r_deq.clear();

	m_theta_deq.clear();
	m_deltaTheta_deq.clear();
	m_deltaThetaBuffer_deq.clear();
	m_deltaTheta_secondBuffer_deq.clear();
	m_dDeltaTheta_secondBuffer = 0.0;
	//m_deltaThetaNEW_deq.clear();

	m_phi_deq.clear();
	m_deltaPhi_deq.clear();
	m_deltaPhiBuffer_deq.clear();
	m_deltaPhi_secondBuffer_deq.clear();
	m_dDeltaPhi_secondBuffer = 0.0;
	//m_deltaPhiNEW_deq.clear();

	m_sma_deq.clear();
	m_smaBuffer_deq.clear();
	m_sma_secondBuffer_deq.clear();
	m_dSma_secondBuffer = 0.0;

	m_movement_deq_smoother.clear();

	m_counter = 0;
	m_intCounter = 0;

	m_movement_deq.clear();
	m_finalMLP_decision_deq.clear();
	delete m_fuzzyEngine;
}



//void FallDetect::startRecordingForAccelerometer(std::string subjectStoringPath_str)
//{
//	USES_CONVERSION;
//
//	//Accelerometer Subfolders
//	std::string accelerometer_TimestampStoringPath_cstr;
//	std::string accelerometer_StoringPath_cstr = subjectStoringPath_str;
//	/*if(m_bAccelerometer_Initialized)
//	{*/
//		accelerometer_TimestampStoringPath_cstr = accelerometer_StoringPath_cstr;
//
//		accelerometer_StoringPath_cstr.append(FILENAME_ACCELEROMETER_ANGLES);
//		accelerometer_TimestampStoringPath_cstr.append(FILENAME_TIMESTAMP);
//	//}
//
//	m_bAccelerometerRecorderStopped = false;
//
//	m_ofs_data.open(accelerometer_StoringPath_cstr);
//	m_ofs_timestamp.open(accelerometer_TimestampStoringPath_cstr);
//
//	RunInThreadAccelerometerRecorder(this);
//
//	//if(m_hThreadAccelerometerRecorder==NULL)
//	//{
//	//	/*if(m_bAccelerometer_Initialized)
//	//	{*/
//	//		m_ofs_data.open(accelerometer_StoringPath_cstr);
//	//		m_ofs_timestamp.open(accelerometer_TimestampStoringPath_cstr);
//	//	//}
//
//	//	m_hThreadAccelerometerRecorder = AfxBeginThread(AFX_THREADPROC(RunInThreadAccelerometerRecorder), 
//	//		this,
//	//		(int)THREAD_PRIORITY_BELOW_NORMAL, //THREAD_PRIORITY_NORMAL
//	//		0,
//	//		CREATE_SUSPENDED, 
//	//		0);
//	//	
//	//	m_hThreadAccelerometerRecorder->m_bAutoDelete = TRUE;
//	//	m_hThreadAccelerometerRecorder->ResumeThread();
//	//	
//	//}
//
//	//return true;
//}
//
//
//void FallDetect::startMonitoringForAccelerometer(std::string subjectStoringPath_str)
//{
//	USES_CONVERSION;
//
//	//if(!m_bAccelerometer_Initialized)
//	//{
//	//	//AddUIString(_T("Accelerometer has to be initialized first..."));
//	//	return false;
//	//}
//
//	//Accelerometer Subfolders
//	std::string accelerometer_TimestampStoringPath_cstr;
//	std::string accelerometer_StoringPath_cstr = subjectStoringPath_str.c_str();
//	/*if(m_bAccelerometer_Initialized)
//	{*/		
//		accelerometer_TimestampStoringPath_cstr = accelerometer_StoringPath_cstr;
//
//		accelerometer_StoringPath_cstr.append(FILENAME_ACCELEROMETER_ANGLES);
//		accelerometer_TimestampStoringPath_cstr.append(FILENAME_TIMESTAMP);
//	//}
//
//	m_bAccelerometerTrackerStopped = false;	
//
//	m_ofs_data.open(accelerometer_StoringPath_cstr);
//	m_ofs_timestamp.open(accelerometer_TimestampStoringPath_cstr);
//
//	RunInThreadAccelerometerTracker(this);
//
//	//boost::thread thrd(&RunInThreadAccelerometerTracker);
//
//	//if(m_hThreadAccelerometerTracker==NULL)
//	//{
//	//	/*if(m_bAccelerometer_Initialized)
//	//	{*/
//	//		m_ofs_data.open(accelerometer_StoringPath_cstr);
//	//		m_ofs_timestamp.open(accelerometer_TimestampStoringPath_cstr);
//	//	//}		
//
//	//	m_hThreadAccelerometerTracker = AfxBeginThread(AFX_THREADPROC(RunInThreadAccelerometerTracker), 
//	//		this,
//	//		(int)THREAD_PRIORITY_BELOW_NORMAL, //THREAD_PRIORITY_NORMAL
//	//		0,
//	//		CREATE_SUSPENDED, 
//	//		0);
//	//	
//	//	m_hThreadAccelerometerTracker->m_bAutoDelete = TRUE;
//	//	m_hThreadAccelerometerTracker->ResumeThread();
//	//}
//
//	//return true;
//}


//double FallDetect::getAccelXVal()
//{
//	if(m_hThreadAccelerometerTracker!=NULL)
//	{
//		return  m_pPhidgetSensor->past_sampleX;
//	}
//	else
//	{
//		return -1.0;
//	}
//}
//
//
//double FallDetect::getAccelYVal()
//{
//	if(m_hThreadAccelerometerTracker!=NULL)
//	{
//		return  m_pPhidgetSensor->past_sampleY;
//	}
//	else
//	{
//		return -1.0;
//	}
//}
//
//
//double FallDetect::getAccelZVal()
//{
//	if(m_hThreadAccelerometerTracker!=NULL)
//	{
//		return  m_pPhidgetSensor->past_sampleZ;
//	}
//	else
//	{
//		return -1.0;
//	}
//}


bool FallDetect::updateAccelValues(double xVal,double yVal,double zVal, Timestamp tmpStmp)
{
	//m_tmpStmp = systemTime();
	m_tmpStmp = tmpStmp;
	m_accel_x = xVal;
	m_accel_y = yVal;
	m_accel_z = zVal;
	m_bUpdatedValues1 = true;
	m_bUpdatedValues2 = true;
	m_bUpdatedValues3 = true;

	m_tmpstmp_diff = m_previousTmpStmp-m_tmpStmp;

	if(m_tmpstmp_diff!=0)
	{
		m_bNewValues = true;
	}
	else
	{
		m_bNewValues = false;
	}

	m_previousTmpStmp = m_tmpStmp;

	return true;
}


bool FallDetect::calcR()
{
	m_r = sqrt(pow(m_accel_x, 2.0) + pow(m_accel_y, 2.0) + pow(m_accel_z, 2.0));
	//m_r = 0.296927036;
	m_r_deq.push_back(m_r);
	m_r_deq.pop_front();
	m_previousR = m_r;

	return true;
}


bool FallDetect::calcPhi()
{
	m_phi = atan2(m_accel_y, m_accel_x);
	m_phi_deq.push_back(m_phi);
	m_phi_deq.pop_front();

	return true;
}


//calcPhi has to run first
bool FallDetect::calcDeltaPhiDer()
{
	m_deltaPhiDer = 1000*(m_phi - m_previousPhi)/m_tmpstmp_diff.timeMilliSeconds();
	m_previousPhi = m_phi;

	return true;
}


bool FallDetect::calcTheta()
{
	m_theta = (PI/2) - atan((m_accel_z)/sqrt(pow(m_accel_x, 2.0) + pow(m_accel_y, 2.0)));
	m_theta_deq.push_back(m_theta);
	m_theta_deq.pop_front();

	return true;
}


//calcTheta has to run first
bool FallDetect::calcDeltaThetaDer()
{
	m_deltaThetaDer = 1000*(m_theta - m_previousTheta)/m_tmpstmp_diff.timeMilliSeconds();
	m_previousTheta = m_theta;

	return true;
}


//bool FallDetect::calcSMA()
//{
//	if(m_counter<ACCELEROMETER_SAMPLES_FOR_SMA)
//	{
//		return -1;
//	}
//
//	double m_sma = 0;
//	int counter = 0;
//	for(int i=0; i<ACCELEROMETER_SAMPLES_FOR_SMA; i++)
//	{
//		m_sma = m_sma + abs(m_accelVals_deq[0][i]) + abs(m_accelVals_deq[1][i]) + abs(m_accelVals_deq[2][i]);
//		counter++;
//	}
//	m_sma = m_sma/counter;
//
//	m_sma_deq.push_back(m_sma);
//	m_sma_deq.pop_front();
//
//	return true;
//}


int FallDetect::RunInThreadAccelerometerRecorder(void* pv_Para)
{
	FallDetect* pointer = (FallDetect*) pv_Para;

	Timestamp tmpstmp;
	Timestamp initialTmpstmp;
	int counter = 0;
	pointer->m_counter = 0;
	std::vector<double> accelFeatures;
	bool bFirstPass = false;
	
	if(pointer)
	{
		while(true)
		{
			::Sleep(ACCELEROMETER_TIMER_PERIOD);
			if( bFirstPass && (tmpstmp.secondsTotal()-initialTmpstmp.secondsTotal() > OBSERVATION_FALL_TIME_IN_SECS + WAITING_TIME_BEFORE_RECORDING_IN_SECS) )
			{
				pointer->m_bAccelerometerRecorderStopped = true;
			}
			if(pointer->m_bAccelerometerRecorderStopped || counter>ACCELEREOMETER_MAXIMUM_RECORDING_CYCLES_NUMBER)
			{
				break;
			}

			if(pointer->m_bVectorsAreUpdated /*&& pointer->m_bNewValues*/)
			{
				tmpstmp = pointer->m_tmpStmp;

				if(!bFirstPass)
				{
					::Beep(7000, 200);
					Sleep(WAITING_TIME_BEFORE_RECORDING_IN_SECS*1000);

					initialTmpstmp = pointer->m_tmpStmp;
					bFirstPass = true;

					::Beep(5000, 200);
				}
				
				pointer->m_ofs_timestamp << std::setfill('0') << std::setw(12) 
					<< tmpstmp.secondsTotal() << "." << std::setfill('0') << std::setw(3) 
					<< tmpstmp.timeMilliSeconds() << "\t" << std::setfill(' ') << "\n";

				//pointer->m_ofs_data << pointer->getCurrentSMA() << "\t" 
				//		<< pointer->getCurrentDeltaTheta() << "\t" 
				//		<< pointer->getCurrentDeltaPhi() << "\n";

				pointer->m_ofs_data << pointer->getR() << "\t" 
						<< pointer->getDeltaTheta() << "\t" 
						<< pointer->getDeltaPhi() << "\n";

				pointer->m_bVectorsAreUpdated = false;
			}
			counter++;
			pointer->m_counter++;
		}

		/*if(pointer->m_bAccelerometer_Initialized)
		{*/
			pointer->m_ofs_timestamp.close();
			pointer->m_ofs_timestamp.clear();

			pointer->m_ofs_data.close();
			pointer->m_ofs_data.clear();

			::Beep(3500, 200);
		//}
	}

	//pointer->m_hThreadAccelerometerRecorder = NULL;

	return 0;
}

int FallDetect::RunInThreadAccelerometerTracker(void* pv_Para)
{
	FallDetect* pointer = (FallDetect*) pv_Para;

	Timestamp tmpstmp;
	int counter = 0;
	pointer->m_counter = 0;
	std::vector<double> accelFeatures;
	//int debug_counter = 0;
	
	if(pointer)
	{
		while(true)
		{
			::Sleep(ACCELEROMETER_TIMER_PERIOD);
			if(pointer->m_bAccelerometerTrackerStopped || counter>ACCELEREOMETER_MAXIMUM_RECORDING_CYCLES_NUMBER )
			{
				break;
			}

			//if(pointer->m_bAccelerometer_Initialized && pointer->m_bVectorsAreUpdated /*&& pointer->m_bNewValues*/)
			if(pointer->m_bVectorsAreUpdated /*&& pointer->m_bNewValues*/)
			{
				//accelFeatures = pointer->m_pPhidgetSensor->getAccelFeaturesInstant();
				tmpstmp = pointer->m_tmpStmp;

				/*pointer->m_accelVals_deq[0].push_back(accelFeatures[0]);
				pointer->m_accelVals_deq[0].pop_front();
				pointer->m_accelVals_deq[1].push_back(accelFeatures[1]);
				pointer->m_accelVals_deq[1].pop_front();
				pointer->m_accelVals_deq[2].push_back(accelFeatures[2]);
				pointer->m_accelVals_deq[2].pop_front();*/

				pointer->m_accelVals_deq[0].push_back(pointer->m_accel_x);
				pointer->m_accelVals_deq[0].pop_front();
				pointer->m_accelVals_deq[1].push_back(pointer->m_accel_y);
				pointer->m_accelVals_deq[1].pop_front();
				pointer->m_accelVals_deq[2].push_back(pointer->m_accel_z);
				pointer->m_accelVals_deq[2].pop_front();

				

				pointer->m_ofs_timestamp << std::setfill('0') << std::setw(12) << tmpstmp.secondsTotal() 
					<< "." << std::setfill('0') << std::setw(3) << tmpstmp.timeMilliSeconds() 
					<< "\t" << std::setfill(' ');


				/*for(int i=0; i<accelFeatures.size(); i++)
				{
					pointer->m_ofs_data << accelFeatures[i];
					pointer->m_ofs_data << "\t";
				}*/

				pointer->m_ofs_data << pointer->m_accel_x;
				pointer->m_ofs_data << "\t";
				pointer->m_ofs_data << pointer->m_accel_y;
				pointer->m_ofs_data << "\t";
				pointer->m_ofs_data << pointer->m_accel_z;
				pointer->m_ofs_data << "\t";

				pointer->m_ofs_timestamp << "\n";
				pointer->m_ofs_data << "\n";
				//myfile << "\n";
				//TRACE("OutPut: %f\n", accelFeatures[0]);

				pointer->m_bVectorsAreUpdated = false;

				/*wchar_t buffer[256];
				wsprintfW(buffer, L"%d", debug_counter);
				OutputDebugString(buffer);
				OutputDebugString(_T("\n"));
				debug_counter++;*/
			}

			counter++;
			pointer->m_counter++;
		}

		/*if(pointer->m_bAccelerometer_Initialized)
		{*/
			pointer->m_ofs_timestamp.close();
			pointer->m_ofs_timestamp.clear();

			pointer->m_ofs_data.close();
			pointer->m_ofs_data.clear();
		//}

		//pointer->m_hThreadAccelerometerTracker = NULL;
	}

	return 0;
}


void FallDetect::reSizeSignature(std::deque <std::deque<double>> &deq, int nbData)
{
	GaussianMixture gMixture;
	deq = gMixture.resizeDeque(deq, nbData);
}


std::deque<double> FallDetect::calcAll_Offline(std::deque<std::deque<double>> inputData_deq, std::deque<std::deque<double>> inputTime_deq)
{
	double dCurrentTime = 0.0;
	double dPreviousTime = inputTime_deq[0][1];

	std::deque<std::deque<double>> timeDiff_deq;
	timeDiff_deq.resize(inputTime_deq.size()-1);
	for(int i=1; i<inputTime_deq.size(); i++)
	{
		dCurrentTime = inputTime_deq[i][1];

		timeDiff_deq[i-1].push_back(i-1);
		timeDiff_deq[i-1].push_back( (dCurrentTime - dPreviousTime)*1000 );

		dPreviousTime = dCurrentTime;
	}

	//trick
	timeDiff_deq[0][1] = timeDiff_deq[1][1];

	reSizeSignature(inputData_deq, OBSERVATION_FALL_TIME_IN_SECS*ACCELEROMETER_FREQUENCY);
	reSizeSignature(timeDiff_deq, OBSERVATION_FALL_TIME_IN_SECS*ACCELEROMETER_FREQUENCY);

	double dCurrent_movement = 0.0;
	std::deque<double> movementBuffer_deq;
	movementBuffer_deq.resize(ceil( (double) (ACCELEROMETERS_SMOOTHING_WINDOW/2) ) );
	std::deque<double> movementSecondBuffer_deq;

	int internalCounter = 0;
	double dSma_secondBuffer = 0.0;
	double dSmaForFIS = 0.0;
	std::deque<double> smaBuffer_deq;
	smaBuffer_deq.resize(0.8*ACCELEROMETERS_SMOOTHING_WINDOW);
	std::deque<double> sma_secondBuffer_deq;
	
	double dDeltaTheta_secondBuffer = 0.0;
	double dDeltaThetaForFIS = 0.0;
	std::deque<double> deltaThetaBuffer_deq;
	deltaThetaBuffer_deq.resize(ACCELEROMETERS_SMOOTHING_WINDOW);
	std::deque<double> deltaTheta_secondBuffer_deq;

	double dDeltaPhi_secondBuffer = 0.0;
	double dDeltaPhiForFIS = 0.0;
	std::deque<double> deltaPhiBuffer_deq;
	deltaPhiBuffer_deq.resize(ACCELEROMETERS_SMOOTHING_WINDOW);
	std::deque<double> deltaPhi_secondBuffer_deq;

	//double maxVal_sma = 0;
	//double maxVal_dt = 0;
	//double maxVal_dp = 0;

	for(int i=0; i<inputData_deq.size(); i++)
	{
		smaBuffer_deq.push_back(inputData_deq[i][1]);
		smaBuffer_deq.pop_front();
		dSma_secondBuffer = getMean(smaBuffer_deq);

		deltaThetaBuffer_deq.push_back(inputData_deq[i][2]);
		deltaThetaBuffer_deq.pop_front();
		dDeltaTheta_secondBuffer = getMean(deltaThetaBuffer_deq)/timeDiff_deq[i][1];
		
		deltaPhiBuffer_deq.push_back(inputData_deq[i][3]);
		deltaPhiBuffer_deq.pop_front();
		dDeltaPhi_secondBuffer = getMean(deltaPhiBuffer_deq)/timeDiff_deq[i][1];

		if(ACCEL_FEATURES_DOWNSAMPLING_RATES == 1)
		{
			dSmaForFIS = dSma_secondBuffer;
			dDeltaThetaForFIS = dDeltaTheta_secondBuffer;
			dDeltaPhiForFIS = dDeltaPhi_secondBuffer;
		}
		else
		{
			if(internalCounter<1/ACCEL_FEATURES_DOWNSAMPLING_RATES)
			{
				sma_secondBuffer_deq.push_back(dSma_secondBuffer);
				deltaTheta_secondBuffer_deq.push_back(dDeltaTheta_secondBuffer);
				deltaPhi_secondBuffer_deq.push_back(dDeltaPhi_secondBuffer);
				internalCounter++;
			}
			else
			{
				dSmaForFIS = getMean(sma_secondBuffer_deq);
				sma_secondBuffer_deq.clear();
				dDeltaThetaForFIS = getMean(deltaTheta_secondBuffer_deq);
				deltaTheta_secondBuffer_deq.clear();
				dDeltaPhiForFIS = getMean(deltaPhi_secondBuffer_deq);
				deltaPhi_secondBuffer_deq.clear();
				internalCounter=0;
			}
		}

		dCurrent_movement = doFIS(dSmaForFIS, abs(dDeltaThetaForFIS), abs(dDeltaPhiForFIS));

		//if(maxVal_sma<dSmaForFIS)
		//{
		//	maxVal_sma=dSmaForFIS;
		//}
		//if(maxVal_dt<abs(dDeltaThetaForFIS))
		//{
		//	maxVal_dt=abs(dDeltaThetaForFIS);
		//}
		//if(maxVal_dp<abs(dDeltaPhiForFIS))
		//{
		//	maxVal_dp=abs(dDeltaPhiForFIS);
		//}
		

		if(!isIndeterminate(dCurrent_movement))
		{
			movementBuffer_deq.push_back(dCurrent_movement);
			movementBuffer_deq.pop_front();
		}
		else
		{
			movementBuffer_deq.push_back(-1.0);
			movementBuffer_deq.pop_front();
		}

		movementSecondBuffer_deq.push_back(getMean(movementBuffer_deq)); 

	}

	return movementSecondBuffer_deq;
}


bool FallDetect::calcAll(double xVal, double yVal, double zVal, Timestamp tmpStmp)
{
	USES_CONVERSION;

	bool retVal = true;

	if(ENABLE_PLOT_R+ENABLE_PLOT_THETA+ENABLE_PLOT_DELTA_THETA+ENABLE_PLOT_PHI+ENABLE_PLOT_DELTA_PHI+ENABLE_PLOT_SMA+ENABLE_PLOT_MOVEMENT>0)
	{
		updateAccelValues(xVal,yVal,zVal,tmpStmp);
	}
	if(ENABLE_PLOT_R+ENABLE_PLOT_SMA+ENABLE_PLOT_MOVEMENT>0)
	{
		retVal = retVal & calcR();
	}
	if(ENABLE_PLOT_THETA+ENABLE_PLOT_DELTA_THETA+ENABLE_PLOT_MOVEMENT>0)
	{
		retVal = retVal & calcTheta();
		calcDeltaThetaDer();
	}
	if(ENABLE_PLOT_PHI+ENABLE_PLOT_DELTA_PHI+ENABLE_PLOT_MOVEMENT>0)
	{
		retVal = retVal & calcPhi();
		calcDeltaPhiDer();
	}
	//if(ENABLE_PLOT_SMA>0)
	//{
	//	retVal = retVal & calcSMA();
	//}

	if(ENABLE_PLOT_SMA+ENABLE_PLOT_MOVEMENT>0)
	{
		if(m_bUpdatedValues1)
		{
			m_bUpdatedValues1 = false;
			m_smaBuffer_deq.push_back(m_r);
		}
		else
		{
			m_smaBuffer_deq.push_back(0);
		}
		m_smaBuffer_deq.pop_front();

		m_dSma_secondBuffer = getMean(m_smaBuffer_deq);
		
		//m_sma_secondBuffer_deq.push_back(getMean(m_smaBuffer_deq));
		//m_sma_deq.push_back(getMean(m_smaBuffer_deq));
		//m_sma_deq.pop_front();
	}

	if(ENABLE_PLOT_DELTA_THETA+ENABLE_PLOT_MOVEMENT>0)
	{
		if(m_bUpdatedValues2)
		{
			m_bUpdatedValues2 = false;
			m_deltaThetaBuffer_deq.push_back(m_deltaThetaDer);
		}
		else
		{
			m_deltaThetaBuffer_deq.push_back(0);
		}
		m_deltaThetaBuffer_deq.pop_front();

		m_dDeltaTheta_secondBuffer = getMean(m_deltaThetaBuffer_deq);

		//m_deltaTheta_secondBuffer_deq.push_back(getMean(m_deltaThetaBuffer_deq));
		//m_deltaTheta_deq.push_back(getMean(m_deltaThetaBuffer_deq));
		//m_deltaTheta_deq.pop_front();
	}

	if(ENABLE_PLOT_DELTA_PHI+ENABLE_PLOT_MOVEMENT>0)
	{
		if(m_bUpdatedValues3)
		{
			m_bUpdatedValues3 = false;
			m_deltaPhiBuffer_deq.push_back(m_deltaPhiDer);
		}
		else
		{
			m_deltaPhiBuffer_deq.push_back(0);
		}
		m_deltaPhiBuffer_deq.pop_front();

		m_dDeltaPhi_secondBuffer = getMean(m_deltaPhiBuffer_deq);

		//m_deltaPhi_secondBuffer_deq.push_back(getMean(m_deltaPhiBuffer_deq));
		//m_deltaPhi_deq.push_back(getMean(m_deltaPhiBuffer_deq));
		//m_deltaPhi_deq.pop_front();
	}

	if(ACCEL_FEATURES_DOWNSAMPLING_RATES == 1)
	{
		m_current_sma = m_dSma_secondBuffer;
		m_sma_deq.push_back(m_current_sma);
		m_sma_deq.pop_front();

		m_current_deltaTheta = m_dDeltaTheta_secondBuffer;
		m_deltaTheta_deq.push_back(m_current_deltaTheta);
		m_deltaTheta_deq.pop_front();

		m_current_deltaPhi = m_dDeltaPhi_secondBuffer;
		m_deltaPhi_deq.push_back(m_current_deltaPhi);
		m_deltaPhi_deq.pop_front();
	}
	else
	{
		if(m_internalCounter<1/ACCEL_FEATURES_DOWNSAMPLING_RATES)
		{
			m_sma_secondBuffer_deq.push_back(m_dSma_secondBuffer);
			m_deltaTheta_secondBuffer_deq.push_back(m_dDeltaTheta_secondBuffer);
			m_deltaPhi_secondBuffer_deq.push_back(m_dDeltaPhi_secondBuffer);
			m_internalCounter++;
		}
		else
		{
			if(ENABLE_PLOT_SMA+ENABLE_PLOT_MOVEMENT>0)
			{
				m_current_sma = getMean(m_sma_secondBuffer_deq);
				m_sma_deq.push_back(m_current_sma);
				m_sma_deq.pop_front();
				m_sma_secondBuffer_deq.clear();
			}
			if(ENABLE_PLOT_DELTA_THETA+ENABLE_PLOT_MOVEMENT>0)
			{
				m_current_deltaTheta = getMean(m_deltaTheta_secondBuffer_deq);
				m_deltaTheta_deq.push_back(m_current_deltaTheta);
				m_deltaTheta_deq.pop_front();
				m_deltaTheta_secondBuffer_deq.clear();
			}
			if(ENABLE_PLOT_DELTA_PHI+ENABLE_PLOT_MOVEMENT>0)
			{
				m_current_deltaPhi = getMean(m_deltaPhi_secondBuffer_deq);
				m_deltaPhi_deq.push_back(m_current_deltaPhi);
				m_deltaPhi_deq.pop_front();
				m_deltaPhi_secondBuffer_deq.clear();
			}
			m_internalCounter=0;
		}
	}

	bIsMovementUpdated = false;
	if(( ENABLE_PLOT_SMA && ENABLE_PLOT_DELTA_PHI && ENABLE_PLOT_DELTA_THETA) || ENABLE_PLOT_MOVEMENT)
	{
		m_current_fis = doFIS(m_current_sma, m_current_deltaTheta, m_current_deltaPhi);

		if(!isIndeterminate(m_current_fis))
		{
			m_movement_deq_smoother.push_back(m_current_fis);
			m_movement_deq_smoother.pop_front();
		}
		else
		{
			m_movement_deq_smoother.push_back(0);
			m_movement_deq_smoother.pop_front();
		}

		m_movement_deq.push_back(getMean(m_movement_deq_smoother)); 
		m_movement_deq.pop_front();

		bIsMovementUpdated = true;
	}

	if(m_intCounter>=ACCELEROMETER_FREQUENCY)
	{
		//if(ENABLE_PLOT_SMA>0)
		//{
		//	//m_deltaThetaNEW_deq.clear();
		//	//downsampleFeatureVector(m_sma_deq, ACCEL_FEATURES_DOWNSAMPLING_RATES);
		//}

		//if(ENABLE_PLOT_DELTA_THETA>0)
		//{
		//	//smoothFeatureVector(m_deltaTheta_deq, ACCELEROMETERS_SMOOTHING_WINDOW);
		//	//m_deltaThetaNEW_deq.clear();
		//	//downsampleFeatureVector(m_deltaTheta_deq, ACCEL_FEATURES_DOWNSAMPLING_RATES);
		//}
		//if(ENABLE_PLOT_DELTA_PHI>0)
		//{
		//	//smoothFeatureVector(m_deltaPhi_deq, ACCELEROMETERS_SMOOTHING_WINDOW);
		//	//m_deltaPhiNEW_deq.clear();
		//	//downsampleFeatureVector(m_deltaPhi_deq, ACCEL_FEATURES_DOWNSAMPLING_RATES);
		//}
		//m_bPlotFlag = true;
		m_bVectorsAreUpdated = true;
	}
	else
	{
		//m_bPlotFlag = false;
		m_bVectorsAreUpdated = false;
		m_intCounter++;
	}

	return retVal;
}



bool FallDetect::checkUsersStatus(double xVal, double yVal, double zVal, Timestamp tmpStmp)
{
	m_psdFreq_deq.clear();
	//m_proportions_deq.clear();

	if(m_counter<ACCELEROMETER_SAMPLES_FOR_SMA)
	{
		m_counter++;
		return false;
	}

	calcAll(xVal, yVal, zVal, tmpStmp);

	//if(ENABLE_PLOT_PSD>0)
	//{
	//	std::deque<std::deque<double>> inv_accelVals_deq = convertRawsToColumns(m_accelVals_deq);
	//	double meanVal = 0;
	//	
	//	for(int i=0; i<inv_accelVals_deq.size(); i++)
	//	{
	//		meanVal = getMean(inv_accelVals_deq[i]);
	//		for(int j=0; j<inv_accelVals_deq[i].size(); j++)
	//		{
	//			inv_accelVals_deq[i][j] = inv_accelVals_deq[i][j] - meanVal;
	//		}
	//	}

	//	m_psdFreq_deq = getPSDFFT3W(inv_accelVals_deq, 0, inv_accelVals_deq.size()-1, fDParams.m_dftSampling);

	//	int a = fDParams.m_dftSampling/2;
	//	int b = ACCELEROMETER_SAMPLES_FOR_SMA;

	//	int minHz_index = (a*minHz)/b;
	//	int maxHz_index = (a*maxHz)/b;

	//	double fullBandSignalPwr=0.0;
	//	double lowerBandSignalPwr=0.0;
	//	double middleBandSignalPwr=0.0;
	//	double higherBandSignalPwr=0.0;
	//	for(int i=0; i<m_psdFreq_deq.size(); i++)
	//	{
	//		fullBandSignalPwr = fullBandSignalPwr + m_psdFreq_deq[i];
	//		if(i<minHz_index)
	//		{
	//			lowerBandSignalPwr = lowerBandSignalPwr + m_psdFreq_deq[i];
	//		}
	//		if( (i>=minHz_index) && (i<=maxHz_index) )
	//		{
	//			middleBandSignalPwr = middleBandSignalPwr + m_psdFreq_deq[i];
	//		}
	//		if(i>maxHz_index)
	//		{
	//			higherBandSignalPwr = higherBandSignalPwr + m_psdFreq_deq[i];;
	//		}
	//	}

	//	char cBuf[512];
	//	std::string strBuf;
	//	sprintf(cBuf, "%f", lowerBandSignalPwr/fullBandSignalPwr);
	//	strBuf = cBuf;
	//	//m_proportions_deq.push_back(strBuf);

	//	sprintf(cBuf, "%f", middleBandSignalPwr/fullBandSignalPwr);
	//	strBuf = cBuf;
	//	//m_proportions_deq.push_back(strBuf);

	//	sprintf(cBuf, "%f", higherBandSignalPwr/fullBandSignalPwr);
	//	strBuf = cBuf;
	//	//m_proportions_deq.push_back(strBuf);
	//}

	return true;
}

int FallDetect::countOnes(std::deque<int> inDeq)
{
	int onesSum = 0;
	for(int i=0; i<inDeq.size(); i++)
	{
		if(inDeq[i]==1)
		{
			onesSum++;
		}
	}

	return onesSum;
}

bool FallDetect::detectFall(std::string storedMLPPath)
{
	int iTimesDecidedFallen = 0;
	double mlpVal =0.0;
	bool bHasFallen = false;
	bool fall = false;

	MLPClass* mlp = new MLPClass();
	mlp->loadNewMLP(storedMLPPath);
	bHasFallen = mlp->checkPattern(getMovement_deq(), mlpVal, MLP_THRESHOLD_VAL);

	if(bHasFallen)
	{
		m_finalMLP_decision_deq.push_back(1);
		m_finalMLP_decision_deq.pop_front();
	}
	else
	{
		m_finalMLP_decision_deq.push_back(0);
		m_finalMLP_decision_deq.pop_front();
	}

	cout << mlpVal << "\n";

	iTimesDecidedFallen = countOnes(m_finalMLP_decision_deq);
	if(iTimesDecidedFallen > MLP_DCISIONS_THRESHOLD_VAL)
	{
		fall = true;
	}
	else
	{
		fall = false;
	}

	if(mlp!=NULL)
	{
		delete mlp;
		mlp = NULL;
	}

	return fall;
}

std::deque< std::deque<double> > FallDetect::convertRawsToColumns(std::deque< std::deque<double> > inputMatrix_deq)
{
	std::deque< std::deque<double> > featuresMatrix;
	featuresMatrix.clear();

	if(inputMatrix_deq.size()==0)
	{
		return featuresMatrix;
	}


	featuresMatrix.resize(inputMatrix_deq[0].size());
	for(int i=0; i<featuresMatrix.size(); i++)
	{
		featuresMatrix[i].resize(inputMatrix_deq.size());
	}

	for(int j=0; j<inputMatrix_deq[0].size(); j++)
	{
		for(int i=0; i<inputMatrix_deq.size(); i++)
		{
			featuresMatrix[j][i] = inputMatrix_deq[i][j];
		}
	}

	return featuresMatrix;
}


//std::deque<double> FallDetect::getPSDFFT3W(std::deque< std::deque<double> > inputData_deq, int startingIndex, int windowSize, int sampling)
//{
//	std::deque<double> psd_deq;
//
//	if(startingIndex+windowSize>inputData_deq.size())
//	{
//		return psd_deq;
//	}
//
//	fftw_complex *inpX, *outX, *inpY, *outY, *inpZ, *outZ;
//	fftw_plan pX, pY, pZ;
//
//	int N = sampling;
//
//	inpX = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
//	outX = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
//	inpY = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
//	outY = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
//	inpZ = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
//	outZ = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * N);
//
//	for (int i = 0; i < windowSize; ++i)
//	{
//		inpX[i][0] = inputData_deq[startingIndex+i][0];
//		inpX[i][1] = 0.;
//
//		inpY[i][0] = inputData_deq[startingIndex+i][1];
//		inpY[i][1] = 0.;
//
//		inpZ[i][0] = inputData_deq[startingIndex+i][2];
//		inpZ[i][1] = 0.;
//	}
//	for (int i=windowSize; i<N; i++)
//	{
//		inpX[i][0] = 0.;
//		inpX[i][1] = 0.;
//		inpY[i][0] = 0.;
//		inpY[i][1] = 0.;
//		inpZ[i][0] = 0.;
//		inpZ[i][1] = 0.;
//	}
//
//	pX = fftw_plan_dft_1d(N, inpX, outX, FFTW_FORWARD, FFTW_ESTIMATE);
//	pY = fftw_plan_dft_1d(N, inpY, outY, FFTW_FORWARD, FFTW_ESTIMATE);
//	pZ = fftw_plan_dft_1d(N, inpZ, outZ, FFTW_FORWARD, FFTW_ESTIMATE);
//
//	fftw_execute(pX); /* repeat as needed */
//	fftw_execute(pY); /* repeat as needed */
//	fftw_execute(pZ); /* repeat as needed */
//
//	std::deque< std::deque<double> > pFrequency_deq;
//	pFrequency_deq.resize(N);
//	psd_deq.resize(N);
//	for(int i = 0; i < N; i++)
//	{
//		pFrequency_deq[i].resize(inputData_deq[0].size()+1);
//		pFrequency_deq[i][0] = sqrt(outX[i][0]*outX[i][0] + outX[i][1]*outX[i][1]);//*0.39894228040143267793994605993438;
//		pFrequency_deq[i][1] = sqrt(outY[i][0]*outY[i][0] + outY[i][1]*outY[i][1]);//*0.39894228040143267793994605993438;
//		pFrequency_deq[i][2] = sqrt(outZ[i][0]*outZ[i][0] + outZ[i][1]*outZ[i][1]);//*0.39894228040143267793994605993438;
//		psd_deq[i] = (pFrequency_deq[i][0] + pFrequency_deq[i][1] + pFrequency_deq[i][2])/3;
//	}
//
//	fftw_destroy_plan(pX);
//	fftw_destroy_plan(pY);
//	fftw_destroy_plan(pZ);
//	fftw_free(inpX);
//	fftw_free(outX);
//	fftw_free(inpY);
//	fftw_free(outY);
//	fftw_free(inpZ);
//	fftw_free(outZ);
//
//	return psd_deq;
//}

double FallDetect::getMean(std::deque<double> sequence)
{

	int seqSize = sequence.size();
	double mean = 0;

	for(int i=0; i<seqSize; i++)
	{
		mean = mean + sequence[i];
	}

	mean = mean/seqSize;

	return mean;
}


double FallDetect::getVariance(std::deque<double> sequence)
{
	double var = 0;
	double sum = 0;

	double mean = getMean(sequence);

	int seqSize = sequence.size();
	for (int i = 0; i<seqSize; i++){
		sum = sum + (sequence[i] - mean)*(sequence[i] - mean);
	}

	var = sum/seqSize;
	return var;
}


double FallDetect::getMinVal(std::deque<double> sequence)
{
	int seqSize = sequence.size();
	if(seqSize==0)
	{
		return 0.0;
	}

	double minVal = sequence[0];

	for (int i = 0; i<sequence.size(); i++)
	{
		if(minVal>sequence[i])
		{
			minVal = sequence[i];
		}
	}
}


double FallDetect::getMaxVal(std::deque<double> sequence)
{
	int seqSize = sequence.size();
	if(seqSize==0)
	{
		return 0.0;
	}

	double maxVal = sequence[0];

	for (int i = 0; i<sequence.size(); i++)
	{
		if(maxVal<sequence[i])
		{
			maxVal = sequence[i];
		}
	}
}


std::string FallDetect::getInternalCounterString()
{
	std::string strCounter;

	char cCounter[MAX_PATH];
	sprintf(cCounter, "%d", m_counter);
	strCounter = cCounter;

	return strCounter;
}

//void FallDetect::smoothFeatureVector(std::deque<double> &deq, int window)
//{
//	int nSamples = (int) deq.size();
//	if (nSamples == 0)
//	{
//		return;
//	}
//
//	//start -> end
//	double* dFeat_AB = new double[nSamples];
//	for (int r=0; r<nSamples; r++)
//	{
//		dFeat_AB[r] = deq[r];
//	}
//	smoothData(dFeat_AB, nSamples, window);
//
//	//end -> start
//	double* dFeat_BA = new double[nSamples];
//	for (int r=0; r<nSamples; r++)
//	{
//		dFeat_BA[nSamples-1-r] = deq[r];
//	}
//	smoothData(dFeat_BA, nSamples, window);
//
//	for (int r=0; r<nSamples; r++)
//	{
//		deq[r] = 0.5 * (dFeat_AB[r] + dFeat_BA[nSamples-1-r]);
//	}
//
//	delete [] dFeat_AB;
//	delete [] dFeat_BA;
//}

//CHECK: maybe integrate in method "smoothFeatureVector"
//void FallDetect::smoothData(double *ft, int nData, int iArea)
//{
//	if (iArea == 0 || nData == 0)
//	{
//		return;
//	}
//
//	for (int i=0; i<nData-iArea; i++)
//	{
//		//local averaging
//		double sum = 0.0;
//		for (int j=i; j<i+iArea; j++)
//		{
//			sum = sum + ft[j];
//		}
//		ft[i] = sum / (double)iArea;
//	}
//
//	for (int i=nData-iArea; i<nData; i++)
//	{
//		double sum = 0.0;
//		int counter = 0;
//		for (int j=i; j<nData; j++)
//		{
//			sum = sum + ft[j];
//			counter++;
//		}
//		ft[i] = sum / (double)counter;
//	}
//}


bool FallDetect::downsampleFeatureVector(std::deque<double> deq, double rate)
{
	std::deque<double> newDeq;

	if(rate>1)
	{
		return false;
	}
	if(rate==1)
	{
		return true;
	}

	int newRate = 1/rate;
	double meanVal = 0;
	for(int i=0; i<deq.size(); i=i+newRate)
	{
		meanVal = 0;
		for(int j=0; j<newRate; j++)
		{
			meanVal = meanVal + deq[i+j];
		}
		meanVal = meanVal/newRate;
		newDeq.push_back(meanVal);
	}

	deq.clear();

	return true;
}


fl::flScalar FallDetect::doFIS(fl::flScalar in_sma, fl::flScalar in_deltaTheta, fl::flScalar in_deltaPhi)
{
	fl::flScalar out_movement;
	std::string m_lVarSma, m_lVarDTheta, m_lVarDPhi, m_lVarMovement;

	//for (in_sma = ACCEL_SMA_LOW_LB; in_sma<ACCEL_SMA_HIGH_UB; in_sma=in_sma+0.1)
	m_fuzzy_sma_var->setInput(in_sma);
	//m_lVarSma = m_fuzzy_movement_var->fuzzify(out_movement);

	//in_theta = (ACCEL_DELTA_THETA_HEAVY_UB+ACCEL_DELTA_THETA_HEAVY_LB)/3;
	m_fuzzy_theta_var->setInput(in_deltaTheta);
	//m_lVarDTheta = m_fuzzy_theta_var->fuzzify(in_theta);

	//in_phi = (ACCEL_DELTA_PHI_HEAVY_UB+ACCEL_DELTA_PHI_HEAVY_LB)/3;
	m_fuzzy_phi_var->setInput(in_deltaPhi);
	//m_lVarDPhi = m_fuzzy_phi_var->fuzzify(in_phi);

	m_fuzzyEngine->process();
	out_movement = m_fuzzy_movement_var->output().defuzzify();

	
	return out_movement;
}

bool FallDetect::isIndeterminate(double pv)
{
	if(pv!=pv)
	{
		return true;
	}
	else
	{
		return false;
	}
}


std::deque<double> FallDetect::readDataFromFile_1Ddeq(std::string filenamePath, bool bPopBackLastCol/*=true*/)
{
	std::ifstream inputFile;
	inputFile.open(filenamePath, std::ifstream::in);
	if (inputFile.is_open())
	{
		std::string useless, number;
		size_t middlePoint1 , previousMiddlePoint;
		double numberD = 0.0;
		int flag = 0;

		inputFile.seekg(0, std::ios_base::beg);

		std::deque<double> feature_vector;

		getline(inputFile, useless);
		do{
			number = useless.substr(0, useless.size());

			numberD = atof(number.c_str());
			feature_vector.push_back((numberD));

		}while(getline(inputFile, useless));

		inputFile.close();

		return feature_vector;
	}
}


std::deque<std::deque<double> > FallDetect::readDataFromFile_2Ddeq(std::string filenamePath, bool bPopBackLastCol/*=true*/)
{
	std::ifstream inputFile;
	inputFile.open(filenamePath, std::ifstream::in);
	if (inputFile.is_open()){
		std::string useless, number;
		size_t middlePoint1 , previousMiddlePoint;
		double numberD = 0.0;
		int flag = 0;

		inputFile.seekg(0, std::ios_base::beg);

		std::deque < std::deque<double> > feature_vector;

		getline(inputFile, useless);
		do{
			std::deque<double> lineDeqeue;
			flag = 0;
			do{

				middlePoint1 = useless.find_first_of("\t");
				previousMiddlePoint = useless.find_last_of("\t");
				if(middlePoint1 == previousMiddlePoint)
					flag++;

				number = useless.substr(0, middlePoint1);

				numberD = atof(number.c_str());
				useless = useless.substr(middlePoint1+1, useless.length());
				lineDeqeue.push_back((numberD));

			}while(flag != 2);
			if(bPopBackLastCol)
			{
				lineDeqeue.pop_back();
			}

			feature_vector.push_back(lineDeqeue);

		}while(getline(inputFile, useless));

		inputFile.close();

		return feature_vector;
	}
}

void FallDetect::storeAngles1D_deq(std::deque<double> input_vec, std::string outputPath_cstr, bool bInsertIndex/*=false*/)
{
	ofstream ofs;
	ofs.open(outputPath_cstr);

	for(int i=0; i<input_vec.size(); i++){
		if(bInsertIndex)
		{
			ofs << i;
			ofs << "\t";
		}
		ofs << input_vec[i];
		if(i!=input_vec.size()-1)
		{
			ofs << "\n";
		}
	}

	ofs.close();
	ofs.clear();
}


void FallDetect::storeAngles2D_deq(std::deque< std::deque<double>> input_vec, std::string outputPath_cstr, bool bInsertIndex/*=false*/)
{
	std::ofstream ofs;
	ofs.open(outputPath_cstr);

	for(int i=0; i<input_vec.size(); i++){
		if(bInsertIndex)
		{
			ofs << i;
			ofs << "\t";
		}
		for(int j=0; j<input_vec[i].size(); j++){
			ofs << input_vec[i][j];
			ofs << "\t";
		}

		if(i<input_vec.size()-1)
		{
			ofs << std::endl;
		}
	}

	ofs.close();
	ofs.clear();
}