#pragma once
#define WIN32_LEAN_AND_MEAN

#include "FallDetectionDLL_Config.h"

#include "api_definitions.h"
#include "MLPClass.h"
#include <string>
#include <iostream>
#include <fstream>
#include <deque>

#include <timestamp.hpp>
#include <limits>
#include "FuzzyLite.h"
#include "FunctionTerm.h"

#include <cv.h>       // opencv general include file
#include <ml.h>		  // opencv machine learning include file


class FALLDETECTIONDLLAPIS FallDetect
{
public:
	FallDetect();
	~FallDetect();
	
	/*void startMonitoringForAccelerometer(std::string subjectStoringPath_str);
	void startRecordingForAccelerometer(std::string subjectStoringPath_str);
	void setAccelerometerTrackerOff() {m_bAccelerometerTrackerStopped = true;};
	void setAccelerometerRecorderOff() {m_bAccelerometerRecorderStopped = true;};*/

	bool detectFall(std::string storedMLPPath);

	bool checkUsersStatus(double xVal, double yVal, double zVal, Timestamp tmpStmp);
	std::deque<double> calcAll_Offline(std::deque<std::deque<double>> inputData_deq, std::deque<std::deque<double>> inputTime_deq);

	std::deque<double> readDataFromFile_1Ddeq(std::string filenamePath, bool bPopBackLastCol=true);
	std::deque<std::deque<double> > readDataFromFile_2Ddeq(std::string filenamePath, bool bPopBackLastCol=true);	
	void storeAngles1D_deq(std::deque<double> input_vec, std::string outputPath_cstr, bool bInsertIndex=false);
	void storeAngles2D_deq(std::deque< std::deque<double>> input_vec, std::string outputPath_cstr, bool bInsertIndex=false);
	std::deque< std::deque<double> > convertRawsToColumns(std::deque< std::deque<double> > inputMatrix_deq);

	bool isMovementUpdated(){return bIsMovementUpdated;};
	
	std::deque<double> getMovement_deq() {return m_movement_deq;};

	fl::flScalar doFIS(fl::flScalar in_sma, fl::flScalar in_deltaTheta, fl::flScalar in_deltaPhi);

	double getR() {return m_r;};
	double getDeltaPhi() {return m_deltaThetaDer;};
	double getDeltaTheta() {return m_deltaPhiDer;};
	Timestamp getTimestamp() {return m_tmpStmp;};

private:
	
	static int RunInThreadAccelerometerTracker(void* pv_Para);
	static int RunInThreadAccelerometerRecorder(void* pv_Para);

	bool calcAll(double xVal, double yVal, double zVal, Timestamp tmpStmp);

	std::deque<double> getPSDFFT3W(std::deque< std::deque<double> > inputData_vec, int startingIndex, int windowSize, int sampling);

	/*double getAccelXVal();
	double getAccelYVal();
	double getAccelZVal();*/

	int countOnes(std::deque<int> inDeq);

	bool updateAccelValues(double xVal,double yVal,double zVal, Timestamp tmpStmp);

	static bool calc(){return false;};

	bool calcR();
	bool calcPhi();
	bool calcDeltaPhiDer();
	bool calcTheta();
	bool calcDeltaThetaDer();
	//bool calcSMA();

	
	double getPhi() {return m_theta;};
	double getTheta() {return m_phi;};
	double getSMA() {return m_sma;};

	double getCurrentSMA() {return m_current_sma;};
	double getCurrentDeltaTheta() {return m_current_deltaTheta;};
	double getCurrentDeltaPhi() {return m_current_deltaPhi;};

	std::string getInternalCounterString();

	std::deque<double> getR_deq() {return m_r_deq;};
	std::deque<double> getPhi_deq() {return m_phi_deq;};
	std::deque<double> getDeltaPhi_deq() {return m_deltaPhi_deq;};
	std::deque<double> getTheta_deq() {return m_theta_deq;};
	std::deque<double> getDeltaTheta_deq() {return m_deltaTheta_deq;};
	std::deque<double> getSMA_deq() {return m_sma_deq;};
	
	void reSizeSignature(std::deque <std::deque<double>> &deq, int nbData);

	std::deque<double> getPSD_deq(){return m_psdFreq_deq;};
	//std::deque<std::string> getProportions_deq(){return m_proportions_deq;};

	bool isIndeterminate(double pv);

	double getMean(std::deque<double> sequence);
	double getVariance(std::deque<double> sequence);
	double getMinVal(std::deque<double> sequence);
	double getMaxVal(std::deque<double> sequence);

public:
	bool m_bVectorsAreUpdated;

	//CWinThread* m_hThreadAccelerometerTracker;
	//CWinThread* m_hThreadAccelerometerRecorder;

	std::ofstream m_ofs_data;
	std::ofstream m_ofs_timestamp;

private:
	int m_counter;
	int m_intCounter;
	int m_internalCounter;

	double m_accel_x;
	double m_accel_y;
	double m_accel_z;

	bool m_bUpdatedValues1;
	bool m_bUpdatedValues2;
	bool m_bUpdatedValues3;

	double m_r;
	double m_previousR;
	double m_previousTheta;
	double m_theta;
	double m_deltaThetaDer;
	double m_previousPhi;
	double m_phi;
	double m_deltaPhiDer;
	double m_sma;

	double m_current_sma;
	double m_current_deltaTheta;
	double m_current_deltaPhi;
	double m_current_fis;

	
	std::ofstream m_ofs_recorder;

	Timestamp m_tmpStmp;
	Timestamp m_previousTmpStmp;
	Timestamp m_tmpstmp_diff;
	
	bool m_bNewValues;
	bool bIsMovementUpdated;
	//bool m_bPlotFlag;

	std::deque<double> m_r_deq;
	std::deque<double> m_theta_deq;
	std::deque<double> m_deltaTheta_deq;
	std::deque<double> m_deltaThetaBuffer_deq;
	std::deque<double> m_deltaTheta_secondBuffer_deq;
	double m_dDeltaTheta_secondBuffer;
	//std::deque<double> m_deltaThetaNEW_deq;
	std::deque<double> m_phi_deq;
	std::deque<double> m_deltaPhi_deq;
	std::deque<double> m_deltaPhiBuffer_deq;
	std::deque<double> m_deltaPhi_secondBuffer_deq;
	double m_dDeltaPhi_secondBuffer;
	//std::deque<double> m_deltaPhiNEW_deq;
	std::deque<double> m_sma_deq;
	std::deque<double> m_smaBuffer_deq;
	std::deque<double> m_sma_secondBuffer_deq;
	double m_dSma_secondBuffer;

	std::deque<double> m_movement_deq_smoother;
	std::deque<double> m_movement_deq;
	std::deque<int> m_finalMLP_decision_deq;
	//Flood::Vector<double> m_mlpOutput_deq;

	bool m_bAccelerometerTrackerStopped;
	bool m_bAccelerometerRecorderStopped;

	std::deque<std::deque<double>> m_accelVals_deq;
	std::deque<double> m_psdFreq_deq;
	//std::deque<std::string> m_proportions_deq;

	//void smoothFeatureVector(std::deque<double> &deq, int window);
	//void smoothData(double *ft, int nData, int iArea);
	bool downsampleFeatureVector(std::deque<double> deq, double rate);

	

	//Create Fuzzy Sets
	fl::FuzzyEngine* m_fuzzyEngine;

	fl::InputLVar* m_fuzzy_sma_var;
	fl::InputLVar* m_fuzzy_theta_var;
	fl::InputLVar* m_fuzzy_phi_var;
	fl::OutputLVar* m_fuzzy_movement_var;
	fl::RuleBlock* m_block;
};