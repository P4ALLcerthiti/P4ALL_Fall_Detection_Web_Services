#pragma once

#include "FallDetectionDLL_Config.h"

#include "api_definitions.h"

#include <iostream>
#include <iomanip>
#include <fstream>

#include <string>
#include <deque>

#include <cv.h>       // opencv general include file
#include <ml.h>		  // opencv machine learning include file


class FALLDETECTIONDLLAPIS MLPClass
{

public:
	MLPClass();
	~MLPClass();

	bool checkPattern(std::deque<double> inPattern, double &mlpVal, double threshold);
	//double openCVMLPTest(std::string inPattern);
	int performMLPTraining(std::string inputDataFilePath, std::string inputResultDataFilePath, std::string logFilePath);

	/*double getMean(std::deque<double> sequence);
	double getVariance(std::deque<double> sequence);
	double getMinVal(std::deque<double> sequence);
	double getMaxVal(std::deque<double> sequence);*/

	bool loadNewMLP(std::string storedMLPPath);

	
private:
	//OpenCV Multi Layer Perceptron
	CvANN_MLP* m_openCV_MLP;

	std::deque<double> readDataFromFile_1Ddeq(std::string filenamePath, bool bPopBackLastCol=true);
	std::deque<std::deque<double> > readDataFromFile_2Ddeq(std::string filenamePath, bool bPopBackLastCol=true);
	void storeAngles2D_deq(std::deque< std::deque<double>> input_vec, std::string outputPath_cstr, bool bInsertIndex=false);
	std::deque< std::deque<double> > convertRawsToColumns(std::deque< std::deque<double> > inputMatrix_deq);
	//bool InitFromIni(std::string iniFilePath);
};