#include "stdafx.h"

#include "MLPClass.h"
//#include "IniFile.h"
#include <math.h>
#include "gmr.h"
#include <iostream>
#include <fstream>
using namespace std;


MLPClass::MLPClass()
{
	//----------------------------------------------------------------------------------------------------------------------
	m_openCV_MLP = NULL;	//MLP variable
}


MLPClass::~MLPClass()
{
	if(m_openCV_MLP!=NULL)
	{
		m_openCV_MLP->clear();
		m_openCV_MLP = NULL;
	}
}


std::deque< std::deque<double> > MLPClass::convertRawsToColumns(std::deque< std::deque<double> > inputMatrix_deq)
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


//double MLPClass::getMean(std::deque<double> sequence)
//{
//
//	int seqSize = sequence.size();
//	double mean = 0;
//
//	for(int i=0; i<seqSize; i++)
//	{
//		mean = mean + sequence[i];
//	}
//
//	mean = mean/seqSize;
//
//	return mean;
//}
//
//
//double MLPClass::getVariance(std::deque<double> sequence)
//{
//	double var = 0;
//	double sum = 0;
//
//	double mean = getMean(sequence);
//
//	int seqSize = sequence.size();
//	for (int i = 0; i<seqSize; i++){
//		sum = sum + (sequence[i] - mean)*(sequence[i] - mean);
//	}
//
//	var = sum/seqSize;
//	return var;
//}
//
//
//double MLPClass::getMinVal(std::deque<double> sequence)
//{
//	int seqSize = sequence.size();
//	if(seqSize==0)
//	{
//		return 0.0;
//	}
//
//	double minVal = sequence[0];
//
//	for (int i = 0; i<sequence.size(); i++)
//	{
//		if(minVal>sequence[i])
//		{
//			minVal = sequence[i];
//		}
//	}
//}
//
//
//double MLPClass::getMaxVal(std::deque<double> sequence)
//{
//	int seqSize = sequence.size();
//	if(seqSize==0)
//	{
//		return 0.0;
//	}
//
//	double maxVal = sequence[0];
//
//	for (int i = 0; i<sequence.size(); i++)
//	{
//		if(maxVal<sequence[i])
//		{
//			maxVal = sequence[i];
//		}
//	}
//}


std::deque<double> MLPClass::readDataFromFile_1Ddeq(std::string filenamePath, bool bPopBackLastCol/*=true*/)
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


std::deque<std::deque<double> > MLPClass::readDataFromFile_2Ddeq(std::string filenamePath, bool bPopBackLastCol/*=true*/)
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


void MLPClass::storeAngles2D_deq(std::deque< std::deque<double>> input_vec, std::string outputPath_cstr, bool bInsertIndex/*=false*/)
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


//double MLPClass::openCVMLPTest(std::string inPattern)
//{
//	USES_CONVERSION;
//
//	char msgBuf[MAX_PATH];
//	std::string msgStr;
//
//	if(m_openCV_MLP==NULL)
//	{
//		return -1.0;
//	}
//
//	std::deque<double> testPattern_deq = readDataFromFile_1Ddeq(inPattern);
//	
//	float _testSample[MLP_NO_OF_INPUT_LAYERS];
//	CvMat testSample_mat = cvMat(1, MLP_NO_OF_INPUT_LAYERS, CV_32FC1, _testSample);
//	float _classificationResult[MLP_NO_OF_OUTPUT_LAYERS];
//	CvMat classificationResult_mat = cvMat(1, MLP_NO_OF_OUTPUT_LAYERS, CV_32FC1, _classificationResult);
//
//	if(testPattern_deq.size()!=MLP_NO_OF_INPUT_LAYERS)
//	{
//		testPattern_deq.clear();
//		return -1;
//	}
//
//	for(int i=0; i<MLP_NO_OF_INPUT_LAYERS; i++)
//	{
//		testSample_mat.data.fl[i] = testPattern_deq[i];
//	}
//	testPattern_deq.clear();
//
//	m_openCV_MLP->predict(&testSample_mat, &classificationResult_mat);		// run neural network prediction
//	double testVal = classificationResult_mat.data.fl[0];
//
//	return testVal;
//}


bool MLPClass::checkPattern(std::deque<double> inPattern, double &mlpVal, double threshold)
{
	USES_CONVERSION;

	char msgBuf[MAX_PATH];
	std::string msgStr;

	if(m_openCV_MLP==NULL)
	{
		return -1.0;
	}
	
	float _testSample[MLP_NO_OF_INPUT_LAYERS];
	CvMat testSample_mat = cvMat(1, MLP_NO_OF_INPUT_LAYERS, CV_32FC1, _testSample);
	float _classificationResult[MLP_NO_OF_OUTPUT_LAYERS];
	CvMat classificationResult_mat = cvMat(1, MLP_NO_OF_OUTPUT_LAYERS, CV_32FC1, _classificationResult);

	if(inPattern.size()!=MLP_NO_OF_INPUT_LAYERS)
	{
		inPattern.clear();
		return -1;
	}

	for(int i=0; i<MLP_NO_OF_INPUT_LAYERS; i++)
	{
		testSample_mat.data.fl[i] = inPattern[i];
	}
	inPattern.clear();		

	m_openCV_MLP->predict(&testSample_mat, &classificationResult_mat);		// run neural network prediction
	mlpVal = classificationResult_mat.data.fl[0];

	if(mlpVal>=threshold)
	{
		return true;
	}
	else
	{
		return false;
	}
}


int MLPClass::performMLPTraining(std::string inputDataFilePath, std::string inputResultDataFilePath, std::string logFilePath)
{
	USES_CONVERSION;

	//Load all in Deques
	//std::deque<std::deque<double>> falseInputData_deq;	//Load False Non-Fall Data Sequences	
	std::deque<std::deque<double>> inputData_deq = readDataFromFile_2Ddeq(inputDataFilePath);		//Load True Fall Data Sequences
	std::deque<double> outputData_deq = readDataFromFile_1Ddeq(inputResultDataFilePath);

	if(inputData_deq.size()==0 || outputData_deq.size()==0)
	{
		return -1;
	}

	inputData_deq = convertRawsToColumns(inputData_deq);

	char msgBuf[MAX_PATH];
	std::string msgStr;

	CvMat trainData1, trainClasses1, sampleWts1;
	CvMat* trainingData_mat = cvCreateMat(inputData_deq.size(), MLP_NO_OF_INPUT_LAYERS, CV_32FC1);
    CvMat* trainingClassifications_mat = cvCreateMat(inputData_deq.size(), MLP_NO_OF_OUTPUT_LAYERS, CV_32FC1);
	CvMat* sampleWts_mat = cvCreateMat(inputData_deq.size(), 1, CV_32FC1);	//The weight of each training data sample. We'll later set all to equal weights.

	cvGetRows(trainingData_mat, &trainData1, 0, inputData_deq.size());
	cvGetRows(trainingClassifications_mat, &trainClasses1, 0, inputData_deq.size());
	cvGetRows(sampleWts_mat, &sampleWts1, 0, inputData_deq.size());

	//if(MLP_SAMPLES_FOR_TRAINING!=falseInputData_deq.size() || MLP_NO_OF_INPUT_LAYERS!=falseInputData_deq[0].size())
	//{
	//	return -1.0;
	//}

	//Load Deques onto Mat
	for (int i=0; i<inputData_deq.size(); i++)
	{
		for(int j=0; j<inputData_deq[i].size(); j++)
		{
			cvSetReal2D(&trainData1, i, j, inputData_deq[i][j]);		//Input
		}
		cvSet1D(&trainClasses1, i, cvScalar(outputData_deq[i]));		//Output
		cvSet1D(&sampleWts1, i, cvScalar(1));							//Weight (setting everything to 1)
	}

	CvMat neuralLayers1;
	CvMat* layers_mat = cvCreateMat(4, 1, CV_32SC1);
	cvGetRows(layers_mat, &neuralLayers1, 0, 4);

   //Setting the number of neurons on each layer of the ANN
   cvSet1D(&neuralLayers1, 0, cvScalar(inputData_deq[0].size()));	//Layer 1: 180 neurons (input layer)
   cvSet1D(&neuralLayers1, 1, cvScalar(MLP_FIRST_LAYER_NEURONS));	//Layer 2: 30 neurons (hidden layer)
   cvSet1D(&neuralLayers1, 2, cvScalar(MLP_SECOND_LAYER_NEURONS));	//Layer 3: 20 neurons (hidden layer)
   cvSet1D(&neuralLayers1, 3, cvScalar(MLP_NO_OF_OUTPUT_LAYERS));	//Layer 4: 1 neurons (1 output)

	if(m_openCV_MLP!=NULL)
	{
		m_openCV_MLP->clear();
		m_openCV_MLP = NULL;
	}
	m_openCV_MLP = new CvANN_MLP;
    m_openCV_MLP->create(layers_mat, CvANN_MLP::SIGMOID_SYM, 0.6, 1);

	//Param #1 terminate the training after either 1000 iterations or a very small change in the network wieghts below the specified value
	//Param #2 use backpropogation for training
	//Param #3 co-efficents for backpropogation training (refer to manual)
	CvANN_MLP_TrainParams openCV_MLP_params = CvANN_MLP_TrainParams(cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 0.000001),
																	CvANN_MLP_TrainParams::BACKPROP, 0.1, 0.1);
	int iterations = m_openCV_MLP->train(trainingData_mat, trainingClassifications_mat, sampleWts_mat, 0, openCV_MLP_params);

	//training has finished
	//display the results
	std::string logFileName = logFilePath.c_str();
	logFileName.append(FILENAME_MULTILAYERPERCEPTRON_DAT);
	m_openCV_MLP->save(logFileName.c_str());

	m_openCV_MLP->clear();
	m_openCV_MLP = NULL;

	cvReleaseMat(&layers_mat);
	cvReleaseMat(&trainingData_mat);
	cvReleaseMat(&trainingClassifications_mat);
	if(m_openCV_MLP!=NULL)
	{
		m_openCV_MLP->clear();
		m_openCV_MLP = NULL;
	}

	return iterations;
}


bool MLPClass::loadNewMLP(std::string storedMLPPath)
{
	USES_CONVERSION;

	if(m_openCV_MLP!=NULL)
	{
		m_openCV_MLP->clear();
		m_openCV_MLP = NULL;
	}
	m_openCV_MLP = new CvANN_MLP;

	std::string initializationData_cstr;
	initializationData_cstr = storedMLPPath;
	//initializationData_cstr.append(FILENAME_MULTILAYERPERCEPTRON_DAT);

	m_openCV_MLP->load(initializationData_cstr.c_str());

	return true;
}