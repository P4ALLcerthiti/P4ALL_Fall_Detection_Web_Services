#include "stdafx.h"
#include "api_definitions.h"
#include "PhidgetSensorEventDetection.h"
#include "FallDetect.h"
#include "MLPClass.h"
#include <deque>
#include <boost\filesystem.hpp>

#include "io.h"


bool addFilesForSignature(std::string anglesFile, std::string tmpstmpFile, int action)
{
	FallDetect* m_fDetection = new FallDetect();

	std::string processedAnglesPath = ".\\..\\FilesForSignature";
	std::string anglesFileName = anglesFile;
	
	std::string timestampFileName = tmpstmpFile;

	boost::filesystem::path dir(processedAnglesPath);
	boost::filesystem::create_directory(dir);
	processedAnglesPath.append("\\");

	if(_access(anglesFileName.c_str(),0)==-1)
	{
		cout << "WARNING: No angles file detected..." << "\n" ;
		return false;
	}

	if(_access(timestampFileName.c_str(),0)==-1)
	{
		cout << "WARNING: No timestamp file detected..." << "\n" ;
		return false;
	}

	std::deque<std::deque<double>> angles_deq = m_fDetection->readDataFromFile_2Ddeq(anglesFileName, false);
	for(int i=0; i<angles_deq.size(); i++)
	{
		angles_deq[i].push_front(i);
	}
	std::deque<std::deque<double>> timestamp_deq = m_fDetection->readDataFromFile_2Ddeq(timestampFileName, true);
	for(int i=0; i<timestamp_deq.size(); i++)
	{
		timestamp_deq[i].push_front(i);
	}

	if(angles_deq.size()==0)
	{
		cout << "WARNING: Empty angles file..." << "\n";
		return false;
	}

	if(angles_deq[0].size()-1!=3)
	{
		cout << "WARNING: Corrupted angles file..." << "\n";
		return false;
	}

	if(timestamp_deq.size()==0)
	{
		cout << "WARNING: Empty timestamp file..." << "\n";
		return false;
	}

	std::deque<double> processedAngles = m_fDetection->calcAll_Offline(angles_deq, timestamp_deq);

	std::vector<boost::filesystem::path> v;       

	int files_counter = 0;
	std::copy(boost::filesystem::directory_iterator(dir), boost::filesystem::directory_iterator(), std::back_inserter(v));
	for (std::vector<boost::filesystem::path>::const_iterator it=v.begin(); it != v.end(); ++it)
	{
		//cout << *it << "\n";
		files_counter++;
	}
	
	std::string s;         
	std::ostringstream convert;  
	convert << files_counter;     
	s = convert.str(); 

	processedAnglesPath.append("processedAngles");
	processedAnglesPath.append(s);
	if(action == 0)
	{
		processedAnglesPath.append("_0.txt");	
	}
	else if(action ==1 )
	{
		processedAnglesPath.append("_1.txt");
	}
	else
	{
		cout << "WARNING: No valid action. Valid inputs are: 1 for Fallings 0 otherwise!" << "\n";
		return false;
	}
	
	m_fDetection->storeAngles1D_deq(processedAngles, processedAnglesPath);

	processedAngles.clear();

	if(m_fDetection!=NULL)
	{
		delete m_fDetection;
		m_fDetection = NULL;
	}

	return true;
}

bool mergeTrainingData()
{
	FallDetect* m_fDetection = new FallDetect();

	std:string folderPath, outputFilePath, outputResultsFilePath, outputPath;
	folderPath = ".\\..\\FilesForSignature";
	boost::filesystem::path dir(folderPath);

	outputPath = "\\FilesForTraining";
	boost::filesystem::create_directory(dir / outputPath);

	outputFilePath = folderPath ;
	outputResultsFilePath = folderPath;
	outputFilePath.append(outputPath);
	outputResultsFilePath.append(outputPath);
	outputFilePath.append("\\");
	outputFilePath.append(FILENAME_ALL_TRAINING_DATA);
	outputResultsFilePath.append("\\");
	outputResultsFilePath.append(FILENAME_ALL_TRAINING_DATA_RESULTS);

	std::deque<std::deque<double>> allMovements_deq;
	std::deque<double> buffer1D_deq;
	std::deque<std::deque<double>> buffer2D_deq;
	std::deque<double> allMovementsResults_deq;

	std::vector<boost::filesystem::path> v;       

	int files_counter = 0;
	std::copy(boost::filesystem::directory_iterator(dir), boost::filesystem::directory_iterator(), std::back_inserter(v));
	for (std::vector<boost::filesystem::path>::const_iterator it=v.begin(); it != v.end(); ++it)
	{
		//cout << *it << "\n";
		ifstream file;
		file.open((it->string()).c_str(), ifstream::in);

		if(_access((it->string()).c_str(),0)!=-1 && file.peek() != std::ifstream::traits_type::eof())
		{

			std::size_t found = (it->string()).find("_1.txt");
			std::size_t found2 =(it->string()).find("_0.txt");

			if (found!=std::string::npos)
			{
				allMovementsResults_deq.push_back(1);
			}
			else if (found2!=std::string::npos)
			{
				allMovementsResults_deq.push_back(0);
			}
			else
			{
				cout << (it->string()) << "\n";
				cout << "WARNING: Error name for processedAngles file" << "\n";
				return false;
			}
			
			

			for(int i=0; i<buffer2D_deq.size(); i++)
			{
				buffer2D_deq[i].clear();
			}
			buffer2D_deq.clear();
			buffer1D_deq.clear();
			buffer2D_deq = m_fDetection->readDataFromFile_2Ddeq((it->string()).c_str());
			buffer1D_deq.resize(buffer2D_deq.size());
			for(int i=0; i<buffer2D_deq.size(); i++)
			{
				buffer1D_deq[i] = buffer2D_deq[i][0];
			}

			allMovements_deq.push_back(buffer1D_deq);
		}
		file.close();
	}

	allMovements_deq = m_fDetection->convertRawsToColumns(allMovements_deq);
	m_fDetection->storeAngles2D_deq(allMovements_deq, outputFilePath);
	m_fDetection->storeAngles1D_deq(allMovementsResults_deq, outputResultsFilePath);

	if(m_fDetection!=NULL)
	{
		delete m_fDetection;
		m_fDetection = NULL;
	}

	allMovements_deq.clear();
	buffer1D_deq.clear();
	buffer2D_deq.clear();
	allMovementsResults_deq.clear();

	return true;

}

bool createSignature(std::string inputDataSet, std::string inputResultsDataSet)
{
	MLPClass* m_mlp = new MLPClass();
	std::string logFilePath = ".\\..\\" ;
	logFilePath.append(FOLDERNAME_MLP);

	boost::filesystem::path dir(logFilePath);
	boost::filesystem::create_directory(dir);

	logFilePath.append("\\");

	if(_access(inputDataSet.c_str(),0)==-1)
	{
		cout << "WARNING: No allTrainingData file detected... " << "\n";
		return false;
	}

	if(_access(inputResultsDataSet.c_str(),0)==-1)
	{
		cout << "WARNING: No allTrainingDataResults file detected... " << "\n";
		return false;
	}

	ifstream file1;
	file1.open(inputResultsDataSet.c_str(), ifstream::in);
	if ( file1.peek() == std::ifstream::traits_type::eof() )
	{
		file1.close();
		cout << "WARNING: Empty allTrainingDataResults file..." << "\n";
		return false;
	}

	ifstream file2;
	file2.open(inputDataSet.c_str(), ifstream::in);
	if ( file2.peek() == std::ifstream::traits_type::eof() )
	{
		file2.close();
		cout << "WARNING: Empty allTrainingData file..." << "\n";
		return false;
	}

	m_mlp->performMLPTraining(inputDataSet, inputResultsDataSet, logFilePath);

	if(m_mlp!=NULL)
	{
		delete m_mlp;
		m_mlp = NULL;
	}
	return true;
}

int _tmain(int argc, _TCHAR* argv[])
{

	std::string anglesFile = "C:\\angles.txt";
	std::string tmpstmpFile = "C:\\index.ini";

	std::string pathToAllTrainingData = "C:\\allTrainingData.txt";
	std::string pathToAllTrainingDataResults = "C:\\allTrainingDataResults.txt";

	/*
		If the given files describe a fall, give action the value of 1 (action=1)
		If the given files don't describe a fall, give action the value of 0 (action=0)
	*/
	
	addFilesForSignature(anglesFile, tmpstmpFile, 0);

	mergeTrainingData();

	createSignature(pathToAllTrainingData, pathToAllTrainingDataResults);

	return 0;

}

