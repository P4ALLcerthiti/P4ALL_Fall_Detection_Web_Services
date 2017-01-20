#pragma once

//#include "CentralUnitDlg.h"
#include "cv.h"
#include "highgui.h"
#include <deque>
#include "timestamp.hpp"

//Phidget Sensor Interface
#include "LightSensorsEventDetector.h"

using namespace std;

typedef struct
{
	Timestamp time;
	int index;
} SequenceInfo;

class HistoryItem
{
public:
	CvRect roi;
	double area;
	CvScalar avgRoiColor;
	CvScalar avgMaskColor;
	int frameID;
};

class EventDetectorParams
{
public:
	//the maximum the number, the longer it takes to learn a region

	int framesBefore;
	int framesAfter;
	int useTopCamera;
	int history_MaxItems;
	int trackDelay;
	//roi size in pixels after resizing (size x size)
	int roi_defaultSize; 

	//updates roi when not changed
	double roi_updateFactor;

	//for matching or not the current roi with the pre-existed ones
	double roi_trainedThreshold;
	
	//detects a change in the current roi
	double changeThreshold_position;
	double changeThreshold_size;
	double changeThreshold_color;
	double changeThreshold_area;
	int stateSensitivityframes;
	int nMaxStates; //not used, max allowed states (uncertainty not counted)
	int nStateFadingInItems; //not used
	int nStateFadingOutItems; //not used

	EventDetectorParams()
	{
		framesBefore = 0;
		framesAfter = 0;
		history_MaxItems = 15;
		roi_defaultSize = 30;
		roi_updateFactor = 0.3;
		roi_trainedThreshold = 100;
		changeThreshold_position = 10;
		changeThreshold_size = 10;
		changeThreshold_color = 50;
		changeThreshold_area = 0.1;
		stateSensitivityframes = 0;
		//Not used:
		nStateFadingInItems = 5; //not used
		nStateFadingOutItems = 5; //not used
		nMaxStates = 3; //not used
	}

	EventDetectorParams operator =(EventDetectorParams obj)
	{
		framesBefore = obj.framesBefore;
		framesAfter = obj.framesAfter;
		history_MaxItems = obj.history_MaxItems;
		roi_defaultSize = obj.roi_defaultSize;
		roi_updateFactor = obj.roi_updateFactor;
		roi_trainedThreshold = obj.roi_trainedThreshold;
		changeThreshold_position = obj.changeThreshold_position;
		changeThreshold_size = obj.changeThreshold_size;
		changeThreshold_color = obj.changeThreshold_color;
		changeThreshold_area = obj.changeThreshold_area;
		stateSensitivityframes = obj.stateSensitivityframes;
		//number of frames to be added in interesting vector before and after the sequence
		nStateFadingInItems = obj.nStateFadingInItems; //not used
		nStateFadingOutItems = obj.nStateFadingOutItems; //not used
		nMaxStates = obj.nMaxStates; //not used

		return *this;
	}
};



class EventDetector
{
private:

	IplImage* currentFrame;
	CvRect currentROI;
	IplImage* currentMask;
	double currentArea;
	double trainedArea;
	int trackDelay[6]; 
	int currFrameID;
	deque <deque <HistoryItem>> history;
	bool weHaveAStaticROI;

	deque <IplImage*> imgStaticROI;
	deque <IplImage*> imgStaticTMP; //assistive
	deque <deque <IplImage*>> vStateRoiImages; //save ROI state images
	deque <deque <IplImage*>> vStateMaskImages; //save ROI state images
	deque <deque <int>> vStateIDs;
	deque <deque <int>> vSensorStates;
	int currentState;
	int interestState; //the state we are interested in
	deque <deque <int>> vInterestFrameIDs;
	deque <deque <int>> vFrameStates; 
	deque <deque <double>> vStateAreas;
	deque <deque <CvRect>> vStateROI;
	deque <deque <CvScalar>> vStateAvgColors;
	deque <deque <CvScalar>> vStateAvgColorsMask;
	deque <EventDetectorParams> mParams;

private:

	CvScalar getAvgRoiColor(bool usingMask);
	double getColorDiff(CvScalar c1, CvScalar c2);
	bool roiChanged(int objectID);
	bool b_panelPressed;
	//experimental, do not use it!
	void addInterestingFrameID(int objectID, int currentFrameID);

	deque <int> removeStateFromVector(deque <int> stateVector, int stateIdToRemove);
	int getNumOfElementsInStateVector(int objectID, int stateId);
	
public:
	int current_event;

	EventDetector();
	~EventDetector();

	deque <EventDetectorParams> getParams() {return mParams;}
	void SetParams(int objectID, EventDetectorParams edp, int useCamera);
	void SetInput(int objectID, IplImage* img, CvRect roi, double area, IplImage* roiMask, int frameId);
	void SetInterestInState(int stateIdx);
	void setTrackDelay(int id, int del) {trackDelay[id] = del;}
	int getTrackDelay(int id){return trackDelay[id];}
	//not used, only for on the fly... deprecated!
	deque <int> GetInterestFrameIDs(int objectID); 

	//returns current state vector (including fuzzy states)
	deque <int> GetStateVector(int objectID);

	//automatically eliminates weak classes
	deque <int> GetCrispStateVector(int objectID,int maxNumOfClasses); 
	
	void ClearAll();

	bool TrackSceneEvents(int objectID, bool objectSelected, double frameRate, int useSensorForEvent);
	bool DoWeHaveAStaticROI();
	bool PanelButtonPressed() {return b_panelPressed;}
	int GetCurrentState(int objectID);
	int GetNumOfStatesDetected(int objectID);

	//Phidget Sensor Methods
	PhidgetSensorEventDetection* PhidgetInterface;

	void initPhidget();

	PhoneMonitoring mPhoneActivity;
	PanelMonitoring mPanelActivity;	
	WorlplacePanelMonitoring mWorkplacePanelActivity;

	int m_tempPhidgetPhoneStatus;
	int m_tempPhidgetPanelStatus;
	int m_tempPhidgetWorkplacePanelStatus;
	int m_tempUltrasonicPhoneStatus;
};