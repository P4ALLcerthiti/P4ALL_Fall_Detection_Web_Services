#include "stdafx.h"
#include "EventDetector.h"
#include <iostream>

EventDetector::EventDetector()
{
	mParams.clear();
	weHaveAStaticROI = false;
	imgStaticROI.clear();
	imgStaticTMP.clear();
	currentFrame = NULL;
	currentROI = cvRect(0,0,0,0);
	currentMask = NULL;
	currentState = -1;
	interestState = -1;
	vInterestFrameIDs.clear();
	history.clear();
	vStateRoiImages.clear();
	vStateMaskImages.clear();
	vFrameStates.clear();
	vStateAvgColors.clear();
	vStateAvgColorsMask.clear();
	vStateAreas.clear();
	vStateROI.clear();
	vSensorStates.clear();
	for (int i = 0; i < 6;i++) {
		imgStaticROI.push_back(NULL);
		imgStaticTMP.push_back(NULL);
		deque <IplImage*> vTempRoiImg;
		deque <IplImage*> vTempMaskImg;
		vStateRoiImages.push_back(vTempRoiImg);
		vStateMaskImages.push_back(vTempMaskImg);
		deque <HistoryItem> vTmpHistory;
		history.push_back(vTmpHistory);
		deque <int> vTmpFrameStates;
		deque <int> vTmpSensorStates;
		vFrameStates.push_back(vTmpFrameStates);
		vSensorStates.push_back(vTmpSensorStates);
		deque <CvScalar> vTmpStateAvgColors;
		vStateAvgColors.push_back(vTmpStateAvgColors);
		deque <CvScalar> vTmpStateAvgColorsMask;
		vStateAvgColorsMask.push_back(vTmpStateAvgColorsMask);
		deque <CvRect> vTmpStateROI;
		vStateROI.push_back(vTmpStateROI);
		deque <double> vTmpStateAreas;
		vStateAreas.push_back(vTmpStateAreas);
	}

	current_event = -1;
	
	m_tempPhidgetPhoneStatus = 1;
	m_tempPhidgetPanelStatus = 1;
	m_tempUltrasonicPhoneStatus = 1;

	mPhoneActivity = ACTIVITY_PHONE_OFF;
	mPanelActivity = ACTIVITY_PANEL_OFF;

	PhidgetInterface = NULL;
}

EventDetector::~EventDetector()
{
	ClearAll();
	for (int j = 0; j<6;j++) {
		vFrameStates.at(j).clear();
//		vSensorStates.at(j).clear();
	}
	vFrameStates.clear();
//	vSensorStates.clear();
	if(PhidgetInterface != NULL)
	{
		PhidgetInterface->releasePhidget();
		delete PhidgetInterface;
		PhidgetInterface = NULL;
	}
}

void EventDetector::initPhidget()
{
	if(PhidgetInterface == NULL)
	{
		PhidgetInterface = new PhidgetSensorEventDetection();
		PhidgetInterface->setEventDetector(this);
		PhidgetInterface->initializePhidget();		
	}
}

void EventDetector::ClearAll()
{
	if (vStateRoiImages.size()>0) {
		for (int j = 0; j<6;j++) {
			if (vStateRoiImages.at(j).size() > 0) {
				for (int i=0; i<(int)vStateRoiImages.at(j).size(); i++) {
					if (vStateRoiImages.at(j).at(i)) {
						cvReleaseImage(&(vStateRoiImages.at(j).at(i)));
						vStateRoiImages.at(j).at(i)=NULL;
					}
				}
				vStateRoiImages.at(j).clear();
			}
			if (vStateMaskImages.at(j).size() > 0) {
				for (int i=0; i<(int)vStateMaskImages.at(j).size(); i++)
				{
					if (vStateMaskImages.at(j).at(i)) {
						cvReleaseImage(&(vStateMaskImages.at(j).at(i)));
						vStateMaskImages.at(j).at(i)=NULL;
					}
				}
				vStateMaskImages.at(j).clear();
			}
			if (history.size() > j && history.at(j).size() > 0) history.at(j).clear();
			if (vStateRoiImages.size() > j && vStateRoiImages.at(j).size() > 0)  vStateRoiImages.at(j).clear();
			if (vStateAvgColors.size() > 0 && vStateAvgColors.at(j).size() > 0)  vStateAvgColors.at(j).clear();
			if (vStateAvgColorsMask.size() > 0 && vStateAvgColorsMask.at(j).size() > 0) vStateAvgColorsMask.at(j).clear();
			if (vStateAreas.size() > 0 && vStateAreas.at(j).size() > 0) vStateAreas.at(j).clear();
			if (vStateROI.size() > 0 && vStateROI.at(j).size() > 0) vStateROI.at(j).clear();
			if (imgStaticROI.at(j)) {
				cvReleaseImage(&imgStaticROI.at(j));
				imgStaticROI.at(j) = NULL;
			}
			if (imgStaticTMP.at(j)) {
//				cvReleaseImage(&imgStaticTMP.at(j));
				imgStaticTMP.at(j) = NULL;
			}
		}
	}
	else if (vSensorStates.size()>0)
	{
		for (int j = 0; j<6;j++) {
			if (vSensorStates.at(j).size() > 0) vSensorStates.at(j).clear();
		}
	}
	history.clear();
	vStateMaskImages.clear();
	vStateRoiImages.clear();
	vSensorStates.clear();
	vStateAvgColors.clear();
	vStateAvgColorsMask.clear();
	vStateAreas.clear();
	vStateROI.clear();
	imgStaticROI.clear();
	imgStaticTMP.clear();

}

CvScalar EventDetector::getAvgRoiColor(bool usingMask)
{
	CvScalar avgColor = cvScalar(-1, -1, -1);

	if (currentROI.width > 0 && currentROI.height > 0)
	{
		cvSetImageROI(currentFrame, currentROI);
		
		if (usingMask)
			avgColor = cvAvg(currentFrame, currentMask);
		else
			avgColor = cvAvg(currentFrame);
		
		cvResetImageROI(currentFrame);
	}

	return avgColor;
}

double EventDetector::getColorDiff(CvScalar c1, CvScalar c2)
{
	double d = 0.0;
	for (int i=0; i<3; i++) //RGB
	{
		d += (c1.val[i]-c2.val[i]) * (c1.val[i]-c2.val[i]);
	}
	d = sqrt(d);

	return d;
}

bool EventDetector::roiChanged(int objectID)
{

	//thresholds 
	if (current_event!=-1) {
		double dposi_th = mParams.at(current_event).changeThreshold_position;
		double dsize_th = mParams.at(current_event).changeThreshold_size;
		double dcolor_th = mParams.at(current_event).changeThreshold_color;
		double darea_th = mParams.at(current_event).changeThreshold_area;

		int nPreviousElements = (int)history.at(objectID).size()-1;
		if (nPreviousElements <= 0) return false;

		double dw = 0;
		double dh = 0;
		double dx = 0;
		double dy = 0;
		double dc = 0;
		double dc2 = 0;
		double da = 0;
		double avg_a = 0;

		for (int t=0; t<nPreviousElements; t++)
		{
			dx += abs(history.at(objectID).at(t).roi.x - currentROI.x);
			dy += abs(history.at(objectID).at(t).roi.y - currentROI.y);
			dw += abs(history.at(objectID).at(t).roi.width - currentROI.width);
			dh += abs(history.at(objectID).at(t).roi.height - currentROI.height);
			dc += getColorDiff(history.at(objectID).at(t).avgRoiColor, history.at(objectID).at(nPreviousElements).avgRoiColor);
			dc2+= getColorDiff(history.at(objectID).at(t).avgMaskColor, history.at(objectID).at(nPreviousElements).avgMaskColor);
			da += history.at(objectID).at(t).area - currentArea;
			avg_a +=history.at(objectID).at(t).area;
		}

		dx = dx / nPreviousElements;
		dy = dy / nPreviousElements;
		dw = dw / nPreviousElements;
		dh = dh / nPreviousElements;
		dc = dc / nPreviousElements;
		dc2 = dc2 / nPreviousElements;
		da = da / nPreviousElements;
		avg_a = avg_a / nPreviousElements;

		//if (trainedArea == 1) trainedArea = avg_a;
		double darea;
		//if (vStateAreas.size()>0) 
		//	darea = avg_a/vStateAreas.at(0);
		//else 
			darea = avg_a/trainedArea;

		//if ((dx > dposi_th || dy > dposi_th ||

		//dw > dsize_th || dh > dsize_th ||
		//dc > dcolor_th ||
	 //   dc2 > dcolor_th)&& 
		if(1-darea > darea_th)
		{
			return true;
		}
	}
	return false;

}

void EventDetector::addInterestingFrameID(int objectID, int currentFrameID)
{
	int dequeSize = (int)vInterestFrameIDs.size();

	//see if it is the first frame of the detected sequence
	//if not the add the previous nMaxHistory...

	bool addPrevious = true;
	if (dequeSize > 0)
	{
		int prevID = vInterestFrameIDs.at(objectID).at(dequeSize-1);
		if ((currentFrameID - prevID) == 1)
			addPrevious = false;
	}

	if (addPrevious)
	{
		for (int i=mParams.at(current_event).nMaxStates-1+mParams.at(current_event).nStateFadingInItems; i>=0; i--)
		{
			int t = currentFrameID - i - 1;

			if (t >= 0) vInterestFrameIDs.at(objectID).push_back(t);
		}
	}

	vInterestFrameIDs.at(objectID).push_back(currentFrameID);
}

deque <int> EventDetector::removeStateFromVector(deque <int> stateVector, int stateIdToRemove)
{
	//copy current vector
	deque <int> vM;
	for (int i=0; i<(int)stateVector.size(); i++)
		vM.push_back(stateVector.at(i));

	int previousCrispIdx = -1;
	int nextCrispIdx = -1; //-1 not set
	vM.at(0) = 0; //reset first stage

	for (int i=0; i<(int)vM.size(); i++)
	{
		int currentState = vM.at(i);

		if (currentState != stateIdToRemove) //if current state is crisp
		{
			//check if previous crisp index is set
			if (previousCrispIdx == -1)
			{
				previousCrispIdx = i;
			}
			else //if previous set, the set next
			{
				nextCrispIdx = i;

				if (vM.at(previousCrispIdx) == 0)
				{
					int nElementsBetween = nextCrispIdx - previousCrispIdx;

					if (nElementsBetween > 1)
					{
						int nElementsBetweenHalf = nElementsBetween/4;


						//change values of all indices between
						for (int j=previousCrispIdx+1; j<previousCrispIdx+nElementsBetweenHalf; j++)
						{
							vM.at(j) = vM.at(previousCrispIdx);
						}
						for (int j=previousCrispIdx+nElementsBetweenHalf; j<nextCrispIdx; j++)
						{
							vM.at(j) = vM.at(nextCrispIdx);
						}
					}
					//reset pointers
					previousCrispIdx = nextCrispIdx;
					nextCrispIdx = -1;
				}
				else
				{
					int nElementsBetween = nextCrispIdx - previousCrispIdx;

					if (nElementsBetween > 1)
					{
						int nElementsBetweenHalf = nElementsBetween/2;

						//change values of all indices between
						for (int j=previousCrispIdx+1; j<previousCrispIdx+nElementsBetweenHalf; j++)
						{
							vM.at(j) = vM.at(previousCrispIdx);
						}
						for (int j=previousCrispIdx+nElementsBetweenHalf; j<nextCrispIdx; j++)
						{
							vM.at(j) = vM.at(nextCrispIdx);
						}

						//reset pointers
						previousCrispIdx = nextCrispIdx;
						nextCrispIdx = -1;
					}
				}
			}
		}
	}

	return vM;
}

int EventDetector::getNumOfElementsInStateVector(int objectID, int stateId)
{
	int counter = 0;

	for (int i=0; i<(int)vFrameStates.at(objectID).size(); i++)
	{
		if (vFrameStates.at(objectID).at(i) == stateId)
			counter++;
	}

	return counter;
}

void EventDetector::SetParams(int objectID,EventDetectorParams edp, int useCamera)
{
	edp.useTopCamera=useCamera;
	mParams.push_back(edp);
	trackDelay[objectID] = 0;
	current_event = objectID;
	b_panelPressed = false;
	if (useCamera==1) {
		if (imgStaticROI.at(objectID)) cvReleaseImage(&imgStaticROI.at(objectID));
		if (imgStaticTMP.at(objectID)) cvReleaseImage(&imgStaticTMP.at(objectID));
		
		imgStaticROI.at(objectID) = cvCreateImage(cvSize(mParams.at(objectID).roi_defaultSize, mParams.at(objectID).roi_defaultSize), IPL_DEPTH_8U, 3);
		cvSetZero(imgStaticROI.at(objectID));
		imgStaticTMP.at(objectID) = cvCloneImage(imgStaticROI.at(objectID));
	}
}

void EventDetector::SetInput(int objectID, IplImage* img, CvRect roi, double area, IplImage* roiMask, int frameId)
{
	currentFrame = img;

	if (vStateRoiImages.at(objectID).size() > 0 && vStateRoiImages.at(objectID).at(0)!=NULL) currentROI = vStateROI.at(objectID).at(0);
	else currentROI = roi;
//	currentROI = roi;
	if (vStateMaskImages.at(objectID).size() > 0 && vStateMaskImages.at(objectID).at(0)!=NULL) {
		currentMask = vStateMaskImages.at(objectID).at(0);
	}
	else currentMask = roiMask;
//	currentMask = roiMask;
	currentArea = area;
	//add information to history
	HistoryItem currItem;
	currItem.roi = currentROI;
	currItem.area = area;
	currItem.avgRoiColor = getAvgRoiColor(false);
	currItem.avgMaskColor = getAvgRoiColor(true);
	currItem.frameID = frameId;
	history.at(objectID).push_back(currItem);

	//keep history file the same
	if ((int)history.at(objectID).size() > mParams.at(objectID).history_MaxItems)
		history.at(objectID).pop_front();
}

void EventDetector::SetInterestInState(int stateIdx)
{
	interestState = stateIdx;
}

bool EventDetector::TrackSceneEvents(int objectID, bool objectSelected, double frameRate, int useSensorForEvent)
{
	currentState = -1;
	
	trackDelay[objectID]++;
	if (useSensorForEvent == 0) // THANOS - Using camera
	{
		if (currentROI.width > 0 && currentROI.height > 0)
		{
			//add current image to ROI_STATIC!
			cvSetImageROI(currentFrame, currentROI);
			cvResize(currentFrame, imgStaticTMP.at(objectID));
			cvResetImageROI(currentFrame);
			if(vStateRoiImages.at(objectID).size()==0)
				cvAddWeighted(imgStaticTMP.at(objectID),mParams.at(objectID).roi_updateFactor , imgStaticROI.at(objectID), 1.0-mParams.at(objectID).roi_updateFactor, 0.0, imgStaticROI.at(objectID));
			else imgStaticROI.at(objectID) = imgStaticTMP.at(objectID);

			int nPreviousElements = (int)history.at(objectID).size()-1;
			//if ((int)history.size() == mParams.history_MaxItems && mParams.trackDelay < 0)
			if ( trackDelay[objectID] > mParams.at(objectID).trackDelay * (10.0/frameRate))
			{
			//cvSetZero(imgTmp);
				//then consider current position as stable
				weHaveAStaticROI = true;
				//and add it to the vector...
				//if and only if it is different of the ones before it
				double minDiff = 10000;
				double dcolor_th = mParams.at(objectID).changeThreshold_color;
				double darea_th = mParams.at(objectID).changeThreshold_area;
				double dc, dc2, da, darea;
				double threshold = mParams.at(objectID).roi_trainedThreshold;;

				dc = 0;
				dc2 = 0;
				da = 0;
				darea = 0;
				CvScalar avgColor = cvScalar(-1, -1, -1);
				CvScalar avgColorMask = cvScalar(-1, -1, -1);
				CvScalar avgColorState = cvScalar(-1, -1, -1);
				trainedArea = currentArea;

				avgColor = getAvgRoiColor(false);
				avgColorMask = getAvgRoiColor(true);
		
				double currAvg = (avgColorMask.val[0] + avgColorMask.val[1] + avgColorMask.val[2])/3;
				double stateAvgMask;
				double currentDiff[4][4] = {{1,1,1,1},{1,1,1,1},{1,1,1,1},{1,1,1,1}};
				if (vStateRoiImages.at(objectID).size()==1)
				{
					currentState = 0;
				}
				else if (vStateRoiImages.at(objectID).size()>0){
					for (int i=0; i<(int)vStateRoiImages.at(objectID).size(); i++)
					{
						for (int j = 0; j<4 ; j++)
						{
							for (int k = 0; k<4; k++)
							{
								if(j==0 || j == 3 || (j==1 && k != 1 && k != 2)||(j==2 && k != 1 && k != 2)) {
									cvSetImageROI(vStateRoiImages.at(objectID).at(i),cvRect((j/4.0)*(float)vStateRoiImages.at(objectID).at(i)->width,(k/4.0)*(float)vStateRoiImages.at(objectID).at(i)->height,1/4.0*(float)vStateRoiImages.at(objectID).at(i)->width,1/4.0*(float)vStateRoiImages.at(objectID).at(i)->height));
									cvSetImageROI(imgStaticROI.at(objectID),cvRect((j/4.0)*(float)imgStaticROI.at(objectID)->width,(k/4.0)*(float)imgStaticROI.at(objectID)->height,1/4.0*(float)imgStaticROI.at(objectID)->width,1/4.0*(float)imgStaticROI.at(objectID)->height));
									avgColor = cvAvg(imgStaticROI.at(objectID));
									avgColorState = cvAvg(vStateRoiImages.at(objectID).at(i));
									cvResetImageROI(vStateRoiImages.at(objectID).at(i));
									cvResetImageROI(imgStaticROI.at(objectID));
									double meanAvgColor = (avgColor.val[0]+avgColor.val[1]+avgColor.val[2])/3;
									double meanAvgStateColor = (avgColorState.val[0]+avgColorState.val[1]+avgColorState.val[2])/3;
									if (meanAvgStateColor/meanAvgColor < threshold)
									{
										currentState = 1;
										objectSelected = true;
										break;
									}
								}
							}
							if (currentState!= -1) break;
						}
					}
					if (currentState != 1) currentState = 0;
				}

				//already learned!
				double currStateAvg;
				double colorCurrDiffRatio;
				if (currentState == 0 &&  vStateRoiImages.at(objectID).size() < 2 )
				{
					bool newState = false;

					for (int j = 0; j<4 ; j++)
					{
						for (int k = 0; k<4; k++)
						{
							if(j==0 || j == 3 || (j==1 && k != 1 && k != 2)||(j==2 && k != 1 && k != 2)) {
								cvSetImageROI(vStateRoiImages.at(objectID).at(0),cvRect((j/4.0)*(float)vStateRoiImages.at(objectID).at(0)->width,(k/4.0)*(float)vStateRoiImages.at(objectID).at(0)->height,1/4.0*(float)vStateRoiImages.at(objectID).at(0)->width,1/4.0*(float)vStateRoiImages.at(objectID).at(0)->height));
								cvSetImageROI(imgStaticROI.at(objectID),cvRect((j/4.0)*(float)imgStaticROI.at(objectID)->width,(k/4.0)*(float)imgStaticROI.at(objectID)->height,1/4.0*(float)imgStaticROI.at(objectID)->width,1/4.0*(float)imgStaticROI.at(objectID)->height));
								avgColor = cvAvg(imgStaticROI.at(objectID));
								avgColorState = cvAvg(vStateRoiImages.at(objectID).at(0));

								cvResetImageROI(vStateRoiImages.at(objectID).at(0));
								cvResetImageROI(imgStaticROI.at(objectID));
								double meanAvgColor = (avgColor.val[0]+avgColor.val[1]+avgColor.val[2])/3;
								double meanAvgStateColor = (avgColorState.val[0]+avgColorState.val[1]+avgColorState.val[2])/3;
								double tmpThreshold;
								if (meanAvgStateColor/meanAvgColor < threshold)
								{
									if(objectSelected && vStateRoiImages.at(objectID).size() < 2) newState = true;
									else if (vFrameStates.at(objectID).at(vFrameStates.at(objectID).size()-1)!= -1){
										objectSelected = true;
										newState = true;
										currentState = 0;
									}
									break;
								}
							}
						}
						if (newState||objectSelected) break;
					}

					if (newState && vStateRoiImages.at(objectID).size() < 2)
					{
						vStateRoiImages.at(objectID).push_back(cvCloneImage(imgStaticROI.at(objectID)));
						vStateMaskImages.at(objectID).push_back(cvCloneImage(vStateMaskImages.at(objectID).at(0)));
						vStateAvgColors.at(objectID).push_back(avgColor);
						vStateAvgColorsMask.at(objectID).push_back(avgColorMask);
						vStateAreas.at(objectID).push_back(trainedArea);
						vStateROI.at(objectID).push_back(currentROI);
						currentState = (int)vStateRoiImages.at(objectID).size()-1;

						darea = trainedArea/vStateAreas.at(objectID).at(currentState);
					}
					

				}
				else if (currentState == -1 && vStateRoiImages.at(objectID).size() < 1 && currentMask != NULL)
				{
					vStateRoiImages.at(objectID).push_back(cvCloneImage(imgStaticROI.at(objectID)));
					vStateMaskImages.at(objectID).push_back(cvCloneImage(currentMask));
					vStateAvgColors.at(objectID).push_back(avgColor);
					vStateAvgColorsMask.at(objectID).push_back(avgColorMask);
					vStateAreas.at(objectID).push_back(trainedArea);
					vStateROI.at(objectID).push_back(currentROI);

					currentState = 0;				
				}
				for (int i=0; i<(int)vStateRoiImages.at(objectID).size(); i++)
				{
					char charBuf[256];
					sprintf_s(charBuf, 256, "State %d of object %d",i,objectID);

					cvNamedWindow(charBuf);
					cvShowImage(charBuf, vStateRoiImages.at(objectID).at(i));
				}
			}
			else if (history.at(objectID).size()!=0) trainedArea = history.at(objectID).at((int)history.at(objectID).size()-1).area;
			else trainedArea = 1;
			//cvShowImage("Event Params", imgTmp);
			//cvSetZero(imgTmp);
	//		cvReleaseImage(&imgStaticROI);
	//		cvReleaseImage(&currentMask);
		}
	}
	else if (useSensorForEvent == 1)//THANOS - Using Phidget Sensor
	{
		if (vSensorStates.at(objectID).size()==1)
		{
			currentState = 0;
		}
		else if (vSensorStates.at(objectID).size()>0)
		{
			if(objectID == 0)
			{
				if (mPhoneActivity == 0)
				{
					currentState = 1; 
				}
				else currentState = 0;
				objectSelected = true;
			}
			if(objectID == 1)
			{
				if (mPanelActivity == 0)
				{
					currentState = 1; 
					b_panelPressed = true;
				}
				else {
					currentState = 0;
					b_panelPressed = false;
				}
				objectSelected = true;
			}
		}

		if(currentState == 0 &&  vSensorStates.at(objectID).size() < 2 )
		{
			bool newState = false;
			if(objectID == 0 && (mPhoneActivity != m_tempPhidgetPhoneStatus))
			{	
				m_tempPhidgetPhoneStatus = mPhoneActivity;
			}
			else if (objectID == 1 && (mPanelActivity != m_tempPhidgetPanelStatus))
			{
				m_tempPhidgetPanelStatus = mPanelActivity;
			}
			if(objectSelected && vSensorStates.at(objectID).size() < 2) newState = true;
			else if (vFrameStates.at(objectID).at(vFrameStates.at(objectID).size()-1)!= -1){
				objectSelected = true;
				newState = true;
				currentState = 0;
			}

			if (newState && vSensorStates.at(objectID).size() < 2)
			{
				currentState = (int)vSensorStates.at(objectID).size()-1;
				vSensorStates.at(objectID).push_back(currentState);
//					SET CURRENT STATE - NEW
			}
		}
		else if (currentState == -1 && vSensorStates.at(objectID).size() < 1)
		{
			if (objectID==0)
				vSensorStates.at(objectID).push_back(0);
			else if (objectID==1)
				vSensorStates.at(objectID).push_back(0);
			currentState = 0;				
		}
	}
	else //EIRINI - Using Ultrasonic Sensor
	{
		if (vSensorStates.at(objectID).size()==1)
		{
			currentState = 0;
		}
		else if (vSensorStates.at(objectID).size()>0)
		{
			if(objectID == 0)
			{
				if (mPhoneActivity == 0) // ACTIVITY_PHONE_ON
				{
					currentState = 1; 
				}
				else // ACTIVITY_PHONE_OFF
				{
					currentState = 0;
				}
				objectSelected = true;
			}
		}

		if(currentState == 0 &&  vSensorStates.at(objectID).size() < 2 )
		{
			bool newState = false;
			if(objectID == 0 && (mPhoneActivity != m_tempUltrasonicPhoneStatus))
			{	
				m_tempUltrasonicPhoneStatus = mPhoneActivity;
			}
			if(objectSelected && vSensorStates.at(objectID).size() < 2) 
				newState = true;
			else if (vFrameStates.at(objectID).at(vFrameStates.at(objectID).size()-1)!= -1)
			{
				objectSelected = true;
				newState = true;
				currentState = 0;
			}

			if (newState && vSensorStates.at(objectID).size() < 2)
			{
				currentState = (int)vSensorStates.at(objectID).size()-1;
				vSensorStates.at(objectID).push_back(currentState);
			}
		}
		else if (currentState == -1 && vSensorStates.at(objectID).size() < 1)
		{
			vSensorStates.at(objectID).push_back(0);
			currentState = 0;				
		}
	}
	
	vFrameStates.at(objectID).push_back(currentState);
	return objectSelected;
}

bool EventDetector::DoWeHaveAStaticROI()
{
	return weHaveAStaticROI;
}

int EventDetector::GetCurrentState(int objectID)
{
	return vFrameStates.at(objectID).at(vFrameStates.at(objectID).size()-1);
	//return currentState;
}

int EventDetector::GetNumOfStatesDetected(int objectID)
{
	return (int)vFrameStates.at(objectID).size();
}

deque <int> EventDetector::GetInterestFrameIDs(int objectID)
{
	return vInterestFrameIDs.at(objectID);
}

deque <int> EventDetector::GetStateVector(int objectID)
{
	return vFrameStates.at(objectID);
}

deque <int> EventDetector::GetCrispStateVector(int objectID, int maxNumOfClasses)
{
	deque <int> vRet = removeStateFromVector(vFrameStates.at(objectID), -1);

	int nStateClasses = GetNumOfStatesDetected(objectID);

	int nClassesToRemove = nStateClasses - maxNumOfClasses;
	deque <int> vStatesNum;

	if (nClassesToRemove > 0)
	{
		//create a vector containing all the class numbers
		for (int i=0; i<nStateClasses; i++)
			vStatesNum.push_back(getNumOfElementsInStateVector(objectID,i));
	
		//remove classes with the smaller number of frames
		for (int i=0; i<nClassesToRemove; i++)
		{
			//select the class to remove
			int minSizedClassIdx = -1;
			int currentMinSize = 100000;
			for (int j=0; j<(int)vStatesNum.size(); j++)
			{
				if (vStatesNum.at(j) < currentMinSize && 
					vStatesNum.at(j) != -1)
				{
					currentMinSize = vStatesNum.at(j);
					minSizedClassIdx = j;
				}
			}

			//set it a high number so that we do not choose it again
			vStatesNum.at(minSizedClassIdx) = -1;
			vRet = removeStateFromVector(vRet, minSizedClassIdx);
		}
	}
	vStatesNum.clear();
	
	return vRet;
}


// deque <int> EventDetector::GetStatesOfFrames(int stateIdToRemove)
// {
// 	if (stateIdToRemove <= -2) return vFrameStates;
// 
// 	//copy vector
// 	deque <int> vM;
// 	for (int i=0; i<(int)vFrameStates.size(); i++)
// 		vM.push_back(vFrameStates.at(i));
// 
// 
// 	int previousCrispIdx = -1;
// 	int nextCrispIdx = -1; //-1 not set
// 	vM.at(0) = 0; //reset first stage
// 
// 	for (int i=0; i<(int)vM.size(); i++)
// 	{
// 		int currentState = vM.at(i);
// 
// 		if (currentState != stateIdToRemove) //if current state is crisp
// 		{
// 			//check if previous crisp index is set
// 			if (previousCrispIdx == -1)
// 			{
// 				previousCrispIdx = i;
// 			}
// 			else //if previous set, the set next
// 			{
// 				nextCrispIdx = i;
// 				int nElementsBetween = nextCrispIdx - previousCrispIdx;
// 				
// 				if (nElementsBetween > 1)
// 				{
// 					int nElementsBetweenHalf = nElementsBetween/2;
// 
// 					//change values of all indices between
// 					for (int j=previousCrispIdx+1; j<previousCrispIdx+nElementsBetweenHalf; j++)
// 					{
// 						vM.at(j) = vM.at(previousCrispIdx);
// 					}
// 					for (int j=previousCrispIdx+nElementsBetweenHalf; j<nextCrispIdx; j++)
// 					{
// 						vM.at(j) = vM.at(nextCrispIdx);
// 					}
// 
// 					//reset pointers
// 					previousCrispIdx = nextCrispIdx;
// 					nextCrispIdx = -1;
// 				}
// 			}
// 		}
// 	}
// 
// 	return vM;
// }