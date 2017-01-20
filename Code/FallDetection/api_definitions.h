#pragma once

#include <string>

#define ENABLE_PLOT_PSD									0
#define ENABLE_PLOT_R									0
#define ENABLE_PLOT_THETA								0
#define ENABLE_PLOT_DELTA_THETA							1
#define ENABLE_PLOT_PHI									0
#define ENABLE_PLOT_DELTA_PHI							1
#define ENABLE_PLOT_SMA									1
#define ENABLE_PLOT_MOVEMENT							1

#define ENABLE_MLP_FUNCTION								0

#define OBSERVATION_FALL_TIME_IN_SECS					3//secs
#define WAITING_TIME_BEFORE_RECORDING_IN_SECS			3//secs

#define ACCELEROMETER_FREQUENCY							60.0
#define REDUCED_ACCELEROMETER_FREQUENCY					60.0
#define ACCEL_FEATURES_DOWNSAMPLING_RATES				REDUCED_ACCELEROMETER_FREQUENCY/ACCELEROMETER_FREQUENCY
#define ACCELEROMETERS_SMOOTHING_WINDOW					6 //CHECK:fixed by Phidget Accelerometer specs - However, may be less in here due to computational delays during processing

#define	PI												3.14159265359

#define FILEPATH_ACCELEROMETER_INIFILE					".\\Config\\activityParams.ini"
#define FILEPATH_MLP_INIFILE							".\\Config\\mlpParams.ini"

#define FOLDERNAME_RECORDING_PATH						"Recording_"
#define FOLDERNAME_SUBJECT_								"Subject_"
#define FOLDERNAME_WALKING								"Walking"
#define FOLDERNAME_RUNNING								"Running"
#define FOLDERNAME_FALLING								"Falling"
#define FOLDERNAME_SITTING								"Sitting"
#define FOLDERNAME_STAIRS_UP							"Stairs_Down"
#define FOLDERNAME_STAIRS_DOWN							"Stairs_Up"

#define FILENAME_BITMAP_RED								".\\Config\\Bitmaps\\red.bmp"
#define FILENAME_BITMAP_GREY							".\\Config\\Bitmaps\\grey.bmp"
#define FILENAME_BITMAP_GREEN							".\\Config\\Bitmaps\\green.bmp"
#define FILENAME_BITMAP_YELLOW							".\\Config\\Bitmaps\\yellow.bmp"

#define FOLDERNAME_ACCELERATION_RECORDINGS				"AccelerationRecordings"	//for training
#define FOLDERNAME_ACCELERATION_TESTS					"AccelerationTests"			//for testing
#define FOLDERNAME_LOG									"Log"
#define FOLDERNAME_MLP									"MLP"
#define FOLDERNAME_OUTPUT								"Output"
#define FOLDERNAME_ACCELEROMETER						"Accelerometer"

#define FILENAME_ACCELEROMETER_ANGLES					"angles.txt"
#define FILENAME_ACCELEROMETER_PROCESSED_ANGLES			"processedAngles.txt"
#define FILENAME_TIMESTAMP								"index.ini"
#define FILENAME_LOG									"log.txt"

//MLP IN
#define FILENAME_ALL_TRAINING_DATA						"allTrainingData.txt"
#define FILENAME_ALL_TRAINING_DATA_RESULTS				"allTrainingDataResults.txt"

//MLP OUT
#define FILENAME_MULTILAYERPERCEPTRON_DAT				"multilayerPerceptron.dat"

#define ACCELEROMETER_DIMENSIONS						3

#define ACCELEROMETER_TIMER_PERIOD						1000/ACCELEROMETER_FREQUENCY
#define ACCELEREOMETER_MAXIMUM_RECORDING_CYCLES_NUMBER	1500000 //~15000 sec-> ~4 Hrs

#define ACCELEROMETER_SAMPLES_FOR_SMA					ACCELEROMETER_FREQUENCY

#define ACCEL_SMA_LOW_LB								0.0
#define ACCEL_SMA_LOW_UB								0.9
#define ACCEL_SMA_HIGH_LB								0.7
#define ACCEL_SMA_HIGH_UB								1.6

#define ACCEL_DELTA_THETA_REST_LB						0.0
#define ACCEL_DELTA_THETA_REST_UB						0.6
#define ACCEL_DELTA_THETA_HEAVY_LB						0.4
#define ACCEL_DELTA_THETA_HEAVY_UB						15

#define ACCEL_DELTA_PHI_REST_LB							0.0
#define ACCEL_DELTA_PHI_REST_UB							0.4
#define ACCEL_DELTA_PHI_HEAVY_LB						0.2
#define ACCEL_DELTA_PHI_HEAVY_UB						20

#define MOV_REST_LB										-0.0001
#define MOV_REST_UB										0.4
#define MOV_AVERAGE_LB									0.3
#define MOV_AVERAGE_UB									0.6
#define MOV_HEAVY_LB									0.5
#define MOV_HEAVY_UB									1.0

#define MLP_NO_OF_INPUT_LAYERS							180 //OBSERVATION_FALL_TIME_IN_SECS sec* ACCELEROMETER_FREQUENCY Hz
#define MLP_NO_OF_OUTPUT_LAYERS							1
#define MLP_NO_OF_HIDDEN_LAYERS							2
#define MLP_FIRST_LAYER_NEURONS							30
#define MLP_SECOND_LAYER_NEURONS						20

#define MLP_THRESHOLD_VAL								0.85
#define MLP_DECISIONS_SIZE								120
#define MLP_DCISIONS_THRESHOLD_VAL						60



typedef enum ActionEnum
{
	WALKING=0,
	RUNNING,
	FALLING,
	SITTING,
	STAIRS_UP,
	STAIRS_DOWN,
	ACTIONENUM_SIZE
}ActionEnum;


static std::string GetStringFromActionEnum(int type)
{	
	switch(type)
	{
	case WALKING:
		return "Walking";
	case RUNNING:
		return "Running";
	case FALLING:
		return "Falling";
	case SITTING:
		return "Sitting";
	case STAIRS_UP:
		return "Stairs_Up";
	case STAIRS_DOWN:
		return "Stairs_Down";
	default:
		return "0";
	}
}

static int GetIntFromActionEnum(int type)
{	
	switch(type)
	{
	case WALKING:
		return 0;
	case RUNNING:
		return 0;
	case FALLING:
		return 1;
	case SITTING:
		return 0;
	case STAIRS_UP:
		return 0;
	case STAIRS_DOWN:
		return 0;
	default:
		return 0;
	}
}