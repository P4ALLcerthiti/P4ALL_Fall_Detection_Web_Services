//Implementation of papaer On Learning, Representing and Generalizing a Task in a Humanoid Robot
//S. Calinon and F. Guenter and A. Billard

//#include "ActivityRecognition_ConfigDLL.h"
#include "GmmMatrix.h"
#include <deque>
#include <vector>

class GaussianMixture {
  /* GMM class, see main.cpp to see some sample code */
 private :
  int m_nState, m_dim;
  GmmMatrix m_mu;
  GmmMatrix *m_sigma;
  float *m_priors;

 public :
  GaussianMixture(int stateNum, int commonLength, int featureVectorSize); 
  GaussianMixture(); 
  ~GaussianMixture();

  std::deque < std::deque<double> > resizeDeque(std::deque < std::deque<double> > deq, int newSize);
  double* resizeVector(std::vector<double> vec, int newSize);

  bool setAnglesFilePath(char* anglesFilePath);
  bool addObservationSequenceToVector();
  bool createObservationSequenceFromFile();
  bool setOutputSignatureFilePath(char* outputFilePath);
  bool train();
  bool saveSignature(bool realWorldCoordinates =false, bool warpedSignature=false);

  bool loadSignature(char* anglesFilePath);
  float doClassification();




private:
	std::string m_outputFilePath;
	std::vector<char* > vAnglesFiles;
	char* anglesFile;
	GmmMatrix anglesDataSet;
	int m_commonLength;
	int m_stateNum;
	int m_featureVectorSize;


private:

	void debug(void);			/* display on screen current parameters */

	GmmMatrix loadDataFile(const char filename []);				// Load the dataset from a file
	bool saveDataFile(const char filename [], GmmMatrix data);	// Save the dataset to a file
	bool saveRegressionResult(const char fileMu[], const char fileSigma[], GmmMatrix inData, GmmMatrix outData, GmmMatrix outSigma[]);	// Save the result of a regression
	bool loadParams(const char filename[]);		/* Load the means, priors probabilies and covariances matrices stored in a file .. (see saveParams )*/
	void saveParams(const char filename []);	/* save current parameters in a file */

	/* do a regression with current parameters : 
	- output a matrix of size nb row of in * nb components in outComponents
	- the SigmaOut pointer will point to an array of nb row of in or out covariances matrices
	- inComponents and outComponents are the index of the dimensions 
	represented in the in and out matrices */
	GmmMatrix doRegression( GmmMatrix in, GmmMatrix * SigmaOut, GmmVector inComponents, GmmVector outComponents);

	float pdfState(GmmVector v,GmmVector Components,int state); 	/* Compute probabilty of vector v ( corresponding to dimension given in the Components vector) for the given state. */
	float pdfState(GmmVector v,int state);	// same as above but v is of same dimension as the GMM 
	
	/* init the GaussianMixture by splitting the dataset into  time (first dimension) slices and computing variances 
	and means for each slices. Once initialisation has been performed, the nb of state is set */
	void initEM_TimeSplit(int nState,GmmMatrix Dataset);
	GmmMatrix HermitteSplineFit(GmmMatrix& inData, int nbSteps, GmmMatrix& outData);	/* Spline fitting to rescale trajectories. */

	/* performs Expectation Maximization on the Dataset, in order to obtain a nState GMM Dataset is a GmmMatrix(nSamples,nDimensions) */
	int doEM(GmmMatrix DataSet);
};

