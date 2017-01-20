//Implementation of papaer On Learning, Representing and Generalizing a Task in a Humanoid Robot
//S. Calinon and F. Guenter and A. Billard


#include "stdafx.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include "MathLib.h"
#include "gmr.h"

#define MAXITER 200


GaussianMixture::GaussianMixture(){
}

GaussianMixture::GaussianMixture(int stateNum, int commonLength, int featureVectorSize){
	m_commonLength = commonLength;
	m_stateNum = stateNum;
	m_featureVectorSize = featureVectorSize;
}; 


GaussianMixture::~GaussianMixture(){
	vAnglesFiles.clear();
}; 


GmmMatrix GaussianMixture::loadDataFile(const char filename [])
{
	// Load the dataset from a file
	GmmMatrix result;
	GmmVector vecTmp;
	float valTmp;
	char tmp[1024];
	unsigned int l=0, c=0;

	std::ifstream f(filename);
	if (f.is_open()){
		// Get number of row
		while(!f.eof()){ 
			f.getline(tmp,1024);
			l++;
			if (l==1){
				// Get number of columns
				std::istringstream strm;
				strm.str(tmp); 
				while (strm >> valTmp)
					c++;
			}
		}
		result.Resize(l-1,c); // then the matrix can be allocated
		f.clear();
		f.seekg(0); // returns to beginning of the file
		for(unsigned int i=0;i<l;i++){ 
			f.getline(tmp,1024);
			std::istringstream strm;
			strm.str(tmp); 
			for(unsigned int j=0;j<c;j++){
				strm >> result(i,j);
			}
		}
		f.close();
	}
	else{
		std::cout  << std::endl << "Error opening file " << filename << std::endl; 
		exit(0);
	}
	return result;
}

bool GaussianMixture::saveDataFile(const char filename [], GmmMatrix data)
{
	// Save the dataset to a file
	std::ofstream f(filename);
	if (f.is_open()) {
		for (unsigned int j = 0; j < data.RowSize(); j++){
			for (unsigned int i = 0; i < data.ColumnSize(); i++)
				f << data(j,i) << " ";
			f << std::endl;
		}
		f.close();
	}
	return 1;
}

bool GaussianMixture::saveRegressionResult(const char fileMu[], const char fileSigma[], GmmMatrix inData, GmmMatrix outData, GmmMatrix outSigma[])
{
	// Save the result of a regression
	std::ofstream Mu_file(fileMu); // regressed data
	std::ofstream Sigma_file(fileSigma); // covariances matrices
	for(unsigned int i=0;i<outData.RowSize();i++) {
		Mu_file << inData(i,0) << " ";
		for(unsigned int j=0;j<outData.ColumnSize();j++) 
			Mu_file << outData(i,j) << " ";
		Mu_file << std::endl;
		for(unsigned int k=0;k<outData.ColumnSize();k++) {
			for(unsigned int j=0;j<outData.ColumnSize();j++)
				Sigma_file << outSigma[i](k,j) << " ";
		}
		Sigma_file << std::endl;
	}
	return 1;
}

bool GaussianMixture::loadParams(const char fileName[])
{
	// Load coefficient of a GMM from a file (stored by saveParams Method or with matlab
	std::ifstream fich(fileName);
	if (!fich.is_open())
		return false;
	fich >> m_dim >> m_nState;
	m_priors = new float[m_nState];
	for (int s = 0; s < m_nState; s++)
	{
		fich >> m_priors[s];
	}
	m_mu.Resize(m_nState,m_dim);
	for (int i = 0; i < m_nState; i++)
	{
		for (int j = 0;j<m_dim;j++)
			fich >> m_mu(i,j);
	}
	m_sigma = new GmmMatrix[m_nState];
	for (int s = 0; s < m_nState; s++)
	{
		m_sigma[s] = GmmMatrix(m_dim,m_dim);
		for (int i = 0; i < m_dim; i++)
		{
			for (int j = 0;j<m_dim;j++)
				fich >> m_sigma[s](i,j);
		}
	}
	return true;
}

void GaussianMixture::debug(void)
{
	/*display on std output info about current parameters */
	GmmVector v;
	GmmMatrix smat;
	std::cout << "Nb state : "<< this->m_nState <<std::endl;
	std::cout << "Dimensions : "<< this->m_dim  <<std::endl;
	std::cout << "Priors : ";
	for(int i = 0;i<m_nState;i++) {
		std::cout << m_priors[i] ;
	}
	std::cout << std::endl;
	std::cout << "Means :";
	m_mu.Print();
	std::cout << "Covariance Matrices :";
	v=GmmVector(2);
	v(0)=0;
	v(1)=2;
	for(int i =0;i<m_nState;i++) {
		//float det;
		//GmmMatrix inv;
		//sigma[i].GetMatrixSpace(v,v,inv);
		//inv.Print();
		//std::cout << det << std::endl;
		m_sigma[i].Print();
	}
}

void GaussianMixture::saveParams(const char filename [])
{
	// save the current GMM parameters, coefficents to a file, to be retrieved 
	// by the loadParams method
	std::ofstream file(filename);
	file << m_dim << "  ";
	file << m_nState << std::endl;
	for(int i=0;i<m_nState;i++) file << m_priors[i] <<"  ";
	file << std::endl;
	for(int s=0;s<m_nState;s++) {
		for(int i=0;i<m_dim;i++) {
			file << m_mu(s,i) <<"  ";
		}
		file << std::endl;
	}
	for(int s=0;s<m_nState;s++) {
		for(int j=0;j<m_dim;j++) {
			for(int i=0;i<m_dim;i++) {
				file << m_sigma[s](i,j) <<"  ";
			}
			file << std::endl;
		}
	}
}

GmmMatrix GaussianMixture::HermitteSplineFit(GmmMatrix& inData, int nbSteps, GmmMatrix& outData)
{
	/* Spline fitting to rescale trajectories. */

	if(nbSteps<=0)
		return outData;

	const int dataSize  = inData.ColumnSize(); 
	const int inSize    = inData.RowSize();
	const int outSize   = nbSteps;

	outData.Resize(outSize,dataSize);
	for(int i=0;i<outSize;i++){
		// Find the nearest data pair
		const float cTime = float(i)/float(outSize-1)*float(inSize-1);
		int   prev, next;
		float prevTime, nextTime;

		prev      = int(floor(cTime));
		next      = prev+1;
		prevTime  = float(prev);
		nextTime  = float(next); 
		const float nphase = (cTime-prevTime)/(nextTime-prevTime);
		const float s1 = nphase;
		const float s2 = s1*nphase;
		const float s3 = s2*nphase; 
		const float h1 =  2.0f*s3 - 3.0f*s2 + 1.0f;
		const float h2 = -2.0f*s3 + 3.0f*s2;
		const float h3 =       s3 - 2.0f*s2 + s1;
		const float h4 =       s3 -      s2;
		// The first column is considered as a temporal value 
		outData(i,0) = (float)i;
		for(int j=1;j<dataSize;j++){
			const float p0 = (prev>0?inData(prev-1,j):inData(prev,j));
			const float p1 = inData(prev,j);
			const float p2 = inData(next,j);
			const float p3 = (next<inSize-1?inData(next+1,j):inData(next,j));
			const float t1 = 0.5f*(p2-p0);
			const float t2 = 0.5f*(p3-p1);
			outData(i,j) = p1*h1+p2*h2+t1*h3+t2*h4;
		}
	}   
	return outData;  
}

void GaussianMixture::initEM_TimeSplit(int nState,GmmMatrix DataSet)
{
	/* init the GaussianMixture by splitting the dataset into 
	time (first dimension) slices and computing variances 
	and means for each slices. 
	once initialisation has been performed, the nb of state is set */

	GmmVector * mean = new GmmVector[nState];
	int nData = DataSet.RowSize();
	this->m_nState = nState;
	this->m_dim = DataSet.ColumnSize();
	float tmax = 0;
	GmmMatrix index(m_nState,nData);
	int * pop = new int[m_nState];
	m_priors = new float[m_nState];
	m_mu.Resize(m_nState,m_dim);
	m_sigma = new GmmMatrix[m_nState];


	GmmMatrix unity(m_dim,m_dim); /* defining unity matrix */
	for(int k = 0;k<m_dim;k++) unity(k,k)=1.0;

	for(int n = 0;n<nData;n++) /* getting the max value for time */
	{
		if(DataSet(n,0) > tmax) tmax = DataSet(n,0); 
	}
	for(int s=0;s<nState;s++) /* clearing values */
	{
		mean[s].Resize(m_dim,true);
		pop[s]=0;
	}

	/* divide the dataset into slices of equal time
	(tmax/nState) and compute the mean for each slice
	the pop table index to wich slice belongs each sample */
	for(int n = 0;n<nData;n++) 
	{
		int s = (int)((DataSet(n,0)/(tmax+1))*nState);
		//std::cout << s << std::endl;
		mean[s] += DataSet.GetRow(n);
		index(s,pop[s]) = (float)n;
		pop[s] +=1;
	}

	for(int s =0;s<nState;s++)
	{
		m_mu.SetRow(mean[s]/(float)pop[s],s); /* initiate the means computed before */
		m_sigma[s]=GmmMatrix(m_dim,m_dim);
		m_priors[s]=1.0f/m_nState; /* set equi-probables states */
		for(int ind=0;ind<pop[s];ind++)
		{
			for(int i=0;i<m_dim;i++) /* Computing covariance matrices */
			{
				for(int j=0;j<m_dim;j++)
				{
					m_sigma[s](i,j) += (DataSet((int)index(s,ind),i) - m_mu(s,i)) \
						*(DataSet((int)index(s,ind),j)-m_mu(s,j));
				}
			}
		}
		m_sigma[s] *= 1.0f/pop[s];
		m_sigma[s] += unity * 1e-5f; /* prevents this matrix from being non-inversible */
	}
}

int GaussianMixture::doEM(GmmMatrix DataSet)
{
	/* perform Expectation/Maximization on the given Dataset :
	GmmMatrix DataSet(nSamples,Dimensions).
	The GaussianMixture Object must be initialised before 
	(see initEM_TimeSplit method ) */

	int nData = DataSet.RowSize();
	int iter = 0;
	float log_lik;
	float log_lik_threshold = 1e-8f;
	float log_lik_old = -1e10f;

	GmmMatrix unity(m_dim,m_dim);
	for(int k = 0;k<m_dim;k++) unity(k,k)=1.0;


	//EM loop

	while(true)
	{
		float * sum_p = new float[nData];
		GmmMatrix pxi(nData,m_nState);
		GmmMatrix pix(nData,m_nState);
		GmmVector E;

		//char strtmp[64];
		//  sprintf(strtmp, "gmms/%03d.gmm",iter);
		//std::cout << strtmp << std::endl;
		//saveParams(strtmp);

		iter++;
		if (iter>MAXITER){
			std::cout << "EM stops here. Max number of iterations has been reached." << std::endl;
			return iter;
		}

		float sum_log = 0; 

		// Expectation Computing 
		for(int i =0;i<nData;i++)
		{
			sum_p[i]=0;
			for(int j=0;j<m_nState;j++)
			{ 
				float p = pdfState(DataSet.GetRow(i),j);  // P(x|i)
				if(p==0) {
					std::cout << p << std::endl;    
					std::cout << "Error: Null probability. Abort.";
					exit(0);
					return -1;
				}
				pxi(i,j)= p;
				sum_p[i] += p*m_priors[j];
			} 
			sum_log += log(sum_p[i]);
		}
		for(int j=0;j<m_nState;j++)
		{
			for(int i=0;i<nData;i++)
			{
				pix(i,j) = pxi(i,j)*m_priors[j]/sum_p[i]; // then P(i|x) 
			}   
		} 

		// here we compute the log likehood 
		log_lik = sum_log/nData; 
		if(fabs((log_lik/log_lik_old)-1) < log_lik_threshold )
		{
			/* if log likehood hasn't move enough, the algorithm has
			converged, exiting the loop */
			//std::cout << "threshold ok" << std::endl;
			return iter;
		}
		//std::cout << "likelihood " << log_lik << std::endl;
		log_lik_old = log_lik;

		// Update Step
		pix.SumRow(E);
		for(int j=0;j<m_nState;j++) 
		{
			m_priors[j]=E(j)/nData; // new priors 
			GmmVector tmu(m_dim);
			GmmMatrix tmsigma(m_dim,m_dim);
			for(int i=0;i<nData;i++) // Means update loop
			{
				tmu += DataSet.GetRow(i)*pix(i,j); 
			}   
			m_mu.SetRow(tmu/E(j),j);

			for(int i=0;i<nData;i++) // Covariances updates
			{
				GmmMatrix Dif(m_dim,1);
				Dif.SetColumn((DataSet.GetRow(i)-m_mu.GetRow(j)),0);
				tmsigma += (Dif*Dif.Transpose())*pix(i,j);
			}
			m_sigma[j] = tmsigma/E(j) + unity * 1e-5f;
		}
	}
	return iter;
}

float GaussianMixture::pdfState(GmmVector Vin,int state)
{
	/* get the probability density for a given state and a given vector */
	GmmMatrix inv_sigma;
	float det_sigma;
	double p;
	GmmVector dif;
	m_sigma[state].Inverse(inv_sigma,&det_sigma); 
	if(m_sigma[state].IsInverseOk())
	{
		dif = Vin - m_mu.GetRow(state);
		p=(double)(dif*(inv_sigma*dif));
		p=exp(-0.5*p)/sqrt(pow(2*3.14159,m_dim)*fabs(det_sigma));
		if(p < 1e-40) return 1e-40f;
		else return (float)p;
	}
	else 
	{
		// sigma[state].Print();
		std::cout << "fail invert sigma matrix" << state << std::endl;
		return 0;
	}
}


float GaussianMixture::pdfState(GmmVector Vin,GmmVector Components,int state) 
{
	/* Compute the probability density function at vector Vin, 
	(given along the dimensions Components), for a given state */ 
	GmmVector mu_s;
	GmmMatrix sig_s;
	GmmMatrix inv_sig_s;
	float det_sig;
	float p;
	int dim_s;
	dim_s = Components.Size();
	m_mu.GetRow(state).GetSubVector(Components,mu_s);
	m_sigma[state].GetMatrixSpace(Components,Components,sig_s);
	sig_s.Inverse(inv_sig_s,&det_sig);
	if(sig_s.IsInverseOk())
	{
		p = (Vin-mu_s) * ( inv_sig_s*(Vin-mu_s));
		p = exp(-0.5f*p)/sqrt(pow(2.0f*3.14159f,dim_s)*fabs(det_sig));
		return p;
	}
	else
	{
		std::cout << "Error in the inversion of sigma" << std::endl;
		exit(0);
		return 0;
	}
}



GmmMatrix GaussianMixture::doRegression(GmmMatrix in,GmmMatrix * SigmaOut,GmmVector inComponents,GmmVector outComponents)
{
	int nData = in.RowSize();
	int outDim = outComponents.Size();
	GmmMatrix Pxi(nData,m_nState);
	GmmMatrix out(nData,outDim);
	GmmMatrix *subSigma;
	GmmMatrix *subSigmaVar;
	GmmMatrix subMu;
	GmmMatrix subMuIn;
	GmmMatrix subMuOut;

	for(int i=0;i<nData;i++)
	{
		float norm_f = 0.0f;
		for(int s=0;s<m_nState;s++){
			float p_i = m_priors[s]*pdfState(in.GetRow(i),inComponents,s);
			Pxi(i,s) = p_i;
			norm_f += p_i;
		}
		Pxi.SetRow(Pxi.GetRow(i)/norm_f,i);
	}

	subSigma = new GmmMatrix[m_nState];
	subSigmaVar = new GmmMatrix[m_nState];
	m_mu.GetColumnSpace(outComponents,subMuOut);
	m_mu.GetColumnSpace(inComponents,subMuIn);

	for(int s=0;s<m_nState;s++)
	{
		GmmMatrix isubSigmaIn;
		GmmMatrix subSigmaOut;
		m_sigma[s].GetMatrixSpace(inComponents,inComponents,subSigmaOut);
		subSigmaOut.Inverse(isubSigmaIn);
		m_sigma[s].GetMatrixSpace(outComponents,inComponents,subSigmaOut);
		subSigma[s] = subSigmaOut*isubSigmaIn;
		m_sigma[s].GetMatrixSpace(outComponents,outComponents,subSigmaOut);
		m_sigma[s].GetMatrixSpace(inComponents,outComponents,isubSigmaIn);
		subSigmaVar[s] = subSigmaOut - subSigma[s]*isubSigmaIn;

	}

	for(int i=0;i<nData;i++)
	{
		GmmVector sv(outDim,true);
		GmmVector sp(outDim,true);
		SigmaOut[i] = GmmMatrix(outDim,outDim);
		for(int s=0;s<m_nState;s++)
		{
			sp = subMuOut.GetRow(s); 
			sp = sp + subSigma[s]*(in.GetRow(i)-subMuIn.GetRow(s));
			sv += sp*Pxi(i,s);

			// CoVariance Computation
			SigmaOut[i]=SigmaOut[i] + subSigmaVar[s]*(Pxi(i,s)*Pxi(i,s));
		}
		out.SetRow(sv,i);
	}
	return out;
}

std::deque < std::deque<double> > GaussianMixture::resizeDeque(std::deque < std::deque<double> > deq, int newSize){

	std::deque < std::deque<double> > resDeq;

	std::string middleStepVal_str;
	char middleStepVal_char[50];

	GmmMatrix rawData, interpol;

	int nbOriginalData = deq.size();
	int nbVar = deq[0].size();

	rawData.Resize(nbOriginalData,nbVar);

	for(int i=0; i<nbOriginalData; i++)
	{
		for(unsigned int j=0; j<nbVar; j++)
		{
			std::istringstream issVal;
			sprintf(middleStepVal_char, "%g", deq[i][j] ); 
			middleStepVal_str = middleStepVal_char;
			issVal.str(middleStepVal_str);
			issVal >> rawData(i,j);
		}
	}


	interpol.Resize(newSize, nbVar);

	HermitteSplineFit(rawData, newSize, interpol);

	resDeq.resize(newSize);
	for(int i=0; i<newSize; i++){
		resDeq[i].resize(nbVar);
	}

	for(int i=0; i<newSize; i++){
		for(int j=0; j<nbVar; j++){
			resDeq[i][j] = interpol(i,j);
		}
	}

	return resDeq;
}

double* GaussianMixture::resizeVector(std::vector<double> vec, int newSize){

	std::deque<double> resVec;
	double *a;

	//std::string middleStepVal_str;
	//char middleStepVal_char[50];

	//GmmMatrix rawData, interpol;

	//int nbOriginalData = vec.size();
	//int nbVar = deq[0].size();

	////rawData.Resize(nbOriginalData,nbVar);

	//for(int i=0; i<nbOriginalData; i++){
	//	for(unsigned int j=0; j<nbVar; j++){
	//		std::istringstream issVal;
	//		sprintf(middleStepVal_char, "%g", deq[i][j] ); 
	//		middleStepVal_str = middleStepVal_char;
	//		issVal.str(middleStepVal_str);
	//		issVal >> rawData(i,j);
	//	}
	//}


	//interpol.Resize(newSize, nbVar);

	//HermitteSplineFit(rawData, newSize, interpol);

	//resDeq.resize(newSize);
	//for(int i=0; i<newSize; i++){
	//	resDeq[i].resize(nbVar);
	//}

	//for(int i=0; i<newSize; i++){
	//	for(int j=0; j<nbVar; j++){
	//		resVec[i][j] = interpol(i,j);
	//	}
	//}

	return a;
}


bool GaussianMixture::train(){

	int noOfFiles = vAnglesFiles.size();
	if(noOfFiles==0){
		return false;
	}


	GmmMatrix* rawData = new GmmMatrix[noOfFiles];
	GmmMatrix* interpolData = new GmmMatrix[m_commonLength];
	
	int nbData = 0;

	for (unsigned int i = 0; i < noOfFiles; i++){
		rawData[i] = loadDataFile(vAnglesFiles[i]); 
		nbData += rawData[i].RowSize();
	}
	nbData = (int) (nbData/noOfFiles);
	int nbVar = rawData[0].ColumnSize();

	//Rescale the raw data and save the result
	GmmMatrix interpol, dataset;
	interpol.Resize(nbData,nbVar);
	for (int i=0; i<noOfFiles; i++){
		HermitteSplineFit(rawData[i], nbData, interpol); 
		dataset = interpol.VCat(dataset);
		interpolData[i]=interpol;
	}

	//Learn the GMM model and save the result
	initEM_TimeSplit(m_stateNum, dataset); // initialize the model
	doEM(dataset); // performs EM

	return true;
}


float GaussianMixture::doClassification(){

	GmmMatrix DataSet = anglesDataSet;
	float logLik = 0.0;
	int nData = DataSet.RowSize();

	float * sum_p = new float[nData];
	GmmMatrix pxi(nData,m_stateNum);
	GmmMatrix pix(nData,m_stateNum);
	float sum_log = 0; 
	//// Expectation Computing 
	for(int i =0;i<nData;i++)
	{
		sum_p[i]=0;
		for(int j=0;j<m_stateNum;j++)
		{ 
			float p = pdfState(DataSet.GetRow(i),j);  // P(x|i)
			if(p==0) {
				std::cout << p << std::endl;    
				std::cout << "Error: Null probability. Abort.";
				exit(0);
				return -1;
			}
			pxi(i,j)= p;
			sum_p[i] += p*m_priors[j];
		} 
		sum_log += log(sum_p[i]);
	}

	//// here we compute the log likehood 
	logLik = sum_log/nData; 

	return logLik;
}


bool GaussianMixture::createObservationSequenceFromFile(){

	GmmMatrix* anglesData = new GmmMatrix[1];
	anglesData[0] = loadDataFile(anglesFile);

	anglesDataSet = anglesData[0].VCat(anglesDataSet);

	return true;
}


bool GaussianMixture::setAnglesFilePath(char* anglesFilePath){

	anglesFile = anglesFilePath;

	return true;
}


bool GaussianMixture::addObservationSequenceToVector(){

	vAnglesFiles.push_back(anglesFile);

	return true;
}


bool GaussianMixture::saveSignature(bool realWorldCoordinates /*=false*/, bool warpedSignature/*=false*/){

	bool retVal = true;

	std::string outputFilename;
	outputFilename = m_outputFilePath;
	if(realWorldCoordinates){
		if(warpedSignature){
			outputFilename.append("warpedgmmRW.txt");
		}else{
			outputFilename.append("gmmRW.txt");
		}
	}else{
		if(warpedSignature){
			outputFilename.append("warpedgmm.txt");
		}else{
			outputFilename.append("gmm.txt");
		}
		
	}
	saveParams(outputFilename.c_str());

	return retVal;
}

bool GaussianMixture::loadSignature(char* signatureFilePath){

	std::string inputFilename;
	inputFilename = signatureFilePath;
	bool retVal = loadParams(inputFilename.c_str());

	return retVal;
}


bool GaussianMixture::setOutputSignatureFilePath(char* outputFilePath){

	m_outputFilePath = outputFilePath;

	return true;
}
