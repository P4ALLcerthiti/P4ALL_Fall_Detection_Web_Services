//Implementation of papaer On Learning, Representing and Generalizing a Task in a Humanoid Robot
//S. Calinon and F. Guenter and A. Billard

#ifndef VECTOR_H
#define VECTOR_H

#include "Macros.h"
#include <math.h>
#include <iostream>

#ifndef NULL
#define NULL 0
#endif

#ifdef  USE_T_EXTENSIONS
template<unsigned int ROW> class TVector;
#endif

class GmmVector
{
  friend class GmmMatrix;
#ifdef  USE_T_EXTENSIONS  
  template<unsigned int ROW> friend class TVector;  
#endif
  
protected:
  static  float undef;
   
          unsigned int   row;
	        float         *_;

public:

  
	inline GmmVector() {
    row = 0;
    _   = NULL;
  }
  
  inline virtual ~GmmVector(){
    Release(); 
  }

  inline GmmVector(const GmmVector &vector)
  {
    row = 0;
    _   = NULL;
    Resize(vector.row,false);
    for (unsigned int i = 0; i < row; i++)
      _[i] = vector._[i];
  }

  inline GmmVector(unsigned int size, bool clear = true)
  {
    row = 0;
    _   = NULL;
    Resize(size,false);
    if(clear)
      Zero();
  }
  
	inline GmmVector(const float _[], unsigned int size)
	{
    row       = 0;
    this->_   = NULL;
    Resize(size,false);
		for (unsigned int i = 0; i < row; i++)
			this->_[i] = _[i];
	}
   
#ifdef  USE_T_EXTENSIONS
  template<unsigned int ROW> inline GmmVector(const TVector<ROW> &vector)
  {
    row = 0;
    _   = NULL;
    Resize(ROW,false);
    for (unsigned int i = 0; i < row; i++)
      _[i] = vector._[i];        
  }
#endif     
   
  inline GmmVector& Zero()
  {
    for (unsigned int i = 0; i < row; i++)
      _[i] = 0.0f;
    return *this;    
  }

  inline GmmVector& One()
  {
    for (unsigned int i = 0; i < row; i++)
      _[i] = 1.0f;
    return *this;    
  }

  inline GmmVector& Random(){
    for (unsigned int j = 0; j < row; j++)
      _[j] =((float)rand())/((float)(RAND_MAX+1.0));    
    return *this;    
  }
    
  inline unsigned int Size() const{
    return row;
  }
  
  inline float *GetArray() const{
    return _;
  }
  
  inline float& operator[] (const unsigned int row)
  {
    if(row<this->row)
      return _[row];
    return undef; 
  }
  
  inline float& operator() (const unsigned int row)
  {
    if(row<this->row)
      return _[row];
    return undef; 
  }
  
	inline GmmVector operator - () const
	{
		GmmVector result(row,false);
		for (unsigned int i = 0; i < row; i++)
			result._[i] = -_[i];
    return result;
	}
  
  inline GmmVector& Set(const GmmVector &vector){
    return (*this)=vector;  
  }
  
  inline GmmVector& operator = (const GmmVector &vector)
  {
    Resize(vector.row,false);
    const unsigned int k = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < k; i++)
      _[i] = vector._[i];
    for (unsigned int i = k; i < row; i++)
      _[i] = 0;
    return *this;    
  }
  
	inline GmmVector& operator += (const GmmVector &vector)
	{
    const unsigned int k = (row<=vector.row?row:vector.row);
		for (unsigned int i = 0; i < k; i++)
			_[i] += vector._[i];
    return *this;
	}

	inline GmmVector& operator -= (const GmmVector &vector)
	{
    const unsigned int k = (row<=vector.row?row:vector.row);
		for (unsigned int i = 0; i < k; i++)
			_[i] -= vector._[i];
    return *this;
	}

  inline GmmVector& operator ^= (const GmmVector &vector)
  {
    const unsigned int k = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < k; i++)
      _[i] *= vector._[i];
    return *this;
  }

  inline GmmVector& operator /= (const GmmVector &vector)
  {
    const unsigned int k = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < k; i++)
      _[i] /= vector._[i];
    return *this;
  }

  inline GmmVector& operator += (float scalar)
  {
    for (unsigned int i = 0; i < row; i++)
      _[i] += scalar;
    return *this;
  }

  inline GmmVector& operator -= (float scalar)
  {
    for (unsigned int i = 0; i < row; i++)
      _[i] -= scalar;
    return *this;
  }

	inline GmmVector& operator *= (float scalar)
	{
		for (unsigned int i = 0; i < row; i++)
			_[i] *= scalar;
    return *this;
	}

	inline GmmVector& operator /= (float scalar)
	{
		scalar = 1.0f / scalar;
		for (unsigned int i = 0; i < row; i++)
			_[i] *= scalar;
    return *this;
	}

  inline GmmVector operator + (const GmmVector &vector) const
  {
    GmmVector result(row,false);
    return Add(vector,result);    
  }
  
  inline GmmVector& Add(const GmmVector &vector, GmmVector& result) const
  {   
    result.Resize(row,false);
    const unsigned int k = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < k; i++)
      result._[i] = _[i] + vector._[i];
    for (unsigned int i = k; i < row; i++)
      result._[i] = _[i];      
    return result;
  }

  inline GmmVector operator - (const GmmVector &vector) const
  {
    GmmVector result(row,false);
    return Sub(vector,result);    
  }
  
  inline GmmVector& Sub(const GmmVector &vector, GmmVector& result) const
  {
    result.Resize(row,false);
    const unsigned int k = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < k; i++)
      result._[i] = _[i] - vector._[i];
    for (unsigned int i = k; i < row; i++)
      result._[i] = _[i];      
    return result; 
  }

  inline GmmVector operator ^ (const GmmVector &vector) const
  {
    GmmVector result(row,false);
    return PMult(vector,result);    
  }
  
  inline GmmVector& PMult(const GmmVector &vector, GmmVector& result) const
  {   
    result.Resize(row,false);
    const unsigned int k = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < k; i++)
      result._[i] = _[i] * vector._[i];
    for (unsigned int i = k; i < row; i++)
      result._[i] = _[i];      
    return result;
  }

  inline GmmVector operator / (const GmmVector &vector) const
  {
    GmmVector result(row,false);
    return PDiv(vector,result);    
  }
  
  inline GmmVector& PDiv(const GmmVector &vector, GmmVector& result) const
  {
    result.Resize(row,false);
    const unsigned int k = (row<=vector.row?row:vector.row);
    for (unsigned int i = 0; i < k; i++)
      result._[i] = _[i] / vector._[i];
    for (unsigned int i = k; i < row; i++)
      result._[i] = _[i];      
    return result; 
  }

  inline float operator * (const GmmVector &vector) const
  { 
    return this->Dot(vector);  
  }
  
  inline GmmVector operator + (float scalar) const
  {
    GmmVector result(row,false);
    return Add(scalar,result);    
  }
  
  inline GmmVector& Add(float scalar, GmmVector& result) const
  {
    result.Resize(row,false);
    for (unsigned int i = 0; i < row; i++)
      result._[i] = _[i] + scalar;
    return result;
  }

  inline GmmVector operator - (float scalar) const
  {
    GmmVector result(row,false);
    return Sub(scalar,result);    
  }
  
  inline GmmVector& Sub(float scalar, GmmVector& result) const
  {
    result.Resize(row,false);
    for (unsigned int i = 0; i < row; i++)
      result._[i] = _[i] - scalar;
    return result;
  }

  inline GmmVector operator * (float scalar) const
  {
    GmmVector result(row,false);
    return Mult(scalar,result);    
  }
  
  inline GmmVector& Mult(float scalar, GmmVector& result) const
  {
    result.Resize(row,false);
    for (unsigned int i = 0; i < row; i++)
      result._[i] = _[i] * scalar;
    return result;
  }

  inline GmmVector operator / (float scalar) const
  {
    GmmVector result(row,false);
    return Div(scalar,result);    
  }
  
  inline GmmVector& Div(float scalar, GmmVector& result) const
  {
    result.Resize(row,false);
    scalar = 1.0f/scalar;
    for (unsigned int i = 0; i < row; i++)
      result._[i] = _[i] * scalar;
    return result;
  }

  inline bool operator == (const GmmVector& vector) const
  {
    if(row!=vector.row) return false;
    for (unsigned int i = 0; i < row; i++)
      if(_[i] != vector._[i]) return false;
    return true;
  }

  inline bool operator != (const GmmVector& vector) const
  {
    return !(*this ==  vector);
  }


  inline float Sum() const 
  {
    float result = 0.0f;
    for (unsigned int i = 0; i < row; i++)
      result += _[i];
    return result;
  }

  inline float Norm() const 
  {
    return sqrtf(Norm2());
  }

  inline float Norm2() const 
  {
    float result = 0.0f;
    for (unsigned int i = 0; i < row; i++)
      result += _[i]*_[i];
    return result;
  }

	inline void Normalize()
	{
		float scalar = 1.0f / Norm();
    (*this)*=scalar;
	}
  
  inline float Distance(const GmmVector &vector) const
  {
    return (*this-vector).Norm();
  }
  
  inline float Distance2(const GmmVector &vector) const
  {
    return (*this-vector).Norm2();  
  }

  inline float Dot(const GmmVector &vector) const
  {
    const unsigned int k = (row<=vector.row?row:vector.row);
    float result = 0.0f;
    for (unsigned int i = 0; i < k; i++)
      result += _[i]*vector._[i];
    return result;     
  }

  inline GmmVector& SetSubVector(unsigned int startPos, const GmmVector &vector)
  {
    if(startPos<row){
      const unsigned int k = (row-startPos<=vector.row?row-startPos:vector.row);
      for (unsigned int i = 0; i < k; i++){
        _[startPos+i] = vector._[i];  
      }
    }
    return *this;   
  }

  inline GmmVector GetSubVector(unsigned int startPos, unsigned int len)
  {
    GmmVector result(len,false);
    return GetSubVector(startPos,len,result);
  }
  

  inline GmmVector& GetSubVector(unsigned int startPos, unsigned int len, GmmVector &result)
  {
    result.Resize(len,false);    
    if(startPos<row){
      const unsigned int k = (row-startPos<=len?row-startPos:len);
      for (unsigned int i = 0; i < k; i++){
        result[i] = _[startPos+i]; 
      }
      for (unsigned int i = k; i < len; i++){
        result[i] = 0.0f;
      }
      
    }else{
      result.Zero();  
    }
    return result;   
  }

  inline float Max(){
    if(row==0)
      return 0.0f;
      
    float res=_[0];
    for(unsigned int i=1;i<row;i++){
      if(_[i]>res) res = _[i];  
    }
    return res;
  }

  inline int MaxId(){
    if(row==0)
      return -1;
      
    float mx  = _[0];
    int   res = 0;
    for(unsigned int i=1;i<row;i++){
      if(_[i]>mx){ mx = _[i]; res = i;}  
    }
    return res;
  }

  inline GmmVector Abs(){
    GmmVector result(row);
    return Abs(result);
  }

  inline GmmVector& Abs(GmmVector &result) const{
    result.Resize(row,false);
    for(unsigned int i=0;i<row;i++){
      result._[i] = fabs(_[i]);
    }
    return result;
  }

  inline GmmVector& GetSubVector(const GmmVector &ids, GmmVector &result) const
  {
    const unsigned int k=ids.Size();
    result.Resize(k);
    for(unsigned int i=0;i<k;i++){
      const unsigned int g = (unsigned int)(fabs(ROUND(ids._[i])));
      if(g<row){
        result._[i] = _[g];
      }else{
        result._[i] = 0.0f;
      }
    }
    return result;     
  }


  void Print() const
  {
    std::cout << "GmmVector " <<row<<std::endl;;
    for (unsigned int i = 0; i < row; i++)
      std::cout << _[i] <<" ";
    std::cout << std::endl;
  }
  

protected:
    inline void Release(){
    if(_!=NULL) delete [] _; 
    row = 0;
    _   = NULL;
  }  
public:  
  inline virtual void Resize(unsigned int size, bool copy = true){
    if(row!=size){
      if(size){
        float *arr = new float[size];
        if(copy){
          unsigned int m = (row<size?row:size);
          for(unsigned int i=0; i<m; i++)
            arr[i] = _[i];
          for(unsigned int i=m; i<size; i++)
            arr[i] = 0.0f;
        }
        if(_!=NULL) delete [] _; 
        _   = arr;
        row = size;        
      }else{
        Release();
      }
    }
  }
};

#endif
