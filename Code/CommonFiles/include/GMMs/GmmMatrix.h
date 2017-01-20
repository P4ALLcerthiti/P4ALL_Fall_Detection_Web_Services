//Implementation of papaer On Learning, Representing and Generalizing a Task in a Humanoid Robot
//S. Calinon and F. Guenter and A. Billard

#ifndef MATRIX_H
#define MATRIX_H

#ifdef WIN32
#include "windows.h"
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "Macros.h"
#include "GmmVector.h"

#ifdef  USE_T_EXTENSIONS
template<unsigned int ROW> class TMatrix;
#endif
    
class GmmMatrix
{
  friend class GmmVector;
#ifdef  USE_T_EXTENSIONS
  template<unsigned int ROW> friend class TMatrix;
#endif
  
protected: 
  static int bInverseOk;
  
  unsigned int  row;
  unsigned int  column;
  float        *_;

public:

  inline GmmMatrix() {
    row    = 0;
    column = 0;
    _      = NULL;
  }
  
  inline virtual ~GmmMatrix(){
    Release(); 
  }

  inline GmmMatrix(const GmmMatrix &matrix)
  {
    row    = 0;
    column = 0;
    _      = NULL;
    Resize(matrix.row,matrix.column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] = matrix._[j*column+i];
  }

  inline GmmMatrix(unsigned int rowSize, unsigned int colSize, bool clear = true)
  {
    row    = 0;
    column = 0;
    _      = NULL;
    Resize(rowSize,colSize,false);
    if(clear)
      Zero();
  }
  
  inline GmmMatrix(const float _[], unsigned int rowSize, unsigned int colSize)
  {
    row       = 0;
    column    = 0;
    this->_   = NULL;
    Resize(rowSize,colSize,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        this->_[j*column+i] = _[j*column+i];
  }

#ifdef  USE_T_EXTENSIONS
  template<unsigned int ROW> inline GmmMatrix(const TMatrix<ROW> &matrix)
  {
    row    = 0;
    column = 0;
    _      = NULL;
    Resize(ROW,ROW,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] = matrix._[j*column+i];
  }
#endif
   
  inline GmmMatrix& Zero()
  {
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] = 0.0f;
    return *this;
  }
    
  inline unsigned int RowSize() const{
    return row;
  }
  inline unsigned int ColumnSize() const{
    return column;
  } 
  inline float *Array() const{
    return _;
  }

  inline float& operator() (const unsigned int row, const unsigned int col)
  {
    if((row<this->row)&&(col<this->column))
      return _[row*column+col];
    return GmmVector::undef; 
  }

  inline GmmVector GetRow(const unsigned int row) const
  {
    GmmVector result(column,false);    
    return GetRow(row,result);     
  }

  inline GmmVector& GetRow(const unsigned int row, GmmVector& result) const
  {
    result.Resize(column,false);
    for (unsigned int i = 0; i < column; i++)
      result._[i] = _[row*column+i];
    return result;     
  }

  inline GmmVector GetColumn(const unsigned int col) const
  {
    GmmVector result(row,false);    
    return GetColumn(col,result);     
  }

  inline GmmVector& GetColumn(const unsigned int col, GmmVector& result) const
  {
    result.Resize(row,false);
    if(col<column){
      for (unsigned int j = 0; j < row; j++)
        result._[j] = _[j*column+col];
    }else{
      result.Zero();
    }
    return result;     
  }

  inline GmmMatrix GetColumnSpace(const unsigned int col, const unsigned int len) const  
  {
    if(len>0){
      GmmMatrix result(row,len,false);    
      return GetColumnSpace(col,len,result);
    }else
      return GmmMatrix();     
  }

  inline GmmMatrix GetRowSpace(const unsigned int row, const unsigned int len) const
  {
    if(len>0){
      GmmMatrix result(len,column,false);    
      return GetRowSpace(row,len,result);
    }else
      return GmmMatrix();     
  }


  inline GmmMatrix& GetColumnSpace(const unsigned int col, const unsigned int len, GmmMatrix &result) const
  {
    if(len>0){
      const unsigned int end  = col+len-1;    
      const unsigned int size = len; 
      result.Resize(row,size,false);
      
      if(col<column){
        const unsigned int k = (end+1<=column?end+1:column);  
        
        for (unsigned int i = col; i < k; i++)
          for (unsigned int j = 0; j < row; j++)
            result._[j*size+(i-col)] = _[j*column+i];
        for (unsigned int i = k; i < end+1; i++)
          for (unsigned int j = 0; j < row; j++)
           result._[j*size+(i-col)] = 0.0f;            
      }else{
        result.Zero();
      }
    }else{
      result.Resize(0,0,false);
    }
    return result;     
  }

  inline GmmMatrix& GetRowSpace(const unsigned int row, const unsigned int len, GmmMatrix &result) const
  {      
    if(len>0){
      const unsigned int end  = row+len-1;
      const unsigned int size = len; 
      result.Resize(size,column,false);
      
      if(row<this->row){
        const unsigned int k = (end+1<=this->row?end+1:this->row);  
        
        for (unsigned int j = 0; j < column; j++)
          for (unsigned int i = row; i < k; i++)
            result._[(i-row)*column+j] = _[i*column+j];
        for (unsigned int j = 0; j < column; j++)
          for (unsigned int i = k; i < end+1; i++)
           result._[(i-row)*column+j] = 0.0f;            
      }else{
        result.Zero();
      }
    }else{
      result.Resize(0,0,false);
    }
    return result;     
  }

  inline GmmMatrix& SetRow(const GmmVector &vector, const unsigned int row)
  {
    if(row<this->row){    
      const unsigned int ki = (column<=vector.row?column:vector.row);
      for (unsigned int i = 0; i < ki; i++)
        _[row*column+i] = vector._[i]; 
    }
    return *this;     
  }

  inline GmmMatrix& SetColumn(const GmmVector &vector, const unsigned int col)
  {
    if(col<this->column){    
      const unsigned int kj = (row<=vector.row?row:vector.row);
      for (unsigned int j = 0; j < kj; j++)
        _[j*column+col] = vector._[j];
    }
    return *this;
  }

  inline GmmMatrix& SetColumnSpace(const GmmMatrix &matrix, const unsigned int col)
  {
    if(col<this->column){    
      const unsigned int kj = (row<=matrix.row?row:matrix.row);
      const unsigned int ki = (col+matrix.column<=this->column?col+matrix.column:this->column);
      for (unsigned int j = 0; j < kj; j++)
        for (unsigned int i = col; i < ki; i++)
          _[j*column+i] = matrix._[j*matrix.column+(i-col)];
    }
    return *this;
  }

  inline GmmMatrix& SetRowSpace(const GmmMatrix &matrix, const unsigned int row)
  {
    if(row<this->row){
      const unsigned int ki = (column<=matrix.column?column:matrix.column);
      const unsigned int kj = (row+matrix.row<=this->row?row+matrix.row:this->row);
      for (unsigned int j = row; j < kj; j++)
        for (unsigned int i = 0; i < ki; i++)
          _[j*column+i] = matrix._[(j-row)*matrix.column+i]; 
    }
    return *this;     
  }

  inline GmmMatrix GetRowSpace(const GmmVector &ids) const
  {
    GmmMatrix result(ids.Size(),column);
    return GetRowSpace(ids,result);
  }

  inline GmmMatrix GetColumnSpace(const GmmVector &ids) const
  {
    GmmMatrix result(row,ids.Size());
    return GetColumnSpace(ids,result);
  }

  inline GmmMatrix GetMatrixSpace(const GmmVector &rowIds,const GmmVector &colIds) const
  {
    GmmMatrix result(rowIds.Size(),colIds.Size());
    return GetMatrixSpace(rowIds,colIds,result);
  }

  inline GmmMatrix& GetColumnSpace(const GmmVector &ids, GmmMatrix &result) const
  {
    const unsigned int k=ids.Size();
    result.Resize(row,k);
    for(unsigned int i=0;i<k;i++){
      const unsigned int g = (unsigned int)(fabs(ROUND(ids._[i])));
      if(g<column){
        for(unsigned int j=0;j<row;j++)
          result._[j*k+i] = _[j*column+g];
      }else{
        for(unsigned int j=0;j<row;j++)
          result._[j*k+i] = 0.0f;        
      }
    }
    return result;     
  }

  inline GmmMatrix& GetRowSpace(const GmmVector &ids, GmmMatrix &result) const
  {
    const unsigned int k=ids.Size();
    result.Resize(k,column);
    for(unsigned int i=0;i<k;i++){
      const unsigned int g = (unsigned int)(fabs(ROUND(ids._[i])));
      if(g<row){
        for(unsigned int j=0;j<column;j++)
          result._[i*column+j] = _[g*column+j];
      }else{
        for(unsigned int j=0;j<column;j++)
          result._[i*column+j] = 0.0f;
      }
    }
    return result;     
  }

  inline GmmMatrix& GetMatrixSpace(const GmmVector &rowIds,const GmmVector &colIds, GmmMatrix &result) const
  {
    const unsigned int k1=rowIds.Size();
    const unsigned int k2=colIds.Size();
    result.Resize(k1,k2);
    for(unsigned int i=0;i<k1;i++){
      const unsigned int g1 = (unsigned int)(fabs(ROUND(rowIds._[i])));
      if(g1<row){
        for(unsigned int j=0;j<k2;j++){      
          const unsigned int g2 = (unsigned int)(fabs(ROUND(colIds._[j])));
          if(g2<column){
            result._[i*k2+j] = _[g1*column+g2];            
          }else{
            result._[i*k2+j] = 0.0f;
          }
        }
      }else{
        for(unsigned int j=0;j<k2;j++)
          result._[i*k2+j] = 0.0f;
      }
    }
    return result;     
  }

  inline GmmMatrix operator - () const
  {
    GmmMatrix result(row,column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = -_[j*column+i];
    return result;
  }
  
  inline virtual GmmMatrix& operator = (const GmmMatrix &matrix)
  {
    Resize(matrix.row,matrix.column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] = matrix._[j*column+i];
    return *this;    
  }

  inline GmmMatrix& operator += (const GmmMatrix &matrix)
  {
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++)
      for (unsigned int i = 0; i < ki; i++)
        _[j*column+i] += matrix._[j*column+i];
    return *this;
  }

  inline GmmMatrix& operator -= (const GmmMatrix &matrix)
  {
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++)
      for (unsigned int i = 0; i < ki; i++)
        _[j*column+i] -= matrix._[j*column+i];
    return *this;
  }

  inline GmmMatrix& operator ^= (const GmmMatrix &matrix)
  {
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++)
      for (unsigned int i = 0; i < ki; i++)
        _[j*column+i] *= matrix._[j*column+i];
    return *this;
  }

  inline GmmMatrix& operator /= (const GmmMatrix &matrix)
  {
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++)
      for (unsigned int i = 0; i < ki; i++)
        _[j*column+i] /= matrix._[j*column+i];
    return *this;
  }

  inline GmmMatrix& operator += (float scalar)
  {
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] += scalar;
    return *this;
  }

  inline GmmMatrix& operator -= (float scalar)
  {
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] -= scalar;
    return *this;
  }

  inline GmmMatrix& operator *= (float scalar)
  {
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] *= scalar;
    return *this;
  }

  inline GmmMatrix& operator /= (float scalar)
  {
    scalar = 1.0f/scalar;
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] *= scalar;
    return *this;
  }
  
  inline GmmMatrix operator + (const GmmMatrix &matrix) const
  {
    GmmMatrix result(row,column,false);  
    return Add(matrix,result);
  }
  
  inline GmmMatrix& Add(const GmmMatrix &matrix, GmmMatrix &result) const
  {   
    result.Resize(row,column,false);
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++){
      for (unsigned int i = 0; i < ki; i++)
        result._[j*column+i] = _[j*column+i] + matrix._[j*column+i];
      for (unsigned int i = ki; i < column; i++)
        result._[j*column+i] = _[j*column+i];  
    }
    for (unsigned int j = kj; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i];  
    return result;
  }
  
  inline GmmMatrix operator - (const GmmMatrix &matrix) const
  {
    GmmMatrix result(row,column,false);  
    return Sub(matrix,result);
  }
  
  inline GmmMatrix& Sub(const GmmMatrix &matrix, GmmMatrix &result) const
  {   
    result.Resize(row,column,false);
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++){
      for (unsigned int i = 0; i < ki; i++)
        result._[j*column+i] = _[j*column+i] - matrix._[j*column+i];
      for (unsigned int i = ki; i < column; i++)
        result._[j*column+i] = _[j*column+i];  
    }
    for (unsigned int j = kj; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i];  
    return result;
  }
  
  inline GmmMatrix operator ^ (const GmmMatrix &matrix) const
  {
    GmmMatrix result(row,column,false);  
    return PMult(matrix,result);
  }
  
  inline GmmMatrix& PMult(const GmmMatrix &matrix, GmmMatrix &result) const
  {   
    result.Resize(row,column,false);
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++){
      for (unsigned int i = 0; i < ki; i++)
        result._[j*column+i] = _[j*column+i] * matrix._[j*column+i];
      for (unsigned int i = ki; i < column; i++)
        result._[j*column+i] = _[j*column+i];  
    }
    for (unsigned int j = kj; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i];  
    return result;
  }
  
  inline GmmMatrix operator / (const GmmMatrix &matrix) const
  {
    GmmMatrix result(row,column,false);  
    return PDiv(matrix,result);
  }
  
  inline GmmMatrix& PDiv(const GmmMatrix &matrix, GmmMatrix &result) const
  {   
    result.Resize(row,column,false);
    const unsigned int kj = (row<=matrix.row?row:matrix.row);
    const unsigned int ki = (column<=matrix.column?column:matrix.column);
    for (unsigned int j = 0; j < kj; j++){
      for (unsigned int i = 0; i < ki; i++)
        result._[j*column+i] = _[j*column+i] / matrix._[j*column+i];
      for (unsigned int i = ki; i < column; i++)
        result._[j*column+i] = _[j*column+i];  
    }
    for (unsigned int j = kj; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i];  
    return result;
  }

  inline GmmMatrix operator + (float scalar) const
  {
    GmmMatrix result(row,column,false);  
    return Add(scalar,result);    
  }

  inline GmmMatrix& Add(float scalar, GmmMatrix& result) const
  {
    result.Resize(row,column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i] + scalar;    
    return result;
  }

  inline GmmMatrix operator - (float scalar) const
  {
    GmmMatrix result(row,column,false);  
    return Sub(scalar,result);    
  }
  
  inline GmmMatrix& Sub(float scalar, GmmMatrix& result) const
  {
    result.Resize(row,column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i] - scalar;    
    return result;
  }

  inline GmmMatrix operator * (float scalar) const
  {
    GmmMatrix result(row,column,false);  
    return Mult(scalar,result);    
  }

  inline GmmMatrix& Mult(float scalar, GmmMatrix& result) const
  {
    result.Resize(row,column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i] * scalar;    
    return result;
  }


  inline GmmMatrix operator / (float scalar) const
  {
    GmmMatrix result(row,column,false);  
    return Div(scalar,result);    
  }

  inline GmmMatrix& Div(float scalar, GmmMatrix& result) const
  {
    result.Resize(row,column,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*column+i] = _[j*column+i] / scalar;    
    return result;    
  }

  inline bool operator == (const GmmMatrix& matrix) const
  {
    if((row!=matrix.row)||(column!=matrix.column)) return false;
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        if(_[j*column+i] != matrix._[j*column+i]) return false;
    return true;
  }

  inline bool operator != (const GmmMatrix& matrix) const
  {
    return !(*this ==  matrix);
  }

  inline GmmVector operator * (const GmmVector &vector) const
  {
    GmmVector result(row,false);  
    return Mult(vector,result);    
  }  

  inline GmmVector Mult(const GmmVector &vector) const
  {
    GmmVector result(row,false);  
    return Mult(vector,result);    
  }

  inline GmmVector& Mult(const GmmVector &vector, GmmVector &result) const
  {
    result.Resize(row,false);
    const unsigned int ki = (column<=vector.row?column:vector.row);
    for (unsigned int j = 0; j < row; j++){
      result._[j] = 0.0f;
      for (unsigned int i = 0; i < ki; i++)
        result._[j] += _[j*column+i] * vector._[i];
    }
    return result;
  }


  inline GmmMatrix operator * (const GmmMatrix &matrix) const  
  {
    GmmMatrix result(row,matrix.column,false);  
    return Mult(matrix,result);
  }  

  inline GmmMatrix& Mult(const GmmMatrix &matrix, GmmMatrix &result) const
  {
    result.Resize(row,matrix.column,false);
    const unsigned int rrow = result.row;
    const unsigned int rcol = result.column;
    const unsigned int kk = (column<=matrix.row?column:matrix.row);
    for (unsigned int j = 0; j < rrow; j++){
      for (unsigned int i = 0; i < rcol; i++){
        result._[j*rcol+i] = 0.0f;
        for(unsigned int k = 0; k< kk; k++)    
          result._[j*rcol+i] += _[j*column+k] * matrix._[k*rcol+i];
      }
    }    
    return result;
  }



  inline GmmMatrix& Identity()
  {
    const unsigned int k = (row>column?column:row);
    Zero();
    for (unsigned int i = 0; i < k; i++)
      _[i*column+i] = 1.0f;
    return *this;    
  }

  inline GmmMatrix& Diag(const GmmVector &vector)
  {
    const unsigned int k = (row>column?column:row);
    const unsigned int k2 = (k>vector.row?vector.row:k);
    Zero();
    for (unsigned int i = 0; i < k2; i++)
      _[i*column+i] = vector._[i];
    return *this;    
  }

  inline GmmMatrix& Random(){
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        _[j*column+i] =((float)rand())/((float)(RAND_MAX+1.0));    
    return *this;    
  }

	inline GmmMatrix Transpose() const
	{
    GmmMatrix result(row,column,false);
    return Transpose(result);    
	}

  inline GmmMatrix& Transpose(GmmMatrix &result) const
  {    
    result.Resize(column,row,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[i*row+j] = _[j*column+i];
    return result;    
  }

  inline GmmMatrix VCat(const GmmMatrix& matrix)
  {
    GmmMatrix result;
    return VCat(matrix,result);    
  }
  
  inline GmmMatrix& VCat(const GmmMatrix& matrix, GmmMatrix & result)
  {
    unsigned int k1 = (column>matrix.column?column:matrix.column);
    result.Resize(row+matrix.row,k1,false);
    for (unsigned int j = 0; j < row; j++){
      for (unsigned int i = 0; i < column; i++)
        result._[j*k1+i] = _[j*column+i];
      for (unsigned int i = column; i < k1; i++)
        result._[j*k1+i] = 0.0f;
    }
    for (unsigned int j = 0; j < matrix.row; j++){
      for (unsigned int i = 0; i < matrix.column; i++)
        result._[(row+j)*k1+i] = matrix._[j*matrix.column+i];
      for (unsigned int i = matrix.column; i < k1; i++)
        result._[(row+j)*k1+i] = 0.0f;
    }
    return result;
  }

  inline GmmMatrix HCat(const GmmMatrix& matrix)
  {
    GmmMatrix result;
    return HCat(matrix,result);    
  }
  
  inline GmmMatrix& HCat(const GmmMatrix& matrix, GmmMatrix & result)
  {
    unsigned int k1 = (row>matrix.row?row:matrix.row);
    unsigned int k2 = column+matrix.column;
    result.Resize(k1,k2,false);
    for (unsigned int j = 0; j < row; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*k2+i] = _[j*column+i];
    for (unsigned int j = row; j < k1; j++)
      for (unsigned int i = 0; i < column; i++)
        result._[j*k2+i] = 0.0f;

    for (unsigned int j = 0; j < matrix.row; j++)
      for (unsigned int i = 0; i < matrix.column; i++)
        result._[j*k2+i+column] = matrix._[j*matrix.column+i];
    for (unsigned int j = matrix.row; j < k1; j++)
      for (unsigned int i = 0; i < matrix.column; i++)
        result._[j*k2+i+column] = 0.0f;
    
    return result;
  }

  inline static int IsInverseOk(){
    return bInverseOk;
  }

  inline GmmMatrix Inverse(float *determinant=NULL) const
  {
    GmmMatrix result;
    return Inverse(result,determinant);
  }  

  inline GmmMatrix& Inverse(GmmMatrix &result, float *determinant=NULL) const
  {
    bInverseOk = true;       
    if(row==column){ // Square matrix
      if(determinant!=NULL) *determinant = 1.0f;
      result.Resize(row,column,false);
      const unsigned int n = row;
      GmmMatrix MM(*this);
      result.Identity();
      for(unsigned int i=0;i<n;i++){
        float pivot = MM._[i*column+i]; 
        if(fabs(pivot)<=EPSILON){
          for(unsigned int j=i+1;j<n;j++){
            if((pivot = MM._[j*column+i])!=0.0f){
              MM.SwapRow(i,j);
              result.SwapRow(i,j);
              break;  
            }
          }            
          if(fabs(pivot)<=EPSILON){
            bInverseOk = false;
            if(determinant!=NULL) *determinant = 0.0f;
            return result;
          }                      
        }
        if(determinant!=NULL) *determinant *= pivot;
        pivot = 1.0f/pivot;
        for(unsigned int j=0;j<n;j++){
          MM._[i*column+j]   *= pivot;
          result._[i*column+j] *= pivot;
        }
        for(unsigned int k=0;k<n;k++){
          if(k!=i){
            const float mki = MM._[k*column+i];
            for(unsigned int j=0;j<n;j++){
               MM._[k*column+j]   -= MM._[i*column+j]   *mki;
               result._[k*column+j] -= result._[i*column+j] *mki;              
            }            
          }
        }
      }        
    }else{ // Moore-Penrose pseudo inverse
      if(determinant!=NULL) *determinant = 0.0f;
      if(row>column){ // (JtJ)^(-1)Jt
        GmmMatrix MT,SQ,SQInv;
        Transpose(MT);
        MT.Mult(*this,SQ);
        SQ.Inverse(SQInv);
        SQInv.Mult(MT,result); 
      }else{ // Jt(JJt)^(-1)
        GmmMatrix MT,SQ,SQInv;
        Transpose(MT);
        Mult(MT,SQ);
        SQ.Inverse(SQInv);
        MT.Mult(SQInv,result);         
      }
    }
    return result;    
  }

  inline GmmMatrix& SwapRow(unsigned int j1, unsigned int j2){
    if((j1<row)&&(j2<row)){
      float tmp;
      for (unsigned int i = 0; i < column; i++){
        tmp            = _[j1*column+i];
        _[j1*column+i] = _[j2*column+i];
        _[j2*column+i] = tmp;        
      }        
    }
    return *this; 
  }
 
  inline GmmMatrix& SwapColumn(unsigned int i1, unsigned int i2){
    if((i1<column)&&(i2<column)){
      float tmp;
      for (unsigned int j = 0; j < row; j++){
        tmp            = _[j*column+i1];
        _[j*column+i1] = _[j*column+i2];
        _[j*column+i2] = tmp;        
      }        
    }
    return *this; 
  }
  
  void Print() const
  {
    std::cout << "GmmMatrix " <<row<<"x"<<column<<std::endl;;
    for (unsigned int j = 0; j < row; j++){
      for (unsigned int i = 0; i < column; i++)
        std::cout << _[j*column+i] <<" ";
      std::cout << std::endl;
    }
  }
  
  void QRDecomposition(GmmMatrix & Q, GmmMatrix & R){
    GmmMatrix QR;
    QRDecomposition(Q,R,QR);
  }
  
  void QRDecomposition(GmmMatrix & Q, GmmMatrix & R, GmmMatrix & QR){    
    if(row>=column){
      QR = *this;
    }else{
      Transpose(QR);
    }
    unsigned int m = QR.row;
    unsigned int n = QR.column;
    GmmVector RDiag(n);
        
    for(unsigned int k=0;k<n;k++){
      float nrm = 0.0f;
      for (unsigned int i = k; i < m; i++) {
        nrm = hypot_s(nrm, QR(i,k));
      }
      if (nrm != 0.0f) {
        if(QR(k,k)<0.0f){
          nrm = -nrm;
        }
        for (unsigned int i = k; i < m; i++) {
            QR(i,k) /= nrm;
        }
        QR(k,k)+=1.0f;

        for (unsigned int j = k + 1; j < n; j++) {
          float s = 0.0f;
          for (unsigned int i = k; i < m; i++) {
              s += QR(i,k) * QR(i,j);
          }
          s = -s / QR(k,k);
          for (unsigned int i = k; i < m; i++) {
              QR(i,j) += s * QR(i,k);
          }
        }
      }
      RDiag(k) = -nrm;
    }
    
    R.Resize(n,n);
    for(unsigned int i = 0; i < n; i++) {
      for(unsigned int j = 0; j < n; j++) {
        if(i<j){
          R(i,j) = QR(i,j); 
        }else if(i==j){
          R(i,j) = RDiag(i);
        }else{
          R(i,j) = 0.0f;
        }
      }
    }

    Q.Resize(m,n);
    for(int k= n-1;k>=0;k--){
      for(unsigned int i = 0; i < m; i++) {
        Q(i,k) = 0.0f;
      }
      Q(k,k)=1.0f;
      for(unsigned int j = k; j < n; j++) {
        if(QR(k,k)!=0.0f){
          float s = 0.0f;
          for(unsigned int i = k; i < m; i++) {
            s += QR(i,k) * Q(i,j);
          }
          s = -s / QR(k,k);
          for(unsigned int i = k; i < m; i++) {
            Q(i,j) = Q(i,j) + s*QR(i,k);
          }
        }
      }       
    }
  }

  
  GmmMatrix& Tridiagonalize(GmmMatrix &result,GmmMatrix &trans){
    result.Resize(2,row);
    GmmMatrix A(*this);
    trans = A;
    if(row==0) return result;
    
    
    int n = row;
    int l,k,j,i;
    float scale,hh,h,g,f;
    for(i=n-1;i>=1;i--){
      l = i-1;
      h = scale = 0.0f;
      if(l>0){
        for(k=0;k<=l;k++)
          scale += fabs(A._[i*column+k]);
        if(scale == 0.0f){
          result._[column+i] = A._[i*column+l];
        }else{
          for(k=0;k<=l;k++){
            A._[i*column+k] /= scale;
            h += A._[i*column+k]*A._[i*column+k];
          }
          f= A._[i*column+l];
          g=(f>=0.0f?-sqrt(h):sqrt(h));
          result._[column+i] = scale*g;
          h-=f*g;
          A._[i*column+l] = f-g;
          f=0.0f;
          for(j=0;j<=l;j++){
            A._[j*column+i] = A._[i*column+j] /h;
            g=0.0f;
            for(k=0;k<=j;k++)
              g+=  A._[j*column+k]*A._[i*column+k];
            for(k=j+1;k<=l;k++)
              g+=  A._[k*column+j]*A._[i*column+k];
            result._[column+j] = g/h;
            f+= result._[column+j]*A._[i*column+j];
          }
          hh = f/(h+h);
          for(j=0;j<=l;j++){
            f = A._[i*column+j];
            result._[column+j]=g=result._[column+j]-hh*f;
            for(k=0;k<=j;k++)
              A._[j*column+k] -= (f*result._[column+k]+g*A._[i*column+k]);            
          }             
        }
      }else{
        result._[column+i] = A._[i*column+l];        
      }
      result._[i]=h;  
    }
    result._[0]=0.0f;  
    result._[column+0]=0.0f;
    for(i=0;i<n;i++){
      l=i-1;
      if(result._[i]){
        for(j=0;j<=l;j++){
          g=0.0f;
          for(k=0;k<=l;k++)
            g+= A._[i*column+k]*A._[k*column+j]; 
          for(k=0;k<=l;k++)
            A._[k*column+j] -= g*A._[k*column+i]; 
        }  
      }
      result._[i] = A._[i*column+i];
      A._[i*column+i] = 1.0f;
      for(j=0;j<=l;j++) A._[j*column+i]=A._[i*column+j]=0.0f;
    }
    trans = A;
    return result;
  }
    
  GmmMatrix& TriDiag(GmmMatrix &tri){
    Resize(tri.ColumnSize(),tri.ColumnSize(),false);
    Zero();
    for(unsigned int i=0;i<column;i++){
      _[i*(column+1)] = tri._[i];
      if(i<column-1)
        _[i*(column+1)+1] = tri._[column+i+1];
      if(i>0)  
        _[i*(column+1)-1] = tri._[column+i];
    }
    return *this;
  }
  
  int TriEigen(GmmVector &eigenValues, GmmMatrix& eigenVectors,int maxIter = 30){
    if(row!=2) return -1;
    if(column==0) return -1;
    GetRow(0,eigenValues);
    GmmVector e;
    GetRow(1,e);
    
    const int n = column;
    int m,l,iter,i,k;
    float s,r,p,g,f,dd,c,b;
    
    for(i=1;i<n;i++) e._[i-1] = e._[i];
    e._[n-1] = 0.0f;

    for(l=0;l<n;l++){
      iter=0;
      do{
        for(m=l;m<=n-2;m++){
          dd = fabs(eigenValues._[m])+fabs(eigenValues._[m+1]);
          if((float)(fabs(e._[m])+dd) == dd) break;  
        }
        if(m!=l){
          if(iter++==maxIter) {
            //printf("Error: too many ierations...\n");
            break;
          }
          g=(eigenValues._[l+1]-eigenValues._[l])/(2.0f*e[l]);
          r=hypot_s(g,1.0f);
          g=eigenValues._[m]-eigenValues._[l]+e._[l]/(g+SIGN2(r,g));
          s=c=1.0f;
          p=0.0f;
          for(i=m-1;i>=l;i--){
            f=s*e._[i];
            b=c*e._[i];
            e._[i+1] =(r=hypot_s(f,g));
            if(r==0.0f){
              eigenValues._[i+1]-=p;
              e._[m] = 0.0f;
              break;  
            }
            s=f/r;
            c=g/r;
            g=eigenValues._[i+1]-p;
            r=(eigenValues._[i]-g)*s+2.0f*c*b;
            eigenValues._[i+1]=g+(p=s*r);
            g=c*r-b;
            for(k=0;k<n;k++){
              f=eigenVectors._[k*n+i+1];
              eigenVectors._[k*n+i+1]=s*eigenVectors._[k*n+i]+c*f;
              eigenVectors._[k*n+i]=c*eigenVectors._[k*n+i]-s*f;                
            }            
          }
          if((r==0.0f)&&(i>=0)) continue;
          eigenValues._[l]-=p;
          e._[l] = g;
          e._[m] = 0.0f;
        }        
      }while(m!=l); 
    } 
       
    return iter;
  }

  GmmMatrix& SortColumnAbs(GmmVector & values){
    const int k = (values.Size()<column?values.Size():column);
    float cmax;
    int maxId;
    for(int i=0;i<k-1;i++){
      cmax  = fabs(values._[i]);
      maxId = i;
      for(int j=i+1;j<k;j++){
        if(cmax<fabs(values._[j])){
          cmax = fabs(values._[j]);
          maxId = j;  
        }            
      }
      if(maxId!=i){
        float tmp       = values._[i];
        values._[i]     = values._[maxId];
        values._[maxId] = tmp;
        SwapColumn(i,maxId);
      }     
    }  
    return *this;
  }

  GmmMatrix& GramSchmidt(GmmVector &base){
    GmmMatrix unit(row,1);
    unit.SetColumn(base,0);
    GmmMatrix ext;
    unit.HCat(*this,ext);
    ext.GramSchmidt();
    (*this) = ext;
    return *this;  
  }

  
  GmmMatrix& GramSchmidt(){
    GmmVector res(row),tmp(row),tmp2(row),tmp3(row);
    for(unsigned int i=0;i<column;i++){
      GetColumn(i,tmp);
      res = tmp;        
      for(unsigned int j=0;j<i;j++){
        GetColumn(j,tmp2);
        res-=tmp2.Mult((tmp2.Dot(tmp)),tmp3);          
      }
      float norm = res.Norm();
      if(norm>EPSILON){
        res /= norm;
      }else{
        res.Zero();
      }
      SetColumn(res,i);  
    }  
    return *this;    
  }

  GmmMatrix& RemoveZeroColumns(){
    int zeroCnt = 0;
    int colCnt  = 0;
    while(colCnt < int(column)-zeroCnt){

      bool bIsZero = true;
      for(unsigned int j=0;j<row;j++){
        if(fabs(_[j*column+colCnt])>EPSILON){
          bIsZero = false;
          break;
        }
      }
      if(bIsZero){
        if(colCnt<int(column)-1-zeroCnt){
          SwapColumn(colCnt,int(column)-1-zeroCnt);
        }
        zeroCnt++;
      }else{
        colCnt++;
      }              
    }
    Resize(row,column-zeroCnt,true);
    return *this;       
  }

  GmmMatrix& ClearColumn(unsigned int col){
    if(col<column){
      for(unsigned int i=0;i<row;i++){
        _[i*column+col] = 0.0f;
      }      
    }  
    return *this;
  }

  GmmVector SumRow(){
    GmmVector result(column);
    return SumRow(result);
  }

  GmmVector SumColumn(){
    GmmVector result(row);
    return SumColumn(result);
  }
  
  GmmVector & SumRow(GmmVector & result){
    result.Resize(column,false);
    result.Zero();
    for(unsigned int i=0;i<column;i++){
      for(unsigned int j=0;j<row;j++){
        result._[i] += _[j*column+i];
      }      
    }
    return result;  
  }

  GmmVector & SumColumn(GmmVector & result){
    result.Resize(row,false);
    result.Zero();
    for(unsigned int j=0;j<row;j++){
      for(unsigned int i=0;i<column;i++){
        result._[j] += _[j*column+i];
      }      
    }
    return result;  
  }
  
protected:

  inline void Release(){
    if(_!=NULL) delete [] _; 
    row    = 0;
    column = 0;
    _      = NULL;
  }  
public:  
  inline virtual void Resize(unsigned int rowSize, unsigned int colSize, bool copy = true){
    if((row!=rowSize)||(column!=colSize)){
      if((rowSize)&&(colSize)){
        float *arr = new float[rowSize*colSize];
        if(copy){
          unsigned int mj = (row<rowSize?row:rowSize);
          unsigned int mi = (column<colSize?column:colSize);
          
          for (unsigned int j = 0; j < mj; j++){
            for (unsigned int i = 0; i < mi; i++)
              arr[j*colSize+i] = _[j*column+i];
            for (unsigned int i = mi; i < colSize; i++)
              arr[j*colSize+i] = 0.0f;
          }
          for (unsigned int j = mj; j < rowSize; j++){
            for (unsigned int i = 0; i < colSize; i++)
              arr[j*colSize+i] = 0.0f;            
          }
        }
        if(_!=NULL) delete [] _; 
        _      = arr;
        row    = rowSize;
        column = colSize;        
      }else{
        Release();
      }
    }
  }
};


#endif
