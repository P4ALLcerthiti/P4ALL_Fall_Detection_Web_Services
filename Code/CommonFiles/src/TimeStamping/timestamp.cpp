#include "stdafx.h"

#include <sstream>
#include <iostream>
#include <iomanip>
#include <cmath>
//#include <math.h>
//#include <sys/time.h>
#include <time.h>
#include <assert.h>
#include "timestamp.hpp"

#define NANO 1000000000.0
//
// constructors
//
Timestamp::Timestamp() :
	_nanoseconds ( 0 ) {}
Timestamp::Timestamp(unsigned int s, unsigned int ns) :
	_nanoseconds ( ns+(long long)(1000000000)*s ) {}
Timestamp::Timestamp(const Timestamp& t) :
	_nanoseconds ( t._nanoseconds ) {}
//Timestamp::Timestamp(timespec ts) :
//	_nanoseconds ( ts.tv_nsec+(long long)(1000000000)*ts.tv_sec ) {}
Timestamp::Timestamp(timeval ts) :
	_nanoseconds ( (long long)(1000)*ts.tv_usec+(long long)(1000000000)*ts.tv_sec ) {}
Timestamp::Timestamp(double seconds) {
    if (seconds > 2000000000)
        throw TimestampException("Timestamp(double seconds) with seconds > 2000000000");
    if (seconds < -2000000000)
        throw TimestampException("Timestamp(double seconds) with seconds < -2000000000");
	//_nanoseconds = (long long)round(1000000000*seconds);	
	_nanoseconds = (long long)(1000000000*seconds+0.5);	
	
}
Timestamp::Timestamp( const std::string& s ) {
    std::istringstream ss( s );
    long seconds;
    unsigned long decimals;

    // Get seconds
    ss >> seconds;

    // Ignore point
    ss.ignore();

    // Get a maxim precision of nanoseconds
    char cDecimals[10];
    ss.get( cDecimals, 10 );
    decimals = atoi(cDecimals);

    // Get number of decimals,
    int numDecimals = ss.gcount();

    // Obtain decimal base to be aplied to obtain nanoseconds
    unsigned int nanoBase = 1;
    for (int i = 9; i > numDecimals; i--) {
        nanoBase = nanoBase*10;
    }

    // Create the timestamp
    _nanoseconds = (long long)(1000000000)*seconds + decimals*nanoBase;
}

Timestamp TimestampFromNanoseconds( long long ns )
{
    Timestamp tmp;
    tmp._nanoseconds = ns;
    return tmp;
}

//
// conversiont to other types
//
//Timestamp::operator const timespec() const
//{
//   if (!positive())
//        throw TimestampException("Timestamp: cannot convert negative time to timespec");
//    timespec temp;
//   temp.tv_sec  = _nanoseconds/1000000000;
//    temp.tv_nsec = _nanoseconds - (long long)(1000000000)*temp.tv_sec;
//    return temp;
//}
Timestamp::operator const timeval() const
{
    if (!positive())
        throw TimestampException("Timestamp: cannot convert negative time to timeval");
    timeval temp;
    temp.tv_sec  = _nanoseconds/1000000000;
    temp.tv_usec = _nanoseconds - (long long)(1000000000)*temp.tv_sec;
    temp.tv_usec /= 1000;
    return temp;
}
Timestamp::operator const std::string() const
{
    std::ostringstream ss;
    ss << secondsTotal()
       << "." << std::setfill('0')
       << std::setw(6)
       << timeMicroSeconds();
    return ss.str();
}

double Timestamp::toDouble() const
{
    return 1e-9 * _nanoseconds;
}

//
// aritmetics
//
bool Timestamp::positive() const
{
	return ( _nanoseconds >= 0 );
}
Timestamp Timestamp::operator+=(const Timestamp& t)
{
    _nanoseconds += t._nanoseconds;
    return *this;
}
Timestamp Timestamp::operator-=(const Timestamp& t)
{
    _nanoseconds -= t._nanoseconds;
     return *this;
}

Timestamp Timestamp::operator+(const Timestamp& t) const
{
    return TimestampFromNanoseconds( _nanoseconds + t._nanoseconds );
}

Timestamp Timestamp::operator-(const Timestamp& t) const
{
    return TimestampFromNanoseconds( _nanoseconds - t._nanoseconds );
}

Timestamp Timestamp::operator*(const double factor) const
{
    return TimestampFromNanoseconds( (long long)(_nanoseconds * factor) );
}

Timestamp Timestamp::operator-() const
{
    return TimestampFromNanoseconds( -_nanoseconds );
}

Timestamp abs(const Timestamp& t)
{
    Timestamp temp(t);
    if (temp > 0)
        return temp;
    else
        return -temp;
}

const bool Timestamp::isSimilar(const Timestamp& t, double epsilon) const
{
    return abs(*this-t) <= epsilon;
}

bool Timestamp::operator==(const Timestamp& t) const {
    return(_nanoseconds == t._nanoseconds);
}

bool Timestamp::operator>(const Timestamp& t) const {
    return _nanoseconds > t._nanoseconds ;
}

bool Timestamp::operator>=(const Timestamp& t) const {
    return _nanoseconds >= t._nanoseconds ;
}

bool Timestamp::operator<(const Timestamp& t) const {
    return _nanoseconds < t._nanoseconds ;}

bool Timestamp::operator<=(const Timestamp& t) const {
    return _nanoseconds <= t._nanoseconds ;
}

bool Timestamp::operator!=(const Timestamp& t) const {
    return _nanoseconds != t._nanoseconds ;
}

long long Timestamp::nanoSecondsTotal() const {
    return _nanoseconds;
}

long long  Timestamp::microSecondsTotal() const {
    return (long long) (_nanoseconds/1000);
}

long long  Timestamp::milliSecondsTotal() const {
    return (long long) (_nanoseconds/1000000);
}

long Timestamp::secondsTotal() const {
    return long(_nanoseconds/1000000000);
}

long Timestamp::minutesTotal() const {
    return (long)(_nanoseconds/60000000000.0);
}

int Timestamp::timeMilliSeconds() const {
    return milliSecondsTotal()-secondsTotal()*1000;
}

int Timestamp::timeMicroSeconds() const {
    return microSecondsTotal()-secondsTotal()*1000000;
}

int Timestamp::timeNanoSeconds() const {
    return nanoSecondsTotal()-secondsTotal()*1000000000;
}

int Timestamp::timeSeconds() const {
    time_t s = secondsTotal();
    const tm* t = localtime(&s);
    return t->tm_sec;
}

int Timestamp::timeMinutes() const {
    time_t s = secondsTotal();
    const tm* t = localtime(&s);
    return t->tm_min;
}

int Timestamp::timeHours() const {
    time_t s = secondsTotal();
    const tm* t = localtime(&s);
    return t->tm_hour;
}

int Timestamp::timeDay() const {
    time_t s = secondsTotal();
    const tm* t = localtime(&s);
    return t->tm_mday;
}

int Timestamp::timeMounth() const {
    time_t s = secondsTotal();
    const tm* t = localtime(&s);
    return t->tm_mon;
}

int Timestamp::timeYear() const {
    time_t s = secondsTotal();
    const tm* t = localtime(&s);
    return t->tm_year;
}


//
// helper functions
//
std::ostream &operator<<(std::ostream &output, const Timestamp &t)
{
    if (t < 1000000000) {
        output << " " << t.toDouble();
    } else {
        output << t.secondsTotal() << ".";
        output << std::setfill('0')
               << std::setw(abs(t).timeMicroSeconds()>0?6:1)
               << abs(t).timeMicroSeconds();
        output << "  or local time "
               << std::setfill('0')
               << std::setw(2) << t.timeHours() << ":"
               << std::setw(2) << t.timeMinutes() << ":"
               << std::setw(2) << t.timeSeconds()  << " "
               << std::setw(2) << t.timeDay() << "."
               << std::setw(2) << t.timeMounth()+1 << "."
               << std::setw(4) << 1900+t.timeYear();
    }
    return output;
}

Timestamp operator+(const double& d, const Timestamp&t) {
    return t + d;
}
Timestamp operator-(const double& d, const Timestamp&t) {
    return t - d;
}

void sleepUntil( const Timestamp& t, int N, double rate )
{
    sleepUntil( t + rate*N );
}

void sleepUntil( const Timestamp& t )
{
    //Timestamp now = systemTime();
    //if ( t > now ) {
    //   timespec temp = t - now;
    //    nanosleep (&temp, 0);
    //}
}

Timestamp systemTime()
{
    timeval temp;
    gettimeofday(&temp, 0);
    Timestamp t( temp.tv_sec, 1000 * temp.tv_usec);
    return t;
}