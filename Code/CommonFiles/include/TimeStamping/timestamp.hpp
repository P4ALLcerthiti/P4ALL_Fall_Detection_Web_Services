/*!
 * \defgroup Timestamp Timestamp
 * \ingroup Utilities
 * \author Joachim Neumann <joachim@gps.tsc.upc.edu>
 * \date 18-Feb-2005
 */
#define WIN32_LEAN_AND_MEAN

#ifndef TIMESTAMP_HPP
#define TIMESTAMP_HPP

// Issue warning 4385 only once.
#pragma warning( once : 4244 )


#include <time.h> // for timespec
#include "timeval.h"
#include <stdexcept>

class TimestampException : public std::logic_error {
    public:
        TimestampException( const std::string& s) :
        std::logic_error(s) {}
};


class Timestamp
{
    friend std::ostream &operator<<(std::ostream &, const Timestamp &);
    friend Timestamp TimestampFromNanoseconds( long long ns );
    public:
        Timestamp();
        Timestamp(const Timestamp& t);
        Timestamp(unsigned int, unsigned int);
        //Timestamp(timespec);
        Timestamp(timeval);
        Timestamp(double t);
        Timestamp( const std::string& );

        Timestamp operator+=(const Timestamp&);
        Timestamp operator-=(const Timestamp&);
        Timestamp operator+(const Timestamp&) const;
        Timestamp operator-(const Timestamp&) const;
        Timestamp operator*(const double) const;
        Timestamp operator-() const;
        bool operator==(const Timestamp&) const;
        bool operator!=(const Timestamp&) const;
        bool operator>=(const Timestamp&) const;
        bool operator<=(const Timestamp&) const;
        bool operator> (const Timestamp&) const;
        bool operator< (const Timestamp&) const;

        const bool isSimilar (const Timestamp&, double epsilon) const;
        //operator const timespec() const;
        operator const timeval () const;
        operator const std::string() const;
        std::string toChilString() const;
        double toDouble() const;
        bool positive() const;

        int timeNanoSeconds() const;
        int timeMicroSeconds() const;
        int timeMilliSeconds() const;
        int timeSeconds() const;
        int timeMinutes() const;
        int timeHours() const;
        int timeDay() const;
        int timeMounth() const;
        int  timeYear() const;
        long long nanoSecondsTotal() const;
        long long microSecondsTotal() const;
        long long milliSecondsTotal() const;
        long secondsTotal() const;
        long minutesTotal() const;
    private:
        long long _nanoseconds;
};

// helper functions
Timestamp operator+(const double&, const Timestamp&);
Timestamp operator-(const double&, const Timestamp&);
Timestamp abs(const Timestamp&);
Timestamp systemTime();
void sleepUntil(const Timestamp&, int N, double rate);
void sleepUntil(const Timestamp&);

//#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
//#define DELTA_EPOCH_IN_MICROSECS  11644473600000000Ui64
//#else
//#define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL
//#endif
//
//struct timezone 
//{
//	int  tz_minuteswest; /* minutes W of Greenwich */
//	int  tz_dsttime;     /* type of dst correction */
//};
//
//static int gettimeofday(struct timeval *tv, struct timezone *tz)
//{
//	FILETIME ft;
//	unsigned __int64 tmpres = 0;
//	static int tzflag;
//
//	if (NULL != tv)
//	{
//		GetSystemTimeAsFileTime(&ft);
//
//		tmpres |= ft.dwHighDateTime;
//		tmpres <<= 32;
//		tmpres |= ft.dwLowDateTime;
//
//		/*converting file time to unix epoch*/
//		tmpres -= DELTA_EPOCH_IN_MICROSECS; 
//		tmpres /= 10;  /*convert into microseconds*/
//		tv->tv_sec = (long)(tmpres / 1000000UL);
//		tv->tv_usec = (long)(tmpres % 1000000UL);
//	}
//
//	if (NULL != tz)
//	{
//		if (!tzflag)
//		{
//			_tzset();
//			tzflag++;
//		}
//		tz->tz_minuteswest = _timezone / 60;
//		tz->tz_dsttime = _daylight;
//	}
//
//	return 0;
//}


#endif // TIMESTAMP_HPP

