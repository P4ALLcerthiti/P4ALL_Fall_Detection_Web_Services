#include "stdafx.h"

// Standard Lib
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sstream>
#include <stdexcept>
#include <iomanip>
#include <indexFile.hpp>
#include <assert.h>

using namespace std;

#ifdef DEBUG
const static bool debug = true;
#else
const static bool debug = false;
#endif


IndexFileEntry:: IndexFileEntry ( Timestamp ts, unsigned int block, int offset ) {
    this->ts     = ts;
    this->block  = block;
    this->offset = offset;
}

IndexFileEntry:: IndexFileEntry () {
    this->ts     = Timestamp();
    this->block  = 0;
    this->offset = 0;
}

void CIndexFile::setFrameRate (double fps)
{
    assert (fps>0);
    this->tsDistance = 1.0 / fps;
}

void CIndexFile::read (string filename, double fps)
{
    ifstream inFile;

    // Save filname
    this->filename   = filename;
    this->tsDistance = 1.0 / fps;

    // Remove current entries
    entries.clear();

    // Open file with input mode
    inFile.open( filename.c_str(), ifstream::in );
    if (!inFile.is_open())
        throw IndexFileException("CIndexFile::read: Error opening IndexFile: " + filename + "\n");

    // Move pointer to file beginning
    inFile.seekg(0, ios_base::beg);

    // Check if first line is a commented line and discard it
    char c;
    inFile >> skipws >> c;
    inFile.unget();
    if ( c == ';' ) {
        string firstLine;
        getline(inFile, firstLine);
        cout << "Header (first line) has been discarted." << endl;
    }

    // Main loop. Read lines and create timestamps vector
    while (!inFile.eof()) {
        string strTimestamp;
        unsigned int block;
        unsigned int offset;

        // Get raw data: time stamp, block and offset
        // TODO: Check the correct format of line
        inFile >> skipws >> strTimestamp >> skipws >> block >> skipws >> offset;

        // Check we didn't arrive to eof
        if (inFile.eof()) break;

        // Push data to vectors
        Timestamp ts( strTimestamp );
        entries.push_back( IndexFileEntry(ts, block, offset ) );

        if (debug) cout << ts << endl;
    }

    inFile.close();
}

vector<IndexFileEntry> CIndexFile::getEntriesVector(void) const {
    return entries;
}

void CIndexFile::clear( void ) {
    entries.clear();
}

void CIndexFile::removeRepeatedFrames( void ) {

    assert(size()>1);

    for (vector<IndexFileEntry>::iterator it = entries.begin()+1; it!=entries.end(); ) {
        if( it->ts == (it-1)->ts )  { it = entries.erase( it ); cout << "Repeated frame removed: " << string(it->ts) << endl; }
        else                        it++;
    }
}

void CIndexFile::addEntry( const Timestamp& ts, unsigned int block, int offset ) {
        entries.push_back( IndexFileEntry(ts, block, offset) );
}
void CIndexFile::addEntry( const IndexFileEntry& newEntry ) {
        entries.push_back( newEntry );
}

void CIndexFile::write( string filename ) {

    ofstream outFile;

    // Open file with output mode
    this->filename = filename;
    outFile.open( this->filename.c_str(), ios::trunc );
    if (!outFile.is_open())
        throw IndexFileException("write: Index file can be opened");

    // Write all data in a row with tabs
    for (vector<IndexFileEntry>::iterator it = entries.begin(); it!=entries.end(); it++ )
        outFile << setfill('0') << setw(12) << it->ts.secondsTotal() << "." << setfill('0') << setw(3) << it->ts.timeMilliSeconds()
                        << "\t" << setfill(' ') << setw(6) << it->block
                        << "\t" << setfill(' ') << setw(6) << it->offset
                        << endl;

    // Add new line to end of file
    outFile << endl << endl;

    // Close file
    outFile.close();
}

bool CIndexFile::hasSimilarTimestamp ( const Timestamp& ts, double epsilon ) const {

    for ( vector<IndexFileEntry>::const_iterator it = entries.begin(); it!=entries.end(); it++) {
        if ( it->ts.isSimilar( ts, epsilon ) ) {
            return true;
        }
    }
    return false;
}

//bool CIndexFile::hasTimestamp ( const Timestamp& ts ) const {
//
//    for ( vector<IndexFileEntry>::const_iterator it = entries.begin(); it!=entries.end(); it++) {
//        if ( it->ts == ts ) {
//            return true;
//        }
//    }
//    return false;
//}

bool CIndexFile::hasTimestamp ( const Timestamp& ts ) const {

	return (getEntry(ts).ts == ts);
}


//IndexFileEntry CIndexFile::getEntry ( const Timestamp& ts ) const {
//
//    for ( vector<IndexFileEntry>::const_iterator it = entries.begin(); it!=entries.end(); it++) {
//        if ( it->ts == ts ) {
//            return *it;
//        }
//    }
//    return IndexFileEntry();
//}

IndexFileEntry CIndexFile::getEntry ( const Timestamp& ts ) const {

    bool end = false;
    for ( vector<IndexFileEntry>::const_iterator it = entries.begin(); !end; it+=2) {
        if(it!=entries.end() ) {
	        if ( it->ts >= ts ) {
	            if 		( it->ts == ts ) return *it;
	            else if ( (it-1)->ts == ts ) return *(it-1);
	        }
	        
	        if(it==entries.end()-1 ) end=true;	        
        }
        else {
        	if ( (it-1)->ts == ts ) return *(it-1); 
        	end=true;
        }        
        
        
        
    }
    return IndexFileEntry();
}

IndexFileEntry CIndexFile::getEntry (const unsigned int block) const {
   for ( vector<IndexFileEntry>::const_iterator it = entries.begin(); it != entries.end(); it++) {
       if ( it->block == block  ) return *it;
    }
    return IndexFileEntry();
}

void CIndexFile::setFirstTimestamp( const Timestamp& first ) {

    // To debug
    assert (entries.size() > 0);
    
    // Remove out of range entries
    // TODO: Improve algorithm to avoid to loop for ALL vector.
    IndexFileEntry firstEntry = getNearestEntry( first );
     
    // Remove entries
    double tsTolerance = tsDistance/2;
    for ( vector<IndexFileEntry>::iterator it = entries.begin(); it != entries.end(); ) {
        if ( ( !(it->ts).isSimilar( firstEntry.ts, tsTolerance ) && (it->ts < firstEntry.ts) ) ) {
            it = entries.erase( it );
        }
        else {
            it++;
        }
    }
    assert ( (firstEntry.ts.isSimilar( entries.at(0).ts, tsTolerance))     || (firstEntry.ts < entries.at(0).ts)      );
}

void CIndexFile::clip( const CIndexFile& file ) {

    // To debug
    assert (entries.size() > 0);
    assert (file.entries.size() > 0);

    // Remove out of range entries
    // TODO: Improve algorithm to avoid to loop for ALL vector.
     IndexFileEntry firstEntry = getNearestEntry ( (*(file.entries.begin())).ts );
     IndexFileEntry lastEntry  = getNearestEntry ( (*(file.entries.end()-1)).ts );

    // Remove entries
    double tsTolerance = tsDistance/2;
    for ( vector<IndexFileEntry>::iterator it = entries.begin(); it != entries.end(); ) {
        if ( ( !(it->ts).isSimilar( firstEntry.ts, tsTolerance ) && (it->ts < firstEntry.ts) ) || 
             ( !(it->ts).isSimilar( lastEntry.ts,  tsTolerance ) && (it->ts > lastEntry.ts ) ) )
        {
            it = entries.erase( it );
        }
        else {
            it++;
        }
    }

    assert ( (firstEntry.ts.isSimilar( entries.at(0).ts, tsTolerance))     || (firstEntry.ts < entries.at(0).ts)      );
    assert ( (lastEntry.ts.isSimilar( (entries.end()-1)->ts, tsTolerance)) || (lastEntry.ts  > (entries.end()-1)->ts) );
}

CIndexFile CIndexFile::getSync( const CIndexFile& master ) const {

    CIndexFile syncFile;

    // Set syncFile frame rate
    syncFile.tsDistance = master.tsDistance;

    // Main loop
    for (vector<IndexFileEntry>::const_iterator it = master.entries.begin(); it!=master.entries.end(); it++) {
    
        // Get entry    
        IndexFileEntry newEntry = getNearestEntry ( it->ts );
        // Edit block and offset
        newEntry.block = it->block;
        newEntry.offset = 0; //TODO: Should be set to 0??
        // Add entry
        syncFile.addEntry( newEntry );

        // If it was a drop frame log it to user
        double tsTolerance = syncFile.tsDistance/2;
        if ( !hasSimilarTimestamp( it->ts, tsTolerance) )
            cout << "Drop frame detected. Added ts: " << string (newEntry.ts) << " as frame " << newEntry.block << endl;
        //else cout << "Added frame " << i << endl;
    }

    return syncFile;
}

void CIndexFile::renameBlocks( void ) {

	size_t block = 0;
    for ( vector<IndexFileEntry>::iterator it = entries.begin(); block < entries.size(); block++, it++ ) {
       it->block = block;
    }
}

void CIndexFile::addTimestampOffset( const Timestamp& offset ) {

    size_t block = 0;
    for ( vector<IndexFileEntry>::iterator it = entries.begin(); block < entries.size(); block++, it++ ) {
       it->ts = it->ts + offset;
    }
}


CIndexFile CIndexFile::getMaster ( double frameRate ) const {

    CIndexFile master = *this;
    master.setFrameRate( frameRate );

    // NOTE: In a master camera a drop frame should be only a clear one to avoid conflicts with variable frame rates. So tolerance should be as big as ts distance
    double tsDistance  = 1.0 / frameRate;
    double tsTolerance = tsDistance;

    // Solve drop frames with expected frames
    // TODO: Improve algorithm using iterators
    for (int i=0; i < master.size()-1; i++)
    {
        Timestamp currentTs  = master.entries.at(i).ts;
        Timestamp expectedTs = master.entries.at(i).ts + tsDistance;
        Timestamp nextTs     = master.entries.at(i+1).ts;

        if ( !nextTs.isSimilar( expectedTs, tsTolerance ) && nextTs > expectedTs ) {
            // Add teorical drop frame following tsDistance 
            IndexFileEntry newEntry = IndexFileEntry( expectedTs, 0, 0 );
            master.entries.insert( (master.entries.begin()+i+1), newEntry );
            cout << "Drop frame detected at master camera. Added teorical ts: " << string(expectedTs) << endl;
        }
        else if (nextTs == currentTs ) {
            // If a ts is repeated in cam it shouldn't in master camera
            master.entries.erase( master.entries.begin()+i );
            i--;
            cout << "Repeated frame detected and removed at master camera." << endl;
        }
    }

    // Rename block on master
    master.renameBlocks();

    return master;
}

IndexFileEntry CIndexFile::getNearestEntry ( const Timestamp& ts ) const {

    IndexFileEntry nearest;

	for ( vector<IndexFileEntry>::const_iterator it = entries.begin(); it != entries.end(); it++) {
        if ( abs(it->ts-ts) < abs(nearest.ts-ts) ) nearest=*it;
    }

    assert( nearest.ts != 0);

    return nearest;
}

void CIndexFile::print( void ) const{

    for ( vector<IndexFileEntry>::const_iterator it = entries.begin(); it!=entries.end(); it++) {
        cout << setfill('0') << setw(12) << it->ts.secondsTotal() << ":" << setfill('0') << setw(9) << it->ts.timeNanoSeconds()
                        << "\t" << setfill(' ') << setw(6) << it->block
                        << "\t" << setfill(' ') << setw(6) << it->offset
                        << endl;
    }
}

int CIndexFile::size( void ) const {
    return entries.size();
}
