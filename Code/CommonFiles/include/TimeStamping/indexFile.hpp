/*!
 * \defgroup indexFile indexFile - read chil format index files
 * \ingroup Utilities
 * \author Albert Gil <algil@gps.tsc.upc.edu>
 * \date 1-Set-2006
 */
#ifndef INDEXFILE_H
#define INDEXFILE_H


#include <timestamp.hpp>
#include <iostream>
#include <fstream>
#include <vector>

class IndexFileException : public std::logic_error {
    public:
        IndexFileException( const std::string& s) :
        std::logic_error(s) {}
};

class IndexFileEntry {
    public:
        IndexFileEntry  ();
        IndexFileEntry ( Timestamp ts, unsigned int block, int offset );
        Timestamp ts;
        unsigned int block;
        int offset;
};

/*!
    * \class CIndexFile
    * \brief To manage timestamps files of CHIL recordings
    *
    * Timestamps files are used in CHIL recordings to associate a timestamp to each frame.
    * These files are usually placed at recording/video/camN/seq.index.
    * This class reads nad writes this files and allows any kind of editing operation.
    *   
    * \ingroup Utilities
    */
class CIndexFile
{
public:
	/*! 
	 * 	This methods reads the content of the file.
	 *	\param fileName The file to read from.
	 * 	\param fps The frame rate expecetd of fileName file. 
	 * 
	 * \note The fps parameter will be used only if you want to create a master from this file.
	 *   
	 */ 
    void    read            (const std::string fileName, double fps=25);
    /*! This methods writes the object content to the file */
    void    write           (const std::string fileName);
        
    void                        setFrameRate            (double fps);  
    bool                        hasSimilarTimestamp     (const Timestamp& ts, double epsilon) const;
    bool                        hasTimestamp            (const Timestamp& ts) const;
    void                        addEntry                (const Timestamp& ts, unsigned int block, int offset);
    void                        addEntry                (const IndexFileEntry& newEntry);
    void                        clear                   (void);
    
    /*! This method cleans all repeated frames of the object.
     * Repeated frames are added in a sync process. */
    void                        removeRepeatedFrames    (void);

    /*! This method clips the object with master. 
     *  Entries with minor timestamp than the first timestamp of master are removed. And also the ones that have a greater timestamp than the maximum of master are removed. */
    void                        clip                    (const CIndexFile& master);
    
    /*! This method clips the object from first timestamp. 
     *  Entries with minor timestamp than the first timestamp are removed. */
    void                        setFirstTimestamp       (const Timestamp& first);
    
    /*! This method creates a new object with same number of frames of master.
     * Each frame of new object have the timestamp of the current object nearest to master timestamp. */    	
    CIndexFile                  getSync                 (const CIndexFile& master) const;
    
    /*! This method check if the object begins with frame 0. If not, all frames are renamed (renumbered) to start with 0. */
    void                        renameBlocks            (void);
    
    IndexFileEntry              getEntry                (const Timestamp& ts) const;
    IndexFileEntry              getEntry                (const unsigned int blockts) const;
    int                         size                    (void) const;
    
    /*! This method creates a new object fixing sync errors of current one.
     * It is considered an error if temporal distance beetwen two consecutive frames are bigger than tsDistance parameter.
     * 
     */
    CIndexFile                  getMaster               (const double tsDistance) const;
    IndexFileEntry              getNearestEntry         (const Timestamp& ts) const;
    
    /*! 
     * This method adds the same offset for all the entries.
     */
    void                        addTimestampOffset      ( const Timestamp& offset ); 

    void            print                 (void) const;
    
    std::vector<IndexFileEntry>      getEntriesVector        (void) const;

protected:

private:
    std::string filename;
    double tsDistance;
    std::vector<IndexFileEntry> entries;

};


#endif
