/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS index class 
 * Author:   Gary Huber, gary@garyhuberart.com
 *
 ******************************************************************************
 * Copyright (c) 2010, Gary Huber, gary@garyhuberart.com
 *
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following 
 * conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright 
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright 
 *       notice, this list of conditions and the following disclaimer in 
 *       the documentation and/or other materials provided 
 *       with the distribution.
 *     * Neither the name of the Martin Isenburg or Iowa Department 
 *       of Natural Resources nor the names of its contributors may be 
 *       used to endorse or promote products derived from this software 
 *       without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS 
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 ****************************************************************************/

#ifndef LIBLAS_LASINDEX_HPP_INCLUDED
#define LIBLAS_LASINDEX_HPP_INCLUDED

#include <liblas/reader.hpp>
#include <liblas/header.hpp>
#include <liblas/bounds.hpp>
#include <liblas/variablerecord.hpp>
#include <liblas/detail/index/indexcell.hpp>
#include <liblas/export.hpp>

// std
#include <stdexcept> // std::out_of_range
#include <cstdio>	// file io
#include <iostream>	// file io
#include <cstdlib> // std::size_t
#include <vector> // std::vector

namespace liblas {

#define LIBLAS_INDEX_MAXMEMDEFAULT	10000000	// 10 megs default
#define LIBLAS_INDEX_MINMEMDEFAULT	1000000	// 1 meg at least has to be allowed
#define LIBLAS_INDEX_VERSIONMAJOR	1
#define LIBLAS_INDEX_VERSIONMINOR	2	// minor version 2 begins 11/15/10
#define LIBLAS_INDEX_MAXSTRLEN	512
#define LIBLAS_INDEX_MAXCELLS	250000
#define LIBLAS_INDEX_OPTPTSPERCELL	100
#define LIBLAS_INDEX_MAXPTSPERCELL	1000
#define LIBLAS_INDEX_RESERVEFILTERDEFAULT	1000000	// 1 million points will be reserved on large files for filter result

// define this in order to fix problem with last bytes of last VLR getting corrupted
// when saved and reloaded from index or las file.
#define LIBLAS_INDEX_PADLASTVLR

typedef std::vector<uint8_t> IndexVLRData;
typedef std::vector<liblas::detail::IndexCell> IndexCellRow;
typedef std::vector<IndexCellRow>	IndexCellDataBlock;

class LAS_DLL IndexData;
class LAS_DLL IndexIterator;

// Index class is the fundamental object for building and filtering a spatial index of points in an LAS file.
// An Index class doesn't do anything until it is configured with an IndexData object (see below).
//		You instantiate an Index object first and then pass it an IndexData or you can construct the Index with an 
//		IndexData. Nothing happens until the Index is told what to do via the configuration with IndexData.
//		For details on configuration options see IndexData below.

// Once an index exists and is configured, it can be used to filter the point set in the LAS file for which it
//		was built. The points have to be what they were when the index was built. The index is not automatically 
//		updated if the point set is changed. The index becomes invalid if either the number of points is changed,
//		the order of points is changed or the location of any points is changed. The Validate function is run to
//		determine as best it can the validity of the stored index but it isn't perfect. It can only determine if
//		the number of points has changed or the spatial extents of the file have changed.

// The user can constrain the memory used in building an index if that is believed to be an issue. 
//		The results will be the same but some efficiency may be lost in the index building process.

//	Data stored in index header can be examined for determining suitability of index for desired purpose.
//		1) presence of z-dimensional cell structure is indicated by GetCellsZ() called on the Index.
//		2) Index author GetIndexAuthorStr() - provided by author at time of creation
//		3) Index comment GetIndexCommentStr() - provided by author at time of creation
//		4) Index creation date GetIndexDateStr() - provided by author at time of creation
//	The latter fields are not validated in any way by the index building code and are just three fields 
//		which can be used as the user sees fit. Maximum length is LIBLAS_INDEX_MAXSTRLEN - 1.

// Obtaining a filtered set of points requires that a valid index exist (see above).
// The IndexData class is used again to pass the extents of the filter to the Index. By making any high/low
//		bounds coordinate pair equal, that dimension is ignored for the purposes of filtering. Note that Z dimension 
//		discrimination is still available even if z-binning was not invoked during index creation. Filtering with
//		the z dimension may be slower in that event but no less successful.

// A filter operation is invoked with the command:
//		const std::vector<uint32_t>& Filter(IndexData const& ParamSrc);
// The return value is a vector of point ID's. The points can be accessed from the LAS file in the standard way
//		as the index in no way modifies them or their sequential order.
// Currently only one, two or three dimensional spatial window filters are supported. See IndexData below for 
//		more info on filtering.

class LAS_DLL Index
{
public:
	Index();
    Index(IndexData const& ParamSrc);
	~Index();
	
    // Blocked copying operations, declared but not defined.
    /// Copy constructor.
    Index(Index const& other);
    /// Assignment operator.
    Index& operator=(Index const& rhs);
    
private:

	Reader *m_reader;
	Reader *m_idxreader;
	Header m_pointheader;
	Header m_idxheader;
	Bounds<double> m_bounds;
	bool m_indexBuilt, m_tempFileStarted, m_readerCreated, m_readOnly, m_writestandaloneindex, m_forceNewIndex;
	int m_debugOutputLevel;
	uint8_t m_versionMajor, m_versionMinor;
    uint32_t m_pointRecordsCount, m_maxMemoryUsage, m_cellsX, m_cellsY, m_cellsZ, m_totalCells, 
		m_DataVLR_ID;
    liblas::detail::TempFileOffsetType m_tempFileWrittenBytes;
    double m_rangeX, m_rangeY, m_rangeZ, m_cellSizeZ, m_cellSizeX, m_cellSizeY;
	std::string m_tempFileName;	
	std::string m_indexAuthor;
	std::string m_indexComment;
	std::string m_indexDate;
	std::vector<uint32_t> m_filterResult;
	std::ostream *m_ofs;
    FILE *m_tempFile, *m_outputFile;
    FILE *m_debugger;
    
	void SetValues(void);
    bool IndexInit(void);
    void ClearOldIndex(void);
	bool BuildIndex(void);
	bool Validate(void);
	uint32_t GetDefaultReserve(void);
	bool LoadIndexVLR(VariableRecord const& vlr);
	void SetCellFilterBounds(IndexData & ParamSrc);
	bool FilterOneVLR(VariableRecord const& vlr, uint32_t& i, IndexData & ParamSrc, bool & VLRDone);
	bool FilterPointSeries(uint32_t & PointID, uint32_t & PointsScanned, 
		uint32_t const PointsToIgnore, uint32_t const x, uint32_t const y, uint32_t const z, 
		liblas::detail::ConsecPtAccumulator const ConsecutivePts, IndexIterator *Iterator, 
		IndexData const& ParamSrc);
	bool VLRInteresting(int32_t MinCellX, int32_t MinCellY, int32_t MaxCellX, int32_t MaxCellY, 
		IndexData const& ParamSrc);
	bool CellInteresting(int32_t x, int32_t y, IndexData const& ParamSrc);
	bool SubCellInteresting(int32_t SubCellID, int32_t XCellID, int32_t YCellID, IndexData const& ParamSrc);
	bool ZCellInteresting(int32_t ZCellID, IndexData const& ParamSrc);
	bool FilterOnePoint(int32_t x, int32_t y, int32_t z, int32_t PointID, int32_t LastPointID, bool &LastPtRead,
		IndexData const& ParamSrc);
	// Determines what X/Y cell in the basic cell matrix a point falls in
	bool IdentifyCell(Point const& CurPt, uint32_t& CurCellX, uint32_t& CurCellY) const;
	// determines what Z cell a point falls in
	bool IdentifyCellZ(Point const& CurPt, uint32_t& CurCellZ) const;
	// Determines what quadrant sub-cell a point falls in
	bool IdentifySubCell(Point const& CurPt, uint32_t x, uint32_t y, uint32_t& CurSubCell) const;
	// Offloads binned cell data while building Index when cell data in memory exceeds maximum set by user
	bool PurgePointsToTempFile(IndexCellDataBlock& CellBlock);
	// Reloads and examines one cell of data from temp file
	bool LoadCellFromTempFile(liblas::detail::IndexCell *CellBlock, 
		uint32_t CurCellX, uint32_t CurCellY);
	// temp file is used to store sorted data while building index
	FILE *OpenTempFile(void);
	// closes and removes the temp file
	void CloseTempFile(void);
	// Creates a Writer from m_ofs and re-saves entire LAS input file with new index
	// Current version does not save any data following the points
	bool SaveIndexInLASFile(void);
	// Creates a Writer from m_ofs and re-saves LAS header with new index, but not with data point records
	bool SaveIndexInStandAloneFile(void);
	// Calculate index bounds dimensions
	void CalcRangeX(void)	{m_rangeX = (m_bounds.max)(0) - (m_bounds.min)(0);}
	void CalcRangeY(void)	{m_rangeY = (m_bounds.max)(1) - (m_bounds.min)(1);}
	void CalcRangeZ(void)	{m_rangeZ = (m_bounds.max)(2) - (m_bounds.min)(2);}

	// error messages
	bool FileError(const char *Reporter);
	bool InputFileError(const char *Reporter) const;
	bool OutputFileError(const char *Reporter) const;
	bool DebugOutputError(const char *Reporter) const;
	bool PointCountError(const char *Reporter) const;
	bool PointBoundsError(const char *Reporter) const;
	bool MemoryError(const char *Reporter) const;
	bool InitError(const char *Reporter) const;
	bool InputBoundsError(const char *Reporter) const;

	// debugging
	bool OutputCellStats(IndexCellDataBlock& CellBlock)  const;
	bool OutputCellGraph(std::vector<uint32_t> CellPopulation, uint32_t MaxPointsPerCell)  const;
	
public:
	// IndexFailed and IndexReady can be used to tell if an Index is ready for a filter operation
    bool IndexFailed(void)  const {return (! m_indexBuilt);}
    bool IndexReady(void)  const {return (m_indexBuilt);}
    // Prep takes the input data and initializes Index values and then either builds or examines the Index
    bool Prep(IndexData const& ParamSrc);
    // Filter performs a point filter using the bounds in ParamSrc
    const std::vector<uint32_t>& Filter(IndexData & ParamSrc);
    IndexIterator* Filter(IndexData const& ParamSrc, uint32_t ChunkSize);
    IndexIterator* Filter(double LowFilterX, double HighFilterX, double LowFilterY, double HighFilterY, 
		double LowFilterZ, double HighFilterZ, uint32_t ChunkSize);
    IndexIterator* Filter(Bounds<double> const& BoundsSrc, uint32_t ChunkSize);
    
    // Return the bounds of the current Index
	double GetMinX(void) const	{return (m_bounds.min)(0);}
	double GetMaxX(void) const	{return (m_bounds.max)(0);}
	double GetMinY(void) const	{return (m_bounds.min)(1);}
	double GetMaxY(void) const	{return (m_bounds.max)(1);}
	double GetMinZ(void) const	{return (m_bounds.min)(2);}
	double GetMaxZ(void) const	{return (m_bounds.max)(2);}
	// Ranges are updated when an index is built or the index header VLR read
	double GetRangeX(void) const	{return m_rangeX;}
	double GetRangeY(void) const	{return m_rangeY;}
	double GetRangeZ(void) const	{return m_rangeZ;}
	Bounds<double> const& GetBounds(void) const	{return m_bounds;}
	// Return the number of points used to build the Index
	uint32_t GetPointRecordsCount(void) const	{return m_pointRecordsCount;}
	// Return the number of cells in the Index
	uint32_t GetCellsX(void) const	{return m_cellsX;}
	uint32_t GetCellsY(void) const	{return m_cellsY;}
	// Return the number of Z-dimension cells in the Index. Value is 1 if no Z-cells were created during Index building
	uint32_t GetCellsZ(void) const	{return m_cellsZ;}
	// 42 is the ID for the Index header VLR and 43 is the normal ID for the Index data VLR's
	// For future expansion, multiple indexes could assign data VLR ID's of their own choosing
	uint32_t GetDataVLR_ID(void) const	{return m_DataVLR_ID;}
	// Since the user can define a Z cell size it is useful to examine that for an existing index
	double GetCellSizeZ(void) const	{return m_cellSizeZ;}
	// Return values used in building or examining index
	FILE *GetDebugger(void) const	{return m_debugger;}
	bool GetReadOnly(void) const	{return m_readOnly;}
	bool GetStandaloneIndex(void) const	{return m_writestandaloneindex;}
	bool GetForceNewIndex(void) const	{return m_forceNewIndex;}
	uint32_t GetMaxMemoryUsage(void) const	{return m_maxMemoryUsage;}
	int GetDebugOutputLevel(void) const {return m_debugOutputLevel;}
	// Not sure if these are more useful than dangerous
	Header *GetPointHeader(void) {return &m_pointheader;}
	Header *GetIndexHeader(void) {return &m_idxheader;}
	Reader *GetReader(void) const {return m_reader;}
	Reader *GetIndexReader(void) const {return m_idxreader;}
	const char *GetTempFileName(void) const {return m_tempFileName.c_str();}
	// Returns the strings set in the index when built
	const char *GetIndexAuthorStr(void)  const;
	const char *GetIndexCommentStr(void)  const;
	const char *GetIndexDateStr(void)  const;
	uint8_t GetVersionMajor(void) const	{return m_versionMajor;}
	uint8_t GetVersionMinor(void) const	{return m_versionMinor;}
	// Methods for setting values used when reading index from file to facilitate moving reading function into
	// separate IndexInput object at a future time to provide symmetry with IndexOutput
	void SetDataVLR_ID(uint32_t DataVLR_ID)	{m_DataVLR_ID = DataVLR_ID;}
	void SetIndexAuthorStr(const char *ias)	{m_indexAuthor = ias;}
	void SetIndexCommentStr(const char *ics)	{m_indexComment = ics;}
	void SetIndexDateStr(const char *ids)	{m_indexDate = ids;}
	void SetMinX(double minX)	{(m_bounds.min)(0, minX);}
	void SetMaxX(double maxX)	{(m_bounds.max)(0, maxX);}
	void SetMinY(double minY)	{(m_bounds.min)(1, minY);}
	void SetMaxY(double maxY)	{(m_bounds.max)(1, maxY);}
	void SetMinZ(double minZ)	{(m_bounds.min)(2, minZ);}
	void SetMaxZ(double maxZ)	{(m_bounds.max)(2, maxZ);}
	void SetPointRecordsCount(uint32_t prc)	{m_pointRecordsCount = prc;}
	void SetCellsX(uint32_t cellsX)	{m_cellsX = cellsX;}
	void SetCellsY(uint32_t cellsY)	{m_cellsY = cellsY;}
	void SetCellsZ(uint32_t cellsZ)	{m_cellsZ = cellsZ;}
	
}; 

// IndexData is used to pass attributes to and from the Index itself.
// How it is initialized determines what action is taken when an Index object is instantiated with the IndexData.
// The choices are:
// a) Build an index for an las file
//		1) std::ostream *ofs must be supplied as well as std::istream *ifs or Reader *reader and a full file path 
//			for writing a temp file, const char *tmpfilenme.
// b) Examine an index for an las file
//		1) std::istream *ifs or Reader *reader must be supplied for the LAS file containing the point data
//		2) if the index to be read is in a standalone file then Reader *idxreader must also be supplied
// Options for building are
//		a) build a new index even if an old one exists, overwriting the old one
//			1) forcenewindex must be true
//			2) std::ostream *ofs must be a valid ostream where there is storage space for the desired output
//		b) only build an index if none exists
//			1) forcenewindex must be false
//			2) std::ostream *ofs must be a valid ostream where there is storage space for the desired output
//		c) do not build a new index under any circumstances
//			1) readonly must be true
// Location of the index can be specified
//		a) build the index within the las file VLR structure
//			1) writestandaloneindex must be false
//			2) std::ostream *ofs must be a valid ostream where there is storage space for a full copy
//				of the LAS file plus the new index which is typically less than 5% of the original file size
//		b) build a stand-alone index outside the las file
//			1) writestandaloneindex must be true
//			2) std::ostream *ofs must be a valid ostream where there is storage space for the new index 
//				which is typically less than 5% of the original file size

// How the index is built is determined also by members of the IndexData class object.
// Options include:
//		a) control the maximum memory used during the build process
//			1) pass a value for maxmem in bytes greater than 0. 0 resolves to default LIBLAS_INDEX_MAXMEMDEFAULT.
//		b) debug messages generated during index creation or filtering. The higher the number, the more messages.
//			0) no debug reports
//			1) general info messages
//			2) status messages
//			3) cell statistics
//			4) progress status
//		c) where debug messages are sent
//			1) default is stderr
//		d) control the creation of z-dimensional cells and what z cell size to use
//			1) to turn on z-dimensional binning, use a value larger than 0 for zbinht
//		e) data can be stored in index header for later use in recognizing the index.
//			1) Index author indexauthor - provided by author at time of creation
//			2) Index comment indexcomment - provided by author at time of creation
//			3) Index creation date indexdate - provided by author at time of creation
//			The fields are not validated in any way by the index building code and are just three fields 
//			which can be used as the user sees fit. Maximum length is LIBLAS_INDEX_MAXSTRLEN - 1.

// Once an index is built, or if an index already exists, the IndexData can be configured
//		to define the bounds of a filter operation. Any dimension whose bounds pair are equal will
//		be disregarded for the purpose of filtering. Filtering on the Z axis can still be performed even if the 
//		index was not built with Z cell sorting. Bounds must be defined in the same units and coordinate
//		system that a liblas::Header returns with the commands GetMin{X|Y|Z} and a liblas::Point returns with 
//		Get{X|Y|Z}

class LAS_DLL IndexData
{
friend class Index;
friend class IndexIterator;

public:
	IndexData(void);
 	IndexData(Index const& index);

 	// use one of these methods to configure the IndexData with the values needed for specific tasks

 	// one comprehensive method to set all the values used in index initialization
	bool SetInitialValues(std::istream *ifs = 0, Reader *reader = 0, std::ostream *ofs = 0, Reader *idxreader = 0, 
		const char *tmpfilenme = 0, const char *indexauthor = 0, 
		const char *indexcomment = 0, const char *indexdate = 0, double zbinht = 0.0, 
		uint32_t maxmem = LIBLAS_INDEX_MAXMEMDEFAULT, int debugoutputlevel = 0, bool readonly = 0, 
		bool writestandaloneindex = 0, bool forcenewindex = 0, FILE *debugger = 0);

	// set the values needed for building an index embedded in existing las file, overriding any existing index
	bool SetBuildEmbedValues(Reader *reader, std::ostream *ofs, const char *tmpfilenme, const char *indexauthor = 0, 
		const char *indexcomment = 0, const char *indexdate = 0, double zbinht = 0.0, 
		uint32_t maxmem = LIBLAS_INDEX_MAXMEMDEFAULT, int debugoutputlevel = 0, FILE *debugger = 0);

	// set the values needed for building an index in a standalone file, overriding any existing index
	bool SetBuildAloneValues(Reader *reader, std::ostream *ofs, const char *tmpfilenme, const char *indexauthor = 0, 
		const char *indexcomment = 0, const char *indexdate = 0, double zbinht = 0.0, 
		uint32_t maxmem = LIBLAS_INDEX_MAXMEMDEFAULT, int debugoutputlevel = 0, FILE *debugger = 0);

	// set the values needed for filtering with an existing index in an las file
	bool SetReadEmbedValues(Reader *reader, int debugoutputlevel = 0, FILE *debugger = 0);

	// set the values needed for filtering with an existing index in a standalone file
	bool SetReadAloneValues(Reader *reader, Reader *idxreader, int debugoutputlevel = 0, FILE *debugger = 0);

	// set the values needed for building an index embedded in existing las file only if no index already exists
	// otherwise, prepare the existing index for filtering
	bool SetReadOrBuildEmbedValues(Reader *reader, std::ostream *ofs, const char *tmpfilenme, const char *indexauthor = 0, 
		const char *indexcomment = 0, const char *indexdate = 0, double zbinht = 0.0, 
		uint32_t maxmem = LIBLAS_INDEX_MAXMEMDEFAULT, int debugoutputlevel = 0, FILE *debugger = 0);

	// set the values needed for building an index in a standalone file only if no index already exists in the las file
	// otherwise, prepare the existing index for filtering
	bool SetReadOrBuildAloneValues(Reader *reader, std::ostream *ofs, const char *tmpfilenme, const char *indexauthor = 0, 
		const char *indexcomment = 0, const char *indexdate = 0, double zbinht = 0.0, 
		uint32_t maxmem = LIBLAS_INDEX_MAXMEMDEFAULT, int debugoutputlevel = 0, FILE *debugger = 0);
	
	// set the bounds for use in filtering
	bool SetFilterValues(double LowFilterX, double HighFilterX, double LowFilterY, double HighFilterY, double LowFilterZ, double HighFilterZ, 
		Index const& index);
	bool SetFilterValues(Bounds<double> const& src, Index const& index);

    /// Copy constructor.
    IndexData(IndexData const& other);
    /// Assignment operator.
    IndexData& operator=(IndexData const& rhs);

private:
	void SetValues(void);
	bool CalcFilterEnablers(void);
	void Copy(IndexData const& other);
	
protected:
	Reader *m_reader;
	Reader *m_idxreader;
	IndexIterator *m_iterator;
	Bounds<double> m_filter;
	std::istream *m_ifs;
	std::ostream *m_ofs;
	const char *m_tempFileName;
	const char *m_indexAuthor;
	const char *m_indexComment;
	const char *m_indexDate;
	double m_cellSizeZ;
	double m_LowXBorderPartCell, m_HighXBorderPartCell, m_LowYBorderPartCell, m_HighYBorderPartCell;
    int32_t m_LowXCellCompletelyIn, m_HighXCellCompletelyIn, m_LowYCellCompletelyIn, m_HighYCellCompletelyIn,
		m_LowZCellCompletelyIn, m_HighZCellCompletelyIn;
    int32_t m_LowXBorderCell, m_HighXBorderCell, m_LowYBorderCell, m_HighYBorderCell,
		m_LowZBorderCell, m_HighZBorderCell;
	uint32_t m_maxMemoryUsage;
	int m_debugOutputLevel;
	bool m_noFilterX, m_noFilterY, m_noFilterZ, m_readOnly, m_writestandaloneindex, m_forceNewIndex, m_indexValid;
	FILE *m_debugger;

	void SetIterator(IndexIterator *setIt) {m_iterator = setIt;}
	IndexIterator *GetIterator(void) {return(m_iterator);}
	
public:
	double GetCellSizeZ(void) const	{return m_cellSizeZ;}
	FILE *GetDebugger(void) const	{return m_debugger;}
	bool GetReadOnly(void) const	{return m_readOnly;}
	bool GetStandaloneIndex(void) const	{return m_writestandaloneindex;}
	bool GetForceNewIndex(void) const	{return m_forceNewIndex;}
	uint32_t GetMaxMemoryUsage(void) const	{return m_maxMemoryUsage;}
	Reader *GetReader(void) const {return m_reader;}
	int GetDebugOutputLevel(void) const {return m_debugOutputLevel;}
	const char *GetTempFileName(void) const {return m_tempFileName;}
	const char *GetIndexAuthorStr(void)  const;
	const char *GetIndexCommentStr(void)  const;
	const char *GetIndexDateStr(void)  const;
	double GetMinFilterX(void) const	{return (m_filter.min)(0);}
	double GetMaxFilterX(void) const	{return (m_filter.max)(0);}
	double GetMinFilterY(void) const	{return (m_filter.min)(1);}
	double GetMaxFilterY(void) const	{return (m_filter.max)(1);}
	double GetMinFilterZ(void) const	{return (m_filter.min)(2);}
	double GetMaxFilterZ(void) const	{return (m_filter.max)(2);}
	void ClampFilterBounds(Bounds<double> const& m_bounds);
	void SetReader(Reader *reader)	{m_reader = reader;}
	void SetIStream(std::istream *ifs)	{m_ifs = ifs;}
	void SetOStream(std::ostream *ofs)	{m_ofs = ofs;}
	void SetTmpFileName(const char *tmpfilenme)	{m_tempFileName = tmpfilenme;}
	void SetIndexAuthor(const char *indexauthor)	{m_indexAuthor = indexauthor;}
	void SetIndexComment(const char *indexcomment)	{m_indexComment = indexcomment;}
	void SetIndexDate(const char *indexdate)	{m_indexDate = indexdate;}
	void SetCellSizeZ(double cellsizez)	{m_cellSizeZ = cellsizez;}
	void SetMaxMem(uint32_t maxmem)	{m_maxMemoryUsage = maxmem;}
	void SetDebugOutputLevel(int debugoutputlevel)	{m_debugOutputLevel = debugoutputlevel;}
	void SetReadOnly(bool readonly)	{m_readOnly = readonly;}
	void SetStandaloneIndex(bool writestandaloneindex)	{m_writestandaloneindex = writestandaloneindex;}
	void SetDebugger(FILE *debugger)	{m_debugger = debugger;}
};

class LAS_DLL IndexIterator
{
friend class Index;

protected:
	IndexData m_indexData;
	Index *m_index;
	uint32_t m_chunkSize, m_advance;
	uint32_t m_curVLR, m_curCellStartPos, m_curCellX, m_curCellY, m_totalPointsScanned, m_ptsScannedCurCell,
		m_ptsScannedCurVLR;
	uint32_t m_conformingPtsFound;

public:
	IndexIterator(Index *IndexSrc, double LowFilterX, double HighFilterX, double LowFilterY, double HighFilterY, 
		double LowFilterZ, double HighFilterZ, uint32_t ChunkSize);
	IndexIterator(Index *IndexSrc, IndexData const& IndexDataSrc, uint32_t ChunkSize);
	IndexIterator(Index *IndexSrc, Bounds<double> const& BoundsSrc, uint32_t ChunkSize);
    /// Copy constructor.
 	IndexIterator(IndexIterator const& other);
    /// Assignment operator.
    IndexIterator& operator=(IndexIterator const& rhs);
private:
 	void Copy(IndexIterator const& other);
	void ResetPosition(void);
	uint8_t MinMajorVersion(void)	{return(1);}
	uint8_t MinMinorVersion(void)	{return(2);}

public:
	/// n=0 or n=1 gives next sequence with no gap, n>1 skips n-1 filter-compliant points, n<0 jumps backwards n compliant points
    const std::vector<uint32_t>& advance(int32_t n);
    /// returns filter-compliant points as though the first point returned is element n in a zero-based array
    const std::vector<uint32_t>& operator()(int32_t n);
	/// returns next set of filter-compliant points with no skipped points
	inline const std::vector<uint32_t>& operator++()	{return (advance(1));}
	/// returns next set of filter-compliant points with no skipped points
	inline const std::vector<uint32_t>& operator++(int)	{return (advance(1));}
	/// returns set of filter-compliant points skipping backwards 1 from the end of the last set
	inline const std::vector<uint32_t>& operator--()	{return (advance(-1));}
	/// returns set of filter-compliant points skipping backwards 1 from the end of the last set
	inline const std::vector<uint32_t>& operator--(int)	{return (advance(-1));}
	/// returns next set of filter-compliant points with n-1 skipped points, for n<0 acts like -=()
	inline const std::vector<uint32_t>& operator+=(int32_t n)	{return (advance(n));}
	/// returns next set of filter-compliant points with n-1 skipped points, for n<0 acts like -()
	inline const std::vector<uint32_t>& operator+(int32_t n)	{return (advance(n));}
	/// returns set of filter-compliant points beginning n points backwards from the end of the last set, for n<0 acts like +=()
	inline const std::vector<uint32_t>& operator-=(int32_t n)	{return (advance(-n));}
	/// returns set of filter-compliant points beginning n points backwards from the end of the last set, for n<0 acts like +()
	inline const std::vector<uint32_t>& operator-(int32_t n)	{return (advance(-n));}
    /// returns filter-compliant points as though the first point returned is element n in a zero-based array
	inline const std::vector<uint32_t>& operator[](int32_t n)	{return ((*this)(n));}
	/// tests viability of index for filtering with iterator
	bool ValidateIndexVersion(uint8_t VersionMajor, uint8_t VersionMinor)	{return (VersionMajor > MinMajorVersion() || (VersionMajor == MinMajorVersion() && VersionMinor >= MinMinorVersion()));}
};

template <typename T, typename Q>
inline void ReadVLRData_n(T& dest, IndexVLRData const& src, Q& pos)
{
    // error if reading past array end
    if (static_cast<size_t>(pos) + sizeof(T) > src.size())
		throw std::out_of_range("liblas::detail::ReadVLRData_n: array index out of range");
	// copy sizeof(T) bytes to destination
    memcpy(&dest, &src[pos], sizeof(T));
    // Fix little-endian
    LIBLAS_SWAP_BYTES_N(dest, sizeof(T));
    // increment the write position to end of written data
    pos = pos + static_cast<Q>(sizeof(T));
}

template <typename T, typename Q>
inline void ReadVLRDataNoInc_n(T& dest, IndexVLRData const& src, Q const& pos)
{
    // error if reading past array end
    if (static_cast<size_t>(pos) + sizeof(T) > src.size())
		throw std::out_of_range("liblas::detail::ReadVLRDataNoInc_n: array index out of range");
	// copy sizeof(T) bytes to destination
    memcpy(&dest, &src[pos], sizeof(T));
    // Fix little-endian
    LIBLAS_SWAP_BYTES_N(dest, sizeof(T));
}

template <typename T, typename Q>
inline void ReadeVLRData_str(char * dest, IndexVLRData const& src, T const srclen, Q& pos)
{
    // error if reading past array end
    if (static_cast<size_t>(pos) + static_cast<size_t>(srclen) > src.size())
		throw std::out_of_range("liblas::detail::ReadeVLRData_str: array index out of range");
 	// copy srclen bytes to destination
	memcpy(dest, &src[pos], srclen);
    // increment the write position to end of written data
    pos = pos + static_cast<Q>(srclen);
}

template <typename T, typename Q>
inline void ReadVLRDataNoInc_str(char * dest, IndexVLRData const& src, T const srclen, Q pos)
{
    // error if reading past array end
    if (static_cast<size_t>(pos) + static_cast<size_t>(srclen) > src.size())
		throw std::out_of_range("liblas::detail::ReadVLRDataNoInc_str: array index out of range");
 	// copy srclen bytes to destination
    std::memcpy(dest, &src[pos], srclen);
}

} // namespace liblas

#endif // LIBLAS_LASINDEX_HPP_INCLUDED
