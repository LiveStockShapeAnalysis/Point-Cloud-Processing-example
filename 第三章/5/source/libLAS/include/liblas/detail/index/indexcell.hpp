/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  index cell implementation for C++ libLAS 
 * Author:   Gary Huber, gary@garyhuberart.com
 *
 ******************************************************************************
 *
 * (C) Copyright Gary Huber 2010, gary@garyhuberart.com
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

#ifndef LIBLAS_DETAIL_INDEXCELL_HPP_INCLUDED
#define LIBLAS_DETAIL_INDEXCELL_HPP_INCLUDED

#include <map>
#include <stdint.h>

namespace liblas { namespace detail {

typedef int16_t ElevExtrema;
typedef uint32_t ElevRange;
typedef uint8_t	ConsecPtAccumulator;
typedef std::map<uint32_t, ConsecPtAccumulator> IndexCellData;
typedef std::map<uint32_t, IndexCellData> IndexSubCellData;
typedef uint64_t	TempFileOffsetType;

class IndexCell
{
public:
	IndexCell();
	
private:
	TempFileOffsetType m_FileOffset;
	uint32_t m_NumPoints;
	ElevExtrema m_MinZ, m_MaxZ;
	IndexCellData m_PtRecords;
	IndexSubCellData m_ZCellRecords;
	IndexSubCellData m_SubCellRecords;

public:
	void SetFileOffset(TempFileOffsetType fos);
	void SetNumPoints(uint32_t nmp);
	TempFileOffsetType GetFileOffset(void) const;
	uint32_t GetNumRecords(void) const;
	uint32_t GetNumPoints(void) const;
	uint32_t GetNumSubCellRecords(void) const;
	uint32_t GetNumZCellRecords(void) const;
	ElevExtrema GetMinZ(void) const {return m_MinZ;}
	ElevExtrema GetMaxZ(void) const {return m_MaxZ;}
	bool RoomToAdd(uint32_t a);
	void AddPointRecord(uint32_t a);
	void AddPointRecord(uint32_t a, uint8_t b);
	bool IncrementPointRecord(uint32_t a);
	void RemoveMainRecords(void);
	void RemoveAllRecords(void);
	void UpdateZBounds(double TestZ);
	ElevRange GetZRange(void) const;
	void AddZCell(uint32_t a, uint32_t b);
	bool IncrementZCell(uint32_t a, uint32_t b);
	void AddSubCell(uint32_t a, uint32_t b);
	bool IncrementSubCell(uint32_t a, uint32_t b);
	uint8_t GetPointRecordCount(uint32_t a);
	const IndexCellData::iterator GetFirstRecord(void);
	const IndexCellData::iterator GetEnd(void);
	const IndexSubCellData::iterator GetFirstSubCellRecord(void);
	const IndexSubCellData::iterator GetEndSubCell(void);
	const IndexSubCellData::iterator GetFirstZCellRecord(void);
	const IndexSubCellData::iterator GetEndZCell(void);
};

}} // namespace liblas::detail

#endif // LIBLAS_DETAIL_INDEXCELL_HPP_INCLUDED
