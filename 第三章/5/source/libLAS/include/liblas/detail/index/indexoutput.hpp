/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  index output implementation for C++ libLAS 
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

#ifndef LIBLAS_DETAIL_INDEXOUTPUT_HPP_INCLUDED
#define LIBLAS_DETAIL_INDEXOUTPUT_HPP_INCLUDED

#include <liblas/index.hpp>
#include <liblas/detail/index/indexcell.hpp>

namespace liblas { namespace detail {

class IndexOutput
{
friend class liblas::Index;
public:
    IndexOutput(liblas::Index *indexsource);

private:
    liblas::Index *m_index;
    liblas::VariableRecord m_indexVLRHeaderRecord, m_indexVLRCellRecord;
    IndexVLRData m_indexVLRHeaderData, m_indexVLRCellPointData, m_indexVLRTempData;
    uint32_t m_VLRCommonDataSize, m_VLRDataSizeLocation, m_FirstCellLocation, m_LastCellLocation, m_VLRPointCountLocation;
    uint32_t  m_DataRecordSize, m_TempWritePos, m_DataPointsThisVLR;
    bool m_FirstCellInVLR, m_SomeDataReadyToWrite;
    
protected:
    bool InitiateOutput(void);
    bool OutputCell(liblas::detail::IndexCell *CellBlock, uint32_t CurCellX, uint32_t CurCellY);
    bool InitializeVLRData(uint32_t CurCellX, uint32_t CurCellY);
    bool FinalizeOutput(void);
    
};

template <typename T, typename Q>
inline void WriteVLRData_n(IndexVLRData& dest, T& src, Q& pos)
{
    // Fix little-endian
    LIBLAS_SWAP_BYTES_N(src, sizeof(T));
    // error if writing past array end
    if (static_cast<size_t>(pos) + sizeof(T) > dest.size())
        dest.resize(dest.size() + (std::numeric_limits<unsigned short>::max)());
    // copy sizeof(T) bytes to destination
    memcpy(&dest[pos], &src, sizeof(T));
    // increment the write position to end of written data
    pos += sizeof(T);
}

template <typename T, typename Q>
inline void WriteVLRDataNoInc_n(IndexVLRData& dest, T& src, Q const& pos)
{
    // Fix little-endian
    LIBLAS_SWAP_BYTES_N(src, sizeof(T));
    // error if writing past array end
    if (static_cast<size_t>(pos) + sizeof(T) > dest.size())
        dest.resize(dest.size() + (std::numeric_limits<unsigned short>::max)());
    // copy sizeof(T) bytes to destination
    memcpy(&dest[pos], &src, sizeof(T));
}

template <typename T, typename Q>
inline void WriteVLRData_str(IndexVLRData& dest, char * const src, T const srclen, Q& pos)
{
    // copy srclen bytes to destination
    std::memcpy(&dest[pos], src, srclen);
    // error if writing past array end
    if (static_cast<size_t>(pos) + static_cast<size_t>(srclen) > dest.size())
        dest.resize(dest.size() + (std::numeric_limits<unsigned short>::max)());
    // increment the write position to end of written data
    pos += srclen;
}

template <typename T, typename Q>
inline void WriteVLRDataNoInc_str(IndexVLRData& dest, char * const src, T const srclen, Q pos)
{
    // error if writing past array end
    if (static_cast<size_t>(pos) + static_cast<size_t>(srclen) > dest.size())
        dest.resize(dest.size() + (std::numeric_limits<unsigned short>::max)());
    // copy srclen bytes to destination
    memcpy(&dest[pos], src, srclen);
}

}} // namespace liblas::detail

#endif // LIBLAS_DETAIL_INDEXOUTPUT_HPP_INCLUDED
