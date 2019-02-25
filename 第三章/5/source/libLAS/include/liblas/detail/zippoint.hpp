/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  laszip helper functions for C++ libLAS 
 * Author:   Michael P. Gerlek (mpg@flaxen.com)
 *
 ******************************************************************************
 * Copyright (c) 2010, Michael P. Gerlek
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

#ifndef LIBLAS_DETAIL_ZIPPOINT_HPP_INCLUDED
#define LIBLAS_DETAIL_ZIPPOINT_HPP_INCLUDED


#include <liblas/detail/fwd.hpp>
#include <liblas/liblas.hpp>

// boost
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>

// liblaszip
class LASzipper;
class LASzip;

namespace liblas { namespace detail { 

class ZipPoint
{
public:
    ZipPoint(PointFormatName, const std::vector<VariableRecord>& vlrs);
    ~ZipPoint();

    void ConstructVLR(VariableRecord&) const;

    // these will return false iff we find a laszip VLR and it doesn't match
    // the point format this object wasd constructed with
    bool ValidateVLR(std::vector<VariableRecord> const& vlrs) const;
    bool ValidateVLR(const VariableRecord& vlr) const;
    
    bool IsZipVLR(const VariableRecord& vlr) const;
    
    LASzip* GetZipper() const { return m_zip.get(); }

private:
    void ConstructItems();

public: // for now
    // LASzip::pack() allocates/sets vlr_data and vlr_num for us, and deletes it for us  ["his"]
    // LASzip::unpack() just reads from the vlr_data we give it (we allocate and delete)  ["our"]
    int his_vlr_num;
    unsigned char* his_vlr_data;

    boost::scoped_ptr<LASzip> m_zip;

    unsigned char** m_lz_point;
    boost::scoped_array<uint8_t> m_lz_point_data;
    unsigned int m_lz_point_size;
};

}} // namespace liblas::detail


#endif // LIBLAS_DETAIL_ZIPPOINT_HPP_INCLUDED
