/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS 1.0 reader implementation for C++ libLAS 
 * Author:   Mateusz Loskot, mateusz@loskot.net
 *
 ******************************************************************************
 * Copyright (c) 2008, Mateusz Loskot
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

#ifndef LIBLAS_DETAIL_READERIMPL_HPP_INCLUDED
#define LIBLAS_DETAIL_READERIMPL_HPP_INCLUDED

#include <liblas/detail/fwd.hpp>
#include <liblas/detail/reader/header.hpp>
#include <liblas/liblas.hpp>
// std
#include <iosfwd>
#include <boost/shared_ptr.hpp>

namespace liblas { namespace detail { 

// typedef boost::shared_ptr< reader::Point > PointReaderPtr;
typedef boost::shared_ptr< reader::Header > HeaderReaderPtr;

class ReaderImpl : public ReaderI
{
public:

    ReaderImpl(std::istream& ifs);
    ~ReaderImpl();

    void ReadHeader();
    liblas::Header const& GetHeader() const {return *m_header;}
    void SetHeader(liblas::Header const& header);
    liblas::Point const& GetPoint() const { return *m_point; }
    void ReadNextPoint();
    liblas::Point const& ReadPointAt(std::size_t n);
    void Seek(std::size_t n);
    
    void Reset();

    void SetFilters(std::vector<liblas::FilterPtr> const& filters);
    std::vector<liblas::FilterPtr> GetFilters() const;

    void SetTransforms(std::vector<liblas::TransformPtr> const& transforms);
    std::vector<liblas::TransformPtr> GetTransforms() const;


protected:

    bool FilterPoint(liblas::Point const& p);
    void TransformPoint(liblas::Point& p);

    typedef std::istream::off_type off_type;
    typedef std::istream::pos_type pos_type;
    
    std::istream& m_ifs;
    uint32_t m_size;
    uint32_t m_current;
    
    // PointReaderPtr m_point_reader;
    HeaderReaderPtr m_header_reader;
    
    HeaderPtr m_header;
    
    PointPtr m_point;

    std::vector<liblas::FilterPtr> m_filters;
    std::vector<liblas::TransformPtr> m_transforms;
    std::vector<uint8_t>::size_type m_record_size;
    bool bNeedHeaderCheck;
    
private:

    // Blocked copying operations, declared but not defined.
    ReaderImpl(ReaderImpl const& other);
    ReaderImpl& operator=(ReaderImpl const& rhs);
};

}} // namespace liblas::detail

#endif // LIBLAS_DETAIL_READERIMPL_HPP_INCLUDED
