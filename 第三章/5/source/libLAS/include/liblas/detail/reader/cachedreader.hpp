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

#ifndef LIBLAS_DETAIL_CACHEDREADERIMPL_HPP_INCLUDED
#define LIBLAS_DETAIL_CACHEDREADERIMPL_HPP_INCLUDED

#include <liblas/detail/fwd.hpp>
#include <liblas/detail/reader/header.hpp>
#include <liblas/liblas.hpp>

// std
#include <iosfwd>
#include <boost/shared_ptr.hpp>

namespace liblas { namespace detail { 


class CachedReaderImpl : public ReaderImpl
{
public:

    CachedReaderImpl(std::istream& ifs, std::size_t cache_size);

    void ReadHeader();
    void ReadNextPoint();
    liblas::Point const& ReadPointAt(std::size_t n);

    void Seek(std::size_t n);
    void Reset();
    void SetFilters(std::vector<liblas::FilterPtr> const& filters);
    std::vector<liblas::FilterPtr> GetFilters() const;

    void SetTransforms(std::vector<liblas::TransformPtr> const& transforms);
    std::vector<liblas::TransformPtr> GetTransforms() const;

protected:

private:

    // Blocked copying operations, declared but not defined.
    CachedReaderImpl(CachedReaderImpl const& other);
    CachedReaderImpl& operator=(CachedReaderImpl const& rhs);
    void ReadCachedPoint(uint32_t position);
    
    void CacheData(uint32_t position);
    void ReadNextUncachedPoint();
    
    typedef std::vector<uint8_t> cache_mask_type;

    cache_mask_type m_mask;
    cache_mask_type::size_type m_cache_size;    
    cache_mask_type::size_type m_cache_start_position;
    cache_mask_type::size_type m_cache_read_position;

    typedef std::vector<liblas::Point*> cache_type;
    cache_type m_cache;
    bool m_cache_initialized;
};


}} // namespace liblas::detail

#endif // LIBLAS_DETAIL_CACHEDREADERIMPL_HPP_INCLUDED
