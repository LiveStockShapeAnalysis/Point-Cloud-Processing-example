/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  Point Writer implementation for C++ libLAS 
 * Author:   Howard Butler, hobu.inc@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2010, Howard Butler
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
 
#ifndef LIBLAS_DETAIL_WRITER_POINT_HPP_INCLUDED
#define LIBLAS_DETAIL_WRITER_POINT_HPP_INCLUDED

#include <liblas/point.hpp>
#include <liblas/schema.hpp>
#include <liblas/header.hpp>
#include <liblas/detail/private_utility.hpp>
#include <liblas/detail/fwd.hpp>

// std
#include <iosfwd>
#include <vector>

namespace liblas { namespace detail { namespace writer {

class Point 
{
public:
    
    Point(std::ostream& ofs, uint32_t& count, HeaderPtr header);
    virtual ~Point();

    // const liblas::Point& GetPoint() const { return m_point; }
    void write( const liblas::Point& );
    
protected:

    typedef std::ostream::off_type off_type;
    typedef std::ostream::pos_type pos_type;
        
private:

    // Blocked copying operations, declared but not defined.
    Point(Point const& other);
    Point& operator=(Point const& rhs);
    
    std::ostream& m_ofs;
    HeaderPtr m_header;
    
    Schema m_format;
    
    std::vector<uint8_t> m_blanks; 

    void setup();
    uint32_t& m_pointCount;
    // void fill();
};

}}} // namespace liblas::detail::writer

#endif // LIBLAS_DETAIL_WRITER_POINT_HPP_INCLUDED
