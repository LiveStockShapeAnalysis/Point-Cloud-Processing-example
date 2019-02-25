/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS writer class 
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

#ifndef LIBLAS_LASWRITER_HPP_INCLUDED
#define LIBLAS_LASWRITER_HPP_INCLUDED

#include <liblas/version.hpp>
#include <liblas/header.hpp>
#include <liblas/point.hpp>
#include <liblas/transform.hpp>
#include <liblas/filter.hpp>
#include <liblas/export.hpp>
// boost
#include <boost/shared_ptr.hpp>
// std
#include <iosfwd> // std::ostream
#include <string>
#include <memory>
#include <cstdlib> // std::size_t

namespace liblas {

/// Defines public interface to LAS writer implementation.
/// This class 
class LAS_DLL Writer
{
public:

    /// Consructor initializes reader with specified output stream and header specification.
    /// @param ofs - stream used as destination for LAS records.
    /// @param header - specifies obligatory properties of LAS file.
    /// @exception std::runtime_error - on failure state of the input stream.
    Writer(std::ostream& ofs, Header const& header);

    Writer(Writer const& other);
    Writer(WriterIPtr ptr);
    
    Writer& operator=(Writer const& rhs);    
    
    /// Destructor does not close file attached to the output stream
    /// Header may be updated after writing operation completed, if necessary
    /// in order to maintain data consistency.
    ~Writer();
    
    /// Provides access to header structure.
    Header const& GetHeader() const;
    
    void SetHeader(Header const& header);
    

    /// \todo TODO: Move point record composition deep into writer implementation.
    /// \todo TODO: How to handle point_source_id in portable way, for LAS 1.0 and 1.1
    bool WritePoint(Point const& point);

    /// Allow in-place writing of header
    void WriteHeader();

    /// Sets filters that are used to determine wither or not to 
    /// keep a point that before we write it
    /// Filters are applied *before* transforms.
    void SetFilters(std::vector<liblas::FilterPtr> const& filters);

    /// Gets the list of filters to be applied to points as they are written
    std::vector<liblas::FilterPtr> GetFilters() const;
    
    /// Sets transforms to apply to points.  Points are transformed in 
    /// place *in the order* of the transform list.
    /// Filters are applied *before* transforms.  
    void SetTransforms(std::vector<liblas::TransformPtr> const& transforms);

    /// Gets the list of transforms to be applied to points as they are read
    std::vector<liblas::TransformPtr> GetTransforms() const;
    
private:

    WriterIPtr m_pimpl;

};

} // namespace liblas

#endif // ndef LIBLAS_LASWRITER_HPP_INCLUDED
