/******************************************************************************
* $Id$
*
* Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
* Purpose:  LAS reader class 
* Author:   Mateusz Loskot, mateusz@loskot.net
*
******************************************************************************
* Copyright (c) 2008, Mateusz Loskot
* Copyright (c) 2008, Phil Vachon
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

#ifndef LIBLAS_LASREADER_HPP_INCLUDED
#define LIBLAS_LASREADER_HPP_INCLUDED

#include <liblas/header.hpp>
#include <liblas/point.hpp>
#include <liblas/variablerecord.hpp>
#include <liblas/spatialreference.hpp>
#include <liblas/transform.hpp>
#include <liblas/filter.hpp>
#include <liblas/export.hpp>
// boost
#include <boost/cstdint.hpp>
// std
#include <cstddef>
#include <iosfwd>
#include <memory>
#include <string>
#include <vector>

namespace liblas {


/// Defines public interface to LAS reader implementation.
class LAS_DLL Reader
{
public:

    /// Consructor initializes reader with input stream as source of LAS records.
    /// @param ifs - stream used as source of LAS records.
    /// @exception std::runtime_error - on failure state of the input stream.
    Reader(std::istream& ifs);
    Reader(ReaderIPtr reader);

    Reader(Reader const& other);
    Reader& operator=(Reader const& rhs);    
    
    
    /// Destructor.
    /// @exception nothrow
    ~Reader();
    
    /// Provides read-only access to header of LAS file being read.
    /// @exception nothrow
    Header const& GetHeader() const;
    
    void SetHeader(Header const& );

    /// Provides read-only access to current point record.
    /// @exception nothrow
    Point const& GetPoint() const;

    /// Fetches next point record in file.
    /// @exception may throw std::exception
    bool ReadNextPoint();

    /// Fetches n-th point record from file.
    /// @exception may throw std::exception
    bool ReadPointAt(std::size_t n);

    /// Reinitializes state of the reader.
    /// @exception may throw std::exception
    void Reset();

    /// Move to the specified point to start 
    /// ReadNextPoint operations
    /// @exception may throw std::exception
    bool Seek(std::size_t n);

    /// Provides index-based access to point records.
    /// The operator is implemented in terms of ReadPointAt method
    /// and is not const-qualified because it updates file stream position.
    /// @exception may throw std::exception
    Point const& operator[](std::size_t n);
    
    /// Sets filters that are used to determine whether or not to 
    /// keep a point that was read from the file.  Filters have *no* 
    /// effect for reading data at specific locations in the file.  
    /// They only affect reading ReadNextPoint-style operations
    /// Filters are applied *before* transforms.
    void SetFilters(std::vector<liblas::FilterPtr> const& filters);
    
    /// Gets the list of filters to be applied to points as they are read
    std::vector<liblas::FilterPtr> GetFilters() const;

    /// Sets transforms to apply to points.  Points are transformed in 
    /// place *in the order* of the transform list.
    /// Filters are applied *before* transforms.  
    void SetTransforms(std::vector<liblas::TransformPtr> const& transforms);

    /// Gets the list of transforms to be applied to points as they are read
    std::vector<liblas::TransformPtr> GetTransforms() const;

private:


    void Init(); // throws on error

    ReaderIPtr m_pimpl;


};

} // namespace liblas

#endif // ndef LIBLAS_LASREADER_HPP_INCLUDED
