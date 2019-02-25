/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS include file
 * Author:   Mateusz Loskot, mateusz@loskot.net
 *
 ******************************************************************************
 * Copyright (c) 2008, Mateusz Loskot
 * Copyright (c) 2008, Phil Vachon
 * Copyright (c) 2008, Howard Butler
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

#ifndef LIBLAS_HPP_INCLUDED
#define LIBLAS_HPP_INCLUDED

// liblas
#include <liblas/version.hpp>
#include <liblas/exception.hpp>
#include <liblas/iterator.hpp>
#include <liblas/bounds.hpp>
#include <liblas/classification.hpp>
#include <liblas/color.hpp>
#include <liblas/error.hpp>
#include <liblas/filter.hpp>
#include <liblas/header.hpp>
#include <liblas/point.hpp>
#include <liblas/reader.hpp>
#include <liblas/schema.hpp>
#include <liblas/spatialreference.hpp>
#include <liblas/transform.hpp>
#include <liblas/variablerecord.hpp>
#include <liblas/version.hpp>
#include <liblas/writer.hpp>
#include <liblas/utility.hpp>
#include <liblas/detail/endian.hpp>
#include <liblas/detail/private_utility.hpp>
#include <liblas/capi/las_version.h>
#include <liblas/export.hpp>
#include <liblas/factory.hpp>

// booost
#include <boost/array.hpp>
#include <boost/concept_check.hpp>

#include <boost/shared_ptr.hpp>

//#define USE_BOOST_IO
#ifdef USE_BOOST_IO
#include <ostream>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/stream_buffer.hpp>
#endif


// std
#include <cstring>
#include <fstream>
#include <string>
#include <vector>
#include <stdint.h>

/// Namespace grouping all elements of libLAS public interface.
/// \note
/// User's may notice there is namespace \em detail nested
/// in the \em liblas namespace. The \em detail should be considered as private
/// namespace dedicated for implementation details, so libLAS users are not
/// supposed to access it directly, nor included headers from the \em detail
/// subdirectory of \em liblas include folder.
namespace liblas
{

/// Open file to read in binary mode.
/// The input file is also attached to input stream.
/// \param ifs - reference to input file stream to
/// which opened file is attached
/// \param filename - name of file to open
/// \return true if file has been opened with success, false otherwise
/// \exception No throw
///
inline bool Open(std::ifstream& ifs, std::string const& filename) // throw()
{
    ifs.open(filename.c_str(), std::ios::in | std::ios::binary);
    return ifs.is_open();
}

inline std::istream* Open(std::string const& filename, std::ios::openmode mode) // throw()
{
#ifdef USE_BOOST_IO
    namespace io = boost::iostreams;
    io::stream<io::file_source>* ifs = new io::stream<io::file_source>();
    ifs->open(filename.c_str(), mode);
    if (ifs->is_open() == false) return NULL;
    return ifs;
#else
    std::ifstream* ifs = new std::ifstream();
    ifs->open(filename.c_str(), mode);
    if (ifs->is_open() == false) return NULL;
    return ifs;
#endif
}

/// Create file and open to write in binary mode.
/// The output file is also attached to output stream.
/// \param ofs - reference to output file stream to
/// which created file is attached
/// \param filename - name of file to open
/// \return true if file has been create with success, false otherwise
/// \exception No throw
///
inline bool Create(std::ofstream& ofs, std::string const& filename) // throw()
{
    ofs.open(filename.c_str(), std::ios::out | std::ios::binary);
    return ofs.is_open();
}

inline std::ostream* Create(std::string const& filename, std::ios::openmode mode)
{
#ifdef USE_BOOST_IO
    namespace io = boost::iostreams;
    io::stream<io::file_sink>* ofs = new io::stream<io::file_sink>();
    ofs->open(filename.c_str(), mode);
    if (ofs->is_open() == false) return NULL;
    return ofs;
#else
    std::ofstream* ofs = new std::ofstream();
    ofs->open(filename.c_str(), mode);
    if (ofs->is_open() == false) return NULL;
    return ofs;
#endif    
}

inline void Cleanup(std::ostream* ofs)
{
    // An ofstream is closeable and deletable, but 
    // an ostream like &std::cout isn't.
    if (!ofs) return;
#ifdef USE_BOOST_IO
    namespace io = boost::iostreams;
    namespace io = boost::iostreams;
    io::stream<io::file_sink>* source = dynamic_cast<io::stream<io::file_sink>*>(ofs);
    if (source)
    {
        source->close();
        delete ofs;
    }

#else
    std::ofstream* source = dynamic_cast<std::ofstream*>(ofs);
    if (source)
    {
        source->close();
        delete ofs;
    }
    
#endif
}

inline void Cleanup(std::istream* ifs)
{
    // An ifstream is closeable and deletable, but 
    // an istream like &std::cin isn't.
    if (!ifs) return;
#ifdef USE_BOOST_IO
    namespace io = boost::iostreams;
    io::stream<io::file_source>* source = dynamic_cast<io::stream<io::file_source>*>(ifs);
    if (source)
    {
        source->close();
        delete ifs;
    }
#else
    std::ifstream* source = dynamic_cast<std::ifstream*>(ifs);
    if (source)
    {
        source->close();
        delete ifs;
    }


#endif
}

class ReaderI
{
public:

    virtual liblas::Header const& GetHeader() const = 0;
    virtual void ReadHeader() = 0;
    virtual void SetHeader(liblas::Header const& header) = 0;
    virtual liblas::Point const& GetPoint() const = 0;
    virtual void ReadNextPoint() = 0;
    virtual Point const& ReadPointAt(std::size_t n) = 0;
    virtual void Seek(std::size_t n) = 0;
    
    virtual void Reset() = 0;
    
    virtual void SetFilters(std::vector<liblas::FilterPtr> const& filters) = 0;
    virtual void SetTransforms(std::vector<liblas::TransformPtr> const& transforms) = 0;
    
    virtual std::vector<liblas::TransformPtr> GetTransforms() const = 0;
    virtual std::vector<liblas::FilterPtr> GetFilters() const = 0;
    
    virtual ~ReaderI() {}
};

class WriterI
{
public:


    virtual liblas::Header& GetHeader() const = 0;
    virtual void WriteHeader() = 0;
    virtual void SetHeader(liblas::Header const& header) = 0;
    
    virtual void UpdatePointCount(uint32_t count) = 0;
    virtual void WritePoint(const Point& point) = 0;

    virtual void SetFilters(std::vector<liblas::FilterPtr> const& filters) = 0;
    virtual void SetTransforms(std::vector<liblas::TransformPtr> const& transforms) = 0;

    virtual std::vector<liblas::TransformPtr> GetTransforms() const = 0;
    virtual std::vector<liblas::FilterPtr> GetFilters() const = 0;


    virtual ~WriterI() {}

};

} // namespace liblas

#endif // LIBLAS_HPP_INCLUDED
