/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  Reader and writer iterator implementation
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

#ifndef LIBLAS_ITERATOR_HPP_INCLUDED
#define LIBLAS_ITERATOR_HPP_INCLUDED

#include <liblas/reader.hpp>
#include <liblas/writer.hpp>
#include <liblas/index.hpp>
#include <liblas/export.hpp>
#include <iterator>
#include <cassert>

namespace liblas {

/// Input iterator associated with liblas::LASReader.
/// It allows to read LAS file records in similar way to elements of STL container,
/// as well as apply STL algorithms that accept pair of input iterators.
/// \sa About Input Iterator at http://www.sgi.com/tech/stl/InputIterator.html
///
template <typename T>
class LAS_DLL reader_iterator
{
public:

    typedef std::input_iterator_tag iterator_category;
    typedef T value_type;
    typedef T const* pointer;
    typedef T const& reference;
    typedef ptrdiff_t difference_type;

    /// Initializes iterator pointing to pass-the-end.
    reader_iterator()
        : m_reader(0)
    {}

    /// Initializes iterator pointing to beginning of LAS file sequence.
    /// No ownership transfer of reader object occurs.
    reader_iterator(liblas::Reader& reader)
        : m_reader(&reader)
    {
        assert(0 != m_reader);
        getval();
    }

    /// Dereference operator.
    /// It is implemented in terms of LASReader::GetPoint function.
    reference operator*() const
    {
        assert(0 != m_reader);
        if (0 != m_reader)
        {
            return m_reader->GetPoint();
        }

        throw std::runtime_error("reader is null and iterator not dereferencable");
    }

    /// Pointer-to-member operator.
    /// It is implemented in terms of LASReader::GetPoint function.
    pointer operator->() const
    {
        return &(operator*());
    }

    /// Pre-increment opertor.
    /// Moves iterator to next record by calling LASReader::ReadNextPoint.
    reader_iterator& operator++()
    {
        assert(0 != m_reader);
        getval();
        return (*this);
    }

    /// Post-increment opertor.
    /// Moves iterator to next record by calling LASReader::ReadNextPoint.
    reader_iterator operator++(int)
    {
        reader_iterator tmp(*this);
        ++(*this);
        return tmp;
    }

    /// Compare passed iterator to this.
    /// Determine if both iterators apply to the same instance of LASReader class.
    bool equal(reader_iterator const& rhs) const
    {
        return m_reader == rhs.m_reader;
    }

private:

    void getval()
    {
        if (0 != m_reader && !(m_reader->ReadNextPoint()))
        {
            m_reader = 0;
        }
    }

    liblas::Reader* m_reader;
};

/// Equality operator implemented in terms of reader_iterator::equal
template <typename T>
bool operator==(reader_iterator<T> const& lhs, reader_iterator<T> const& rhs)
{
    return lhs.equal(rhs);
}

/// Inequality operator implemented in terms of reader_iterator::equal
template <typename T>
bool operator!=(reader_iterator<T> const& lhs, reader_iterator<T> const& rhs)
{
    return (!(lhs == rhs));
}

/// Output iterator associated with liblas::LASWriter.
/// It allows to store a sequence of records into LAS file.
/// \sa About Output Iterator at http://www.sgi.com/tech/stl/OutputIterator.html
///
template <typename T>
class LAS_DLL writer_iterator
{
public:

    typedef std::output_iterator_tag iterator_category;
    typedef void value_type;
    typedef void pointer;
    typedef T const& reference;
    typedef void difference_type;

    /// Initialize iterator with given writer.
    /// The writer position is not changed.
    /// No ownership transfer of writer object occurs.
    writer_iterator(liblas::Writer& writer)
        : m_writer(&writer)
    {
        assert(0 != m_writer);
    }

    /// Dereference assignment operator.
    /// Implemented in terms of LASWriter::WritePoint function.
    writer_iterator& operator=(reference value) 
    {
        assert(0 != m_writer);

        bool ret = false;
        ret = m_writer->WritePoint(value);
        assert(ret);

        return (*this);
    }

    /// Dereference operator.
    writer_iterator& operator*()
    {
        // pretend to return designated value
        return (*this);
    }

    /// Pre-increment operator.
    writer_iterator& operator++()
    {
        // pretend to preincrement
        return (*this);
    }

    /// Post-increment operator.
    writer_iterator operator++(int)
    {
        // pretend to postincrement
        return (*this);
    }

private:

    liblas::Writer* m_writer;
};

template <typename T>
class index_filter_iterator
{
public:

    typedef std::input_iterator_tag iterator_category;
    typedef T value_type;
    typedef T const* pointer;
    typedef T const& reference;
    typedef ptrdiff_t difference_type;

    /// Initializes iterator pointing to pass-the-end.
    index_filter_iterator()
        : m_index(0)
    {}

    /// Initializes iterator pointing to beginning of Index's filtered points sequence.
    /// No ownership transfer of index object occurs.
    index_filter_iterator(liblas::Index& index)
        : m_index(&index)
    {
        assert(0 != m_index);
        getval();
    }

    /// Dereference operator.
    /// It is implemented in terms of Index::GetNextID function.
    reference operator*() const
    {
        assert(0 != m_index);
        if (0 != m_index)
        {
            // return m_index->GetNextID();
        }

        throw std::runtime_error("index is null and iterator not dereferencable");
    }

    /// Pointer-to-member operator.
    /// It is implemented in terms of Index::GetPoint function.
    pointer operator->() const
    {
        return &(operator*());
    }

    /// Pre-increment operator.
    /// Moves iterator to next record by calling Index::GetNextID.
    index_filter_iterator& operator++()
    {
        assert(0 != m_index);
        getval();
        return (*this);
    }

    /// Post-increment operator.
    /// Moves iterator to next record by calling Index::FindNextID.
    index_filter_iterator operator++(int)
    {
        index_filter_iterator tmp(*this);
        ++(*this);
        return tmp;
    }

    /// Compare passed iterator to this.
    /// Determine if both iterators apply to the same instance of liblas::Index class.
    bool equal(index_filter_iterator const& rhs) const
    {
        return m_index == rhs.m_index;
    }

private:

    void getval()
    {
        // if (0 != m_index && !(m_index->FindNextID()))
        // {
        //     m_index = 0;
        // }
    }

    liblas::Index* m_index;
};

// Declare specializations for user's convenience

/// Public specialization of LASReader input iterator for liblas::LASPoint type.
typedef reader_iterator<Point> lasreader_iterator;

/// Public specialization of LASWriter output iterator for liblas::LASPoint type.
typedef writer_iterator<Point> laswriter_iterator;

// Needed for C++ DLL exports
#ifdef _MSC_VER
template class LAS_DLL reader_iterator<Point>;
template class LAS_DLL writer_iterator<Point>;
#endif

} // namespace liblas

#endif // LIBLAS_ITERATOR_HPP_INCLUDED
