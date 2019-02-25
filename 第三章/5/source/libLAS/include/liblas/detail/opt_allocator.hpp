/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  Allocator to allow mapping allocation strategy
 * Author:   Andrew Bell, andrew.bell.ia@gmail.com
 *
 ******************************************************************************
 * Copyright (c) 2011, Andrew Bell
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

#ifndef LIBLAS_DETAIL_MAP_ALLOCATOR_HPP_INCLUDED
#define LIBLAS_DETAIL_MAP_ALLOCATOR_HPP_INCLUDED

#include <map>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>

namespace liblas {
namespace detail {

template <typename T>
class opt_allocator
{
public:
    typedef T value_type;
    typedef size_t size_type;
    typedef ptrdiff_t difference_type;
    typedef T *pointer;
    typedef T const *const_pointer;
    typedef T& reference;
    typedef T const & const_reference;

private:
    typedef std::map<pointer, boost::interprocess::mapped_region *> RegVec;
    typedef typename RegVec::iterator RegIterator;

    static RegVec m_regions;
    static size_type m_next_offset;
    static boost::interprocess::file_mapping *m_file_p;
    static size_type m_max_size;
    static bool m_initialized;

public:
    pointer address(reference r) const
        { return &r; }

    const_pointer address(const_reference r) const
        { return &r; }

    opt_allocator()
    {
        if (m_initialized && m_file_p)
        {
            throw std::bad_alloc();
        }
        m_initialized = true;
    }

    opt_allocator(const std::string& file_name)
    {
        using namespace boost::interprocess;
        using namespace std;

        if (m_initialized && !m_file_p)
        {
            throw std::bad_alloc();
        }
        m_initialized = true;
        if (!m_file_p)
        {
            // Determine file size so that we don't over-allocate.
            filebuf fbuf;
            fbuf.open(file_name.c_str(), ios_base::in);
            m_max_size = fbuf.pubseekoff(0, ios_base::end);
            fbuf.close();

            m_file_p = new file_mapping(file_name.c_str(), read_write);
        }
    }

    opt_allocator(const opt_allocator<T>& other )
    {
        boost::ignore_unused_variable_warning(other);
    }

    template <typename U>
    opt_allocator(opt_allocator<U> const &)
    {
    }

    ~opt_allocator()
    {
    }

    size_type max_size() const
    {
        size_type s = m_file_p ? m_max_size : size_type(-1);
        return s / sizeof(T);
    }

    pointer allocate(size_type num, void *hint = 0)
    {
        pointer p;
        size_t size = num * sizeof(T);

        if (m_file_p)
        {
            using namespace boost::interprocess;

            (void)hint;

            if (m_next_offset + size > m_max_size)
                throw std::bad_alloc();
            mapped_region *reg;
            reg = new mapped_region(*m_file_p, read_write, m_next_offset, size);
            m_next_offset += size;
            p = static_cast<pointer>(reg->get_address());
            m_regions[p] = reg;
        }
        else
            p = static_cast<pointer>(::operator new(size)); 
        return p;
    }

    void deallocate(pointer p, size_type num)
    {
        if (m_file_p)
        {
            RegIterator ri;
            if ((ri = m_regions.find(p)) != m_regions.end())
            {
                delete ri->second;
                m_regions.erase(ri);
            }
        }
        else
            ::operator delete(p);

        boost::ignore_unused_variable_warning(num);
    }

    void construct(pointer p, const_reference value)
    {
        new(p) T(value);
    }

    void destroy(pointer p)
    {
        p->~T();
        boost::ignore_unused_variable_warning(p);
    }

    template <typename U>
    struct rebind {
        typedef opt_allocator<U> other;
    };
};

template <typename T>
bool opt_allocator<T>::m_initialized = false;

template <typename T>
typename opt_allocator<T>::RegVec opt_allocator<T>::m_regions;

template <typename T>
typename opt_allocator<T>::size_type opt_allocator<T>::m_next_offset = 0;

template <typename T>
typename opt_allocator<T>::size_type opt_allocator<T>::m_max_size = 0;

template <typename T>
boost::interprocess::file_mapping *opt_allocator<T>::m_file_p = NULL;

} // namespace detail
} // namespace liblas

#endif // LIBLAS_DETAIL_MAP_ALLOCATOR_HPP_INCLUDED
