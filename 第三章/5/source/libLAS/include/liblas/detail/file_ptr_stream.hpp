/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  Stream wrapper for FILE pointer.
 * Author:   Mateusz Loskot <mateusz@loskot.net>
 *
 ******************************************************************************
 * Copyright (c) 2010, Mateusz Loskot
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

#ifndef LIBLAS_DETAIL_FILE_PTR_STREAM_HPP_INCLUDED
#define LIBLAS_DETAIL_FILE_PTR_STREAM_HPP_INCLUDED

#include <cstdio>
#include <istream>
#include <ostream>
#include <streambuf>

namespace liblas { namespace detail {

class file_ptr_streambuf : public std::streambuf
{
public:

    file_ptr_streambuf(FILE* fp) : std::streambuf() , fp(fp) {}

protected:

    virtual int overflow(int c)
    {
        return EOF != c ? std::fputc(c, fp) : EOF;
    }

    virtual int underflow()
    {
        int const c = std::getc(fp);
        if (c != EOF)
        {
            std::ungetc(c, fp);
        }
        return c;
    }

    virtual int uflow()
    {
        return std::getc(fp);
    }

    virtual int pbackfail(int c)
    {
        return EOF != c ? std::ungetc(c, fp) : EOF;
    }

    virtual int sync()
    {
        return std::fflush(fp);
    }

private:
    FILE* fp;
};

#pragma warning(push)
#pragma warning(disable: 4355)

class file_ptr_istream : private file_ptr_streambuf, public std::istream
{
public:
    explicit file_ptr_istream(FILE* fp) : file_ptr_streambuf(fp), std::istream(this) {}
};

class file_ptr_ostream : private file_ptr_streambuf, public std::ostream
{
public:
    explicit file_ptr_ostream(FILE* fp) : file_ptr_streambuf(fp), std::ostream(this) {}
};

#pragma warning(pop)

}} //namespace liblas::detail

#endif // LIBLAS_DETAIL_FILE_PTR_STREAM_HPP_INCLUDED