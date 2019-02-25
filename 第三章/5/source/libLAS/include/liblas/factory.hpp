/******************************************************************************
* $Id$
*
* Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
* Purpose:  LAS factories 
* Author:   Howard Butler, hobu.inc@gmail.com
*
******************************************************************************
* Copyright (c) 2008, Mateusz Loskot
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

#ifndef LIBLAS_FACTORY_HPP_INCLUDED
#define LIBLAS_FACTORY_HPP_INCLUDED

#include <liblas/reader.hpp>
#include <liblas/writer.hpp>
#include <liblas/export.hpp>


namespace liblas {


class LAS_DLL ReaderFactory
{
public:


    ReaderFactory() {}

    ReaderFactory(ReaderFactory const& other);
    ReaderFactory& operator=(ReaderFactory const& rhs);    

    Reader CreateWithImpl(ReaderIPtr r);
    
    Reader CreateCached(std::istream& stream, uint32_t cache_size);
    Reader CreateWithStream(std::istream& stream);
    
    // help function to create an input stream
    // returns NULL if failed to open
    // static std::istream* FileOpen(std::string const& filename, std::ios::openmode mode);
    // static void FileClose(std::istream*);

    /// Destructor.
    /// @exception nothrow
    ~ReaderFactory() {}


private:

};

class LAS_DLL WriterFactory
{
public:
    enum FileType
    {
        FileType_Unknown,
        FileType_LAS,
        FileType_LAZ
    };

    WriterFactory() {}

    WriterFactory(WriterFactory const& other);
    WriterFactory& operator=(WriterFactory const& rhs);    

    Writer CreateWithImpl(WriterIPtr w);

    // makes a WriterImpl or a ZipWriterImpl, depending on header type
    static WriterIPtr CreateWithStream(std::ostream& stream, Header const& header); 
    
    /// Destructor.
    /// @exception nothrow
    ~WriterFactory() {}

    // returns Unknown, unless we find a .laz or .las extension
    static FileType InferFileTypeFromExtension(const std::string&);

// 
private:

};
} // namespace liblas

#endif // ndef LIBLAS_FACTORY_HPP_INCLUDED
