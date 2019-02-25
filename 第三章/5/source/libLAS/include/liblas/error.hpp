/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  An error encapsulation class
 * Author:   Mateusz Loskot, mateusz@loskot.net
 *
 ******************************************************************************
 * Copyright (c) 2008, Mateusz Loskot
 * Copyright (c) 2008, Howard Butler
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

#ifndef LIBLAS_LASERROR_HPP_INCLUDED
#define LIBLAS_LASERROR_HPP_INCLUDED

#include <liblas/export.hpp>
//std
#include <iosfwd>
#include <string>

namespace liblas {

/// Definition of error notification used on the level of C API.
/// This class describes details of error condition occured in
/// libLAS core. All errors are stacked by C API layer, so it's
/// possible to track problem down to its source.
class LAS_DLL Error
{
public:

    /// Custom constructor.
    /// This is the main and the only tool to initialize error instance.
    Error(int code, std::string const& message, std::string const& method);

    /// Copy constructor.
    /// Error objects are copyable.
    Error(Error const& other);

    /// Assignment operator.
    /// Error objects are assignable.
    Error& operator=(Error const& rhs);

    // TODO - mloskot: What about replacing string return by copy with const char* ?
    //        char const* GetMethod() const { return m_method.c_str(); }, etc.

    int GetCode() const { return m_code; }
    std::string GetMessage() const { return m_message; }
    std::string GetMethod() const { return m_method; }   

private:

    int m_code;
    std::string m_message;
    std::string m_method;
};

} // namespace liblas

#endif // LIBLAS_LASERROR_HPP_INCLUDED
