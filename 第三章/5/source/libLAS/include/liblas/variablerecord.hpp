/******************************************************************************
 * $Id: lasvariablerecord.hpp 889 2008-09-28 04:17:22Z hobu $
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS record header class 
 * Author:   Phil Vachon, philippe@cowpig.ca
 *
 ******************************************************************************
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

#ifndef LIBLAS_LASVARIABLERECORD_HPP_INCLUDED
#define LIBLAS_LASVARIABLERECORD_HPP_INCLUDED

#include <liblas/detail/private_utility.hpp>
#include <liblas/external/property_tree/ptree.hpp>
#include <liblas/export.hpp>
// boost
#include <boost/array.hpp>
// std
#include <string>
#include <vector>
#include <iostream>

namespace liblas {

/// Representation of variable-length record data.
class LAS_DLL VariableRecord
{
public:

    /// Default constructor.
    /// Zero-initialization of record data.
    /// \exception No throw
    VariableRecord(); 

    /// Copy constructor.
    /// Construction of new record object as a copy of existing one.
    /// \exception No throw
    VariableRecord(VariableRecord const& other);
    
    ~VariableRecord();

    /// Assignment operator.
    /// Construction and initializition of record object by
    /// assignment of another one.
    /// \exception No throw
    VariableRecord& operator=(VariableRecord const& rhs);

    /// Get record signature (LAS 1.0) or reserved bytes (LAS 1.1).
    /// \exception No throw
    uint16_t GetReserved() const;
    
    void SetReserved(uint16_t data);

    /// Get identifier of user which created the record.
    /// The character data is up to 16 bytes long.
    /// \exception No throw
    std::string GetUserId(bool pad /*= false*/) const;
    
    void SetUserId(std::string const& id);

    /// Get identifier of record.
    /// The record ID is closely related to the user ID.
    /// \exception No throw
    uint16_t GetRecordId() const;
    
    void SetRecordId(uint16_t id);

    /// Get record length after the header.
    /// \exception No throw
    uint16_t GetRecordLength() const;
    
    void SetRecordLength(uint16_t length);

    /// Get text description of data in the record.
    /// The character data is up to 32 bytes long.
    /// \exception No throw
    std::string GetDescription(bool pad /*= false*/) const;
    
    void SetDescription(std::string const& text);

    /// Get the data for this VLR
    std::vector<uint8_t> const& GetData() const;
    
    void SetData(std::vector<uint8_t> const& data);

    /// Compare actual header object against the other.
    /// \exception No throw
    bool equal(VariableRecord const& other) const;

    /// Get the total size of the VLR in bytes
    std::size_t GetTotalSize() const;
    
    liblas::property_tree::ptree GetPTree() const;    

    enum
    {
        eUserIdSize = 16,
        eDescriptionSize = 32
    };

private:

    std::vector<uint8_t> m_data;
    boost::array<char, 32> m_description;
    boost::array<char, 16> m_user_id;    
    uint16_t m_reserved;
    uint16_t m_record_id;
    uint16_t m_record_size; // length after header
};

/// Equality operator.
/// Implemented in terms of VariableRecord::equal member function.
/// \exception No throw
inline bool operator==(VariableRecord const& lhs, VariableRecord const& rhs)
{
    return lhs.equal(rhs);
}

/// Inequality operator.
/// Implemented in terms of LASRecordHeader::equal member function.
/// \exception No throw
inline bool operator!=(VariableRecord const& lhs, VariableRecord const& rhs)
{
    return (!(lhs == rhs));
}

LAS_DLL std::ostream& operator<<(std::ostream& os, liblas::VariableRecord const&);


} // namespace liblas

#endif // LIBLAS_LASVARIABLERECORD_HPP_INCLUDED
