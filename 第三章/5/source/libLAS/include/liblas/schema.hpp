/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS Schema implementation for C++ libLAS 
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

#ifndef LIBLAS_SCHEMA_HPP_INCLUDED
#define LIBLAS_SCHEMA_HPP_INCLUDED

#include <liblas/version.hpp>
#include <liblas/external/property_tree/ptree.hpp>
#include <liblas/variablerecord.hpp>
#include <liblas/dimension.hpp>
#include <liblas/export.hpp>

// boost
#include <boost/any.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/array.hpp>
#include <boost/optional.hpp>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/sequenced_index.hpp>
#include <boost/multi_index/mem_fun.hpp>
#include <boost/multi_index/random_access_index.hpp>

// std
#include <iosfwd>
#include <limits>
#include <string>
#include <vector>
#include <algorithm>
#include <boost/unordered_map.hpp>

namespace liblas {  

typedef std::vector<Dimension> DimensionArray;

struct name{};
struct position{};
struct index{};


typedef boost::multi_index::multi_index_container<
  Dimension,
  boost::multi_index::indexed_by<
    // sort by Dimension::operator<
    boost::multi_index::ordered_unique<boost::multi_index::tag<position>, boost::multi_index::identity<Dimension> >,
    
    // Random access
    boost::multi_index::random_access<boost::multi_index::tag<index> >,
    // sort by less<string> on GetName
    boost::multi_index::hashed_unique<boost::multi_index::tag<name>, boost::multi_index::const_mem_fun<Dimension,std::string const&,&Dimension::GetName> >
      >
> IndexMap;

typedef IndexMap::index<name>::type index_by_name;
typedef IndexMap::index<position>::type index_by_position;
typedef IndexMap::index<index>::type index_by_index;


/// Schema definition
class LAS_DLL Schema
{
public:
    
    // Schema();
    Schema(PointFormatName data_format_id);
    Schema(std::vector<VariableRecord> const& vlrs);
    Schema& operator=(Schema const& rhs);

    bool operator==(const Schema& other) const;
    bool operator!=(const Schema& other) const { return !(*this == other); }
        
    Schema(Schema const& other);
    
    ~Schema() {}

    /// Fetch byte size
    std::size_t GetByteSize() const;

    std::size_t GetBitSize() const;
    void CalculateSizes();

    /// Get the base size (only accounting for Time, Color, etc )
    std::size_t GetBaseByteSize() const;

    PointFormatName GetDataFormatId() const { return m_data_format_id; }
    void SetDataFormatId(PointFormatName const& value);
    
    void AddDimension(Dimension const& dim);
    boost::optional< Dimension const& > GetDimension(std::string const& n) const;
    boost::optional< Dimension const& > GetDimension(index_by_index::size_type t) const;
    
    // DimensionPtr GetDimension(std::size_t index) const;
    void RemoveDimension(Dimension const& dim);
    
    void SetDimension(Dimension const& dim);
    
    std::vector<std::string> GetDimensionNames() const;
    IndexMap const& GetDimensions() const { return m_index; }
    liblas::property_tree::ptree GetPTree() const;
    
    uint16_t GetSchemaVersion() const { return m_schemaversion; }
    void SetSchemaVersion(uint16_t v) { m_schemaversion = v; }
    
    bool IsCustom() const;
    VariableRecord GetVLR() const;

protected:
    
    PointFormatName m_data_format_id;
    uint32_t m_nextpos;
    std::size_t m_bit_size;
    std::size_t m_base_bit_size;
    uint16_t m_schemaversion;
    
private:

    IndexMap m_index;
    
    void add_record0_dimensions();
    void add_time();
    void add_color();
    void update_required_dimensions(PointFormatName data_format_id);
    bool IsSchemaVLR(VariableRecord const& vlr);
    liblas::property_tree::ptree LoadPTree(VariableRecord const& v);
    IndexMap LoadDimensions(liblas::property_tree::ptree tree);

};

bool inline sort_dimensions(Dimension i, Dimension j) 
{ 
    return i < j; 
}

LAS_DLL std::ostream& operator<<(std::ostream& os, liblas::Schema const&);


} // namespace liblas

#endif // LIBLAS_SCHEMA_HPP_INCLUDED
