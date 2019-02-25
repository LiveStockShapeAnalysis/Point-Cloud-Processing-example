/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS header class 
 * Author:   Mateusz Loskot, mateusz@loskot.net
 *
 ******************************************************************************
 * Copyright (c) 2010, Mateusz Loskot
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

#ifndef LIBLAS_LASHEADER_HPP_INCLUDED
#define LIBLAS_LASHEADER_HPP_INCLUDED

#include <boost/uuid/uuid.hpp>
#include <liblas/bounds.hpp>
#include <liblas/schema.hpp>
#include <liblas/spatialreference.hpp>
#include <liblas/variablerecord.hpp>
#include <liblas/version.hpp>
#include <liblas/external/property_tree/ptree.hpp>
#include <liblas/export.hpp>
#include <liblas/detail/singleton.hpp>
// boost
#include <boost/foreach.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

//std
#include <cstddef>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>

namespace liblas {

/// Definition of public header block.
/// The header contains set of generic data and metadata
/// describing a family of ASPRS LAS files. The header is stored
/// at the beginning of every valid ASPRS LAS file.
///
/// \todo  TODO (low-priority): replace static-size char arrays as data members
///        with std::string and return const-reference to string object.
///
class LAS_DLL Header
{
public:

    /// Official signature of ASPRS LAS file format, always \b "LASF".
    static char const* const FileSignature;

    /// Default system identifier used by libLAS, always \b "libLAS".
    static char const* const SystemIdentifier;

    /// Default software identifier used by libLAS, always \b "libLAS X.Y".
    static char const* const SoftwareIdentifier;

    /// Array of 5 elements - numbers of points recorded by each return.
    /// \todo TODO: Consider replacing with {boost|std::tr1}::array<T, 5> --mloskot
    typedef std::vector<uint32_t> RecordsByReturnArray;

    /// Default constructor.
    /// The default constructed header is configured according to the ASPRS
    /// LAS 1.2 Specification, point data format set to 0.
    /// Other fields filled with 0.
    Header();

    /// Copy constructor.
    Header(Header const& other);

    /// Assignment operator.
    Header& operator=(Header const& rhs);
    
    /// Comparison operator.
    bool operator==(const Header& other) const;

    /// Get ASPRS LAS file signature.
    /// \return 4-characters long string - \b "LASF".
    std::string GetFileSignature() const;

    /// Set ASPRS LAS file signature.
    /// The only value allowed as file signature is \b "LASF",
    /// defined as FileSignature constant.
    /// \exception std::invalid_argument - if invalid signature given.
    /// \param v - string contains file signature, at least 4-bytes long
    /// with "LASF" as first four bytes.
    void SetFileSignature(std::string const& v);

    /// Get file source identifier.
    /// \exception No throw
    uint16_t GetFileSourceId() const;

    /// Set file source identifier.
    /// \param v - should be set to a value between 1 and 65535.
    /// \exception No throw
    ///
    /// \todo TODO: Should we warn or throw about type overflow when user passes 65535 + 1 = 0
    void SetFileSourceId(uint16_t v);

    /// Get value field reserved by the ASPRS LAS Specification.
    /// \note This field is always filled with 0.
    ///
    /// \todo TODO: Should we warn or throw about type overflow when user passes 65535 + 1 = 0
    uint16_t GetReserved() const;

    /// Set reserved value for the header identifier.
    /// \param v - should be set to a value between 1 and 65535.
    /// \exception No throw
    void SetReserved(uint16_t v);

    /// Get project identifier.
    /// \return Global Unique Identifier as an instance of boost::uuid::uuid class.
    boost::uuids::uuid GetProjectId() const;

    /// Set project identifier.
    void SetProjectId(boost::uuids::uuid const& v);

    /// Get major component of version of LAS format.
    /// \return Always 1 is returned as the only valid value.
    uint8_t GetVersionMajor() const;

    /// Set major component of version of LAS format.
    /// \exception std::out_of_range - invalid value given.
    /// \param v - value between eVersionMajorMin and eVersionMajorMax.
    void SetVersionMajor(uint8_t v);

    /// Get minor component of version of LAS format.
    /// \return Valid values are 0, 1, 2, 3.
    uint8_t GetVersionMinor() const;

    /// Set minor component of version of LAS format.
    /// \exception std::out_of_range - invalid value given.
    /// \param v - value between eVersionMinorMin and eVersionMinorMax.
    void SetVersionMinor(uint8_t v);

    /// Get system identifier.
    /// Default value is \b "libLAS" specified as the SystemIdentifier constant.
    /// \param pad - if true the returned string is padded right with spaces and
    /// its length is 32 bytes, if false (default) no padding occurs and
    /// length of the returned string is <= 32 bytes.
    /// \return value of system identifier field.
    std::string GetSystemId(bool pad = false) const;

    /// Set system identifier.
    /// \exception std::invalid_argument - if identifier longer than 32 bytes.
    /// \param v - system identifiers string.
    void SetSystemId(std::string const& v);

    /// Get software identifier.
    /// Default value is \b "libLAS 1.0", specified as the SoftwareIdentifier constant.
    /// \param pad - if true the returned string is padded right with spaces and its length is 32 bytes,
    /// if false (default) no padding occurs and length of the returned string is <= 32 bytes.
    /// \return value of generating software identifier field.
    std::string GetSoftwareId(bool pad = false) const;

    /// Set software identifier.
    /// \exception std::invalid_argument - if identifier is longer than 32 bytes.
    /// \param v - software identifiers string.
    void SetSoftwareId(std::string const& v);

    /// Get day of year of file creation date.
    /// \todo TODO: Use full date structure instead of Julian date number.
    uint16_t GetCreationDOY() const;

    /// Set day of year of file creation date.
    /// \exception std::out_of_range - given value is higher than number 366.
    /// \todo TODO: Use full date structure instead of Julian date number.
    void SetCreationDOY(uint16_t v);

    /// Set year of file creation date.
    /// \todo TODO: Remove if full date structure is used.
    uint16_t GetCreationYear() const;

    /// Get year of file creation date.
    /// \exception std::out_of_range - given value is higher than number 9999.
    /// \todo TODO: Remove if full date structure is used.
    void SetCreationYear(uint16_t v);

    /// Get number of bytes of generic verion of public header block storage.
    /// Standard version of the public header block is 227 bytes long.
    uint16_t GetHeaderSize() const;

    /// Sets the header size.  Note that this is not the same as the offset to 
    /// point data. 
    void SetHeaderSize(uint16_t v);
    
    /// Get number of bytes from the beginning to the first point record.
    uint32_t GetDataOffset() const;

    /// Set number of bytes from the beginning to the first point record.
    /// \exception std::out_of_range - if given offset is bigger than 227+2 bytes
    /// for the LAS 1.0 format and 227 bytes for the LAS 1.1 format.
    void SetDataOffset(uint32_t v);

    /// Get number of bytes from the end of the VLRs to the GetDataOffset.
    uint32_t GetHeaderPadding() const;

    /// Set the number of bytes from the end of the VLRs in the header to the 
    /// beginning of point data.
    /// \exception std::out_of_range - if given offset is bigger than 227+2 bytes
    /// for the LAS 1.0 format and 227 bytes for the LAS 1.1 format.
    void SetHeaderPadding(uint32_t v);

    /// Get number of variable-length records.
    uint32_t GetRecordsCount() const;

    /// Set number of variable-length records.
    void SetRecordsCount(uint32_t v);
    
    /// Get identifier of point data (record) format.
    PointFormatName GetDataFormatId() const;

    /// Set identifier of point data (record) format.
    void SetDataFormatId(PointFormatName v);

    /// The length in bytes of each point.  All points in the file are 
    /// considered to be fixed in size, and the PointFormatName is used 
    /// to determine the fixed portion of the dimensions in the point.  Any 
    /// other byte space in the point record beyond the liblas::Schema::GetBaseByteSize() 
    /// can be used for other, optional, dimensions.  If no schema is 
    /// available for the file in the form of a liblas.org VLR schema record,
    /// These extra bytes are available via liblas::Point::GetExtraData().
    uint16_t GetDataRecordLength() const;
    
    /// Get total number of point records stored in the LAS file.
    uint32_t GetPointRecordsCount() const;

    /// Set number of point records that will be stored in a new LAS file.
    void SetPointRecordsCount(uint32_t v);
    
    /// Get array of the total point records per return.
    RecordsByReturnArray const& GetPointRecordsByReturnCount() const;

    /// Set values of 5-elements array of total point records per return.
    /// \exception std::out_of_range - if index is bigger than 4.
    /// \param index - subscript (0-4) of array element being updated.
    /// \param v - new value to assign to array element identified by index.
    void SetPointRecordsByReturnCount(std::size_t index, uint32_t v);
    
    /// Get scale factor for X coordinate.
    double GetScaleX() const;

    /// Get scale factor for Y coordinate.
    double GetScaleY() const;
    
    /// Get scale factor for Z coordinate.
    double GetScaleZ() const;

    /// Set values of scale factor for X, Y and Z coordinates.
    void SetScale(double x, double y, double z);

    /// Get X coordinate offset.
    double GetOffsetX() const;
    
    /// Get Y coordinate offset.
    double GetOffsetY() const;
    
    /// Get Z coordinate offset.
    double GetOffsetZ() const;

    /// Set values of X, Y and Z coordinates offset.
    void SetOffset(double x, double y, double z);

    /// Get minimum value of extent of X coordinate.
    double GetMaxX() const;

    /// Get maximum value of extent of X coordinate.
    double GetMinX() const;

    /// Get minimum value of extent of Y coordinate.
    double GetMaxY() const;

    /// Get maximum value of extent of Y coordinate.
    double GetMinY() const;

    /// Get minimum value of extent of Z coordinate.
    double GetMaxZ() const;

    /// Get maximum value of extent of Z coordinate.
    double GetMinZ() const;

    /// Set maximum values of extent of X, Y and Z coordinates.
    void SetMax(double x, double y, double z);

    /// Set minimum values of extent of X, Y and Z coordinates.
    void SetMin(double x, double y, double z);

    /// Adds a variable length record to the header
    void AddVLR(VariableRecord const& v);
    
    /// Returns a VLR 
    VariableRecord const& GetVLR(uint32_t index) const;
    
    /// Returns all of the VLRs
    const std::vector<VariableRecord>& GetVLRs() const;

    /// Removes a VLR from the the header.
    void DeleteVLR(uint32_t index);
    void DeleteVLRs(std::string const& name, uint16_t id);

    /// Rewrite variable-length record with georeference infomation, if available.
    void SetGeoreference();
    
    /// Fetch the georeference
    SpatialReference GetSRS() const;
    
    /// Set the georeference
    void SetSRS(SpatialReference& srs);
    
    /// Returns the schema.
    Schema const& GetSchema() const;

    /// Sets the schema
    void SetSchema(const Schema& format);

    /// Return the liblas::Bounds.  This is a 
    /// combination of the GetMax and GetMin 
    /// (or GetMinX, GetMaxY, etc) data.
    const Bounds<double>& GetExtent() const;

    /// Set the liblas::Bounds.  This is a 
    /// combination of the GetMax and GetMin 
    /// (or GetMinX, GetMaxY, etc) data, and it is equivalent to setting 
    /// all of these values.
    void SetExtent(Bounds<double> const& extent);

    /// Returns a property_tree that contains 
    /// all of the header data in a structured format.
    liblas::property_tree::ptree GetPTree() const;
    
    /// Returns true iff the file is compressed (laszip),
    /// as determined by the high bit in the point type
    bool Compressed() const;

    /// Sets whether or not the points are compressed.
    void SetCompressed(bool b);
    
    uint32_t GetVLRBlockSize() const;

    void to_rst(std::ostream& os) const;
    void to_xml(std::ostream& os) const;
    void to_json(std::ostream& os) const;
    
private:
    
    typedef detail::Point<double> PointScales;
    typedef detail::Point<double> PointOffsets;

    enum
    {
        eDataSignatureSize = 2,
        eFileSignatureSize = 4,
        ePointsByReturnSize = 7,
        eProjectId4Size = 8,
        eSystemIdSize = 32,
        eSoftwareIdSize = 32,
        eHeaderSize = 227, 
        eFileSourceIdMax = 65535
    };

    // TODO (low-priority): replace static-size char arrays
    // with std::string and return const-reference to string object.
    
    //
    // Private function members
    //
    void Init();

    //
    // Private data members
    //
    char m_signature[eFileSignatureSize]; // TODO: replace with boost::array --mloskot
    boost::uint16_t m_sourceId;
    boost::uint16_t m_reserved;
    boost::uuids::uuid m_projectGuid;
    // boost::uint32_t m_projectId1;
    // boost::uint16_t m_projectId2;
    // boost::uint16_t m_projectId3;
    // boost::uint8_t m_projectId4[eProjectId4Size];
    boost::uint8_t m_versionMajor;
    boost::uint8_t m_versionMinor;

    char m_systemId[eSystemIdSize]; // TODO: replace with boost::array --mloskot
    char m_softwareId[eSoftwareIdSize];
    uint16_t m_createDOY;
    uint16_t m_createYear;
    uint16_t m_headerSize;
    uint32_t m_dataOffset;
    uint32_t m_recordsCount;
    uint32_t m_pointRecordsCount;
    RecordsByReturnArray m_pointRecordsByReturn;
    PointScales m_scales;
    PointOffsets m_offsets;
    Bounds<double> m_extent;
    std::vector<VariableRecord> m_vlrs;
    SpatialReference m_srs;
    Schema m_schema;
    bool m_isCompressed;
    uint32_t m_headerPadding;
};

LAS_DLL std::ostream& operator<<(std::ostream& os, liblas::Header const&);

/// Singleton used for all empty points upon construction.  If 
/// a reader creates the point, the HeaderPtr from the file that was 
/// read will be used, but all stand-alone points will have EmptyHeader 
/// as their base.
class LAS_DLL DefaultHeader : public Singleton<Header>
{
public:
    ~DefaultHeader() {}


protected:
    DefaultHeader();
    DefaultHeader( DefaultHeader const&);
    DefaultHeader& operator=( DefaultHeader const&);
    
};


} // namespace liblas

#endif // LIBLAS_LASHEADER_HPP_INCLUDED
