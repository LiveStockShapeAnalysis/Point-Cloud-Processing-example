/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS point class 
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

#ifndef LIBLAS_LASPOINT_HPP_INCLUDED
#define LIBLAS_LASPOINT_HPP_INCLUDED

#include <liblas/classification.hpp>
#include <liblas/color.hpp>
#include <liblas/schema.hpp>
#include <liblas/detail/pointrecord.hpp>
#include <liblas/detail/fwd.hpp>
#include <liblas/detail/private_utility.hpp>
#include <liblas/external/property_tree/ptree.hpp>
#include <liblas/export.hpp>

// boost
#include <boost/array.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/any.hpp>
// std
#include <stdexcept> // std::out_of_range
#include <cstdlib> // std::size_t
#include <vector> // std::vector


namespace liblas {


/// Point data record composed with X, Y, Z coordinates and attributes.
class LAS_DLL Point
{
public:

    enum DataMemberFlag
    {
        eReturnNumber = 1,
        eNumberOfReturns = 2,
        eScanDirection = 4,
        eFlightLineEdge = 8,
        eClassification = 16,
        eScanAngleRank = 32,
        eTime = 64
    };

    enum ClassificationType
    {
        eCreated = 0,
        eUnclassified,
        eGround,
        eLowVegetation,
        eMediumVegetation,
        eHighVegetation,
        eBuilding,
        eLowPoint,
        eModelKeyPoint,
        eWater = 9,
        // = 10 // reserved for ASPRS Definition
        // = 11 // reserved for ASPRS Definition
        eOverlapPoints = 12
        // = 13-31 // reserved for ASPRS Definition
    };

    enum ScanAngleRankRange
    {
        eScanAngleRankMin = -90,
        eScanAngleRankMax = 90
    };

    // Point();
    ~Point() {}
    Point(Header const* header);
    Point(Point const& other);
    Point& operator=(Point const& rhs);

    double GetX() const;
    double GetY() const;
    double GetZ() const;
    
    int32_t GetRawX() const;
    int32_t GetRawY() const;
    int32_t GetRawZ() const;
        
    void SetCoordinates(double const& x, double const& y, double const& z);
    
    void SetX(double const& value);
    void SetY(double const& value);
    void SetZ(double const& value);

    void SetRawX(int32_t const& value);
    void SetRawY(int32_t const& value);
    void SetRawZ(int32_t const& value);
    
    uint16_t GetIntensity() const;
    void SetIntensity(uint16_t const& intensity);

    /// Gets all scanning flags encoded as single byte.
    /// The flags are (mandatory):
    /// - Return Number (bits 0, 1, 2);
    /// - Number of Returns - given pulse (bits 3, 4, 5);
    /// - Scan Direction Flag (bit 6);
    /// - Edge of Flight Line (bit 7).
    uint8_t GetScanFlags() const;

    /// Sets all scanning flags passed as a single byte.
    /// \sa Documentation of GetScanFlags method for flags details.
    void SetScanFlags(uint8_t const& flags);
    
    uint16_t GetReturnNumber() const;
    void SetReturnNumber(uint16_t const& num);

    uint16_t GetNumberOfReturns() const;
    void SetNumberOfReturns(uint16_t const& num);

    uint16_t GetScanDirection() const;
    void SetScanDirection(uint16_t const& dir);
    
    uint16_t GetFlightLineEdge() const;
    void SetFlightLineEdge(uint16_t const& edge);

    //Classification& GetClassification();
    Classification GetClassification() const;
    void SetClassification(Classification const& cls);
    void SetClassification(Classification::bitset_type const& flags);
    void SetClassification(uint8_t const& flags);

    int8_t GetScanAngleRank() const;
    void SetScanAngleRank(int8_t const& rank);

    /// Fetch value of File Marker (LAS 1.0) or User Data (LAS 1.1).
    uint8_t GetUserData() const;

    /// Set value of File Marker (LAS 1.0) or User Data (LAS 1.1).
    void SetUserData(uint8_t const& data);

    /// Fetch value of User Bit Field (LAS 1.0) or Point Source ID (LAS 1.1).
    uint16_t GetPointSourceID() const;

    /// Set value of User Bit Field (LAS 1.0) or Point Source ID (LAS 1.1).
    void SetPointSourceID(uint16_t const& id);

    /// Fetch color value associated with this point (LAS 1.2)
    Color GetColor() const;

    /// Set color value associated with this point (LAS 1.2)
    void SetColor(Color const& value);

    double GetTime() const;
    void SetTime(double const& time);

    /// Const version of index operator providing access to XYZ coordinates of point record.
    /// Valid index values are 0, 1 or 2.
    /// \exception std::out_of_range if requested index is out of range (> 2).
    double operator[](std::size_t const& index) const;

    /// \todo TODO: Should we compare other data members, but not only coordinates?
    bool equal(Point const& other) const;

    bool Validate() const;
    bool IsValid() const;


    std::vector<uint8_t> const& GetData() const {return m_data; }
    std::vector<uint8_t> & GetData() {return m_data; }
    void SetData(std::vector<uint8_t> const& v) { m_data = v;}
    
    void SetHeader(Header const* header); 
    Header const* GetHeader() const;
    property_tree::ptree GetPTree() const;    
    boost::any GetValue(Dimension const& d) const;

private:

    std::vector<uint8_t> m_data;
    
    std::vector<uint8_t>::size_type GetDimensionBytePosition(std::size_t dim_pos) const;
    Header const* m_header;
    Header const& m_default_header;

};

/// Equal-to operator implemented in terms of Point::equal method.
inline bool operator==(Point const& lhs, Point const& rhs)
{
    return lhs.equal(rhs);
}

/// Not-equal-to operator implemented in terms of Point::equal method.
inline bool operator!=(Point const& lhs, Point const& rhs)
{
    return (!(lhs == rhs));
}

inline double Point::operator[](std::size_t const& index) const
{
    
    if (index == 0) 
        return GetX();
    if (index == 1) 
        return GetY();
    if (index == 2)
        return GetZ();

    throw std::out_of_range("coordinate subscript out of range");
    
}


LAS_DLL std::ostream& operator<<(std::ostream& os, liblas::Point const&);


} // namespace liblas

#endif // LIBLAS_LASPOINT_HPP_INCLUDED
