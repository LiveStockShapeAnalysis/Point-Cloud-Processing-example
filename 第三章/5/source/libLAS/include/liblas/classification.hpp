/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  Definition of LASClassification type.
 * Author:   Mateusz Loskot, mateusz@loskot.net
 *
 ******************************************************************************
 * Copyright (c) 2009, Mateusz Loskot
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

#ifndef LIBLAS_LASCLASSIFICATION_HPP_INCLUDED
#define LIBLAS_LASCLASSIFICATION_HPP_INCLUDED

#include <liblas/export.hpp>
// std
#include <cassert>
#include <cstddef>
#include <bitset>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <stdint.h>

// I hate you windows
#ifdef _MSC_VER
#ifdef GetClassName
#undef GetClassName
#endif
#endif

namespace liblas {

/// Class definition to manipulate properties of point record classification.
///
class LAS_DLL Classification
{
public:

    /// Alias on std::bitset<8> used as collection of flags.
    typedef std::bitset<8> bitset_type;


    /// Number of classes in lookup table as defined in ASPRS LAS 1.1+.
    /// For LAS 1.0, this static number may be invalid and
    /// extend up to 255 classes stored in variable-length records.
    /// @note Currently, libLAS does not support classification based on table
    /// stored in variable-length record. Only Standard ASPRS classification
    /// table is supported.
    static uint32_t const class_table_size;

    /// Values of indexes in the set of bit flags.
    enum BitPosition
    {
        eClassBit     = 0, ///< First bit position of 0:4 range.
        eSyntheticBit = 5, ///< Synthetic flag.
        eKeyPointBit  = 6, ///< Key-point flag.
        eWithheldBit  = 7  ///< Withheld flag.
    };

    /// Default initialization constructor.
    /// Initializes all flags of classification as set to 0.
    /// Operation semantic is equivalent to bitset_type::reset().
    Classification() {}

    /// Initializes classification flags using given set of 8 bits.
    /// @param flags [in] - contains 8 bits representing classification flags.
    explicit Classification(bitset_type const& flags)
        : m_flags(flags)
    {}

    /// Initializes classification flags using 8 bits of integral type.
    /// @param flags [in] - contains 8 bits representing classification flags.
    explicit Classification(uint8_t const& flags)
        : m_flags(flags)
    {}

    /// Initializes classification with values of given compounds.
    /// @param cls [in] - index of Standard ASPRS classification as
    /// defined in the lookup table, from 0 to class_table_size - 1.
    /// @param s [in] - If set then this point was created by a technique other than
    /// LIDAR collection such as digitized from a photogrammetric stereo model.
    /// @param k [in] - If set, this point is considered to be a model keypoint and
    /// thus generally should not be withheld in a thinning algorithm.
    /// @param w [in] - If set, this point should not be included in processing.
    Classification(uint32_t cls, bool s, bool k, bool w)
    {
        SetClass(cls);
        SetSynthetic(s);
        SetKeyPoint(k);
        SetWithheld(w);
    }

    /// Copy constructor.
    Classification(Classification const& other)
    {
        m_flags = other.m_flags;
    }

    /// Assignment operator.
    Classification& operator=(Classification const& rhs)
    {
        if (&rhs != this)
        {    
            m_flags = rhs.m_flags;
        }
        return *this;
    }

    /// Conversion operator.
    /// Returns classification object as in form of std::bitset<8>.
    operator bitset_type() const
    {
        return bitset_type(m_flags);
    }
    
    /// Named accessor converting Classification to std::bitset<8>.
    bitset_type GetFlags() const
    {
        return bitset_type(m_flags);
    }

    /// Raturns name of class as defined in LAS 1.1+
    /// Finds class name in lookup table based on class index
    /// as defined in classification object.
    std::string GetClassName() const;

    /// Returns index of ASPRS classification as defined in the lookup table.
    uint8_t GetClass() const;

    /// Updates index of ASPRS classification as defined in the lookup table.
    /// Valid index is in range from 0 to class_table_size - 1.
    /// For LAS 1.0, this static number may be invalid and
    /// extend up to 255 classes stored in variable-length records.
    /// @note Currently, libLAS does not support classification based on table
    /// stored in variable-length record. Only Standard ASPRS classification
    /// table is supported.
    /// @exception Theoretically, may throw std::out_of_range in case index 
    /// value is not in range between 0 and class_table_size - 1.
    void SetClass(uint32_t index);

    /// Sets if this point was created by a technique other than LIDAR
    /// collection such as digitized from a photogrammetric stereo model.
    void SetSynthetic(bool flag)
    {
        m_flags[eSyntheticBit] = flag;
    }

    /// Tests if this point was created by a technique other than LIDAR collection.
    bool IsSynthetic() const
    {
        return m_flags[eSyntheticBit];
    }

    /// Sets if this point is considered to be a model keypoint and
    /// thus generally should not be withheld in a thinning algorithm.
    void SetKeyPoint(bool flag)
    {
        m_flags[eKeyPointBit] = flag;
    }

    /// Tests if this point is considered to be a model keypoint.
    bool IsKeyPoint() const
    {
        return m_flags[eKeyPointBit];
    }

    /// SetTests if this point should excluded from processing.
    void SetWithheld(bool flag)
    {
        m_flags[eWithheldBit] = flag;
    }

    /// Tests if this point should excluded from processing.
    bool IsWithheld() const
    {
        return m_flags[eWithheldBit];
    }

    /// Compares this classification object with other one.
    /// Comparison is preformed against set of bit flags stored 
    /// in both objects.
    bool equal(Classification const& other) const
    {
        return (other.m_flags == m_flags);
    }

private:

    bitset_type m_flags;

    void check_class_index(uint32_t index) const;
};

/// Equal-to operator implemented in terms of Classification::equal.
inline bool operator==(Classification const& lhs, Classification const& rhs)
{
    return lhs.equal(rhs);
}

/// Not-equal-to operator implemented in terms of Classification::equal.
inline bool operator!=(Classification const& lhs, Classification const& rhs)
{
    return (!(lhs == rhs));
}

/// The output stream operator is based on std::bitset<N>::operator<<.
/// It outputs classification flags in form of string.
/// Effects promised as by 
/// @link http://www.open-std.org/Jtc1/sc22/wg21/ Standard for Programming Language C++ @endlink,
/// 23.3.5.2:
/// Each character is determined by the value of its corresponding bit
/// position in *this. Character position N - 1 corresponds to bit position
/// zero. Subsequent decreasing character positions correspond to increasing
/// bit positions. Bit value zero becomes the character 0, bit value one
/// becomes the character 1.
inline std::ostream& operator<<(std::ostream& os, Classification const& cls)
{
    Classification::bitset_type flags(cls);
    return (os << flags);
}

} // namespace liblas

#endif // LIBLAS_LASCLASSIFICATION_HPP_INCLUDED
