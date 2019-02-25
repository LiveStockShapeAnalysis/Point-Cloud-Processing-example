/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  LAS filter class 
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

#ifndef LIBLAS_LASSUMMARY_HPP_INCLUDED
#define LIBLAS_LASSUMMARY_HPP_INCLUDED

#include <liblas/header.hpp>
#include <liblas/point.hpp>
#include <liblas/detail/fwd.hpp>
#include <liblas/external/property_tree/ptree.hpp>
#include <liblas/export.hpp>
#include <liblas/filter.hpp>
// boost
#include <boost/foreach.hpp>
// std
#include <vector>
#include <functional>
#include <string>
#include <memory>

using liblas::property_tree::ptree;
typedef boost::array<uint32_t, 32> classes_type;

namespace liblas {

/// A summarization utililty for LAS points
class LAS_DLL Summary : public FilterI
{
public:
    
    Summary();
    Summary(Summary const& other);
    Summary& operator=(Summary const& rhs);
    bool filter(const Point& point);

    void AddPoint(liblas::Point const& p);
    ptree GetPTree() const;
    void SetHeader(liblas::Header const& h);
    
    ~Summary() {}
private:

    classes_type classes;
    uint32_t synthetic;
    uint32_t withheld;
    uint32_t keypoint;
    uint32_t count;
    boost::array<uint32_t, 8> points_by_return; 
    boost::array<uint32_t, 8> returns_of_given_pulse;
    bool first;
    boost::shared_ptr<liblas::Point> minimum;
    boost::shared_ptr<liblas::Point> maximum;
    liblas::Header m_header;
    bool bHaveHeader; 
    bool bHaveColor;
    bool bHaveTime;
};

class LAS_DLL CoordinateSummary : public FilterI
{
public:
    
    CoordinateSummary();
    CoordinateSummary(CoordinateSummary const& other);
    CoordinateSummary& operator=(CoordinateSummary const& rhs);
    bool filter(const Point& point);

    void AddPoint(liblas::Point const& p);
    ptree GetPTree() const;
    void SetHeader(liblas::Header const& h);
    
    ~CoordinateSummary() {}
    
private:

    uint32_t count;
    boost::array<uint32_t, 8> points_by_return; 
    boost::array<uint32_t, 8> returns_of_given_pulse;
    bool first;
    boost::shared_ptr<liblas::Point> minimum;
    boost::shared_ptr<liblas::Point> maximum;
    liblas::Header m_header;
    bool bHaveHeader; 
    bool bHaveColor;
    bool bHaveTime;    
};

LAS_DLL std::ostream& operator<<(std::ostream& os, liblas::Summary const& s);

LAS_DLL uint32_t GetStreamPrecision(double scale);

} // namespace liblas

#endif // ndef LIBLAS_LASSUMMARY_HPP_INCLUDED
