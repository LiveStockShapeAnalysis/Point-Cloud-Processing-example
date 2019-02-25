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

#ifndef LIBLAS_LASFILTER_HPP_INCLUDED
#define LIBLAS_LASFILTER_HPP_INCLUDED

#include <liblas/version.hpp>
#include <liblas/header.hpp>
#include <liblas/point.hpp>
#include <liblas/detail/fwd.hpp>
#include <liblas/export.hpp>
// boost
#include <boost/function.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
// std
#include <vector>
#include <functional>
#include <string>

namespace liblas {

/// Defines public interface to LAS filter implementation.
class LAS_DLL FilterI
{
public:
    
    /// Determines whether or not the filter keeps or rejects points that meet 
    /// filtering criteria
    enum FilterType
    {
        eExclusion = 0, ///< Filter removes point that meet the criteria of filter(const Point& point)
        eInclusion = 1 ///< Filter keeps point that meet the criteria of filter(const Point& point)
    };
    
    /// Function called by liblas::Reader::ReadNextPoint to apply the (list)
    /// of filter to the point.  If the function returns true, the point 
    /// passes the filter and is kept.
    virtual bool filter(const Point& point) = 0;
    
    /// Sets whether the filter is one that keeps data that matches 
    /// construction criteria or rejects them.
    void SetType(FilterType t) {m_type = t;}

    /// Gets the type of filter.
    FilterType GetType() const {return m_type; }

    virtual ~FilterI() {}

    /// Base constructor.  Initializes the FilterType
    FilterI(FilterType t) : m_type(t) {}
    
private:

    FilterI(FilterI const& other);
    FilterI& operator=(FilterI const& rhs);

    FilterType m_type;
};

typedef boost::shared_ptr<FilterI> FilterPtr;

/// A filter for keeping or rejecting points that fall within a 
/// specified bounds.
class LAS_DLL BoundsFilter: public FilterI
{
public:
    
    BoundsFilter(double minx, double miny, double maxx, double maxy);
    BoundsFilter(double minx, double miny, double minz, double maxx, double maxy, double maxz);
    BoundsFilter(Bounds<double> const& b);
    bool filter(const Point& point);

private:
    
    liblas::Bounds<double> bounds;

    BoundsFilter(BoundsFilter const& other);
    BoundsFilter& operator=(BoundsFilter const& rhs);
};

/// A filter for keeping or rejecting a list of classification ids
class LAS_DLL ClassificationFilter: public FilterI
{
public:
    
    typedef std::vector<liblas::Classification> class_list_type;

    ClassificationFilter(class_list_type classes);
    bool filter(const Point& point);
    
private:

    class_list_type m_classes;

    ClassificationFilter(ClassificationFilter const& other);
    ClassificationFilter& operator=(ClassificationFilter const& rhs);
};

/// A filter simple decimation
class LAS_DLL ThinFilter: public liblas::FilterI
{
public:
    
    /// Default constructor.  Keep every thin'th point.
    ThinFilter(uint32_t thin);
    bool filter(const liblas::Point& point);


private:

    ThinFilter(ThinFilter const& other);
    ThinFilter& operator=(ThinFilter const& rhs);
    
    uint32_t thin_amount;
    uint32_t thin_count;
};


/// A filter for keeping or rejecting a list of return ids.
class LAS_DLL ReturnFilter: public FilterI
{
public:
    
    typedef std::vector<uint16_t> return_list_type;

    ReturnFilter(return_list_type returns, bool last_only);
    bool filter(const Point& point);
    
private:

    return_list_type m_returns;
    bool last_only;

    ReturnFilter(ReturnFilter const& other);
    ReturnFilter& operator=(ReturnFilter const& rhs);
};


class LAS_DLL ValidationFilter: public FilterI
{
public:

    ValidationFilter();
    bool filter(const Point& point);
    
private:

    ValidationFilter(ValidationFilter const& other);
    ValidationFilter& operator=(ValidationFilter const& rhs);
};


/// A templated class that allows you 
/// to create complex filters using functions that are callable 
/// from the liblas::Point class.  See laskernel.cpp for examples 
/// how to use it for filtering intensity and time values.  
template <typename T>
class LAS_DLL ContinuousValueFilter: public FilterI
{
    

    // 
public:
    typedef boost::function<T (const Point*)> filter_func;
    typedef boost::function<bool(T, T)> compare_func;

    /// Construct the filter with a filter_func, a comparison value, 
    /// and a compare_func.  To use this we must take in a filtering function that returns 
    ///  us a value from the point, and a binary_predicate comparator
    /// (ie, std::less, std::greater, std::equal_to, etc).  
    /// \param f - The function to compare with from the liblas::Point.  It 
    /// must be one of the functions that returns an integer type or double type
    /// \param value - The value to use for one-way comparison
    /// \param c - the std::binary_predicate to use for comparison

    /// To use this we must take in a filtering function that returns 
    /// us a value from the point, and a binary_predicate comparator
    /// (ie, std::less, std::greater, std::equal_to, etc).  
    
    /// Example:
    /// GetIntensity returns a uint16_t, so we use that for our template 
    /// value.  This filter would keep all points with intensities that are 
    /// less than 100.
    
    /// liblas::ContinuousValueFilter<uint16_t>::compare_func c = std::less<uint16_t>();
    /// liblas::ContinuousValueFilter<uint16_t>::filter_func f = &liblas::Point::GetIntensity;
    /// liblas::ContinuousValueFilter<uint16_t>* intensity_filter = new liblas::ContinuousValueFilter<uint16_t>(f, 100, c);
    /// intensity_filter->SetType(liblas::FilterI::eInclusion);
    ContinuousValueFilter(filter_func f, T value, compare_func c)
        : liblas::FilterI(eInclusion), f(f), c(c),value(value)
    {}

        
    /// Construct the filter with a filter_func and a simple 
    /// expression.  
    /// \param f - The function to compare with from the liblas::Point.  It 
    /// must be one of the functions that returns an integer type or double type
    /// \param filter_string - A string to use for the filter.  Supports taking 
    /// in a simple expression and turning it into 
    /// a comparator we can use.  We support dead simple stuff:
    /// >200
    /// ==150
    /// >=32
    /// <=150
    /// <100
    
    /// We don't strip whitespace, and we don't support complex 
    /// comparisons (ie, two function  10<x<300)
    
    /// In addition to explicitly setting your comparator function, you can 
    /// also use the constructor that takes in a simple expression string 
    /// and constructs the basic comparators.  See the source code or las2las2 
    /// help output for the forms that are supported.  This may be 
    /// improved in the future.
    
    /// std::string intensities("<100")
    /// liblas::ContinuousValueFilter<uint16_t>::filter_func f = &liblas::Point::GetIntensity;
    /// liblas::ContinuousValueFilter<uint16_t>* intensity_filter = new liblas::ContinuousValueFilter<uint16_t>(f, intensities);
    /// intensity_filter->SetType(liblas::FilterI::eInclusion);
    
    ContinuousValueFilter(filter_func f, std::string const& filter_string)
        : liblas::FilterI(eInclusion), f(f)
    {
        compare_func compare;

        bool gt = HasPredicate(filter_string, ">");
        bool gte = HasPredicate(filter_string, ">=");
        bool lt = HasPredicate(filter_string, "<");
        bool lte = HasPredicate(filter_string, "<=");
        bool eq = HasPredicate(filter_string, "==");

        std::string::size_type pos=0;
        std::string out;

        if (gte) // >=
        {
            // std::cout<<"have gte!" << std::endl;
            c = std::greater_equal<T>();
            pos = filter_string.find_first_of("=") + 1;
        }
        else if (gt) // .
        {
            // std::cout<<"have gt!" << std::endl;
            c = std::greater<T>();
            pos = filter_string.find_first_of(">") + 1;
        }
        else if (lte) // <=
        {
            // std::cout<<"have lte!" << std::endl;
            c = std::less_equal<T>();
            pos = filter_string.find_first_of("=") +1;
        }
        else if (lt) // <
        {
            // std::cout<<"have le!" << std::endl;
            c = std::less<T>();    
            pos = filter_string.find_first_of("<") + 1;
        }
        else if (eq) // ==
        {
            // std::cout<<"have eq!" << std::endl;
            c = std::equal_to<T>();
            pos = filter_string.find_last_of("=") + 1;
    
        }

        out = filter_string.substr(pos, filter_string.size());
        
        value =  boost::lexical_cast<T>(out);
        // std::cout << "Value is: " << value << " pos " << pos << " out " << out << std::endl;
    }
            
    bool filter(const liblas::Point& p)
    {
        bool output = false;

        T v = f(&p);
        // std::cout << std::endl<< "Checking c(v, value) v: " << v << " value: " << value;
        if (c(v, value)){
            // std::cout<< " ... succeeded "<<std::endl;
            if (GetType() == eInclusion) {
                output = true;
            } else {
                // std::cout << "Filter type is eExclusion and test passed" << std::endl;
                output = false;
            }    
        } else {
            // std::cout<<" ... failed" <<std::endl;
            if (GetType() == eInclusion) {
                output = false;
            } else {
                // std::cout << "Filter type is eExclusion and test failed" << std::endl;
                output = true;
            }    
        }
        // std::cout << " returning " << output << std::endl;
        return output;
    }
    
private:

    ContinuousValueFilter(ContinuousValueFilter const& other);
    ContinuousValueFilter& operator=(ContinuousValueFilter const& rhs);
    filter_func f;
    compare_func c;
    T value;

    bool HasPredicate(std::string const& parse_string, std::string predicate)
    {
        // Check if the given string contains all of the characters of predicate
        // For example, does '>=300' have both > and = (as given in the predicate string)
        bool output = false;
        // We must have all of the characters in the predicate to return true
        for (std::string::const_iterator i = predicate.begin(); i!=predicate.end(); ++i) {
            std::string::size_type pred = parse_string.find_first_of(*i);
            if (pred != std::string::npos) {
                output = true;
            } else {
                return false;
            }
        }
        return output;
    }


};

/// A filter for color ranges
class LAS_DLL ColorFilter: public FilterI
{
public:

    ColorFilter(liblas::Color const& low, 
                liblas::Color const& high);
                
    ColorFilter(liblas::Color::value_type low_red, 
                liblas::Color::value_type high_red,
                liblas::Color::value_type low_blue,
                liblas::Color::value_type high_blue,
                liblas::Color::value_type low_green,
                liblas::Color::value_type high_green);
    bool filter(const Point& point);
    
private:
    
    liblas::Color m_low;
    liblas::Color m_high;

    ColorFilter(ColorFilter const& other);
    ColorFilter& operator=(ColorFilter const& rhs);
    bool DoExclude();
};

} // namespace liblas

#endif // ndef LIBLAS_LASFILTER_HPP_INCLUDED
