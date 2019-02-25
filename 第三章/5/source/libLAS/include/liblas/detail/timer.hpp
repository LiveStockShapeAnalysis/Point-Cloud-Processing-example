/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  A simple timing class for performance tuning
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

#ifndef LIBLAS_DETAIL_TIMER_HPP_INCLUDED
#define LIBLAS_DETAIL_TIMER_HPP_INCLUDED

#ifdef WIN32
#include <windows.h>
#else
#include <sys/time.h> // gettimeofday
#endif

namespace liblas { namespace detail {

class Timer
{
public:

    Timer()
    {
#ifdef WIN32
        LARGE_INTEGER rfreq = { 0 };
        QueryPerformanceFrequency(&rfreq);
        m_freq = rfreq.LowPart / double(1000);
#endif
  }

  void start()
  {
#ifdef WIN32
      QueryPerformanceCounter(&m_start);
#else
      gettimeofday(&m_start, 0);
#endif
  }

  double stop()
  {
#ifdef WIN32
  QueryPerformanceCounter(&m_stop);
  return (m_stop.LowPart - m_start.LowPart) / m_freq;
#else
  gettimeofday(&m_stop, 0);
  return (m_stop.tv_sec - m_start.tv_sec) * double(1000)
          + (m_stop.tv_usec - m_start.tv_usec) / double(1000);
#endif
  }

private:

#ifdef WIN32
  double m_freq;
  LARGE_INTEGER m_start;
  LARGE_INTEGER m_stop;
#else
  timeval m_start;
  timeval m_stop;
#endif

};

}} // namespace liblas::detail

#endif // LIBLAS_DETAIL_TIMER_HPP_INCLUDED
