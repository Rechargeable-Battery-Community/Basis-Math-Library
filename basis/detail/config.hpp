/**
 * \file config.hpp
 * 
 * \section COPYRIGHT
 *
 * Basis: A 3D Mathematics Library
 *
 * ---------------------------------------------------------------------
 *
 * Copyright (c) 2011, Don Olmstead
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of organization nor the names of its contributors may be
 *     used to endorse or promote products derived from this software without
 *     specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef BASIS_DETAIL_CONFIG_HPP_INCLUDED
#define BASIS_DETAIL_CONFIG_HPP_INCLUDED

//--------------------------------------------------------------------------
// STL includes
//--------------------------------------------------------------------------

#include <cstdint>

//--------------------------------------------------------------------------
// Inlining macro
//--------------------------------------------------------------------------

// Uncomment or define BASIS_FORCE_INLINE to use __forceinline
// #define BASIS_FORCE_INLINE

#if defined(DEBUG)
#define BASIS_INLINE inline
#else
	#if defined(BASIS_FORCE_INLINE)
	#define BASIS_INLINE __forceinline
	#else
	#define BASIS_INLINE inline
	#endif
#endif

//--------------------------------------------------------------------------
// Define the SSE level to compile for
//
// SSE2 defines SIMD floating point operations.
// SSE3 adds support for horizontal operations.
// SSE4 adds a dot product instruction.
//--------------------------------------------------------------------------

// Uncomment or define BASIS_USE_SSE4 to compile for SSE4
// #define BASIS_USE_SSE4
// Uncomment or define BASIS_USE_SSE3 to compile for SSE3
#define BASIS_USE_SSE3
// Uncomment or define BASIS_USE_SSE2 to compile for SSE2
// #define BASIS_USE_SSE2

#if defined(BASIS_USE_SSE4)
#define BASIS_SSE_LEVEL 4
#elif defined(BASIS_USE_SSE3)
#define BASIS_SSE_LEVEL 3
#elif defined(BASIS_USE_SSE2)
#define BASIS_SSE_LEVEL 2
#else
#define BASIS_SSE_LEVEL 0
#endif

#if BASIS_SSE_LEVEL > 0
#include <smmintrin.h>
#endif

//--------------------------------------------------------------------------
// Define prefernce between speed and accuracy with divisions.
//
// Allows accuracy to be traded for speed when computing divisions.
// By default accuracy is chosen.
//--------------------------------------------------------------------------

// Uncomment or define BASIS_ESTIMATE_DIVISION to estimate divisions
//#define BASIS_ESTIMATE_DIVISION

//--------------------------------------------------------------------------
// Define preference between speed and accuracy with the square root.
//
// Allows accuracy to be traded for speed when computing the
// square root. By default accuracy is chosen.
//
// In the case of SSE the fast sqrt relies on the identity
// x / sqrt(x) = sqrt(x)
// To improve the accuracy a Newton-Raphson iteration can be added.
//
// The SSE implementation was inspired by an article by Elan Ruskin
// who timed various methods of computing the sqrt
// <http://assemblyrequired.crashworks.org/2009/10/16/timing-square-root/>
//--------------------------------------------------------------------------

// Uncomment or define BASIS_ESTIMATE_SQRT to use the fast sqrt method
// #define BASIS_ESTIMATE_SQRT

// Uncomment or define BASIS_USE_NEWTON_RAPHSON_ITERATION
// #define BASIS_USE_NEWTON_RAPHSON_ITERATION

//--------------------------------------------------------------------------
// Define whether vector3 should use dot4 instead of dot3
//
// The dot4 operation in most cases uses less instructions than
// the dot3 operation. If the 3rd component in the vector3 is always
// zero, which should be the case if the data is initialized, then
// the dot4 operation can be used.
//--------------------------------------------------------------------------

// Uncomment or define BASIS_VECTOR3_USE_DOT4 to use dot4 instead of dot3
// #define BASIS_VECTOR3_USE_DOT4

//--------------------------------------------------------------------------
// Custom assert macro
//--------------------------------------------------------------------------

#include <cassert>

/**
 * Custom assert macro.
 *
 * \param expression The expression to check.
 * \param message The error message.
 */
#define BASIS_ASSERT(expression, message) assert(expression && message)

//--------------------------------------------------------------------------
// Define whether ostream operators should be created
//--------------------------------------------------------------------------

// Uncomment or define to create ostream operators
#define BASIS_USE_OSTREAM_OPERATORS

#ifdef BASIS_USE_OSTREAM_OPERATORS
#include <ostream>
#endif

#endif // end BASIS_DETAIL_CONFIG_HPP_INCLUDED
