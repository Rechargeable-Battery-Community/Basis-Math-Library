/**
 * \file sse_type.hpp
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

#ifndef BASIS_DETAIL_SSE_TYPE_HPP_INCLUDED
#define BASIS_DETAIL_SSE_TYPE_HPP_INCLUDED

#include <basis/detail/config.hpp>

// Check if SSE instructions are available
#if BASIS_SSE_LEVEL > 0

namespace basis { namespace detail
{
	//------------------------------------------------------------
	// Types
	//------------------------------------------------------------

	/// An SIMD type containing 4 32-bit floating point numbers.
	typedef __m128 sse_type;
	/**
	 * Input parameter type.
	 *
	 * Microsoft recommends passing the __m128 data type by
	 * reference to functions. This is the behavior of the
	 * XNA Math Library.
	 */
	typedef const __m128& sse_param;
	/// Output parameter type.
	typedef __m128& sse_outparam;

	//------------------------------------------------------------
	// Initialization
	//------------------------------------------------------------

	/**
	 * Replicates a floating point value into all 4 components.
	 *
	 * \param value The value to replicate.
	 * \returns A SIMD type whose components are all equal to value.
	 */
	BASIS_INLINE sse_type simd_replicate(float value)
	{
		return _mm_set_ps1(value);
	}

	/**
	 * Creates a vector using 4 floating point values.
	 *
	 * \param value1 The first component.
	 * \param value2 The second component.
	 * \param value3 The third component.
	 * \param value4 The fourth component.
	 * \returns A SIMD type with the components specified.
	 */
	BASIS_INLINE sse_type simd_set(float value1, float value2, float value3, float value4)
	{
		return _mm_set_ps(value4, value3, value2, value1);
	}

	/**
	 * Creates a SIMD type whose components are all set to zero.
	 *
	 * \returns A SIMD type whose components are all set to zero.
	 */
	BASIS_INLINE sse_type simd_zero()
	{
		return _mm_setzero_ps();
	}

	/**
	 * Creates a SIMD type whose components are all set to one.
	 *
	 * \returns A SIMD type whose components are all set to one.
	 */
	BASIS_INLINE sse_type simd_one()
	{
		return _mm_set1_ps(1.0f);
	}

	//------------------------------------------------------------
	// Element access
	//------------------------------------------------------------

	/**
	 * Gets the value of the component at the specified index.
	 *
	 * \param v The SIMD type.
	 * \param i The index to access.
	 * \returns The value of the component at the specified index.
	 */
	BASIS_INLINE float simd_get_element(sse_param v, int i)
	{
		BASIS_ASSERT((i >= 0) && (i < 4), "Element out of range");

	#if defined _MSC_VER
		return v.m128_f32[i];
	#elif defined __GNUC__
	#else
	#endif
	}

	/**
	 * Sets the value of the component at the specified index.
	 *
	 * \param v The SIMD type.
	 * \param f The value to use.
	 * \param i The index to access.
	 */
	BASIS_INLINE void simd_set_element(sse_outparam v, float f, int i)
	{
		BASIS_ASSERT((i >= 0) && (i < 4), "Element out of range");

	#if defined _MSC_VER
		v.m128_f32[i] = f;
	#elif defined __GNUC__
	#else
	#endif
	}

	//------------------------------------------------------------
	// Comparison operations
	//------------------------------------------------------------

	//------------------------------------------------------------
	// Arithmetic operations
	//------------------------------------------------------------

	/**
	 * Computes the negation of a SIMD type.
	 *
	 * \param v The SIMD type to negate.
	 * \returns The negation of the SIMD type.
	 */
	BASIS_INLINE sse_type simd_negate(sse_param v)
	{
		return _mm_sub_ps(_mm_setzero_ps(), v);
	}

	/**
	 * Computes the sum of two SIMD types.
	 *
	 * \param v1 First SIMD type to add.
	 * \param v2 Second SIMD type to add.
	 * \returns The sum of v1 and v2.
	 */
	BASIS_INLINE sse_type simd_add(sse_param v1, sse_param v2)
	{
		return _mm_add_ps(v1, v2);
	}

	/**
	 * Computes the difference of two SIMD types.
	 *
	 * \param v1 First SIMD type.
	 * \param v2 SIMD type to subtract from v1.
	 * \returns The difference of v1 and v2.
	 */
	BASIS_INLINE sse_type simd_subtract(sse_param v1, sse_param v2)
	{
		return _mm_sub_ps(v1, v2);
	}

	/**
	 * Computes the product of two SIMD types.
	 *
	 * \param v1 First SIMD type to multiply.
	 * \param v2 Second SIMD type to multiply.
	 * \returns The product of v1 and v2.
	 */
	BASIS_INLINE sse_type simd_multiply(sse_param v1, sse_param v2)
	{
		return _mm_mul_ps(v1, v2);
	}

	/**
	 * Computes the division of two SIMD types.
	 *
	 * \param v1 The dividends of the operation.
	 * \param v2 The divisors of the operation.
	 * \returns The quotient of v1 and v2.
	 */
	BASIS_INLINE sse_type simd_divide(sse_param v1, sse_param v2)
	{
		return _mm_div_ps(v1, v2);
	}

	/**
	 * Estimates the division of two SIMD types.
	 *
	 * Offers increased performance at the expense of reduced accuracy.
	 *
	 * \code
	 * result[0] = v1[0] * (1.0 / v2[0]);
	 * result[1] = v1[1] * (1.0 / v2[1]);
	 * result[2] = v1[2] * (1.0 / v2[2]);
	 * result[3] = v1[3] * (1.0 / v2[3]);
	 * \endcode
	 *
	 * \param v1 The dividends of the operation.
	 * \param v2 The divisors of the operation.
	 * \returns The quotient of v1 and v2.
	 */
	BASIS_INLINE sse_type simd_divide_est(sse_param v1, sse_param v2)
	{
		return _mm_mul_ps(v1, _mm_rcp_ps(v2));
	}

	/**
	 * Computes the product of the first two SIMD types added to the third SIMD type.
	 *
	 * \code
	 * result[0] = (v1[0] * v2[0]) + v3[0];
	 * result[1] = (v1[1] * v2[1]) + v3[1];
	 * result[2] = (v1[2] * v2[2]) + v3[2];
	 * result[3] = (v1[3] * v2[3]) + v3[3];
	 * \endcode
	 *
	 * \param v1 SIMD type multiplier.
	 * \param v2 SIMD type multiplicand.
	 * \param v3 SIMD type addend.
	 * \returns The product sum of the SIMD types.
	 */
	BASIS_INLINE sse_type simd_multiply_add(sse_param v1, sse_param v2, sse_param v3)
	{
		return _mm_add_ps(_mm_mul_ps(v1, v2), v3);
	}

	/**
	 * Computes the difference of a third SIMD type and the product of the first two SIMD types.
	 *
	 * \code
	 * result[0] = v3[0] - (v1[0] * v2[0]);
	 * result[1] = v3[1] - (v1[1] * v2[1]);
	 * result[2] = v3[2] - (v1[2] * v2[2]);
	 * result[3] = v3[3] - (v1[3] * v2[3]);
	 * \endcode
	 *
	 * \param v1 SIMD type multiplier.
	 * \param v2 SIMD type multiplicand.
	 * \param v3 SIMD type subtrahend.
	 * \returns The product difference of the SIMD types.
	 */
	BASIS_INLINE sse_type simd_negative_multiply_subtract(sse_param v1, sse_param v2, sse_param v3)
	{
		return _mm_sub_ps(v3, _mm_mul_ps(v1, v2));
	}

	/**
	 * Computes the reciprocal of a SIMD type.
	 *
	 * \param v The SIMD type to compute the reciprocal of.
	 * \returns The reciprocal of the SIMD type.
	 */
	BASIS_INLINE sse_type simd_reciprocal(sse_param v)
	{
		return _mm_div_ps(_mm_set1_ps(1.0f), v);
	}

	/**
	 * Estimates the reciprocal of a SIMD type.
	 *
	 * Offers increased performance at the expense of reduced accuracy.
	 *
	 * \param v The SIMD type to compute the reciprocal of.
	 * \returns The reciprocal of the SIMD type.
	 */
	BASIS_INLINE sse_type simd_reciprocal_est(sse_param v)
	{
		return _mm_rcp_ps(v);
	}

	/**
	 * Computes the square root of a SIMD type.
	 *
	 * \param v The SIMD type to compute the square root of.
	 * \returns The square root of the SIMD type.
	 */
	BASIS_INLINE sse_type simd_sqrt(sse_param v)
	{
		return _mm_sqrt_ps(v);
	}

	/**
	 * Estimates the square root of a SIMD type.
	 *
	 * Offers increased performance at the expense of reduced accuracy.
	 *
	 * \param v The SIMD type to compute the square root of.
	 * \returns The square root of the SIMD type.
	 */
	BASIS_INLINE sse_type simd_sqrt_est(sse_param v)
	{
	#ifdef BASIS_USE_NEWTON_RAPHSON_ITERATION
		// Perform a Newton-Raphson iteration on the reciprocal
		// yn+1 = (yn * (3 - xyn^2)) / 2
		const __m128 yn = _mm_rsqrt_ps(v);
		const __m128 xyn2 = _mm_mul_ps(_mm_mul_ps(v, yn), yn);

		return _mm_mul_ps(_mm_mul_ps(_mm_set_ps1(0.5f), _mm_mul_ps(v, yn)), _mm_sub_ps(_mm_set_ps1(3.0f), xyn2));
	#else
		return _mm_mul_ps(v, _mm_rsqrt_ps(v));
	#endif
	}

	/**
	 * Computes the reciprocal square root of a SIMD type.
	 *
	 * \param v The SIMD type to compute the reciprocal square root of.
	 * \returns The reciprocal square root of the SIMD type.
	 */
	BASIS_INLINE sse_type simd_reciprocal_sqrt(sse_param v)
	{
		return _mm_div_ps(_mm_set1_ps(1.0f), _mm_sqrt_ps(v));
	}

	/**
	 * Estimates the reciprocal square root of a SIMD type.
	 *
	 * Offers increased performance at the expense of reduced accuracy.
	 *
	 * \param v The SIMD type to compute the reciprocal square root of.
	 * \returns The reciprocal square root of the SIMD type.
	 */
	BASIS_INLINE sse_type simd_reciprocal_sqrt_est(sse_param v)
	{
	#ifdef BASIS_USE_NEWTON_RAPHSON_ITERATION
		// Perform a Newton-Raphson iteration on the reciprocal
		// yn+1 = (yn * (3 - xyn^2)) / 2
		const __m128 yn = _mm_rsqrt_ps(v);
		const __m128 xyn2 = _mm_mul_ps(_mm_mul_ps(v, yn), yn);

		return _mm_mul_ps(_mm_mul_ps(_mm_set_ps1(0.5f), yn), _mm_sub_ps(_mm_set_ps1(3.0f), xyn2));
	#else
		return _mm_rsqrt_ps(v);
	#endif
	}

	//------------------------------------------------------------
	// Max/Min operations
	//------------------------------------------------------------

	/**
	 * Compares two SIMD types and selects the largest components.
	 *
	 * \code
	 * result[0] = (v1[0] > v2[0]) ? v1[0] : v2[0];
	 * result[1] = (v1[1] > v2[1]) ? v1[1] : v2[1];
	 * result[2] = (v1[2] > v2[2]) ? v1[2] : v2[2];
	 * result[3] = (v1[3] > v2[3]) ? v1[3] : v2[3];
	 * \endcode
	 *
	 * \param v1 First SIMD type to compare.
	 * \param v2 Second SIMD type to compare.
	 * \returns A SIMD type containing the largest components between the two SIMD types.
	 */
	BASIS_INLINE sse_type simd_max(sse_param v1, sse_param v2)
	{
		return _mm_max_ps(v1, v2);
	}

	/**
	 * Compares two SIMD types and selects the smallest components.
	 *
	 * \code
	 * result[0] = (v1[0] < v2[0]) ? v1[0] : v2[0];
	 * result[1] = (v1[1] < v2[1]) ? v1[1] : v2[1];
	 * result[2] = (v1[2] < v2[2]) ? v1[2] : v2[2];
	 * result[3] = (v1[3] < v2[3]) ? v1[3] : v2[3];
	 * \endcode
	 *
	 * \param v1 First SIMD type to compare.
	 * \param v2 Second SIMD type to compare.
	 * \returns A SIMD type containing the smallest components between the two SIMD types.
	 */
	BASIS_INLINE sse_type simd_min(sse_param v1, sse_param v2)
	{
		return _mm_min_ps(v1, v2);
	}

	/**
	 * Clamps the components of the SIMD type to a specified range.
	 *
	 * \code
	 * result[0] = min(max[0], max(min[0], v[1]));
	 * result[1] = min(max[1], max(min[1], v[1]));
	 * result[2] = min(max[2], max(min[2], v[2]));
	 * result[3] = min(max[3], max(min[3], v[3]));
	 * \endcode
	 *
	 * \param v The SIMD type to clamp.
	 * \param min The minimum values.
	 * \param max The maximum values.
	 * \returns A SIMD type clamped to the specified range.
	 */
	BASIS_INLINE sse_type simd_clamp(sse_param v, sse_param min, sse_param max)
	{
		return _mm_min_ps(max, _mm_max_ps(min, v));
	}

	/**
	 * Clamps the components of the SIMD type to [0.0, 1.0]
	 *
	 * \code
	 * result[0] = min(1.0, max(0.0, v[1]));
	 * result[1] = min(1.0, max(0.0, v[1]));
	 * result[2] = min(1.0, max(0.0, v[2]));
	 * result[3] = min(1.0, max(0.0, v[3]));
	 * \endcode
	 *
	 * \param v The SIMD type to clamp.
	 * \returns A SIMD type clamped to [0.0, 1.0].
	 */
	BASIS_INLINE sse_type simd_saturate(sse_param v)
	{
		return _mm_min_ps(_mm_set_ps1(1.0f), _mm_max_ps(_mm_setzero_ps(), v));
	}

	//------------------------------------------------------------
	// Horizontal arithmetic operations
	//------------------------------------------------------------

	/**
	 * Computes the horizontal addition of two SIMD types.
	 *
	 * Adds the adjacent elements in the SIMD types.
	 *
	 * \code
	 * result[0] = v1[0] + v1[1];
	 * result[1] = v1[2] + v1[3];
	 * result[2] = v2[0] + v2[1];
	 * result[3] = v2[2] + v2[3];
	 * \endcode
	 *
	 * \param v1 First SIMD type.
	 * \param v2 Second SIMD type.
	 * \returns The horizontal sum of the SIMD types.
	 */
	BASIS_INLINE sse_type simd_horizontal_add(sse_param v1, sse_param v2)
	{
	#if BASIS_SSE_LEVEL > 2
		return _mm_hadd_ps(v1, v2);
	#else
		const __m128 t1 = _mm_shuffle_ps(v1._values, v2._values, _MM_SHUFFLE(2, 0, 2, 0));
		const __m128 t2 = _mm_shuffle_ps(v1._values, v2._values, _MM_SHUFFLE(3, 1, 3, 1));

		return _mm_add_ps(t1, t2);
	#endif
	}

	/**
	 * Computes the horizontal subtraction of two SIMD types.
	 *
	 * Subtracts the adjacent elements in the SIMD types.
	 *
	 * \code
	 * result[0] = v1[0] - v1[1];
	 * result[1] = v1[2] - v1[3];
	 * result[2] = v2[0] - v2[1];
	 * result[3] = v2[2] - v2[3];
	 * \endcode
	 *
	 * \param v1 First SIMD type.
	 * \param v2 Second SIMD type.
	 * \returns The horizontal difference of the SIMD types.
	 */
	BASIS_INLINE sse_type simd_horizontal_subtract(sse_param v1, sse_param v2)
	{
	#if BASIS_SSE_LEVEL > 2
		return _mm_hsub_ps(v1, v2);
	#else
		const __m128 t1 = _mm_shuffle_ps(v1._values, v2._values, _MM_SHUFFLE(2, 0, 2, 0));
		const __m128 t2 = _mm_shuffle_ps(v1._values, v2._values, _MM_SHUFFLE(3, 1, 3, 1));

		return _mm_sub_ps(t1, t2);		
	#endif
	}

	//------------------------------------------------------------
	// Vector operations
	//------------------------------------------------------------

	/**
	 * Computes the dot product between 2D vectors.
	 *
	 * \param v1 First SIMD type.
	 * \param v2 Second SIMD type.
	 * \returns The dot product between v1 and v2.
	 */
	BASIS_INLINE sse_type dot2(sse_param v1, sse_param v2)
	{
	#if BASIS_SSE_LEVEL >= 4
		return _mm_dp_ps(v1, v2, 0x3F);
	#else
		const __m128 t0 = _mm_mul_ps(v1, v2);
		const __m128 t1 = _mm_shuffle_ps(t0, t0, _MM_SHUFFLE(0,0,0,0));
		const __m128 t2 = _mm_shuffle_ps(t0, t0, _MM_SHUFFLE(1,1,1,1));

		return sse_type(_mm_add_ps(t1, t2));
	#endif
	}

	/**
	 * Computes the dot product between 3D vectors.
	 *
	 * \param v1 First SIMD type.
	 * \param v2 Second SIMD type.
	 * \returns The dot product between v1 and v2.
	 */
	BASIS_INLINE sse_type dot3(sse_param v1, sse_param v2)
	{
	#if BASIS_SSE_LEVEL >= 4
		return _mm_dp_ps(v1, v2, 0x7F);
	#else
		const __m128 t0 = _mm_mul_ps(v1, v2);
		const __m128 t1 = _mm_shuffle_ps(t0, t0, _MM_SHUFFLE(0,0,0,0));
		const __m128 t2 = _mm_shuffle_ps(t0, t0, _MM_SHUFFLE(1,1,1,1));
		const __m128 t3 = _mm_shuffle_ps(t0, t0, _MM_SHUFFLE(2,2,2,2));

		return _mm_add_ps(t1, _mm_add_ps(t2, t3));
	#endif
	}

	/**
	 * Computes the dot product between 4D vectors.
	 *
	 * \param v1 First SIMD type.
	 * \param v2 Second SIMD type.
	 * \returns The dot product between v1 and v2.
	 */
	BASIS_INLINE sse_type dot4(sse_param v1, sse_param v2)
	{
	#if BASIS_SSE_LEVEL >= 4
		return _mm_dp_ps(v1, v2);
	#elif BASIS_SSE_LEVEL >= 3
		const __m128 t0 = _mm_mul_ps(v1, v2);
		const __m128 t1 = _mm_hadd_ps(t0, t0);

		return _mm_hadd_ps(t1, t1);
	#else
		const __m128 t0 = _mm_mul_ps(v1, v2);
		const __m128 t1 = _mm_shuffle_ps(t0, t0, _MM_SHUFFLE(1,0,3,2));
		const __m128 t2 = _mm_add_ps(t0, t1);
		const __m128 t3 = _mm_shuffle_ps(t2, t2, _MM_SHUFFLE(2,3,0,1));

		return _mm_add_ps(t2, t3);
	#endif
	}

	//------------------------------------------------------------
	// Shuffle/Swizzle operations
	//------------------------------------------------------------

	/**
	 * Joins values from two SIMD types.
	 *
	 * \code
	 * result[0] = v1[_1];
	 * result[1] = v1[_2];
	 * result[2] = v2[_3];
	 * result[3] = v2[_4];
	 * \endcode
	 *
	 * \tparam _1 The element from v1 to select for the 1st slot.
	 * \tparam _2 The element from v1 to select for the 2nd slot.
	 * \tparam _3 The element from v2 to select for the 3rd slot.
	 * \tparam _4 The element from v2 to select for the 4th slot.
	 *
	 * \param v1 First SIMD type.
	 * \param v2 Second SIMD type.
	 * \returns The result of the shuffle operation.
	 */
	template <int _1, int _2, int _3, int _4>
	BASIS_INLINE sse_type simd_shuffle(sse_param v1, sse_param v2)
	{
		const unsigned int mask = (_4 << 6) | (_3 << 4) | (_2 << 2) | _1;

		return _mm_shuffle_ps(v1, v2, mask);
	}

	/**
	 * Swizzles a SIMD type.
	 *
	 * \code
	 * result[0] = v[_1];
	 * result[1] = v[_2];
	 * result[2] = v[_3];
	 * result[3] = v[_4];
	 * \endcode
	 *
	 * \tparam _1 The element to select for the 1st slot.
	 * \tparam _2 The element to select for the 2nd slot.
	 * \tparam _3 The element to select for the 3rd slot.
	 * \tparam _4 The element to select for the 4th slot.
	 *
	 * \param v A SIMD type.
	 * \returns The result of the swizzle operation.
	 */
	template <int _1, int _2, int _3, int _4>
	BASIS_INLINE sse_type simd_swizzle(sse_param v)
	{
		return simd_shuffle<_1, _2, _3, _4>(v, v);
	}

} } // end namespace basis::detail

#endif // end #if (BASIS_SSE_LEVEL > 0)

#endif // end BASIS_DETAIL_SSE_TYPE_HPP_INCLUDED
