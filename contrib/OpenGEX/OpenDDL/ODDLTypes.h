/*
	OpenDDL Library Software License
	==================================

	OpenDDL Library, version 1.1
	Copyright 2014-2015, Eric Lengyel
	All rights reserved.

	The OpenDDL Library is free software published on the following website:

		http://openddl.org/

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

	1. Redistributions of source code must retain the entire text of this license,
	comprising the above copyright notice, this list of conditions, and the following
	disclaimer.
	
	2. Redistributions of any modified source code files must contain a prominent
	notice immediately following this license stating that the contents have been
	modified from their original form.

	3. Redistributions in binary form must include attribution to the author in any
	listing of credits provided with the distribution. If there is no listing of
	credits, then attribution must be included in the documentation and/or other
	materials provided with the distribution. The attribution must be exactly the
	statement "This software contains the OpenDDL Library by Eric Lengyel" (without
	quotes) in the case that the distribution contains the original, unmodified
	OpenDDL Library, or it must be exactly the statement "This software contains a
	modified version of the OpenDDL Library by Eric Lengyel" (without quotes) in the
	case that the distribution contains a modified version of the OpenDDL Library.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
	ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
	WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
	IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
	INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
	NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
	PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
	WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
	ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef ODDLTypes_h
#define ODDLTypes_h


#include <stddef.h>
#include <math.h>
#include <new>

#if !defined(ODDL_CXX11)
	#if defined(_MSC_VER)
		#if _MSC_VER >= 1800
			#define ODDL_CXX11 1
		#endif
	#else
		#if __cplusplus > 199711L
			#define ODDL_CXX11 1
		#endif
	#endif
#endif

#if defined(ODDL_CXX11)
	#define ODDL_DELETE delete
	#define ODDL_FINAL final
	#define ODDL_OVERRIDE override
	#define ODDL_HAS_CXX11_RVALUE_REFERENCES 1
#else
	#define ODDL_DELETE
	#define ODDL_FINAL
	#define ODDL_OVERRIDE
	#define ODDL_HAS_CXX11_RVALUE_REFERENCES 0
	#if !defined(nullptr)
		#define nullptr 0
	#endif
#endif

#define restrict __restrict

namespace ODDL
{
#if defined (_WIN32) || defined(WIN32)
	typedef signed char				int8;
	typedef unsigned char			unsigned_int8;

	typedef short					int16;
	typedef unsigned short			unsigned_int16;

	typedef int						int32;
	typedef unsigned int			unsigned_int32;

	typedef __int64					int64;
	typedef unsigned __int64		unsigned_int64;

	#if defined(_WIN64)

		typedef __int64				machine;
		typedef unsigned __int64	unsigned_machine;

	#else

		typedef long				machine;
		typedef unsigned long		unsigned_machine;

	#endif

#else

	typedef int8_t					int8;
	typedef uint8_t					unsigned_int8;

	typedef int16_t					int16;
	typedef uint16_t				unsigned_int16;

	typedef int32_t						int32;
	typedef uint32_t				unsigned_int32;

	typedef int64_t					int64;
	typedef uint64_t				unsigned_int64;

	typedef ptrdiff_t				machine;
	typedef size_t					unsigned_machine;

#endif

	inline int32 Abs(int32 x)
	{
		int32 a = x >> 31;
		return ((x ^ a) - a);
	}

	inline int32 Min(int32 x, int32 y)
	{
		int32 a = x - y;
		return (x - (a & ~(a >> 31)));
	}

	inline int32 Max(int32 x, int32 y)
	{
		int32 a = x - y;
		return (x - (a & (a >> 31)));
	}

	inline int32 MinZero(int32 x)
	{
		return (x & (x >> 31));
	}

	inline int32 MaxZero(int32 x)
	{
		return (x & ~(x >> 31));
	}


	template <class type> class AutoDelete
	{
		private:

			type	*reference;

			AutoDelete(const AutoDelete&);

		public:

			explicit AutoDelete(type *ptr)
			{
				reference = ptr;
			}

			~AutoDelete()
			{
				delete reference;
			}

			operator type *(void) const
			{
				return (reference);
			}

			type *const *operator &(void) const
			{
				return (&reference);
			}

			type *operator ->(void) const
			{
				return (reference);
			}

			AutoDelete& operator =(type *ptr)
			{
				reference = ptr;
				return (*this);
			}
	};
}


#endif
