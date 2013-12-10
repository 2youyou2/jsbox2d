/*
* Copyright (c) 2010 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef B2_GROWABLE_STACK_H
#define B2_GROWABLE_STACK_H
#include <Box2D/Common/b2Settings.h>
#include <memory.h>

/// This is a growable LIFO stack with an initial capacity of N.
/// If the stack size exceeds the initial capacity, the heap is used
/// to increase the size of the stack.
template <typename T, int32 N>
class b2GrowableStack
{
public:
	b2GrowableStack()
	{
		this->m_stack = this->m_array;
		this->m_count = 0;
		this->m_capacity = N;
	}

	~b2GrowableStack()
	{
		if (this->m_stack != this->m_array)
		{
			b2Free(this->m_stack);
			this->m_stack = null;
		}
	}

	void Push(const T& element)
	{
		if (this->m_count == this->m_capacity)
		{
			T* old = this->m_stack;
			this->m_capacity *= 2;
			this->m_stack = (T*)b2Alloc(this->m_capacity * sizeof(T));
			memcpy(this->m_stack, old, this->m_count * sizeof(T));
			if (old != this->m_array)
			{
				b2Free(old);
			}
		}

		this->m_stack[this->m_count] = element;
		++this->m_count;
	}

	T Pop()
	{
		b2Assert(this->m_count > 0);
		--this->m_count;
		return this->m_stack[this->m_count];
	}

	int32 GetCount()
	{
		return this->m_count;
	}

private:
	T* m_stack;
	T m_array[N];
	int32 m_count;
	int32 m_capacity;
};


#endif
