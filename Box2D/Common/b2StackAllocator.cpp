/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Common/b2StackAllocator.h>
#include <Box2D/Common/b2Math.h>

b2StackAllocator::b2StackAllocator()
{
	this->m_index = 0;
	this->m_allocation = 0;
	this->m_maxAllocation = 0;
	this->m_entryCount = 0;
}

b2StackAllocator::~b2StackAllocator()
{
	b2Assert(this->m_index == 0);
	b2Assert(this->m_entryCount == 0);
}

void* b2StackAllocator::Allocate(int32 size)
{
	b2Assert(this->m_entryCount < b2_maxStackEntries);

	b2StackEntry* entry = this->m_entries + this->m_entryCount;
	entry->size = size;
	if (this->m_index + size > b2_stackSize)
	{
		entry->data = (char*)b2Alloc(size);
		entry->usedMalloc = true;
	}
	else
	{
		entry->data = this->m_data + this->m_index;
		entry->usedMalloc = false;
		this->m_index += size;
	}

	this->m_allocation += size;
	this->m_maxAllocation = b2Max(this->m_maxAllocation, this->m_allocation);
	++this->m_entryCount;

	return entry->data;
}

void b2StackAllocator::Free(void* p)
{
	b2Assert(this->m_entryCount > 0);
	b2StackEntry* entry = this->m_entries + this->m_entryCount - 1;
	b2Assert(p == entry->data);
	if (entry->usedMalloc)
	{
		b2Free(p);
	}
	else
	{
		this->m_index -= entry->size;
	}
	this->m_allocation -= entry->size;
	--this->m_entryCount;

	p = null;
}

int32 b2StackAllocator::GetMaxAllocation() const
{
	return this->m_maxAllocation;
}
