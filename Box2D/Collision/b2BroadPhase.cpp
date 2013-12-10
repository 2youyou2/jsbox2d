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

#include <Box2D/Collision/b2BroadPhase.h>

b2BroadPhase::b2BroadPhase()
{
	this->m_proxyCount = 0;

	this->m_pairCapacity = 16;
	this->m_pairCount = 0;
	this->m_pairBuffer = (b2Pair*)b2Alloc(this->m_pairCapacity * sizeof(b2Pair));

	this->m_moveCapacity = 16;
	this->m_moveCount = 0;
	this->m_moveBuffer = (int32*)b2Alloc(this->m_moveCapacity * sizeof(int32));
}

b2BroadPhase::~b2BroadPhase()
{
	b2Free(this->m_moveBuffer);
	b2Free(this->m_pairBuffer);
}

int32 b2BroadPhase::CreateProxy(const b2AABB& aabb, void* userData)
{
	int32 proxyId = this->m_tree.CreateProxy(aabb, userData);
	++this->m_proxyCount;
	BufferMove(proxyId);
	return proxyId;
}

void b2BroadPhase::DestroyProxy(int32 proxyId)
{
	UnBufferMove(proxyId);
	--this->m_proxyCount;
	this->m_tree.DestroyProxy(proxyId);
}

void b2BroadPhase::MoveProxy(int32 proxyId, const b2AABB& aabb, const b2Vec2& displacement)
{
	bool buffer = this->m_tree.MoveProxy(proxyId, aabb, displacement);
	if (buffer)
	{
		BufferMove(proxyId);
	}
}

void b2BroadPhase::TouchProxy(int32 proxyId)
{
	BufferMove(proxyId);
}

void b2BroadPhase::BufferMove(int32 proxyId)
{
	if (this->m_moveCount == this->m_moveCapacity)
	{
		int32* oldBuffer = this->m_moveBuffer;
		this->m_moveCapacity *= 2;
		this->m_moveBuffer = (int32*)b2Alloc(this->m_moveCapacity * sizeof(int32));
		memcpy(this->m_moveBuffer, oldBuffer, this->m_moveCount * sizeof(int32));
		b2Free(oldBuffer);
	}

	this->m_moveBuffer[this->m_moveCount] = proxyId;
	++this->m_moveCount;
}

void b2BroadPhase::UnBufferMove(int32 proxyId)
{
	for (int32 i = 0; i < this->m_moveCount; ++i)
	{
		if (this->m_moveBuffer[i] == proxyId)
		{
			this->m_moveBuffer[i] = b2BroadPhase::e_nullProxy;
		}
	}
}

// This is called from b2DynamicTree::Query when we are gathering pairs.
bool b2BroadPhase::QueryCallback(int32 proxyId)
{
	// A proxy cannot form a pair with itself.
	if (proxyId == this->m_queryProxyId)
	{
		return true;
	}

	// Grow the pair buffer as needed.
	if (this->m_pairCount == this->m_pairCapacity)
	{
		b2Pair* oldBuffer = this->m_pairBuffer;
		this->m_pairCapacity *= 2;
		this->m_pairBuffer = (b2Pair*)b2Alloc(this->m_pairCapacity * sizeof(b2Pair));
		memcpy(this->m_pairBuffer, oldBuffer, this->m_pairCount * sizeof(b2Pair));
		b2Free(oldBuffer);
	}

	this->m_pairBuffer[this->m_pairCount].proxyIdA = b2Min(proxyId, this->m_queryProxyId);
	this->m_pairBuffer[this->m_pairCount].proxyIdB = b2Max(proxyId, this->m_queryProxyId);
	++this->m_pairCount;

	return true;
}
