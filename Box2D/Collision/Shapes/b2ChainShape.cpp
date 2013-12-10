/*
* Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/Shapes/b2ChainShape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <new>
#include <memory.h>

b2ChainShape::~b2ChainShape()
{
	b2Free(this->m_vertices);
	this->m_vertices = null;
	this->m_count = 0;
}

void b2ChainShape::CreateLoop(const b2Vec2* vertices, int32 count)
{
	b2Assert(this->m_vertices == null && this->m_count == 0);
	b2Assert(count >= 3);
	for (int32 i = 1; i < count; ++i)
	{
		b2Vec2 v1 = vertices[i-1];
		b2Vec2 v2 = vertices[i];
		// If the code crashes here, it means your vertices are too close together.
		b2Assert(b2DistanceSquared(v1, v2) > b2_linearSlop * b2_linearSlop);
	}

	this->m_count = count + 1;
	this->m_vertices = (b2Vec2*)b2Alloc(this->m_count * sizeof(b2Vec2));
	memcpy(this->m_vertices, vertices, count * sizeof(b2Vec2));
	this->m_vertices[count] = this->m_vertices[0];
	this->m_prevVertex = this->m_vertices[this->m_count - 2];
	this->m_nextVertex = this->m_vertices[1];
	this->m_hasPrevVertex = true;
	this->m_hasNextVertex = true;
}

void b2ChainShape::CreateChain(const b2Vec2* vertices, int32 count)
{
	b2Assert(this->m_vertices == null && this->m_count == 0);
	b2Assert(count >= 2);
	for (int32 i = 1; i < count; ++i)
	{
		b2Vec2 v1 = vertices[i-1];
		b2Vec2 v2 = vertices[i];
		// If the code crashes here, it means your vertices are too close together.
		b2Assert(b2DistanceSquared(v1, v2) > b2_linearSlop * b2_linearSlop);
	}

	this->m_count = count;
	this->m_vertices = (b2Vec2*)b2Alloc(count * sizeof(b2Vec2));
	memcpy(this->m_vertices, vertices, this->m_count * sizeof(b2Vec2));

	this->m_hasPrevVertex = false;
	this->m_hasNextVertex = false;

	this->m_prevVertex.SetZero();
	this->m_nextVertex.SetZero();
}

void b2ChainShape::SetPrevVertex(const b2Vec2& prevVertex)
{
	this->m_prevVertex = prevVertex;
	this->m_hasPrevVertex = true;
}

void b2ChainShape::SetNextVertex(const b2Vec2& nextVertex)
{
	this->m_nextVertex = nextVertex;
	this->m_hasNextVertex = true;
}

b2Shape* b2ChainShape::Clone(b2BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b2ChainShape));
	b2ChainShape* clone = new (mem) b2ChainShape;
	clone->CreateChain(this->m_vertices, this->m_count);
	clone->m_prevVertex = this->m_prevVertex;
	clone->m_nextVertex = this->m_nextVertex;
	clone->m_hasPrevVertex = this->m_hasPrevVertex;
	clone->m_hasNextVertex = this->m_hasNextVertex;
	return clone;
}

int32 b2ChainShape::GetChildCount() const
{
	// edge count = vertex count - 1
	return this->m_count - 1;
}

void b2ChainShape::GetChildEdge(b2EdgeShape* edge, int32 index) const
{
	b2Assert(0 <= index && index < this->m_count - 1);
	edge->m_type = b2Shape::e_edge;
	edge->m_radius = this->m_radius;

	edge->m_vertex1 = this->m_vertices[index + 0];
	edge->m_vertex2 = this->m_vertices[index + 1];

	if (index > 0)
	{
		edge->m_vertex0 = this->m_vertices[index - 1];
		edge->m_hasVertex0 = true;
	}
	else
	{
		edge->m_vertex0 = this->m_prevVertex;
		edge->m_hasVertex0 = this->m_hasPrevVertex;
	}

	if (index < this->m_count - 2)
	{
		edge->m_vertex3 = this->m_vertices[index + 2];
		edge->m_hasVertex3 = true;
	}
	else
	{
		edge->m_vertex3 = this->m_nextVertex;
		edge->m_hasVertex3 = this->m_hasNextVertex;
	}
}

bool b2ChainShape::TestPoint(const b2Transform& xf, const b2Vec2& p) const
{
	B2_NOT_USED(xf);
	B2_NOT_USED(p);
	return false;
}

bool b2ChainShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
							const b2Transform& xf, int32 childIndex) const
{
	b2Assert(childIndex < this->m_count);

	b2EdgeShape edgeShape;

	int32 i1 = childIndex;
	int32 i2 = childIndex + 1;
	if (i2 == this->m_count)
	{
		i2 = 0;
	}

	edgeShape.m_vertex1 = this->m_vertices[i1];
	edgeShape.m_vertex2 = this->m_vertices[i2];

	return edgeShape.RayCast(output, input, xf, 0);
}

void b2ChainShape::ComputeAABB(b2AABB* aabb, const b2Transform& xf, int32 childIndex) const
{
	b2Assert(childIndex < this->m_count);

	int32 i1 = childIndex;
	int32 i2 = childIndex + 1;
	if (i2 == this->m_count)
	{
		i2 = 0;
	}

	b2Vec2 v1 = b2Mul_t_v2(xf, this->m_vertices[i1]);
	b2Vec2 v2 = b2Mul_t_v2(xf, this->m_vertices[i2]);

	aabb->lowerBound = b2Min_v2(v1, v2);
	aabb->upperBound = b2Max_v2(v1, v2);
}

void b2ChainShape::ComputeMass(b2MassData* massData, float32 density) const
{
	B2_NOT_USED(density);

	massData->mass = 0.0;
	massData->center.SetZero();
	massData->I = 0.0;
}
