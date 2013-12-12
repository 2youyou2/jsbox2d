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

#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <new>

void b2EdgeShape::Set(const b2Vec2& v1, const b2Vec2& v2)
{
	this->m_vertex1.Assign(v1);
	this->m_vertex2.Assign(v2);
	this->m_hasVertex0 = false;
	this->m_hasVertex3 = false;
}

b2Shape* b2EdgeShape::Clone(b2BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b2EdgeShape));
	b2EdgeShape* clone = new (mem) b2EdgeShape;
	*clone = *this;
	return clone;
}

int32 b2EdgeShape::GetChildCount() const
{
	return 1;
}

bool b2EdgeShape::TestPoint(const b2Transform& xf, const b2Vec2& p) const
{
	B2_NOT_USED(xf);
	B2_NOT_USED(p);
	return false;
}

// p = p1 + t * d
// v = v1 + s * e
// p1 + t * d = v1 + s * e
// s * e - t * d = p1 - v1
bool b2EdgeShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
							const b2Transform& xf, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

	// Put the ray into the edge's frame of reference.
	b2Vec2 p1 = b2MulT_r_v2(xf.q, b2Vec2::Subtract(input.p1, xf.p));
	b2Vec2 p2 = b2MulT_r_v2(xf.q, b2Vec2::Subtract(input.p2, xf.p));
	b2Vec2 d = b2Vec2::Subtract(p2, p1);

	b2Vec2 v1 = this->m_vertex1;
	b2Vec2 v2 = this->m_vertex2;
	b2Vec2 e = b2Vec2::Subtract(v2, v1);
	b2Vec2 normal(e.y, -e.x);
	normal.Normalize();

	// q = p1 + t * d
	// dot(normal, q - v1) = 0
	// dot(normal, p1 - v1) + t * dot(normal, d) = 0
	float32 numerator = b2Dot_v2_v2(normal, b2Vec2::Subtract(v1, p1));
	float32 denominator = b2Dot_v2_v2(normal, d);

	if (denominator == 0.0)
	{
		return false;
	}

	float32 t = numerator / denominator;
	if (t < 0.0 || input.maxFraction < t)
	{
		return false;
	}

	b2Vec2 q = b2Vec2::Add(p1, b2Vec2::Multiply(t, d));

	// q = v1 + s * r
	// s = dot(q - v1, r) / dot(r, r)
	b2Vec2 r = b2Vec2::Subtract(v2, v1);
	float32 rr = b2Dot_v2_v2(r, r);
	if (rr == 0.0)
	{
		return false;
	}

	float32 s = b2Dot_v2_v2(b2Vec2::Subtract(q, v1), r) / rr;
	if (s < 0.0 || 1.0 < s)
	{
		return false;
	}

	output->fraction = t;
	if (numerator > 0.0)
	{
		output->normal.Assign(b2Mul_r_v2(xf.q, normal).Negate());
	}
	else
	{
		output->normal.Assign(b2Mul_r_v2(xf.q, normal));
	}
	return true;
}

void b2EdgeShape::ComputeAABB(b2AABB* aabb, const b2Transform& xf, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

	b2Vec2 v1 = b2Mul_t_v2(xf, this->m_vertex1);
	b2Vec2 v2 = b2Mul_t_v2(xf, this->m_vertex2);

	b2Vec2 lower = b2Min_v2(v1, v2);
	b2Vec2 upper = b2Max_v2(v1, v2);

	b2Vec2 r(this->m_radius, this->m_radius);
	aabb->lowerBound.Assign(b2Vec2::Subtract(lower, r));
	aabb->upperBound.Assign(b2Vec2::Add(upper, r));
}

void b2EdgeShape::ComputeMass(b2MassData* massData, float32 density) const
{
	B2_NOT_USED(density);

	massData->mass = 0.0;
	massData->center.Assign(b2Vec2::Multiply(0.5, b2Vec2::Add(this->m_vertex1, this->m_vertex2)));
	massData->I = 0.0;
}
