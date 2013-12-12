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

#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <new>

b2Shape* b2CircleShape::Clone(b2BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b2CircleShape));
	b2CircleShape* clone = new (mem) b2CircleShape;
	*clone = *this;
	return clone;
}

int32 b2CircleShape::GetChildCount() const
{
	return 1;
}

bool b2CircleShape::TestPoint(const b2Transform& transform, const b2Vec2& p) const
{
	b2Vec2 center = b2Vec2::Add(transform.p, b2Mul_r_v2(transform.q, this->m_p));
	b2Vec2 d = b2Vec2::Subtract(p, center);
	return b2Dot_v2_v2(d, d) <= this->m_radius * this->m_radius;
}

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius
bool b2CircleShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
							const b2Transform& transform, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

	b2Vec2 position = b2Vec2::Add(transform.p, b2Mul_r_v2(transform.q, this->m_p));
	b2Vec2 s = b2Vec2::Subtract(input.p1, position);
	float32 b = b2Dot_v2_v2(s, s) - this->m_radius * this->m_radius;

	// Solve quadratic equation.
	b2Vec2 r = b2Vec2::Subtract(input.p2, input.p1);
	float32 c = b2Dot_v2_v2(s, r);
	float32 rr = b2Dot_v2_v2(r, r);
	float32 sigma = c * c - rr * b;

	// Check for negative discriminant and short segment.
	if (sigma < 0.0 || rr < b2_epsilon)
	{
		return false;
	}

	// Find the point of intersection of the line with the circle.
	float32 a = -(c + b2Sqrt(sigma));

	// Is the intersection point on the segment?
	if (0.0 <= a && a <= input.maxFraction * rr)
	{
		a /= rr;
		output->fraction = a;
		output->normal.Assign(b2Vec2::Add(s, b2Vec2::Multiply(a, r)));
		output->normal.Normalize();
		return true;
	}

	return false;
}

void b2CircleShape::ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const
{
	B2_NOT_USED(childIndex);

	b2Vec2 p = b2Vec2::Add(transform.p, b2Mul_r_v2(transform.q, this->m_p));
	aabb->lowerBound.Set(p.x - this->m_radius, p.y - this->m_radius);
	aabb->upperBound.Set(p.x + this->m_radius, p.y + this->m_radius);
}

void b2CircleShape::ComputeMass(b2MassData* massData, float32 density) const
{
	massData->mass = density * b2_pi * this->m_radius * this->m_radius;
	massData->center.Assign(this->m_p);

	// inertia about the local origin
	massData->I = massData->mass * (0.5 * this->m_radius * this->m_radius + b2Dot_v2_v2(this->m_p, this->m_p));
}
