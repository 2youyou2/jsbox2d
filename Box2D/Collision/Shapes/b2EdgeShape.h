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

#ifndef B2_EDGE_SHAPE_H
#define B2_EDGE_SHAPE_H

#include <Box2D/Collision/Shapes/b2Shape.h>

/// A line segment (edge) shape. These can be connected in chains or loops
/// to other edge shapes. The connectivity information is used to ensure
/// correct contact normals.
class b2EdgeShape : public b2Shape
{
public:
	b2EdgeShape();

	/// Set this as an isolated edge.
	void Set(const b2Vec2& v1, const b2Vec2& v2);

	/// Implement b2Shape.
	b2Shape* Clone(b2BlockAllocator* allocator) const;

	/// @see b2Shape::GetChildCount
	int32 GetChildCount() const;

	/// @see b2Shape::TestPoint
	bool TestPoint(const b2Transform& transform, const b2Vec2& p) const;

	/// Implement b2Shape.
	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
				const b2Transform& transform, int32 childIndex) const;

	/// @see b2Shape::ComputeAABB
	void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int32 childIndex) const;

	/// @see b2Shape::ComputeMass
	void ComputeMass(b2MassData* massData, float32 density) const;
	
	/// These are the edge vertices
	b2Vec2 m_vertex1, m_vertex2;

	/// Optional adjacent vertices. These are used for smooth collision.
	b2Vec2 m_vertex0, m_vertex3;
	bool m_hasVertex0, m_hasVertex3;

	b2EdgeShape &operator =(const b2EdgeShape &r)
	{
		this->m_hasVertex0 = r.m_hasVertex0;
		this->m_hasVertex3 = r.m_hasVertex3;
		this->m_vertex0.Assign(r.m_vertex0);
		this->m_vertex1.Assign(r.m_vertex1);
		this->m_vertex2.Assign(r.m_vertex2);
		this->m_vertex3.Assign(r.m_vertex3);

		this->m_type = r.m_type;
		this->m_radius = r.m_radius;

		return *this;
	}
};

inline b2EdgeShape::b2EdgeShape()
{
	this->m_type = b2Shape::e_edge;
	this->m_radius = b2_polygonRadius;
	this->m_vertex0.x = 0.0;
	this->m_vertex0.y = 0.0;
	this->m_vertex3.x = 0.0;
	this->m_vertex3.y = 0.0;
	this->m_hasVertex0 = false;
	this->m_hasVertex3 = false;
}

#endif
