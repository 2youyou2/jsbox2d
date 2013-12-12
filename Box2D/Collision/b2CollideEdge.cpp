/*
 * Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include <Box2D/Collision/b2Collision.h>
#include <Box2D/Collision/Shapes/b2CircleShape.h>
#include <Box2D/Collision/Shapes/b2EdgeShape.h>
#include <Box2D/Collision/Shapes/b2PolygonShape.h>


// Compute contact points for edge versus circle.
// This accounts for edge connectivity.
void b2CollideEdgeAndCircle(b2Manifold* manifold,
							const b2EdgeShape* edgeA, const b2Transform& xfA,
							const b2CircleShape* circleB, const b2Transform& xfB)
{
	manifold->pointCount = 0;
	
	// Compute circle in frame of edge
	b2Vec2 Q = b2MulT_t_v2(xfA, b2Mul_t_v2(xfB, circleB->m_p));
	
	b2Vec2 A = edgeA->m_vertex1, B = edgeA->m_vertex2;
	b2Vec2 e = b2Vec2::Subtract(B, A);
	
	// Barycentric coordinates
	float32 u = b2Dot_v2_v2(e, b2Vec2::Subtract(B, Q));
	float32 v = b2Dot_v2_v2(e, b2Vec2::Subtract(Q, A));
	
	float32 radius = edgeA->m_radius + circleB->m_radius;
	
	b2ContactFeature cf;
	cf.indexB = 0;
	cf.typeB = b2ContactFeature::e_vertex;
	
	// Region A
	if (v <= 0.0)
	{
		b2Vec2 P = A;
		b2Vec2 d = b2Vec2::Subtract(Q, P);
		float32 dd = b2Dot_v2_v2(d, d);
		if (dd > radius * radius)
		{
			return;
		}
		
		// Is there an edge connected to A?
		if (edgeA->m_hasVertex0)
		{
			b2Vec2 A1 = edgeA->m_vertex0;
			b2Vec2 B1 = A;
			b2Vec2 e1 = b2Vec2::Subtract(B1, A1);
			float32 u1 = b2Dot_v2_v2(e1, b2Vec2::Subtract(B1, Q));
			
			// Is the circle in Region AB of the previous edge?
			if (u1 > 0.0)
			{
				return;
			}
		}
		
		cf.indexA = 0;
		cf.typeA = b2ContactFeature::e_vertex;
		manifold->pointCount = 1;
		manifold->type = b2Manifold::e_circles;
		manifold->localNormal.SetZero();
		manifold->localPoint.Assign(P);
		manifold->points[0].id.key = 0;
		manifold->points[0].id.cf = cf;
		manifold->points[0].localPoint.Assign(circleB->m_p);
		return;
	}
	
	// Region B
	if (u <= 0.0)
	{
		b2Vec2 P = B;
		b2Vec2 d = b2Vec2::Subtract(Q, P);
		float32 dd = b2Dot_v2_v2(d, d);
		if (dd > radius * radius)
		{
			return;
		}
		
		// Is there an edge connected to B?
		if (edgeA->m_hasVertex3)
		{
			b2Vec2 B2 = edgeA->m_vertex3;
			b2Vec2 A2 = B;
			b2Vec2 e2 = b2Vec2::Subtract(B2, A2);
			float32 v2 = b2Dot_v2_v2(e2, b2Vec2::Subtract(Q, A2));
			
			// Is the circle in Region AB of the next edge?
			if (v2 > 0.0)
			{
				return;
			}
		}
		
		cf.indexA = 1;
		cf.typeA = b2ContactFeature::e_vertex;
		manifold->pointCount = 1;
		manifold->type = b2Manifold::e_circles;
		manifold->localNormal.SetZero();
		manifold->localPoint.Assign(P);
		manifold->points[0].id.key = 0;
		manifold->points[0].id.cf = cf;
		manifold->points[0].localPoint.Assign(circleB->m_p);
		return;
	}
	
	// Region AB
	float32 den = b2Dot_v2_v2(e, e);
	b2Assert(den > 0.0);
	b2Vec2 P = b2Vec2::Multiply((1.0 / den), b2Vec2::Add(b2Vec2::Multiply(u, A), b2Vec2::Multiply(v, B)));
	b2Vec2 d = b2Vec2::Subtract(Q, P);
	float32 dd = b2Dot_v2_v2(d, d);
	if (dd > radius * radius)
	{
		return;
	}
	
	b2Vec2 n(-e.y, e.x);
	if (b2Dot_v2_v2(n, b2Vec2::Subtract(Q, A)) < 0.0)
	{
		n.Set(-n.x, -n.y);
	}
	n.Normalize();
	
	cf.indexA = 0;
	cf.typeA = b2ContactFeature::e_face;
	manifold->pointCount = 1;
	manifold->type = b2Manifold::e_faceA;
	manifold->localNormal.Assign(n);
	manifold->localPoint.Assign(A);
	manifold->points[0].id.key = 0;
	manifold->points[0].id.cf = cf;
	manifold->points[0].localPoint.Assign(circleB->m_p);
}

// This structure is used to keep track of the best separating axis.
struct b2EPAxis
{
	int type;
	int32 index;
	float32 separation;

	static const int e_unknown = 0;
	static const int e_edgeA = 1;
	static const int e_edgeB = 2;
};

// This holds polygon B expressed in frame A.
struct b2TempPolygon
{
	b2Vec2 vertices[b2_maxPolygonVertices];
	b2Vec2 normals[b2_maxPolygonVertices];
	int32 count;
};

// Reference face used for clipping
struct b2ReferenceFace
{
	int32 i1, i2;
	
	b2Vec2 v1, v2;
	
	b2Vec2 normal;
	
	b2Vec2 sideNormal1;
	float32 sideOffset1;
	
	b2Vec2 sideNormal2;
	float32 sideOffset2;
};

// This class collides and edge and a polygon, taking into account edge adjacency.
struct b2EPCollider
{
	void Collide(b2Manifold* manifold, const b2EdgeShape* edgeA, const b2Transform& xfA,
				 const b2PolygonShape* polygonB, const b2Transform& xfB);
	b2EPAxis ComputeEdgeSeparation();
	b2EPAxis ComputePolygonSeparation();
	
	b2TempPolygon m_polygonB;
	
	b2Transform m_xf;
	b2Vec2 m_centroidB;
	b2Vec2 m_v0, m_v1, m_v2, m_v3;
	b2Vec2 m_normal0, m_normal1, m_normal2;
	b2Vec2 m_normal;
	int m_type1, m_type2;
	b2Vec2 m_lowerLimit, m_upperLimit;
	float32 m_radius;
	bool m_front;

	static const int e_isolated = 0;
	static const int e_concave = 1;
	static const int e_convex = 2;
};

// Algorithm:
// 1. Classify v1 and v2
// 2. Classify polygon centroid as front or back
// 3. Flip normal if necessary
// 4. Initialize normal range to [-pi, pi] about face normal
// 5. Adjust normal range according to adjacent edges
// 6. Visit each separating axes, only accept axes within the range
// 7. Return if _any_ axis indicates separation
// 8. Clip
void b2EPCollider::Collide(b2Manifold* manifold, const b2EdgeShape* edgeA, const b2Transform& xfA,
						   const b2PolygonShape* polygonB, const b2Transform& xfB)
{
	this->m_xf.Assign(b2MulT_t_t(xfA, xfB));
	
	this->m_centroidB.Assign(b2Mul_t_v2(this->m_xf, polygonB->m_centroid));
	
	this->m_v0.Assign(edgeA->m_vertex0);
	this->m_v1.Assign(edgeA->m_vertex1);
	this->m_v2.Assign(edgeA->m_vertex2);
	this->m_v3.Assign(edgeA->m_vertex3);
	
	bool hasVertex0 = edgeA->m_hasVertex0;
	bool hasVertex3 = edgeA->m_hasVertex3;
	
	b2Vec2 edge1 = b2Vec2::Subtract(this->m_v2, this->m_v1);
	edge1.Normalize();
	this->m_normal1.Set(edge1.y, -edge1.x);
	float32 offset1 = b2Dot_v2_v2(this->m_normal1, b2Vec2::Subtract(this->m_centroidB, this->m_v1));
	float32 offset0 = 0.0, offset2 = 0.0;
	bool convex1 = false, convex2 = false;
	
	// Is there a preceding edge?
	if (hasVertex0)
	{
		b2Vec2 edge0 = b2Vec2::Subtract(this->m_v1, this->m_v0);
		edge0.Normalize();
		this->m_normal0.Set(edge0.y, -edge0.x);
		convex1 = b2Cross_v2_v2(edge0, edge1) >= 0.0;
		offset0 = b2Dot_v2_v2(this->m_normal0, b2Vec2::Subtract(this->m_centroidB, this->m_v0));
	}
	
	// Is there a following edge?
	if (hasVertex3)
	{
		b2Vec2 edge2 = b2Vec2::Subtract(this->m_v3, this->m_v2);
		edge2.Normalize();
		this->m_normal2.Set(edge2.y, -edge2.x);
		convex2 = b2Cross_v2_v2(edge1, edge2) > 0.0;
		offset2 = b2Dot_v2_v2(this->m_normal2, b2Vec2::Subtract(this->m_centroidB, this->m_v2));
	}
	
	// Determine front or back collision. Determine collision normal limits.
	if (hasVertex0 && hasVertex3)
	{
		if (convex1 && convex2)
		{
			this->m_front = offset0 >= 0.0 || offset1 >= 0.0 || offset2 >= 0.0;
			if (this->m_front)
			{
				this->m_normal.Assign(this->m_normal1);
				this->m_lowerLimit.Assign(this->m_normal0);
				this->m_upperLimit.Assign(this->m_normal2);
			}
			else
			{
				this->m_normal.Assign(this->m_normal1.Negate());
				this->m_lowerLimit.Assign(this->m_normal1.Negate());
				this->m_upperLimit.Assign(this->m_normal1.Negate());
			}
		}
		else if (convex1)
		{
			this->m_front = offset0 >= 0.0 || (offset1 >= 0.0 && offset2 >= 0.0);
			if (this->m_front)
			{
				this->m_normal.Assign(this->m_normal1);
				this->m_lowerLimit.Assign(this->m_normal0);
				this->m_upperLimit.Assign(this->m_normal1);
			}
			else
			{
				this->m_normal.Assign(this->m_normal1.Negate());
				this->m_lowerLimit.Assign(this->m_normal2.Negate());
				this->m_upperLimit.Assign(this->m_normal1.Negate());
			}
		}
		else if (convex2)
		{
			this->m_front = offset2 >= 0.0 || (offset0 >= 0.0 && offset1 >= 0.0);
			if (this->m_front)
			{
				this->m_normal.Assign(this->m_normal1);
				this->m_lowerLimit.Assign(this->m_normal1);
				this->m_upperLimit.Assign(this->m_normal2);
			}
			else
			{
				this->m_normal.Assign(this->m_normal1.Negate());
				this->m_lowerLimit.Assign(this->m_normal1.Negate());
				this->m_upperLimit.Assign(this->m_normal0.Negate());
			}
		}
		else
		{
			this->m_front = offset0 >= 0.0 && offset1 >= 0.0 && offset2 >= 0.0;
			if (this->m_front)
			{
				this->m_normal.Assign(this->m_normal1);
				this->m_lowerLimit.Assign(this->m_normal1);
				this->m_upperLimit.Assign(this->m_normal1);
			}
			else
			{
				this->m_normal.Assign(this->m_normal1.Negate());
				this->m_lowerLimit.Assign(this->m_normal2.Negate());
				this->m_upperLimit.Assign(this->m_normal0.Negate());
			}
		}
	}
	else if (hasVertex0)
	{
		if (convex1)
		{
			this->m_front = offset0 >= 0.0 || offset1 >= 0.0;
			if (this->m_front)
			{
				this->m_normal.Assign(this->m_normal1);
				this->m_lowerLimit.Assign(this->m_normal0);
				this->m_upperLimit.Assign(this->m_normal1.Negate());
			}
			else
			{
				this->m_normal.Assign(this->m_normal1.Negate());
				this->m_lowerLimit.Assign(this->m_normal1);
				this->m_upperLimit.Assign(this->m_normal1.Negate());
			}
		}
		else
		{
			this->m_front = offset0 >= 0.0 && offset1 >= 0.0;
			if (this->m_front)
			{
				this->m_normal.Assign(this->m_normal1);
				this->m_lowerLimit.Assign(this->m_normal1);
				this->m_upperLimit.Assign(this->m_normal1.Negate());
			}
			else
			{
				this->m_normal.Assign(this->m_normal1.Negate());
				this->m_lowerLimit.Assign(this->m_normal1);
				this->m_upperLimit.Assign(this->m_normal0.Negate());
			}
		}
	}
	else if (hasVertex3)
	{
		if (convex2)
		{
			this->m_front = offset1 >= 0.0 || offset2 >= 0.0;
			if (this->m_front)
			{
				this->m_normal.Assign(this->m_normal1);
				this->m_lowerLimit.Assign(this->m_normal1.Negate());
				this->m_upperLimit.Assign(this->m_normal2);
			}
			else
			{
				this->m_normal.Assign(this->m_normal1.Negate());
				this->m_lowerLimit.Assign(this->m_normal1.Negate());
				this->m_upperLimit.Assign(this->m_normal1);
			}
		}
		else
		{
			this->m_front = offset1 >= 0.0 && offset2 >= 0.0;
			if (this->m_front)
			{
				this->m_normal.Assign(this->m_normal1);
				this->m_lowerLimit.Assign(this->m_normal1.Negate());
				this->m_upperLimit.Assign(this->m_normal1);
			}
			else
			{
				this->m_normal.Assign(this->m_normal1.Negate());
				this->m_lowerLimit.Assign(this->m_normal2.Negate());
				this->m_upperLimit.Assign(this->m_normal1);
			}
		}		
	}
	else
	{
		this->m_front = offset1 >= 0.0;
		if (this->m_front)
		{
			this->m_normal.Assign(this->m_normal1);
			this->m_lowerLimit.Assign(this->m_normal1.Negate());
			this->m_upperLimit.Assign(this->m_normal1.Negate());
		}
		else
		{
			this->m_normal.Assign(this->m_normal1.Negate());
			this->m_lowerLimit.Assign(this->m_normal1);
			this->m_upperLimit.Assign(this->m_normal1);
		}
	}
	
	// Get polygonB in frameA
	this->m_polygonB.count = polygonB->m_count;
	for (int32 i = 0; i < polygonB->m_count; ++i)
	{
		this->m_polygonB.vertices[i].Assign(b2Mul_t_v2(this->m_xf, polygonB->m_vertices[i]));
		this->m_polygonB.normals[i].Assign(b2Mul_r_v2(this->m_xf.q, polygonB->m_normals[i]));
	}
	
	this->m_radius = 2.0 * b2_polygonRadius;
	
	manifold->pointCount = 0;
	
	b2EPAxis edgeAxis = ComputeEdgeSeparation();
	
	// If no valid normal can be found than this edge should not collide.
	if (edgeAxis.type == b2EPAxis::e_unknown)
	{
		return;
	}
	
	if (edgeAxis.separation > this->m_radius)
	{
		return;
	}
	
	b2EPAxis polygonAxis = ComputePolygonSeparation();
	if (polygonAxis.type != b2EPAxis::e_unknown && polygonAxis.separation > this->m_radius)
	{
		return;
	}
	
	// Use hysteresis for jitter reduction.
	const float32 k_relativeTol = 0.98f;
	const float32 k_absoluteTol = 0.001f;
	
	b2EPAxis primaryAxis;
	if (polygonAxis.type == b2EPAxis::e_unknown)
	{
		primaryAxis = edgeAxis;
	}
	else if (polygonAxis.separation > k_relativeTol * edgeAxis.separation + k_absoluteTol)
	{
		primaryAxis = polygonAxis;
	}
	else
	{
		primaryAxis = edgeAxis;
	}
	
	b2ClipVertex ie[2];
	b2ReferenceFace rf;
	if (primaryAxis.type == b2EPAxis::e_edgeA)
	{
		manifold->type = b2Manifold::e_faceA;
		
		// Search for the polygon normal that is most anti-parallel to the edge normal.
		int32 bestIndex = 0;
		float32 bestValue = b2Dot_v2_v2(this->m_normal, this->m_polygonB.normals[0]);
		for (int32 i = 1; i < this->m_polygonB.count; ++i)
		{
			float32 value = b2Dot_v2_v2(this->m_normal, this->m_polygonB.normals[i]);
			if (value < bestValue)
			{
				bestValue = value;
				bestIndex = i;
			}
		}
		
		int32 i1 = bestIndex;
		int32 i2 = i1 + 1 < this->m_polygonB.count ? i1 + 1 : 0;
		
		ie[0].v.Assign(this->m_polygonB.vertices[i1]);
		ie[0].id.cf.indexA = 0;
		ie[0].id.cf.indexB = static_cast<uint8>(i1);
		ie[0].id.cf.typeA = b2ContactFeature::e_face;
		ie[0].id.cf.typeB = b2ContactFeature::e_vertex;
		
		ie[1].v.Assign(this->m_polygonB.vertices[i2]);
		ie[1].id.cf.indexA = 0;
		ie[1].id.cf.indexB = static_cast<uint8>(i2);
		ie[1].id.cf.typeA = b2ContactFeature::e_face;
		ie[1].id.cf.typeB = b2ContactFeature::e_vertex;
		
		if (this->m_front)
		{
			rf.i1 = 0;
			rf.i2 = 1;
			rf.v1.Assign(this->m_v1);
			rf.v2.Assign(this->m_v2);
			rf.normal.Assign(this->m_normal1);
		}
		else
		{
			rf.i1 = 1;
			rf.i2 = 0;
			rf.v1.Assign(this->m_v2);
			rf.v2.Assign(this->m_v1);
			rf.normal.Assign(this->m_normal1.Negate());
		}		
	}
	else
	{
		manifold->type = b2Manifold::e_faceB;
		
		ie[0].v.Assign(this->m_v1);
		ie[0].id.cf.indexA = 0;
		ie[0].id.cf.indexB = static_cast<uint8>(primaryAxis.index);
		ie[0].id.cf.typeA = b2ContactFeature::e_vertex;
		ie[0].id.cf.typeB = b2ContactFeature::e_face;
		
		ie[1].v.Assign(this->m_v2);
		ie[1].id.cf.indexA = 0;
		ie[1].id.cf.indexB = static_cast<uint8>(primaryAxis.index);		
		ie[1].id.cf.typeA = b2ContactFeature::e_vertex;
		ie[1].id.cf.typeB = b2ContactFeature::e_face;
		
		rf.i1 = primaryAxis.index;
		rf.i2 = rf.i1 + 1 < this->m_polygonB.count ? rf.i1 + 1 : 0;
		rf.v1.Assign(this->m_polygonB.vertices[rf.i1]);
		rf.v2.Assign(this->m_polygonB.vertices[rf.i2]);
		rf.normal.Assign(this->m_polygonB.normals[rf.i1]);
	}
	
	rf.sideNormal1.Set(rf.normal.y, -rf.normal.x);
	rf.sideNormal2.Assign(rf.sideNormal1.Negate());
	rf.sideOffset1 = b2Dot_v2_v2(rf.sideNormal1, rf.v1);
	rf.sideOffset2 = b2Dot_v2_v2(rf.sideNormal2, rf.v2);
	
	// Clip incident edge against extruded edge1 side edges.
	b2ClipVertex clipPoints1[2];
	b2ClipVertex clipPoints2[2];
	int32 np;
	
	// Clip to box side 1
	np = b2ClipSegmentToLine(clipPoints1, ie, rf.sideNormal1, rf.sideOffset1, rf.i1);
	
	if (np < b2_maxManifoldPoints)
	{
		return;
	}
	
	// Clip to negative box side 1
	np = b2ClipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2);
	
	if (np < b2_maxManifoldPoints)
	{
		return;
	}
	
	// Now clipPoints2 contains the clipped points.
	if (primaryAxis.type == b2EPAxis::e_edgeA)
	{
		manifold->localNormal.Assign(rf.normal);
		manifold->localPoint.Assign(rf.v1);
	}
	else
	{
		manifold->localNormal.Assign(polygonB->m_normals[rf.i1]);
		manifold->localPoint.Assign(polygonB->m_vertices[rf.i1]);
	}
	
	int32 pointCount = 0;
	for (int32 i = 0; i < b2_maxManifoldPoints; ++i)
	{
		float32 separation;
		
		separation = b2Dot_v2_v2(rf.normal, b2Vec2::Subtract(clipPoints2[i].v, rf.v1));
		
		if (separation <= this->m_radius)
		{
			b2ManifoldPoint* cp = manifold->points + pointCount;
			
			if (primaryAxis.type == b2EPAxis::e_edgeA)
			{
				cp->localPoint.Assign(b2MulT_t_v2(this->m_xf, clipPoints2[i].v));
				cp->id = clipPoints2[i].id;
			}
			else
			{
				cp->localPoint.Assign(clipPoints2[i].v);
				cp->id.cf.typeA = clipPoints2[i].id.cf.typeB;
				cp->id.cf.typeB = clipPoints2[i].id.cf.typeA;
				cp->id.cf.indexA = clipPoints2[i].id.cf.indexB;
				cp->id.cf.indexB = clipPoints2[i].id.cf.indexA;
			}
			
			++pointCount;
		}
	}
	
	manifold->pointCount = pointCount;
}

b2EPAxis b2EPCollider::ComputeEdgeSeparation()
{
	b2EPAxis axis;
	axis.type = b2EPAxis::e_edgeA;
	axis.index = this->m_front ? 0 : 1;
	axis.separation = FLT_MAX;
	
	for (int32 i = 0; i < this->m_polygonB.count; ++i)
	{
		float32 s = b2Dot_v2_v2(this->m_normal, b2Vec2::Subtract(this->m_polygonB.vertices[i], this->m_v1));
		if (s < axis.separation)
		{
			axis.separation = s;
		}
	}
	
	return axis;
}

b2EPAxis b2EPCollider::ComputePolygonSeparation()
{
	b2EPAxis axis;
	axis.type = b2EPAxis::e_unknown;
	axis.index = -1;
	axis.separation = -FLT_MAX;

	b2Vec2 perp(-this->m_normal.y, this->m_normal.x);

	for (int32 i = 0; i < this->m_polygonB.count; ++i)
	{
		b2Vec2 n = this->m_polygonB.normals[i].Negate();
		
		float32 s1 = b2Dot_v2_v2(n, b2Vec2::Subtract(this->m_polygonB.vertices[i], this->m_v1));
		float32 s2 = b2Dot_v2_v2(n, b2Vec2::Subtract(this->m_polygonB.vertices[i], this->m_v2));
		float32 s = b2Min(s1, s2);
		
		if (s > this->m_radius)
		{
			// No collision
			axis.type = b2EPAxis::e_edgeB;
			axis.index = i;
			axis.separation = s;
			return axis;
		}
		
		// Adjacency
		if (b2Dot_v2_v2(n, perp) >= 0.0)
		{
			if (b2Dot_v2_v2(b2Vec2::Subtract(n, this->m_upperLimit), this->m_normal) < -b2_angularSlop)
			{
				continue;
			}
		}
		else
		{
			if (b2Dot_v2_v2(b2Vec2::Subtract(n, this->m_lowerLimit), this->m_normal) < -b2_angularSlop)
			{
				continue;
			}
		}
		
		if (s > axis.separation)
		{
			axis.type = b2EPAxis::e_edgeB;
			axis.index = i;
			axis.separation = s;
		}
	}
	
	return axis;
}

void b2CollideEdgeAndPolygon(	b2Manifold* manifold,
							 const b2EdgeShape* edgeA, const b2Transform& xfA,
							 const b2PolygonShape* polygonB, const b2Transform& xfB)
{
	b2EPCollider collider;
	collider.Collide(manifold, edgeA, xfA, polygonB, xfB);
}
