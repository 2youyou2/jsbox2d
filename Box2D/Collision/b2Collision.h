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

#ifndef B2_COLLISION_H
#define B2_COLLISION_H

#include <Box2D/Common/b2Math.h>
#include <limits.h>

/// @file
/// Structures and functions used for computing contact points, distance
/// queries, and TOI queries.

class b2Shape;
class b2CircleShape;
class b2EdgeShape;
class b2PolygonShape;

const uint8 b2_nullFeature = UCHAR_MAX;

/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
struct b2ContactFeature
{
	uint8 indexA;		///< Feature index on shapeA
	uint8 indexB;		///< Feature index on shapeB
	uint8 typeA;		///< The feature type on shapeA
	uint8 typeB;		///< The feature type on shapeB

	static const int e_vertex = 0;
	static const int e_face = 1;
};

/// Contact ids to facilitate warm starting.
union b2ContactID
{
	b2ContactFeature cf;
	uint32 key;					///< Used to quickly compare contact ids.
};

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleB
/// -e_faceA: the local center of cirlceB or the clip point of polygonB
/// -e_faceB: the clip point of polygonA
/// This structure is stored across time steps, so we keep it small.
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
struct b2ManifoldPoint
{
	b2Vec2 localPoint;		///< usage depends on manifold type
	float32 normalImpulse;	///< the non-penetration impulse
	float32 tangentImpulse;	///< the friction impulse
	b2ContactID id;			///< uniquely identifies a contact point between two shapes
};

/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleA
/// -e_faceA: the center of faceA
/// -e_faceB: the center of faceB
/// Similarly the local normal usage:
/// -e_circles: not used
/// -e_faceA: the normal on polygonA
/// -e_faceB: the normal on polygonB
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.
struct b2Manifold
{
	b2ManifoldPoint points[b2_maxManifoldPoints];	///< the points of contact
	b2Vec2 localNormal;								///< not use for Type::e_points
	b2Vec2 localPoint;								///< usage depends on manifold type
	int type;
	int32 pointCount;								///< the number of manifold points
	
	static const int e_circles = 0;
	static const int e_faceA = 1;
	static const int e_faceB = 2;

	/// This is used for determining the state of contact points.
	static const int b2_nullState = 0;		///< point does not exist
	static const int b2_addState = 1;		///< point was added in the update
	static const int b2_persistState = 2;	///< point persisted across the update
	static const int b2_removeState = 3;		///< point was removed in the update
};

/// This is used to compute the current state of a contact manifold.
struct b2WorldManifold
{
	/// Evaluate the manifold with supplied transforms. This assumes
	/// modest motion from the original state. This does not change the
	/// point count, impulses, etc. The radii must come from the shapes
	/// that generated the manifold.
	void Initialize(const b2Manifold* manifold,
					const b2Transform& xfA, float32 radiusA,
					const b2Transform& xfB, float32 radiusB);

	b2Vec2 normal;								///< world vector pointing from A to B
	b2Vec2 points[b2_maxManifoldPoints];		///< world contact point (point of intersection)
	float32 separations[b2_maxManifoldPoints];	///< a negative value indicates overlap, in meters
};

/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
void b2GetPointStates(int state1[b2_maxManifoldPoints], int state2[b2_maxManifoldPoints],
					  const b2Manifold* manifold1, const b2Manifold* manifold2);

/// Used for computing contact manifolds.
struct b2ClipVertex
{
	b2Vec2 v;
	b2ContactID id;

	b2ClipVertex &operator= (const b2ClipVertex &l)
	{
		this->v.Assign(l.v);
		this->id = l.id;

		return *this;
	}
};

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
struct b2RayCastInput
{
	b2Vec2 p1, p2;
	float32 maxFraction;
};

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
struct b2RayCastOutput
{
	b2Vec2 normal;
	float32 fraction;

	b2RayCastOutput &operator= (const b2RayCastOutput &l)
	{
		this->normal.Assign(l.normal);
		this->fraction = l.fraction;

		return *this;
	}
};

/// An axis aligned bounding box.
struct b2AABB
{
	void Assign(const b2AABB &l)
	{
		this->lowerBound.Assign(l.lowerBound);
		this->upperBound.Assign(l.upperBound);
	}

	/// Verify that the bounds are sorted.
	bool IsValid() const;

	/// Get the center of the AABB.
	b2Vec2 GetCenter() const
	{
		return b2Vec2::Multiply(0.5, b2Vec2::Add(this->lowerBound, this->upperBound));
	}

	/// Get the extents of the AABB (half-widths).
	b2Vec2 GetExtents() const
	{
		return b2Vec2::Multiply(0.5, b2Vec2::Subtract(this->upperBound, this->lowerBound));
	}

	/// Get the perimeter length
	float32 GetPerimeter() const
	{
		float32 wx = this->upperBound.x - this->lowerBound.x;
		float32 wy = this->upperBound.y - this->lowerBound.y;
		return 2.0 * (wx + wy);
	}

	/// Combine an AABB into this one.
	void Combine(const b2AABB& aabb)
	{
		this->lowerBound.Assign(b2Min_v2(this->lowerBound, aabb.lowerBound));
		this->upperBound.Assign(b2Max_v2(this->upperBound, aabb.upperBound));
	}

	/// Combine two AABBs into this one.
	void Combine(const b2AABB& aabb1, const b2AABB& aabb2)
	{
		this->lowerBound.Assign(b2Min_v2(aabb1.lowerBound, aabb2.lowerBound));
		this->upperBound.Assign(b2Max_v2(aabb1.upperBound, aabb2.upperBound));
	}

	/// Does this aabb contain the provided AABB.
	bool Contains(const b2AABB& aabb) const
	{
		bool result = true;
		result = result && this->lowerBound.x <= aabb.lowerBound.x;
		result = result && this->lowerBound.y <= aabb.lowerBound.y;
		result = result && aabb.upperBound.x <= this->upperBound.x;
		result = result && aabb.upperBound.y <= this->upperBound.y;
		return result;
	}

	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input) const;

	b2Vec2 lowerBound;	///< the lower vertex
	b2Vec2 upperBound;	///< the upper vertex

private:
	b2AABB &operator =(const b2AABB &r)
	{
		B2_NOT_USED(r);
		return *this;
	}
};

/// Compute the collision manifold between two circles.
void b2CollideCircles(b2Manifold* manifold,
					  const b2CircleShape* circleA, const b2Transform& xfA,
					  const b2CircleShape* circleB, const b2Transform& xfB);

/// Compute the collision manifold between a polygon and a circle.
void b2CollidePolygonAndCircle(b2Manifold* manifold,
							   const b2PolygonShape* polygonA, const b2Transform& xfA,
							   const b2CircleShape* circleB, const b2Transform& xfB);

/// Compute the collision manifold between two polygons.
void b2CollidePolygons(b2Manifold* manifold,
					   const b2PolygonShape* polygonA, const b2Transform& xfA,
					   const b2PolygonShape* polygonB, const b2Transform& xfB);

/// Compute the collision manifold between an edge and a circle.
void b2CollideEdgeAndCircle(b2Manifold* manifold,
							   const b2EdgeShape* polygonA, const b2Transform& xfA,
							   const b2CircleShape* circleB, const b2Transform& xfB);

/// Compute the collision manifold between an edge and a circle.
void b2CollideEdgeAndPolygon(b2Manifold* manifold,
							   const b2EdgeShape* edgeA, const b2Transform& xfA,
							   const b2PolygonShape* circleB, const b2Transform& xfB);

/// Clipping for contact manifolds.
int32 b2ClipSegmentToLine(b2ClipVertex vOut[2], const b2ClipVertex vIn[2],
							const b2Vec2& normal, float32 offset, int32 vertexIndexA);

/// Determine if two generic shapes overlap.
bool b2TestOverlap(	const b2Shape* shapeA, int32 indexA,
					const b2Shape* shapeB, int32 indexB,
					const b2Transform& xfA, const b2Transform& xfB);

// ---------------- Inline Functions ------------------------------------------

inline bool b2AABB::IsValid() const
{
	b2Vec2 d = b2Vec2::Subtract(this->upperBound, this->lowerBound);
	bool valid = d.x >= 0.0 && d.y >= 0.0;
	valid = valid && this->lowerBound.IsValid() && this->upperBound.IsValid();
	return valid;
}

inline bool b2TestOverlap(const b2AABB& a, const b2AABB& b)
{
	b2Vec2 d1, d2;
	d1.Assign(b2Vec2::Subtract(b.lowerBound, a.upperBound));
	d2.Assign(b2Vec2::Subtract(a.lowerBound, b.upperBound));

	if (d1.x > 0.0 || d1.y > 0.0)
		return false;

	if (d2.x > 0.0 || d2.y > 0.0)
		return false;

	return true;
}

#endif
