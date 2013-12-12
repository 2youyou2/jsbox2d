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
#include <Box2D/Collision/b2Distance.h>

void b2WorldManifold::Initialize(const b2Manifold* manifold,
						  const b2Transform& xfA, float32 radiusA,
						  const b2Transform& xfB, float32 radiusB)
{
	if (manifold->pointCount == 0)
	{
		return;
	}

	switch (manifold->type)
	{
	case b2Manifold::e_circles:
		{
			this->normal.Set(1.0, 0.0);
			b2Vec2 pointA = b2Mul_t_v2(xfA, manifold->localPoint);
			b2Vec2 pointB = b2Mul_t_v2(xfB, manifold->points[0].localPoint);
			if (b2DistanceSquared(pointA, pointB) > b2_epsilon * b2_epsilon)
			{
				this->normal.Assign(b2Vec2::Subtract(pointB, pointA));
				this->normal.Normalize();
			}

			b2Vec2 cA = b2Vec2::Add(pointA, b2Vec2::Multiply(radiusA, this->normal));
			b2Vec2 cB = b2Vec2::Subtract(pointB, b2Vec2::Multiply(radiusB, this->normal));
			this->points[0].Assign(b2Vec2::Multiply(0.5, b2Vec2::Add(cA, cB)));
			this->separations[0] = b2Dot_v2_v2(b2Vec2::Subtract(cB, cA), this->normal);
		}
		break;

	case b2Manifold::e_faceA:
		{
			this->normal.Assign(b2Mul_r_v2(xfA.q, manifold->localNormal));
			b2Vec2 planePoint = b2Mul_t_v2(xfA, manifold->localPoint);

			for (int32 i = 0; i < manifold->pointCount; ++i)
			{
				b2Vec2 clipPoint = b2Mul_t_v2(xfB, manifold->points[i].localPoint);
				b2Vec2 cA = b2Vec2::Add(clipPoint, b2Vec2::Multiply((radiusA - b2Dot_v2_v2(b2Vec2::Subtract(clipPoint, planePoint), this->normal)), this->normal));
				b2Vec2 cB = b2Vec2::Subtract(clipPoint, b2Vec2::Multiply(radiusB, this->normal));
				this->points[i].Assign(b2Vec2::Multiply(0.5, b2Vec2::Add(cA, cB)));
				this->separations[i] = b2Dot_v2_v2(b2Vec2::Subtract(cB, cA), this->normal);
			}
		}
		break;

	case b2Manifold::e_faceB:
		{
			this->normal.Assign(b2Mul_r_v2(xfB.q, manifold->localNormal));
			b2Vec2 planePoint = b2Mul_t_v2(xfB, manifold->localPoint);

			for (int32 i = 0; i < manifold->pointCount; ++i)
			{
				b2Vec2 clipPoint = b2Mul_t_v2(xfA, manifold->points[i].localPoint);
				b2Vec2 cB = b2Vec2::Add(clipPoint, b2Vec2::Multiply((radiusB - b2Dot_v2_v2(b2Vec2::Subtract(clipPoint, planePoint), this->normal)), this->normal));
				b2Vec2 cA = b2Vec2::Subtract(clipPoint, b2Vec2::Multiply(radiusA, this->normal));
				this->points[i].Assign(b2Vec2::Multiply(0.5, b2Vec2::Add(cA, cB)));
				this->separations[i] = b2Dot_v2_v2(b2Vec2::Subtract(cA, cB), this->normal);
			}

			// Ensure normal points from A to B.
			this->normal.Assign(this->normal.Negate());
		}
		break;
	}
}

void b2GetPointStates(int state1[b2_maxManifoldPoints], int state2[b2_maxManifoldPoints],
					  const b2Manifold* manifold1, const b2Manifold* manifold2)
{
	for (int32 i = 0; i < b2_maxManifoldPoints; ++i)
	{
		state1[i] = b2Manifold::b2_nullState;
		state2[i] = b2Manifold::b2_nullState;
	}

	// Detect persists and removes.
	for (int32 i = 0; i < manifold1->pointCount; ++i)
	{
		b2ContactID id = manifold1->points[i].id;

		state1[i] = b2Manifold::b2_removeState;

		for (int32 j = 0; j < manifold2->pointCount; ++j)
		{
			if (manifold2->points[j].id.Get() == id.Get())
			{
				state1[i] = b2Manifold::b2_persistState;
				break;
			}
		}
	}

	// Detect persists and adds.
	for (int32 i = 0; i < manifold2->pointCount; ++i)
	{
		b2ContactID id = manifold2->points[i].id;

		state2[i] = b2Manifold::b2_addState;

		for (int32 j = 0; j < manifold1->pointCount; ++j)
		{
			if (manifold1->points[j].id.Get() == id.Get())
			{
				state2[i] = b2Manifold::b2_persistState;
				break;
			}
		}
	}
}

// From Real-time Collision Detection, p179.
bool b2AABB::RayCast(b2RayCastOutput* output, const b2RayCastInput& input) const
{
	float32 tmin = -b2_maxFloat;
	float32 tmax = b2_maxFloat;

	b2Vec2 p = input.p1;
	b2Vec2 d = b2Vec2::Subtract(input.p2, input.p1);
	b2Vec2 absD = b2Abs_v2(d);

	b2Vec2 normal;

	for (int32 i = 0; i < 2; ++i)
	{
		if (absD.get_i(i) < b2_epsilon)
		{
			// Parallel.
			if (p.get_i(i) < this->lowerBound.get_i(i) || this->upperBound.get_i(i) < p.get_i(i))
			{
				return false;
			}
		}
		else
		{
			float32 inv_d = 1.0 / d.get_i(i);
			float32 t1 = (this->lowerBound.get_i(i) - p.get_i(i)) * inv_d;
			float32 t2 = (this->upperBound.get_i(i) - p.get_i(i)) * inv_d;

			// Sign of the normal vector.
			float32 s = -1.0;

			if (t1 > t2)
			{
				float32 temp = t2;
				t2 = t1;
				t1 = temp;
				s = 1.0;
			}

			// Push the min up
			if (t1 > tmin)
			{
				normal.SetZero();
				normal.set_i(i, s);
				tmin = t1;
			}

			// Pull the max down
			tmax = b2Min(tmax, t2);

			if (tmin > tmax)
			{
				return false;
			}
		}
	}

	// Does the ray start inside the box?
	// Does the ray intersect beyond the max fraction?
	if (tmin < 0.0 || input.maxFraction < tmin)
	{
		return false;
	}

	// Intersection.
	output->fraction = tmin;
	output->normal.Assign(normal);
	return true;
}

// Sutherland-Hodgman clipping.
int32 b2ClipSegmentToLine(b2ClipVertex vOut[2], const b2ClipVertex vIn[2],
						const b2Vec2& normal, float32 offset, int32 vertexIndexA)
{
	// Start with no output points
	int32 numOut = 0;

	// Calculate the distance of end points to the line
	float32 distance0 = b2Dot_v2_v2(normal, vIn[0].v) - offset;
	float32 distance1 = b2Dot_v2_v2(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0) vOut[numOut++] = vIn[0];
	if (distance1 <= 0.0) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0)
	{
		// Find intersection point of edge and plane
		float32 interp = distance0 / (distance0 - distance1);
		vOut[numOut].v.Assign(b2Vec2::Add(vIn[0].v, b2Vec2::Multiply(interp, b2Vec2::Subtract(vIn[1].v, vIn[0].v))));

		// VertexA is hitting edgeB.
		vOut[numOut].id.indexA = static_cast<uint8>(vertexIndexA);
		vOut[numOut].id.indexB = vIn[0].id.indexB;
		vOut[numOut].id.typeA = b2ContactID::e_vertex;
		vOut[numOut].id.typeB = b2ContactID::e_face;
		++numOut;
	}

	return numOut;
}

bool b2TestOverlap(	const b2Shape* shapeA, int32 indexA,
					const b2Shape* shapeB, int32 indexB,
					const b2Transform& xfA, const b2Transform& xfB)
{
	b2DistanceInput input;
	input.proxyA.Set(shapeA, indexA);
	input.proxyB.Set(shapeB, indexB);
	input.transformA.Assign(xfA);
	input.transformB.Assign(xfB);
	input.useRadii = true;

	b2SimplexCache cache;
	cache.count = 0;

	b2DistanceOutput output;

	b2DistanceFunc(&output, &cache, &input);

	return output.distance < 10.0 * b2_epsilon;
}
