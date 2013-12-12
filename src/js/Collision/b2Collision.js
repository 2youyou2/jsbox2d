"use strict";

var b2_nullFeature = 255;

/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
function b2ContactID()
{
}

b2ContactID.prototype =
{
	indexA: 0,		///< Feature index on shapeA
	indexB: 0,		///< Feature index on shapeB
	typeA: 0,		///< The feature type on shapeA
	typeB: 0,		///< The feature type on shapeB

	Reset: function()
	{
		this.indexA = this.indexB = this.typeA = this.typeB = 0;
	},

	Get: function()
	{
		return this.indexA | (this.indexB << 8) | (this.typeA << 16) | (this.typeB << 24);
	},

	Assign: function(k)
	{
		this.indexA = k.indexA;
		this.indexB = k.indexB;
		this.typeA = k.typeA;
		this.typeB = k.typeB;
	}
};

b2ContactID.e_vertex = 0;
b2ContactID.e_face = 1;

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
function b2ManifoldPoint()
{
	this.localPoint = new b2Vec2();		///< usage depends on manifold type
	this.normalImpulse = 0;				///< the non-penetration impulse
	this.tangentImpulse = 0;			///< the friction impulse
	this.id = new b2ContactID();		///< uniquely identifies a contact point between two shapes
};

b2ManifoldPoint.prototype =
{
	Clone: function()
	{
		var point = new b2ManifoldPoint();
		point.localPoint.Assign(this.localPoint);
		point.normalImpulse = this.normalImpulse;
		point.tangentImpulse = this.tangentImpulse;
		point.id.Assign(this.id);
		return point;
	}
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
function b2Manifold()
{
	this.points = new Array(b2_maxManifoldPoints);	///< the points of contact
	this.localNormal = new b2Vec2();				///< not use for Type::e_points
	this.localPoint = new b2Vec2();						///< usage depends on manifold type
	this.type = 0;
	this.pointCount = 0;							///< the number of manifold points
};

b2Manifold.prototype =
{
	Clone: function()
	{
		var manifold = new b2Manifold();

		manifold.pointCount = this.pointCount;
		manifold.type = this.type;
		manifold.localPoint.Assign(this.localPoint);
		manifold.localNormal.Assign(this.localNormal);

		for (var i = 0; i < this.pointCount; ++i)
			manifold.points[i] = this.points[i].Clone();

		return manifold;
	}
};

b2Manifold.e_circles = 0;
b2Manifold.e_faceA = 1;
b2Manifold.e_faceB = 2;

/// This is used for determining the state of contact points.
b2Manifold.b2_nullState = 0;		///< point does not exist
b2Manifold.b2_addState = 1;		///< point was added in the update
b2Manifold.b2_persistState = 2;	///< point persisted across the update
b2Manifold.b2_removeState = 3;		///< point was removed in the update

/// This is used to compute the current state of a contact manifold.
function b2WorldManifold()
{
	this.normal = new b2Vec2();							///< world vector pointing from A to B
	this.points = new Array(b2_maxManifoldPoints);		///< world contact point (point of intersection)
	this.separations = new Array(b2_maxManifoldPoints);	///< a negative value indicates overlap, in meters
}

b2WorldManifold.prototype =
{
	/// Evaluate the manifold with supplied transforms. This assumes
	/// modest motion from the original state. This does not change the
	/// point count, impulses, etc. The radii must come from the shapes
	/// that generated the manifold.
	Initialize: function(manifold,
					xfA, radiusA,
					xfB, radiusB)
	{
		if (manifold.pointCount == 0)
		{
			return;
		}

		switch (manifold.type)
		{
		case b2Manifold.e_circles:
			{
				this.normal.Set(1.0, 0.0);
				var pointA = b2Mul_t_v2(xfA, manifold.localPoint);
				var pointB = b2Mul_t_v2(xfB, manifold.points[0].localPoint);
				if (b2DistanceSquared(pointA, pointB) > b2_epsilon * b2_epsilon)
				{
					this.normal = b2Vec2.Subtract(pointB, pointA);
					this.normal.Normalize();
				}

				var cA = b2Vec2.Add(pointA, b2Vec2.Multiply(radiusA, this.normal));
				var cB = b2Vec2.Subtract(pointB, b2Vec2.Multiply(radiusB, this.normal));
				this.points[0] = b2Vec2.Multiply(0.5, b2Vec2.Add(cA, cB));
				this.separations[0] = b2Dot_v2_v2(b2Vec2.Subtract(cB, cA), this.normal);
			}
			break;

		case b2Manifold.e_faceA:
			{
				this.normal = b2Mul_r_v2(xfA.q, manifold.localNormal);
				var planePoint = b2Mul_t_v2(xfA, manifold.localPoint);

				for (var i = 0; i < manifold.pointCount; ++i)
				{
					var clipPoint = b2Mul_t_v2(xfB, manifold.points[i].localPoint);
					var cA = b2Vec2.Add(clipPoint, b2Vec2.Multiply((radiusA - b2Dot_v2_v2(b2Vec2.Subtract(clipPoint, planePoint), this.normal)), this.normal));
					var cB = b2Vec2.Subtract(clipPoint, b2Vec2.Multiply(radiusB, this.normal));
					this.points[i] = b2Vec2.Multiply(0.5, b2Vec2.Add(cA, cB));
					this.separations[i] = b2Dot_v2_v2(b2Vec2.Subtract(cB, cA), this.normal);
				}
			}
			break;

		case b2Manifold.e_faceB:
			{
				this.normal = b2Mul_r_v2(xfB.q, manifold.localNormal);
				var planePoint = b2Mul_t_v2(xfB, manifold.localPoint);

				for (var i = 0; i < manifold.pointCount; ++i)
				{
					var clipPoint = b2Mul_t_v2(xfA, manifold.points[i].localPoint);
					var cB = b2Vec2.Add(clipPoint, b2Vec2.Multiply((radiusB - b2Dot_v2_v2(b2Vec2.Subtract(clipPoint, planePoint), this.normal)), this.normal));
					var cA = b2Vec2.Subtract(clipPoint, b2Vec2.Multiply(radiusA, this.normal));
					this.points[i] = b2Vec2.Multiply(0.5, b2Vec2.Add(cA, cB));
					this.separations[i] = b2Dot_v2_v2(b2Vec2.Subtract(cA, cB), this.normal);
				}

				// Ensure normal points from A to B.
				this.normal = this.normal.Negate();
			}
			break;
		}
	}
};

/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
function b2GetPointStates(state1, state2,
					  manifold1, manifold2)
{
	for (var i = 0; i < b2_maxManifoldPoints; ++i)
	{
		state1[i] = b2Manifold.b2_nullState;
		state2[i] = b2Manifold.b2_nullState;
	}

	// Detect persists and removes.
	for (var i = 0; i < manifold1.pointCount; ++i)
	{
		var id = manifold1.points[i].id;

		state1[i] = b2Manifold.b2_removeState;

		for (var j = 0; j < manifold2.pointCount; ++j)
		{
			if (manifold2.points[j].id.Get() == id.Get())
			{
				state1[i] = b2Manifold.b2_persistState;
				break;
			}
		}
	}

	// Detect persists and adds.
	for (var i = 0; i < manifold2.pointCount; ++i)
	{
		var id = manifold2.points[i].id;

		state2[i] = b2Manifold.b2_addState;

		for (var j = 0; j < manifold1.pointCount; ++j)
		{
			if (manifold1.points[j].id.Get() == id.Get())
			{
				state2[i] = b2Manifold.b2_persistState;
				break;
			}
		}
	}
}

/// Used for computing contact manifolds.
function b2ClipVertex()
{
	this.v = new b2Vec2();
	this.id = new b2ContactID();
};

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
function b2RayCastInput()
{
	this.p1 = new b2Vec2(), this.p2 = new b2Vec2();
	this.maxFraction = 0;
};

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from b2RayCastInput.
function b2RayCastOutput()
{
	this.normal = new b2Vec2();
	this.fraction = 0;
};

/// An axis aligned bounding box.
function b2AABB()
{
	this.lowerBound = new b2Vec2(); ///< the lower vertex
	this.upperBound = new b2Vec2(); ///< the upper vertex
}

b2AABB.prototype =
{
	Assign: function(other)
	{
		this.lowerBound.Assign(other.lowerBound);
		this.upperBound.Assign(other.upperBound);
	},

	Clone: function()
	{
		var clone = new b2AABB();
		clone.lowerBound.Assign(this.lowerBound);
		clone.upperBound.Assign(this.upperBound);
		return clone;
	},

	/// Verify that the bounds are sorted.
	IsValid: function()
	{
		var d = b2Vec2.Subtract(this.upperBound, this.lowerBound);
		var valid = d.x >= 0.0 && d.y >= 0.0;
		valid = valid && this.lowerBound.IsValid() && this.upperBound.IsValid();
		return valid;
	},

	/// Get the center of the AABB.
	GetCenter: function()
	{
		return b2Vec2.Multiply(0.5, b2Vec2.Add(this.lowerBound, this.upperBound));
	},

	/// Get the extents of the AABB (half-widths).
	GetExtents: function()
	{
		return b2Vec2.Multiply(0.5, b2Vec2.Subtract(this.upperBound, this.lowerBound));
	},

	/// Get the perimeter length
	GetPerimeter: function()
	{
		var wx = this.upperBound.x - this.lowerBound.x;
		var wy = this.upperBound.y - this.lowerBound.y;
		return 2.0 * (wx + wy);
	},

	/// Combine one or two AABBs into this one.
	Combine: function(aabb1, aabb2)
	{
		if (typeof(aabb2) !== 'undefined')
		{
			this.lowerBound.Assign(b2Min_v2(aabb1.lowerBound, aabb2.lowerBound));
			this.upperBound.Assign(b2Max_v2(aabb1.upperBound, aabb2.upperBound));
		}
		else
		{
			this.lowerBound.Assign(b2Min_v2(this.lowerBound, aabb.lowerBound));
			this.upperBound.Assign(b2Max_v2(this.upperBound, aabb.upperBound));
		}
	},

	/// Does this aabb contain the provided AABB.
	Contains: function(aabb)
	{
		var result = true;
		result = result && this.lowerBound.x <= aabb.lowerBound.x;
		result = result && this.lowerBound.y <= aabb.lowerBound.y;
		result = result && aabb.upperBound.x <= this.upperBound.x;
		result = result && aabb.upperBound.y <= this.upperBound.y;
		return result;
	},

	RayCast: function(output, input)
	{
		var tmin = -b2_maxFloat;
		var tmax = b2_maxFloat;

		var p = input.p1;
		var d = b2Vec2.Subtract(input.p2, input.p1);
		var absD = b2Abs_v2(d);

		var normal = new b2Vec2();

		for (var i = 0; i < 2; ++i)
		{
			if (absD.get_i(i) < b2_epsilon)
			{
				// Parallel.
				if (p.get_i(i) < this.lowerBound.get_i(i) || this.upperBound.get_i(i) < p.get_i(i))
				{
					return false;
				}
			}
			else
			{
				var inv_d = 1.0 / d.get_i(i);
				var t1 = (this.lowerBound.get_i(i) - p.get_i(i)) * inv_d;
				var t2 = (this.upperBound.get_i(i) - p.get_i(i)) * inv_d;

				// Sign of the normal vector.
				var s = -1.0;

				if (t1 > t2)
				{
					var temp = t2;
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
		output.fraction = tmin;
		output.normal.Assign(normal);
		return true;
	}
};

/// Compute the collision manifold between two circles.
function b2CollideCircles(manifold,
					  circleA, xfA,
					  circleB, xfB)
{
	manifold.pointCount = 0;

	var pA = b2Mul_t_v2(xfA, circleA.m_p);
	var pB = b2Mul_t_v2(xfB, circleB.m_p);

	var d = b2Vec2.Subtract(pB, pA);
	var distSqr = b2Dot_v2_v2(d, d);
	var rA = circleA.m_radius, rB = circleB.m_radius;
	var radius = rA + rB;
	if (distSqr > radius * radius)
	{
		return;
	}

	manifold.type = b2Manifold.e_circles;
	manifold.localPoint = circleA.m_p;
	manifold.localNormal.SetZero();
	manifold.pointCount = 1;

	manifold.points[0] = new b2ManifoldPoint();
	manifold.points[0].localPoint.Assign(circleB.m_p);
	manifold.points[0].id.Reset();
}

/// Compute the collision manifold between a polygon and a circle.
function b2CollidePolygonAndCircle(manifold,
							   polygonA, xfA,
							   circleB, xfB)
{
	manifold.pointCount = 0;

	// Compute circle position in the frame of the polygon.
	var c = b2Mul_t_v2(xfB, circleB.m_p);
	var cLocal = b2MulT_t_v2(xfA, c);

	// Find the min separating edge.
	var normalIndex = 0;
	var separation = -b2_maxFloat;
	var radius = polygonA.m_radius + circleB.m_radius;
	var vertexCount = polygonA.m_count;
	var vertices = polygonA.m_vertices;
	var normals = polygonA.m_normals;

	for (var i = 0; i < vertexCount; ++i)
	{
		var s = b2Dot_v2_v2(normals[i], b2Vec2.Subtract(cLocal, vertices[i]));

		if (s > radius)
		{
			// Early out.
			return;
		}

		if (s > separation)
		{
			separation = s;
			normalIndex = i;
		}
	}

	// Vertices that subtend the incident face.
	var vertIndex1 = normalIndex;
	var vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
	var v1 = vertices[vertIndex1];
	var v2 = vertices[vertIndex2];

	// If the center is inside the polygon ...
	if (separation < b2_epsilon)
	{
		manifold.pointCount = 1;
		manifold.type = b2Manifold.e_faceA;
		manifold.localNormal.Assign(normals[normalIndex]);
		manifold.localPoint.Assign(b2Vec2.Multiply(0.5, b2Vec2.Add(v1, v2)));
		manifold.points[0] = new b2ManifoldPoint();
		manifold.points[0].localPoint.Assign(circleB.m_p);
		manifold.points[0].id.Reset();
		return;
	}

	// Compute barycentric coordinates
	var u1 = b2Dot_v2_v2(b2Vec2.Subtract(cLocal, v1), b2Vec2.Subtract(v2, v1));
	var u2 = b2Dot_v2_v2(b2Vec2.Subtract(cLocal, v2), b2Vec2.Subtract(v1, v2));
	if (u1 <= 0.0)
	{
		if (b2DistanceSquared(cLocal, v1) > radius * radius)
		{
			return;
		}

		manifold.pointCount = 1;
		manifold.type = b2Manifold.e_faceA;
		manifold.localNormal.Assign(b2Vec2.Subtract(cLocal, v1));
		manifold.localNormal.Normalize();
		manifold.localPoint.Assign(v1);
		manifold.points[0] = new b2ManifoldPoint();
		manifold.points[0].localPoint.Assign(circleB.m_p);
		manifold.points[0].id.Reset();
	}
	else if (u2 <= 0.0)
	{
		if (b2DistanceSquared(cLocal, v2) > radius * radius)
		{
			return;
		}

		manifold.pointCount = 1;
		manifold.type = b2Manifold.e_faceA;
		manifold.localNormal.Assign(b2Vec2.Subtract(cLocal, v2));
		manifold.localNormal.Normalize();
		manifold.localPoint.Assign(v2);
		manifold.points[0] = new b2ManifoldPoint();
		manifold.points[0].localPoint.Assign(circleB.m_p);
		manifold.points[0].id.Reset();
	}
	else
	{
		var faceCenter = b2Vec2.Multiply(0.5, b2Vec2.Add(v1, v2));
		var separation = b2Dot_v2_v2(b2Vec2.Subtract(cLocal, faceCenter), normals[vertIndex1]);
		if (separation > radius)
		{
			return;
		}

		manifold.pointCount = 1;
		manifold.type = b2Manifold.e_faceA;
		manifold.localNormal.Assign(normals[vertIndex1]);
		manifold.localPoint.Assign(faceCenter);
		manifold.points[0] = new b2ManifoldPoint();
		manifold.points[0].localPoint.Assign(circleB.m_p);
		manifold.points[0].id.Reset();
	}
}

// Find the max separation between poly1 and poly2 using edge normals from poly1.
function b2FindMaxSeparation(edgeIndex,
								 poly1, xf1,
								 poly2, xf2)
{
	var count1 = poly1.m_count;
	var count2 = poly2.m_count;
	var n1s = poly1.m_normals;
	var v1s = poly1.m_vertices;
	var v2s = poly2.m_vertices;
	var xf = b2MulT_t_t(xf2, xf1);

	var bestIndex = 0;
	var maxSeparation = -b2_maxFloat;
	for (var i = 0; i < count1; ++i)
	{
		// Get poly1 normal in frame2.
		var n = b2Mul_r_v2(xf.q, n1s[i]);
		var v1 = b2Mul_t_v2(xf, v1s[i]);

		// Find deepest point for normal i.
		var si = b2_maxFloat;
		for (var j = 0; j < count2; ++j)
		{
			var sij = b2Dot_v2_v2(n, b2Vec2.Subtract(v2s[j], v1));
			if (sij < si)
			{
				si = sij;
			}
		}

		if (si > maxSeparation)
		{
			maxSeparation = si;
			bestIndex = i;
		}
	}

	edgeIndex[0] = bestIndex;
	return maxSeparation;
}

function b2FindIncidentEdge(c,
							 poly1, xf1, edge1,
							 poly2, xf2)
{
	var normals1 = poly1.m_normals;

	var count2 = poly2.m_count;
	var vertices2 = poly2.m_vertices;
	var normals2 = poly2.m_normals;

	b2Assert(0 <= edge1 && edge1 < poly1.m_count);

	// Get the normal of the reference edge in poly2's frame.
	var normal1 = b2MulT_r_v2(xf2.q, b2Mul_r_v2(xf1.q, normals1[edge1]));

	// Find the incident edge on poly2.
	var index = 0;
	var minDot = b2_maxFloat;
	for (var i = 0; i < count2; ++i)
	{
		var dot = b2Dot_v2_v2(normal1, normals2[i]);
		if (dot < minDot)
		{
			minDot = dot;
			index = i;
		}
	}

	// Build the clip vertices for the incident edge.
	var i1 = index;
	var i2 = i1 + 1 < count2 ? i1 + 1 : 0;

	c[0] = new b2ClipVertex();
	c[0].v = b2Mul_t_v2(xf2, vertices2[i1]);
	c[0].id.indexA = edge1;
	c[0].id.indexB = i1;
	c[0].id.typeA = b2ContactID.e_face;
	c[0].id.typeB = b2ContactID.e_vertex;

	c[1] = new b2ClipVertex();
	c[1].v = b2Mul_t_v2(xf2, vertices2[i2]);
	c[1].id.indexA = edge1;
	c[1].id.indexB = i2;
	c[1].id.typeA = b2ContactID.e_face;
	c[1].id.typeB = b2ContactID.e_vertex;
}

/// Compute the collision manifold between two polygons.
function b2CollidePolygons(manifold,
					   polyA, xfA,
					   polyB, xfB)
{
	manifold.pointCount = 0;
	var totalRadius = polyA.m_radius + polyB.m_radius;

	var edgeA = [0];
	var separationA = b2FindMaxSeparation(edgeA, polyA, xfA, polyB, xfB);
	if (separationA > totalRadius)
		return;

	var edgeB = [0];
	var separationB = b2FindMaxSeparation(edgeB, polyB, xfB, polyA, xfA);
	if (separationB > totalRadius)
		return;

	var poly1;	// reference polygon
	var poly2;	// incident polygon
	var xf1 = new b2Transform(), xf2 = new b2Transform();
	var edge1 = 0;					// reference edge
	var flip = 0;
	var k_tol = 0.1 * b2_linearSlop;

	if (separationB > separationA + k_tol)
	{
		poly1 = polyB;
		poly2 = polyA;
		xf1 = xfB;
		xf2 = xfA;
		edge1 = edgeB[0];
		manifold.type = b2Manifold.e_faceB;
		flip = 1;
	}
	else
	{
		poly1 = polyA;
		poly2 = polyB;
		xf1 = xfA;
		xf2 = xfB;
		edge1 = edgeA[0];
		manifold.type = b2Manifold.e_faceA;
		flip = 0;
	}

	var incidentEdge = new Array(2);
	b2FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2);

	var count1 = poly1.m_count;
	var vertices1 = poly1.m_vertices;

	var iv1 = edge1;
	var iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;

	var v11 = vertices1[iv1];
	var v12 = vertices1[iv2];

	var localTangent = b2Vec2.Subtract(v12, v11);
	localTangent.Normalize();

	var localNormal = b2Cross_v2_f(localTangent, 1.0);
	var planePoint = b2Vec2.Multiply(0.5, b2Vec2.Add(v11, v12));

	var tangent = b2Mul_r_v2(xf1.q, localTangent);
	var normal = b2Cross_v2_f(tangent, 1.0);

	v11 = b2Mul_t_v2(xf1, v11);
	v12 = b2Mul_t_v2(xf1, v12);

	// Face offset.
	var frontOffset = b2Dot_v2_v2(normal, v11);

	// Side offsets, extended by polytope skin thickness.
	var sideOffset1 = -b2Dot_v2_v2(tangent, v11) + totalRadius;
	var sideOffset2 = b2Dot_v2_v2(tangent, v12) + totalRadius;

	// Clip incident edge against extruded edge1 side edges.
	var clipPoints1 = new Array(2);
	var clipPoints2 = new Array(2);
	var np;

	// Clip to box side 1
	np = b2ClipSegmentToLine(clipPoints1, incidentEdge, tangent.Negate(), sideOffset1, iv1);

	if (np < 2)
		return;

	// Clip to negative box side 1
	np = b2ClipSegmentToLine(clipPoints2, clipPoints1,  tangent, sideOffset2, iv2);

	if (np < 2)
	{
		return;
	}

	// Now clipPoints2 contains the clipped points.
	manifold.localNormal.Assign(localNormal);
	manifold.localPoint.Assign(planePoint);

	var pointCount = 0;
	for (var i = 0; i < b2_maxManifoldPoints; ++i)
	{
		var separation = b2Dot_v2_v2(normal, clipPoints2[i].v) - frontOffset;

		if (separation <= totalRadius)
		{
			var cp = manifold.points[pointCount] = new b2ManifoldPoint();
			cp.localPoint.Assign(b2MulT_t_v2(xf2, clipPoints2[i].v));
			cp.id.Assign(clipPoints2[i].id);
			if (flip)
			{
				// Swap features
				var cf = new b2ContactID();
				cf.Assign(cp.id);
				cp.id.indexA = cf.indexB;
				cp.id.indexB = cf.indexA;
				cp.id.typeA = cf.typeB;
				cp.id.typeB = cf.typeA;
			}
			++pointCount;
		}
	}

	manifold.pointCount = pointCount;
}

/// Compute the collision manifold between an edge and a circle.
function b2CollideEdgeAndCircle(manifold,
							   edgeA, xfA,
							   circleB, xfB)
{
	manifold.pointCount = 0;

	// Compute circle in frame of edge
	var Q = b2MulT_t_v2(xfA, b2Mul_t_v2(xfB, circleB.m_p));

	var A = edgeA.m_vertex1, B = edgeA.m_vertex2;
	var e = b2Vec2.Subtract(B, A);

	// Barycentric coordinates
	var u = b2Dot_v2_v2(e, b2Vec2.Subtract(B, Q));
	var v = b2Dot_v2_v2(e, b2Vec2.Subtract(Q, A));

	var radius = edgeA.m_radius + circleB.m_radius;

	var cf = new b2ContactID();
	cf.indexB = 0;
	cf.typeB = b2ContactID.e_vertex;

	// Region A
	if (v <= 0.0)
	{
		var P = A;
		var d = b2Vec2.Subtract(Q, P);
		var dd = b2Dot_v2_v2(d, d);
		if (dd > radius * radius)
		{
			return;
		}

		// Is there an edge connected to A?
		if (edgeA.m_hasVertex0)
		{
			var A1 = edgeA.m_vertex0;
			var B1 = A;
			var e1 = b2Vec2.Subtract(B1, A1);
			var u1 = b2Dot_v2_v2(e1, b2Vec2.Subtract(B1, Q));

			// Is the circle in Region AB of the previous edge?
			if (u1 > 0.0)
			{
				return;
			}
		}

		cf.indexA = 0;
		cf.typeA = b2ContactID.e_vertex;
		manifold.pointCount = 1;
		manifold.type = b2Manifold.e_circles;
		manifold.localNormal.SetZero();
		manifold.localPoint.Assign(P);
		manifold.points[0] = new b2ManifoldPoint();
		manifold.points[0].id.Assign(cf);
		manifold.points[0].localPoint.Assign(circleB.m_p);
		return;
	}

	// Region B
	if (u <= 0.0)
	{
		var P = B;
		var d = b2Vec2.Subtract(Q, P);
		var dd = b2Dot_v2_v2(d, d);
		if (dd > radius * radius)
		{
			return;
		}

		// Is there an edge connected to B?
		if (edgeA.m_hasVertex3)
		{
			var B2 = edgeA.m_vertex3;
			var A2 = B;
			var e2 = b2Vec2.Subtract(B2, A2);
			var v2 = b2Dot_v2_v2(e2, b2Vec2.Subtract(Q, A2));

			// Is the circle in Region AB of the next edge?
			if (v2 > 0.0)
			{
				return;
			}
		}

		cf.indexA = 1;
		cf.typeA = b2ContactID.e_vertex;
		manifold.pointCount = 1;
		manifold.type = b2Manifold.e_circles;
		manifold.localNormal.SetZero();
		manifold.localPoint.Assign(P);
		manifold.points[0] = new b2ManifoldPoint();
		manifold.points[0].id.Assign(cf);
		manifold.points[0].localPoint.Assign(circleB.m_p);
		return;
	}

	// Region AB
	var den = b2Dot_v2_v2(e, e);
	b2Assert(den > 0.0);
	var P = b2Vec2.Multiply((1.0 / den), b2Vec2.Add(b2Vec2.Multiply(u, A), b2Vec2.Multiply(v, B)));
	var d = b2Vec2.Subtract(Q, P);
	var dd = b2Dot_v2_v2(d, d);
	if (dd > radius * radius)
	{
		return;
	}

	var n = new b2Vec2(-e.y, e.x);
	if (b2Dot_v2_v2(n, b2Vec2.Subtract(Q, A)) < 0.0)
	{
		n.Set(-n.x, -n.y);
	}
	n.Normalize();

	cf.indexA = 0;
	cf.typeA = b2ContactID.e_face;
	manifold.pointCount = 1;
	manifold.type = b2Manifold.e_faceA;
	manifold.localNormal.Assign(n);
	manifold.localPoint.Assign(A);
	manifold.points[0] = new b2ManifoldPoint();
	manifold.points[0].id.Assign(cf);
	manifold.points[0].localPoint.Assign(circleB.m_p);
}

// This structure is used to keep track of the best separating axis.
function b2EPAxis()
{
	this.type = 0;
	this.index = 0;
	this.separation = 0;
}

b2EPAxis.e_unknown = 0;
b2EPAxis.e_edgeA = 1;
b2EPAxis.e_edgeB = 2;


// This holds polygon B expressed in frame A.
function b2TempPolygon()
{
	this.vertices = new Array(b2_maxPolygonVertices);
	this.normals = new Array(b2_maxPolygonVertices);
	this.count = 0;
};

// Reference face used for clipping
function b2ReferenceFace()
{
	this.i1 = 0, this.i2 = 0;

	this.v1 = new b2Vec2(), this.v2 = new b2Vec2();

	this.normal = new b2Vec2();

	this.sideNormal1 = new b2Vec2();
	this.sideOffset1 = 0;

	this.sideNormal2 = new b2Vec2();
	this.sideOffset2 = 0;
};

// This class collides and edge and a polygon, taking into account edge adjacency.
function b2EPCollider()
{
	this.m_polygonB = new b2TempPolygon();

	this.m_xf = new b2Transform();
	this.m_centroidB = new b2Vec2();
	this.m_v0 = new b2Vec2(), this.m_v1 = new b2Vec2(), this.m_v2 = new b2Vec2(), this.m_v3 = new b2Vec2();
	this.m_normal0 = new b2Vec2(), this.m_normal1 = new b2Vec2(), this.m_normal2 = new b2Vec2();
	this.m_normal = new b2Vec2();
	this.m_type1 = 0, this.m_type2 = 0;
	this.m_lowerLimit = new b2Vec2(), this.m_upperLimit = new b2Vec2();
	this.m_radius = 0;
	this.m_front = false;
}

b2EPCollider.prototype =
{
	// Algorithm:
	// 1. Classify v1 and v2
	// 2. Classify polygon centroid as front or back
	// 3. Flip normal if necessary
	// 4. Initialize normal range to [-pi, pi] about face normal
	// 5. Adjust normal range according to adjacent edges
	// 6. Visit each separating axes, only accept axes within the range
	// 7. Return if _any_ axis indicates separation
	// 8. Clip
	Collide: function(manifold, edgeA, xfA,
				 polygonB, xfB)
	{
		this.m_xf.Assign(b2MulT_t_t(xfA, xfB));

		this.m_centroidB.Assign(b2Mul_t_v2(this.m_xf, polygonB.m_centroid));

		this.m_v0.Assign(edgeA.m_vertex0);
		this.m_v1.Assign(edgeA.m_vertex1);
		this.m_v2.Assign(edgeA.m_vertex2);
		this.m_v3.Assign(edgeA.m_vertex3);

		var hasVertex0 = edgeA.m_hasVertex0;
		var hasVertex3 = edgeA.m_hasVertex3;

		var edge1 = b2Vec2.Subtract(this.m_v2, this.m_v1);
		edge1.Normalize();
		this.m_normal1.Set(edge1.y, -edge1.x);
		var offset1 = b2Dot_v2_v2(this.m_normal1, b2Vec2.Subtract(this.m_centroidB, this.m_v1));
		var offset0 = 0.0, offset2 = 0.0;
		var convex1 = false, convex2 = false;

		// Is there a preceding edge?
		if (hasVertex0)
		{
			var edge0 = b2Vec2.Subtract(this.m_v1, this.m_v0);
			edge0.Normalize();
			this.m_normal0.Set(edge0.y, -edge0.x);
			convex1 = b2Cross_v2_v2(edge0, edge1) >= 0.0;
			offset0 = b2Dot_v2_v2(this.m_normal0, b2Vec2.Subtract(this.m_centroidB, this.m_v0));
		}

		// Is there a following edge?
		if (hasVertex3)
		{
			var edge2 = b2Vec2.Subtract(this.m_v3, this.m_v2);
			edge2.Normalize();
			this.m_normal2.Set(edge2.y, -edge2.x);
			convex2 = b2Cross_v2_v2(edge1, edge2) > 0.0;
			offset2 = b2Dot_v2_v2(this.m_normal2, b2Vec2.Subtract(this.m_centroidB, this.m_v2));
		}

		// Determine front or back collision. Determine collision normal limits.
		if (hasVertex0 && hasVertex3)
		{
			if (convex1 && convex2)
			{
				this.m_front = offset0 >= 0.0 || offset1 >= 0.0 || offset2 >= 0.0;
				if (this.m_front)
				{
					this.m_normal = this.m_normal1;
					this.m_lowerLimit = this.m_normal0;
					this.m_upperLimit = this.m_normal2;
				}
				else
				{
					this.m_normal = this.m_normal1.Negate();
					this.m_lowerLimit = this.m_normal1.Negate();
					this.m_upperLimit = this.m_normal1.Negate();
				}
			}
			else if (convex1)
			{
				this.m_front = offset0 >= 0.0 || (offset1 >= 0.0 && offset2 >= 0.0);
				if (this.m_front)
				{
					this.m_normal = this.m_normal1;
					this.m_lowerLimit = this.m_normal0;
					this.m_upperLimit = this.m_normal1;
				}
				else
				{
					this.m_normal = this.m_normal1.Negate();
					this.m_lowerLimit = this.m_normal2.Negate();
					this.m_upperLimit = this.m_normal1.Negate();
				}
			}
			else if (convex2)
			{
				this.m_front = offset2 >= 0.0 || (offset0 >= 0.0 && offset1 >= 0.0);
				if (this.m_front)
				{
					this.m_normal = this.m_normal1;
					this.m_lowerLimit = this.m_normal1;
					this.m_upperLimit = this.m_normal2;
				}
				else
				{
					this.m_normal = this.m_normal1.Negate();
					this.m_lowerLimit = this.m_normal1.Negate();
					this.m_upperLimit = this.m_normal0.Negate();
				}
			}
			else
			{
				this.m_front = offset0 >= 0.0 && offset1 >= 0.0 && offset2 >= 0.0;
				if (this.m_front)
				{
					this.m_normal = this.m_normal1;
					this.m_lowerLimit = this.m_normal1;
					this.m_upperLimit = this.m_normal1;
				}
				else
				{
					this.m_normal = this.m_normal1.Negate();
					this.m_lowerLimit = this.m_normal2.Negate();
					this.m_upperLimit = this.m_normal0.Negate();
				}
			}
		}
		else if (hasVertex0)
		{
			if (convex1)
			{
				this.m_front = offset0 >= 0.0 || offset1 >= 0.0;
				if (this.m_front)
				{
					this.m_normal = this.m_normal1;
					this.m_lowerLimit = this.m_normal0;
					this.m_upperLimit = this.m_normal1.Negate();
				}
				else
				{
					this.m_normal = this.m_normal1.Negate();
					this.m_lowerLimit = this.m_normal1;
					this.m_upperLimit = this.m_normal1.Negate();
				}
			}
			else
			{
				this.m_front = offset0 >= 0.0 && offset1 >= 0.0;
				if (this.m_front)
				{
					this.m_normal = this.m_normal1;
					this.m_lowerLimit = this.m_normal1;
					this.m_upperLimit = this.m_normal1.Negate();
				}
				else
				{
					this.m_normal = this.m_normal1.Negate();
					this.m_lowerLimit = this.m_normal1;
					this.m_upperLimit = this.m_normal0.Negate();
				}
			}
		}
		else if (hasVertex3)
		{
			if (convex2)
			{
				this.m_front = offset1 >= 0.0 || offset2 >= 0.0;
				if (this.m_front)
				{
					this.m_normal = this.m_normal1;
					this.m_lowerLimit = this.m_normal1.Negate();
					this.m_upperLimit = this.m_normal2;
				}
				else
				{
					this.m_normal = this.m_normal1.Negate();
					this.m_lowerLimit = this.m_normal1.Negate();
					this.m_upperLimit = this.m_normal1;
				}
			}
			else
			{
				this.m_front = offset1 >= 0.0 && offset2 >= 0.0;
				if (this.m_front)
				{
					this.m_normal = this.m_normal1;
					this.m_lowerLimit = this.m_normal1.Negate();
					this.m_upperLimit = this.m_normal1;
				}
				else
				{
					this.m_normal = this.m_normal1.Negate();
					this.m_lowerLimit = this.m_normal2.Negate();
					this.m_upperLimit = this.m_normal1;
				}
			}
		}
		else
		{
			this.m_front = offset1 >= 0.0;
			if (this.m_front)
			{
				this.m_normal = this.m_normal1;
				this.m_lowerLimit = this.m_normal1.Negate();
				this.m_upperLimit = this.m_normal1.Negate();
			}
			else
			{
				this.m_normal = this.m_normal1.Negate();
				this.m_lowerLimit = this.m_normal1;
				this.m_upperLimit = this.m_normal1;
			}
		}

		// Get polygonB in frameA
		this.m_polygonB.count = polygonB.m_count;
		for (var i = 0; i < polygonB.m_count; ++i)
		{
			this.m_polygonB.vertices[i] = b2Mul_t_v2(this.m_xf, polygonB.m_vertices[i]);
			this.m_polygonB.normals[i] = b2Mul_r_v2(this.m_xf.q, polygonB.m_normals[i]);
		}

		this.m_radius = 2.0 * b2_polygonRadius;

		manifold.pointCount = 0;

		var edgeAxis = this.ComputeEdgeSeparation();

		// If no valid normal can be found than this edge should not collide.
		if (edgeAxis.type == b2EPAxis.e_unknown)
		{
			return;
		}

		if (edgeAxis.separation > this.m_radius)
		{
			return;
		}

		var polygonAxis = this.ComputePolygonSeparation();
		if (polygonAxis.type != b2EPAxis.e_unknown && polygonAxis.separation > this.m_radius)
		{
			return;
		}

		// Use hysteresis for jitter reduction.
		var k_relativeTol = 0.98;
		var k_absoluteTol = 0.001;

		var primaryAxis = new b2EPAxis();
		if (polygonAxis.type == b2EPAxis.e_unknown)
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

		var ie = new Array(2);
		var rf = new b2ReferenceFace();
		if (primaryAxis.type == b2EPAxis.e_edgeA)
		{
			manifold.type = b2Manifold.e_faceA;

			// Search for the polygon normal that is most anti-parallel to the edge normal.
			var bestIndex = 0;
			var bestValue = b2Dot_v2_v2(this.m_normal, this.m_polygonB.normals[0]);
			for (var i = 1; i < this.m_polygonB.count; ++i)
			{
				var value = b2Dot_v2_v2(this.m_normal, this.m_polygonB.normals[i]);
				if (value < bestValue)
				{
					bestValue = value;
					bestIndex = i;
				}
			}

			var i1 = bestIndex;
			var i2 = i1 + 1 < this.m_polygonB.count ? i1 + 1 : 0;

			ie[0] = new b2ClipVertex();
			ie[0].v.Assign(this.m_polygonB.vertices[i1]);
			ie[0].id.indexA = 0;
			ie[0].id.indexB = i1;
			ie[0].id.typeA = b2ContactID.e_face;
			ie[0].id.typeB = b2ContactID.e_vertex;

			ie[1] = new b2ClipVertex();
			ie[1].v.Assign(this.m_polygonB.vertices[i2]);
			ie[1].id.indexA = 0;
			ie[1].id.indexB = i2;
			ie[1].id.typeA = b2ContactID.e_face;
			ie[1].id.typeB = b2ContactID.e_vertex;

			if (this.m_front)
			{
				rf.i1 = 0;
				rf.i2 = 1;
				rf.v1.Assign(this.m_v1);
				rf.v2.Assign(this.m_v2);
				rf.normal.Assign(this.m_normal1);
			}
			else
			{
				rf.i1 = 1;
				rf.i2 = 0;
				rf.v1.Assign(this.m_v2);
				rf.v2.Assign(this.m_v1);
				rf.normal.Assign(this.m_normal1.Negate());
			}
		}
		else
		{
			manifold.type = b2Manifold.e_faceB;

			ie[0] = new b2ClipVertex();
			ie[0].v = this.m_v1;
			ie[0].id.indexA = 0;
			ie[0].id.indexB = primaryAxis.index;
			ie[0].id.typeA = b2ContactID.e_vertex;
			ie[0].id.typeB = b2ContactID.e_face;

			ie[1] = new b2ClipVertex();
			ie[1].v = this.m_v2;
			ie[1].id.indexA = 0;
			ie[1].id.indexB = primaryAxis.index;
			ie[1].id.typeA = b2ContactID.e_vertex;
			ie[1].id.typeB = b2ContactID.e_face;

			rf.i1 = primaryAxis.index;
			rf.i2 = rf.i1 + 1 < this.m_polygonB.count ? rf.i1 + 1 : 0;
			rf.v1 = this.m_polygonB.vertices[rf.i1];
			rf.v2 = this.m_polygonB.vertices[rf.i2];
			rf.normal = this.m_polygonB.normals[rf.i1];
		}

		rf.sideNormal1.Set(rf.normal.y, -rf.normal.x);
		rf.sideNormal2 = rf.sideNormal1.Negate();
		rf.sideOffset1 = b2Dot_v2_v2(rf.sideNormal1, rf.v1);
		rf.sideOffset2 = b2Dot_v2_v2(rf.sideNormal2, rf.v2);

		// Clip incident edge against extruded edge1 side edges.
		var clipPoints1 = new Array(2);
		var clipPoints2 = new Array(2);
		var np;

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
		if (primaryAxis.type == b2EPAxis.e_edgeA)
		{
			manifold.localNormal.Assign(rf.normal);
			manifold.localPoint.Assign(rf.v1);
		}
		else
		{
			manifold.localNormal.Assign(polygonB.m_normals[rf.i1]);
			manifold.localPoint.Assign(polygonB.m_vertices[rf.i1]);
		}

		var pointCount = 0;
		for (var i = 0; i < b2_maxManifoldPoints; ++i)
		{
			var separation;

			separation = b2Dot_v2_v2(rf.normal, b2Vec2.Subtract(clipPoints2[i].v, rf.v1));

			if (separation <= this.m_radius)
			{
				var cp = manifold.points[pointCount] = new b2ManifoldPoint();

				if (primaryAxis.type == b2EPAxis.e_edgeA)
				{
					cp.localPoint.Assign(b2MulT_t_v2(this.m_xf, clipPoints2[i].v));
					cp.id.Assign(clipPoints2[i].id);
				}
				else
				{
					cp.localPoint.Assign(clipPoints2[i].v);
					cp.id.typeA = clipPoints2[i].id.typeB;
					cp.id.typeB = clipPoints2[i].id.typeA;
					cp.id.indexA = clipPoints2[i].id.indexB;
					cp.id.indexB = clipPoints2[i].id.indexA;
				}

				++pointCount;
			}
		}

		manifold.pointCount = pointCount;
	},

	ComputeEdgeSeparation: function()
	{
		var axis = new b2EPAxis();
		axis.type = b2EPAxis.e_edgeA;
		axis.index = this.m_front ? 0 : 1;
		axis.separation = Number.MAX_VALUE;

		for (var i = 0; i < this.m_polygonB.count; ++i)
		{
			var s = b2Dot_v2_v2(this.m_normal, b2Vec2.Subtract(this.m_polygonB.vertices[i], this.m_v1));
			if (s < axis.separation)
			{
				axis.separation = s;
			}
		}

		return axis;
	},
	ComputePolygonSeparation: function()
	{
		var axis = new b2EPAxis();
		axis.type = b2EPAxis.e_unknown;
		axis.index = -1;
		axis.separation = -Number.MAX_VALUE;

		var perp = new b2Vec2(-this.m_normal.y, this.m_normal.x);

		for (var i = 0; i < this.m_polygonB.count; ++i)
		{
			var n = this.m_polygonB.normals[i].Negate();

			var s1 = b2Dot_v2_v2(n, b2Vec2.Subtract(this.m_polygonB.vertices[i], this.m_v1));
			var s2 = b2Dot_v2_v2(n, b2Vec2.Subtract(this.m_polygonB.vertices[i], this.m_v2));
			var s = b2Min(s1, s2);

			if (s > this.m_radius)
			{
				// No collision
				axis.type = b2EPAxis.e_edgeB;
				axis.index = i;
				axis.separation = s;
				return axis;
			}

			// Adjacency
			if (b2Dot_v2_v2(n, perp) >= 0.0)
			{
				if (b2Dot_v2_v2(b2Vec2.Subtract(n, this.m_upperLimit), this.m_normal) < -b2_angularSlop)
				{
					continue;
				}
			}
			else
			{
				if (b2Dot_v2_v2(b2Vec2.Subtract(n, this.m_lowerLimit), this.m_normal) < -b2_angularSlop)
				{
					continue;
				}
			}

			if (s > axis.separation)
			{
				axis.type = b2EPAxis.e_edgeB;
				axis.index = i;
				axis.separation = s;
			}
		}

		return axis;
	}
};

b2EPCollider.e_isolated = 0;
b2EPCollider.e_concave = 1;
b2EPCollider.e_convex = 2;

/// Compute the collision manifold between an edge and a circle.
function b2CollideEdgeAndPolygon(manifold,
							   edgeA, xfA,
							   polygonB, xfB)
{
	var collider = new b2EPCollider();
	collider.Collide(manifold, edgeA, xfA, polygonB, xfB);
}

/// Clipping for contact manifolds.
function b2ClipSegmentToLine(vOut, vIn,
							normal, offset, vertexIndexA)
{
	// Start with no output points
	var numOut = 0;

	// Calculate the distance of end points to the line
	var distance0 = b2Dot_v2_v2(normal, vIn[0].v) - offset;
	var distance1 = b2Dot_v2_v2(normal, vIn[1].v) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0) vOut[numOut++] = vIn[0];
	if (distance1 <= 0.0) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0)
	{
		// Find intersection point of edge and plane
		var interp = distance0 / (distance0 - distance1);
		vOut[numOut] = new b2ClipVertex();
		vOut[numOut].v.Assign(b2Vec2.Add(vIn[0].v, b2Vec2.Multiply(interp, b2Vec2.Subtract(vIn[1].v, vIn[0].v))));

		// VertexA is hitting edgeB.
		vOut[numOut].id.indexA = vertexIndexA;
		vOut[numOut].id.indexB = vIn[0].id.indexB;
		vOut[numOut].id.typeA = b2ContactID.e_vertex;
		vOut[numOut].id.typeB = b2ContactID.e_face;
		++numOut;
	}

	return numOut;
}

/// Determine if two generic shapes overlap.
function b2TestShapeOverlap(shapeA, indexA,
					shapeB, indexB,
					xfA, xfB)
{
	var input = new b2DistanceInput();
	input.proxyA.Set(shapeA, indexA);
	input.proxyB.Set(shapeB, indexB);
	input.transformA = xfA;
	input.transformB = xfB;
	input.useRadii = true;

	var cache = new b2SimplexCache();
	cache.count = 0;

	var output = new b2DistanceOutput();

	b2DistanceFunc(output, cache, input);

	return output.distance < 10.0 * b2_epsilon;
}

function b2TestOverlap(a, b)
{
	var d1, d2;
	d1 = b2Vec2.Subtract(b.lowerBound, a.upperBound);
	d2 = b2Vec2.Subtract(a.lowerBound, b.upperBound);

	if (d1.x > 0.0 || d1.y > 0.0)
		return false;

	if (d2.x > 0.0 || d2.y > 0.0)
		return false;

	return true;
}