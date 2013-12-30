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
		point.localPoint.x = this.localPoint.x;//.Assign(this.localPoint);
		point.localPoint.y = this.localPoint.y;
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
		manifold.localPoint.x = this.localPoint.x;//.Assign(this.localPoint);
		manifold.localPoint.y = this.localPoint.y;
		manifold.localNormal.x = this.localNormal.x;//.Assign(this.localNormal);
		manifold.localNormal.y = this.localNormal.y;

		for (var i = 0; i < this.pointCount; ++i)
			manifold.points[i] = this.points[i].Clone();

		return manifold;
	},

	// NOTE: this version of assign is very specific and
	// should NOT be used in your own code.
	Assign: function(manifold)
	{
		this.pointCount = manifold.pointCount;
		this.type = manifold.type;
		this.localPoint.x = manifold.localPoint.x;//.Assign(manifold.localPoint);
		this.localPoint.y = manifold.localPoint.y;
		this.localNormal.x = manifold.localNormal.x;//.Assign(manifold.localNormal);
		this.localNormal.y = manifold.localNormal.y;

		for (var i = 0; i < this.pointCount; ++i)
			this.points[i] = manifold.points[i].Clone();
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
				this.normal.x = 1; this.normal.y = 0;//.Set(1.0, 0.0);
				var pointAx = (xfA.q.c * manifold.localPoint.x - xfA.q.s * manifold.localPoint.y) + xfA.p.x;//b2Mul_t_v2(xfA, manifold.localPoint);
				var pointAy = (xfA.q.s * manifold.localPoint.x + xfA.q.c * manifold.localPoint.y) + xfA.p.y;
				var pointBx = (xfB.q.c * manifold.points[0].localPoint.x - xfB.q.s * manifold.points[0].localPoint.y) + xfB.p.x;//b2Mul_t_v2(xfB, manifold.points[0].localPoint);
				var pointBy = (xfB.q.s * manifold.points[0].localPoint.x + xfB.q.c * manifold.points[0].localPoint.y) + xfB.p.y;

				var cx = pointAx - pointBx;
				var cy = pointAy - pointBy;
				if ((cx * cx + cy * cy)/*b2DistanceSquared(pointA, pointB)*/ > b2_epsilon * b2_epsilon)
				{
					this.normal.x = pointBx - pointAx;// = b2Vec2.Subtract(pointB, pointA);
					this.normal.y = pointBy - pointAy;
					this.normal.Normalize();
				}

				//var cA = b2Vec2.Add(pointA, b2Vec2.Multiply(radiusA, this.normal));
				var cAx = pointAx + (radiusA * this.normal.x);
				var cAy = pointAy + (radiusA * this.normal.y);
				//var cB = b2Vec2.Subtract(pointB, b2Vec2.Multiply(radiusB, this.normal));
				var cBx = pointBx - (radiusB * this.normal.x);
				var cBy = pointBy - (radiusB * this.normal.y);
				//this.points[0] = b2Vec2.Multiply(0.5, b2Vec2.Add(cA, cB));
				this.points[0] = new b2Vec2(0.5 * (cAx + cBx), 0.5 * (cAy + cBy));
				this.separations[0] = (cBx - cAx) * this.normal.x + (cBy - cAy) * this.normal.y;//b2Dot_v2_v2(b2Vec2.Subtract(cB, cA), this.normal);
			}
			break;

		case b2Manifold.e_faceA:
			{
				this.normal.x = xfA.q.c * manifold.localNormal.x - xfA.q.s * manifold.localNormal.y;//b2Mul_r_v2(xfA.q, manifold.localNormal);
				this.normal.y = xfA.q.s * manifold.localNormal.x + xfA.q.c * manifold.localNormal.y;
				var planePointx = (xfA.q.c * manifold.localPoint.x - xfA.q.s * manifold.localPoint.y) + xfA.p.x;//b2Mul_t_v2(xfA, manifold.localPoint);
				var planePointy = (xfA.q.s * manifold.localPoint.x + xfA.q.c * manifold.localPoint.y) + xfA.p.y;

				for (var i = 0; i < manifold.pointCount; ++i)
				{
					var clipPointx = (xfB.q.c * manifold.points[i].localPoint.x - xfB.q.s * manifold.points[i].localPoint.y) + xfB.p.x;//b2Mul_t_v2(xfB, manifold.points[i].localPoint);
					var clipPointy = (xfB.q.s * manifold.points[i].localPoint.x + xfB.q.c * manifold.points[i].localPoint.y) + xfB.p.y;
					//var cA = b2Vec2.Add(clipPoint, b2Vec2.Multiply((radiusA - b2Dot_v2_v2(b2Vec2.Subtract(clipPoint, planePoint), this.normal)), this.normal));
					var d = (clipPointx - planePointx) * this.normal.x + (clipPointy - planePointy) * this.normal.y;
					var cAx = clipPointx + ((radiusA - d) * this.normal.x);
					var cAy = clipPointy + ((radiusA - d) * this.normal.y);
					//var cB = b2Vec2.Subtract(clipPoint, b2Vec2.Multiply(radiusB, this.normal));
					var cBx = (clipPointx - (radiusB * this.normal.x));
					var cBy = (clipPointy - (radiusB * this.normal.y));
					//this.points[i] = b2Vec2.Multiply(0.5, b2Vec2.Add(cA, cB));
					this.points[i] = new b2Vec2(0.5 * (cAx + cBx), 0.5 * (cAy + cBy));
					//this.separations[i] = b2Dot_v2_v2(b2Vec2.Subtract(cB, cA), this.normal);
					this.separations[i] = (cBx - cAx) * this.normal.x + (cBy - cAy) * this.normal.y;//b2Dot_v2_v2(b2Vec2.Subtract(cB, cA), this.normal);
				}
			}
			break;

		case b2Manifold.e_faceB:
			{
				this.normal.x = xfB.q.c * manifold.localNormal.x - xfB.q.s * manifold.localNormal.y;//b2Mul_r_v2(xfB.q, manifold.localNormal);
				this.normal.y = xfB.q.s * manifold.localNormal.x + xfB.q.c * manifold.localNormal.y;
				var planePointx = (xfB.q.c * manifold.localPoint.x - xfB.q.s * manifold.localPoint.y) + xfB.p.x;//b2Mul_t_v2(xfB, manifold.localPoint);
				var planePointy = (xfB.q.s * manifold.localPoint.x + xfB.q.c * manifold.localPoint.y) + xfB.p.y;

				for (var i = 0; i < manifold.pointCount; ++i)
				{
					var clipPointx = (xfA.q.c * manifold.points[i].localPoint.x - xfA.q.s * manifold.points[i].localPoint.y) + xfA.p.x;//b2Mul_t_v2(xfA, manifold.points[i].localPoint);
					var clipPointy = (xfA.q.s * manifold.points[i].localPoint.x + xfA.q.c * manifold.points[i].localPoint.y) + xfA.p.y;
					//var cB = b2Vec2.Add(clipPoint, b2Vec2.Multiply((radiusB - b2Dot_v2_v2(b2Vec2.Subtract(clipPoint, planePoint), this.normal)), this.normal));
					var d = (clipPointx - planePointx) * this.normal.x + (clipPointy - planePointy) * this.normal.y;
					var cBx = clipPointx + ((radiusB - d) * this.normal.x);
					var cBy = clipPointy + ((radiusB - d) * this.normal.y);
					//var cA = b2Vec2.Subtract(clipPoint, b2Vec2.Multiply(radiusA, this.normal));
					var cAx = (clipPointx - (radiusA * this.normal.x));
					var cAy = (clipPointy - (radiusA * this.normal.y));
					this.points[i] = new b2Vec2(0.5 * (cAx + cBx), 0.5 * (cAy + cBy));
					this.separations[i] = (cAx - cBx) * this.normal.x + (cAy - cBy) * this.normal.y;//b2Dot_v2_v2(b2Vec2.Subtract(cA, cB), this.normal);
				}

				// Ensure normal points from A to B.
				this.normal.x = -this.normal.x;// = this.normal.Negate();
				this.normal.y = -this.normal.y;
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
		this.lowerBound.x = other.lowerBound.x;
		this.lowerBound.y = other.lowerBound.y;//.Assign(other.lowerBound);
		this.upperBound.x = other.upperBound.x;//.Assign(other.upperBound);
		this.upperBound.y = other.upperBound.y;
	},

	Clone: function()
	{
		var clone = new b2AABB();
		clone.lowerBound.x = this.lowerBound.x; //.Assign(this.lowerBound);
		clone.lowerBound.y = this.lowerBound.y;
		clone.upperBound.x = this.upperBound.x;//.Assign(this.upperBound);
		clone.upperBound.y = this.upperBound.y;
		return clone;
	},

	/// Verify that the bounds are sorted.
	IsValid: function()
	{
		return (this.upperBound.x - this.lowerBound.x) >= 0.0 && (this.upperBound.y - this.lowerBound.y) >= 0.0 && this.lowerBound.IsValid() && this.upperBound.IsValid();
	},

	/// Get the center of the AABB.
	GetCenter: function()
	{
		return new b2Vec2(0.5 * (this.lowerBound.x + this.upperBound.x), 0.5 * (this.lowerBound.y + this.upperBound.y));
	},

	/// Get the extents of the AABB (half-widths).
	GetExtents: function()
	{
		return new b2Vec2(0.5 * (this.upperBound.x - this.lowerBound.x), 0.5 * (this.upperBound.y - this.lowerBound.y));
	},

	/// Get the perimeter length
	GetPerimeter: function()
	{
		return 2.0 * ((this.upperBound.x - this.lowerBound.x) + (this.upperBound.y - this.lowerBound.y));
	},

	/// Combine one or two AABBs into this one.
	Combine: function(aabb1, aabb2)
	{
		if (aabb2)
		{
			//this.lowerBound.Assign(b2Min_v2(aabb1.lowerBound, aabb2.lowerBound));
			this.lowerBound.x = b2Min(aabb1.lowerBound.x, aabb2.lowerBound.x);
			this.lowerBound.y = b2Min(aabb1.lowerBound.y, aabb2.lowerBound.y);
			//this.upperBound.Assign(b2Max_v2(aabb1.upperBound, aabb2.upperBound));
			this.upperBound.x = b2Max(aabb1.upperBound.x, aabb2.upperBound.x);
			this.upperBound.y = b2Max(aabb1.upperBound.y, aabb2.upperBound.y);
		}
		else
		{
			//this.lowerBound.Assign(b2Min_v2(this.lowerBound, aabb1.lowerBound));
			this.lowerBound.x = b2Min(this.lowerBound.x, aabb1.lowerBound.x);
			this.lowerBound.y = b2Min(this.lowerBound.y, aabb1.lowerBound.y);
			//this.upperBound.Assign(b2Max_v2(this.upperBound, aabb1.upperBound));
			this.upperBound.x = b2Max(this.upperBound.x, aabb1.upperBound.x);
			this.upperBound.y = b2Max(this.upperBound.y, aabb1.upperBound.y);
		}
	},

	/// Does this aabb contain the provided AABB.
	Contains: function(aabb)
	{
		return	   this.lowerBound.x <= aabb.lowerBound.x
				&& this.lowerBound.y <= aabb.lowerBound.y
				&& aabb.upperBound.x <= this.upperBound.x
				&& aabb.upperBound.y <= this.upperBound.y;
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
					normal.x = normal.y = 0;
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
		output.normal.x = normal.x;
		output.normal.y = normal.y;
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

	var dx = pB.x - pA.x;//b2Vec2.Subtract(pB, pA);
	var dy = pB.y - pA.y;
	var distSqr = dx * dx + dy * dy;//b2Dot_v2_v2(d, d);
	var rA = circleA.m_radius, rB = circleB.m_radius;
	var radius = rA + rB;
	if (distSqr > radius * radius)
	{
		return;
	}

	manifold.type = b2Manifold.e_circles;
	manifold.localPoint.x = circleA.m_p.x;
	manifold.localPoint.y = circleA.m_p.y;
	manifold.localNormal.x = manifold.localNormal.y = 0;
	manifold.pointCount = 1;

	manifold.points[0] = new b2ManifoldPoint();
	manifold.points[0].localPoint.x = circleB.m_p.x;
	manifold.points[0].localPoint.y = circleB.m_p.y;
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
		var s = normals[i].x * (cLocal.x - vertices[i].x) + normals[i].y * (cLocal.y - vertices[i].y);//b2Dot_v2_v2(normals[i], b2Vec2.Subtract(cLocal, vertices[i]));

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
		manifold.localNormal.x = normals[normalIndex].x;//.Assign(normals[normalIndex]);
		manifold.localNormal.y = normals[normalIndex].y;
		manifold.localPoint.x = 0.5 * (v1.x + v2.x);//.Assign(b2Vec2.Multiply(0.5, b2Vec2.Add(v1, v2)));
		manifold.localPoint.y = 0.5 * (v1.y + v2.y);
		manifold.points[0] = new b2ManifoldPoint();
		manifold.points[0].localPoint.x = circleB.m_p.x;
		manifold.points[0].localPoint.y = circleB.m_p.y;
		manifold.points[0].id.Reset();
		return;
	}

	// Compute barycentric coordinates
	var u1 = (cLocal.x - v1.x) * (v2.x - v1.x) + (cLocal.y - v1.y) * (v2.y - v1.y);//b2Dot_v2_v2(b2Vec2.Subtract(cLocal, v1), b2Vec2.Subtract(v2, v1));
	var u2 = (cLocal.x - v2.x) * (v1.x - v2.x) + (cLocal.y - v2.y) * (v1.y - v2.y);//b2Dot_v2_v2(b2Vec2.Subtract(cLocal, v2), b2Vec2.Subtract(v1, v2));
	if (u1 <= 0.0)
	{
		if (b2DistanceSquared(cLocal, v1) > radius * radius)
		{
			return;
		}

		manifold.pointCount = 1;
		manifold.type = b2Manifold.e_faceA;
		manifold.localNormal.x = cLocal.x - v1.x;//.Assign(b2Vec2.Subtract(cLocal, v1));
		manifold.localNormal.y = cLocal.y - v1.y;
		manifold.localNormal.Normalize();
		manifold.localPoint.x = v1.x;
		manifold.localPoint.y = v1.y;
		manifold.points[0] = new b2ManifoldPoint();
		manifold.points[0].localPoint.x = circleB.m_p.x;
		manifold.points[0].localPoint.y = circleB.m_p.y;
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
		manifold.localNormal.x = cLocal.x - v2.x;//.Assign(b2Vec2.Subtract(cLocal, v2));
		manifold.localNormal.y = cLocal.y - v2.y;
		manifold.localNormal.Normalize();
		manifold.localPoint.x = v2.x;
		manifold.localPoint.y = v2.y;
		manifold.points[0] = new b2ManifoldPoint();
		manifold.points[0].localPoint.x = circleB.m_p.x;
		manifold.points[0].localPoint.y = circleB.m_p.y;
		manifold.points[0].id.Reset();
	}
	else
	{
		var faceCenterx = 0.5 * (v1.x + v2.x);//b2Vec2.Multiply(0.5, b2Vec2.Add(v1, v2));
		var faceCentery = 0.5 * (v1.y + v2.y);
		var separation = (cLocal.x - faceCenterx) * normals[vertIndex1].x + (cLocal.y - faceCentery) * normals[vertIndex1].y;//b2Dot_v2_v2(b2Vec2.Subtract(cLocal, faceCenter), normals[vertIndex1]);
		if (separation > radius)
		{
			return;
		}

		manifold.pointCount = 1;
		manifold.type = b2Manifold.e_faceA;
		manifold.localNormal.x = normals[vertIndex1].x;
		manifold.localNormal.y = normals[vertIndex1].y;
		manifold.localPoint.x = faceCenterx;
		manifold.localPoint.y = faceCentery;
		manifold.points[0] = new b2ManifoldPoint();
		manifold.points[0].localPoint.x = circleB.m_p.x;
		manifold.points[0].localPoint.y = circleB.m_p.y;
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
		var nx = xf.q.c * n1s[i].x - xf.q.s * n1s[i].y;//b2Mul_r_v2(xf.q, n1s[i]);
		var ny = xf.q.s * n1s[i].x + xf.q.c * n1s[i].y;
		var v1x = (xf.q.c * v1s[i].x - xf.q.s * v1s[i].y) + xf.p.x;//b2Mul_t_v2(xf, v1s[i]);
		var v1y = (xf.q.s * v1s[i].x + xf.q.c * v1s[i].y) + xf.p.y;

		// Find deepest point for normal i.
		var si = b2_maxFloat;
		for (var j = 0; j < count2; ++j)
		{
			var sij = nx * (v2s[j].x - v1x) + ny * (v2s[j].y - v1y);//b2Dot_v2_v2(n, b2Vec2.Subtract(v2s[j], v1));
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

'#if @DEBUG';
	b2Assert(0 <= edge1 && edge1 < poly1.m_count);
'#endif';

	// Get the normal of the reference edge in poly2's frame.
	//var normal1 = b2MulT_r_v2(xf2.q, b2Mul_r_v2(xf1.q, normals1[edge1]));
	var t1x = xf1.q.c * normals1[edge1].x - xf1.q.s * normals1[edge1].y;
	var t1y = xf1.q.s * normals1[edge1].x + xf1.q.c * normals1[edge1].y;
	var normal1x = xf2.q.c * t1x + xf2.q.s * t1y;
	var normal1y = -xf2.q.s * t1x + xf2.q.c * t1y;

	// Find the incident edge on poly2.
	var index = 0;
	var minDot = b2_maxFloat;
	for (var i = 0; i < count2; ++i)
	{
		var dot = normal1x * normals2[i].x + normal1y * normals2[i].y;//b2Dot_v2_v2(normal1, normals2[i]);
		if (dot < minDot)
		{
			minDot = dot;
			index = i;
		}
	}

	// Build the clip vertices for the incident edge.
	var i1 = index;
	var i2 = i1 + 1 < count2 ? i1 + 1 : 0;

	c[0].v.x = (xf2.q.c * vertices2[i1].x - xf2.q.s * vertices2[i1].y) + xf2.p.x;//.Assign(b2Mul_t_v2(xf2, vertices2[i1]));
	c[0].v.y = (xf2.q.s * vertices2[i1].x + xf2.q.c * vertices2[i1].y) + xf2.p.y;
	c[0].id.indexA = edge1;
	c[0].id.indexB = i1;
	c[0].id.typeA = b2ContactID.e_face;
	c[0].id.typeB = b2ContactID.e_vertex;

	c[1].v.x = (xf2.q.c * vertices2[i2].x - xf2.q.s * vertices2[i2].y) + xf2.p.x;//.Assign(b2Mul_t_v2(xf2, vertices2[i2]));
	c[1].v.y = (xf2.q.s * vertices2[i2].x + xf2.q.c * vertices2[i2].y) + xf2.p.y;
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
	var xf1, xf2;
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

	b2FindIncidentEdge(b2CollidePolygons._local_incidentEdges, poly1, xf1, edge1, poly2, xf2);

	var count1 = poly1.m_count;
	var vertices1 = poly1.m_vertices;

	var iv1 = edge1;
	var iv2 = edge1 + 1 < count1 ? edge1 + 1 : 0;

	var v11 = vertices1[iv1];
	var v12 = vertices1[iv2];

	b2CollidePolygons._localTangent.x = v12.x - v11.x;
	b2CollidePolygons._localTangent.y = v12.y - v11.y;
	b2CollidePolygons._localTangent.Normalize();

	var localNormalx = 1.0 * b2CollidePolygons._localTangent.y;//b2Cross_v2_f(b2CollidePolygons._localTangent, 1.0);
	var localNormaly = -1.0 * b2CollidePolygons._localTangent.x;
	var planePointx = 0.5 * (v11.x + v12.x);//b2Vec2.Multiply(0.5, b2Vec2.Add(v11, v12));
	var planePointy = 0.5 * (v11.y + v12.y);

	var tangentx = xf1.q.c * b2CollidePolygons._localTangent.x - xf1.q.s * b2CollidePolygons._localTangent.y;//b2Mul_r_v2(xf1.q, b2CollidePolygons._localTangent);
	var tangenty = xf1.q.s * b2CollidePolygons._localTangent.x + xf1.q.c * b2CollidePolygons._localTangent.y;
	var normalx = 1.0 * tangenty;//b2Cross_v2_f(tangent, 1.0);
	var normaly = -1.0 * tangentx;

	v11 = b2Mul_t_v2(xf1, v11);
	v12 = b2Mul_t_v2(xf1, v12);

	// Face offset.
	var frontOffset = normalx * v11.x + normaly * v11.y;//b2Dot_v2_v2(normal, v11);

	// Side offsets, extended by polytope skin thickness.
	var sideOffset1 = /*-b2Dot_v2_v2(tangent, v11)*/-(tangentx * v11.x + tangenty * v11.y) + totalRadius;
	var sideOffset2 = /*b2Dot_v2_v2(tangent, v12)*/(tangentx * v12.x + tangenty * v12.y) + totalRadius;

	// Clip incident edge against extruded edge1 side edges.
	var clipPoints1 = new Array(2);
	var clipPoints2 = new Array(2);
	var np;

	// Clip to box side 1
	np = b2ClipSegmentToLine(clipPoints1, b2CollidePolygons._local_incidentEdges, -tangentx, -tangenty, sideOffset1, iv1);

	if (np < 2)
		return;

	// Clip to negative box side 1
	np = b2ClipSegmentToLine(clipPoints2, clipPoints1, tangentx, tangenty, sideOffset2, iv2);

	if (np < 2)
	{
		return;
	}

	// Now clipPoints2 contains the clipped points.
	manifold.localNormal.x = localNormalx;//.Assign(localNormal);
	manifold.localNormal.y = localNormaly;
	manifold.localPoint.x = planePointx;//.Assign(planePoint);
	manifold.localPoint.y = planePointy;

	var pointCount = 0;
	for (var i = 0; i < b2_maxManifoldPoints; ++i)
	{
		var separation = /*b2Dot_v2_v2(normal, clipPoints2[i].v)*/(normalx * clipPoints2[i].v.x + normaly * clipPoints2[i].v.y) - frontOffset;

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

b2CollidePolygons._localTangent = new b2Vec2();

b2CollidePolygons._local_incidentEdges = [new b2ClipVertex(), new b2ClipVertex()];

/// Compute the collision manifold between an edge and a circle.
function b2CollideEdgeAndCircle(manifold,
							   edgeA, xfA,
							   circleB, xfB)
{
	manifold.pointCount = 0;

	// Compute circle in frame of edge
	var Q = b2MulT_t_v2(xfA, b2Mul_t_v2(xfB, circleB.m_p));

	var A = edgeA.m_vertex1, B = edgeA.m_vertex2;
	var ex = B.x - A.x;//b2Vec2.Subtract(B, A);
	var ey = B.y - A.y;

	// Barycentric coordinates
	var u = ex * (B.x - Q.x) + ey * (B.y - Q.y);//b2Dot_v2_v2(e, b2Vec2.Subtract(B, Q));
	var v = ex * (Q.x - A.x) + ey * (Q.y - A.y);//b2Dot_v2_v2(e, b2Vec2.Subtract(Q, A));

	var radius = edgeA.m_radius + circleB.m_radius;

	var cf = new b2ContactID();
	cf.indexB = 0;
	cf.typeB = b2ContactID.e_vertex;

	// Region A
	if (v <= 0.0)
	{
		var P = A;
		var dx = Q.x - P.x;//b2Vec2.Subtract(Q, P);
		var dy = Q.y - P.y;
		var dd = dx * dx + dy * dy;//b2Dot_v2_v2(d, d);
		if (dd > radius * radius)
		{
			return;
		}

		// Is there an edge connected to A?
		if (edgeA.m_hasVertex0)
		{
			var A1 = edgeA.m_vertex0;
			var B1 = A;
			var e1x = B1.x - A1.x;//b2Vec2.Subtract(B1, A1);
			var e1y = B1.y - A1.y;
			var u1 = e1x * (B1.x - Q.x) + e1y * (B1.y - Q.y);//b2Dot_v2_v2(e1, b2Vec2.Subtract(B1, Q));

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
		manifold.localNormal.x = manifold.localNormal.y = 0;
		manifold.localPoint.x = P.x;
		manifold.localPoint.y = P.y;
		manifold.points[0] = new b2ManifoldPoint();
		manifold.points[0].id.Assign(cf);
		manifold.points[0].localPoint.x = circleB.m_p.x;
		manifold.points[0].localPoint.y = circleB.m_p.y;
		return;
	}

	// Region B
	if (u <= 0.0)
	{
		var P = B;
		var dx = Q.x - P.x;//b2Vec2.Subtract(Q, P);
		var dy = Q.y - P.y;
		var dd = dx * dx + dy * dy;//b2Dot_v2_v2(d, d);
		if (dd > radius * radius)
		{
			return;
		}

		// Is there an edge connected to B?
		if (edgeA.m_hasVertex3)
		{
			var B2 = edgeA.m_vertex3;
			var A2 = B;
			var e2x = B2.x - A2.x;//b2Vec2.Subtract(B2, A2);
			var e2y = B2.y - A2.y;
			var v2 = e2x * (Q.x - A2.x) + e2y * (Q.y - A2.y);//b2Dot_v2_v2(e2, b2Vec2.Subtract(Q, A2));

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
		manifold.localNormal.x = manifold.localNormal.y = 0;
		manifold.localPoint.x = P.x;
		manifold.localPoint.y = P.y;
		manifold.points[0] = new b2ManifoldPoint();
		manifold.points[0].id.Assign(cf);
		manifold.points[0].localPoint.x = circleB.m_p.x;
		manifold.points[0].localPoint.y = circleB.m_p.y;
		return;
	}

	// Region AB
	var den = ex * ex + ey * ey;//b2Dot_v2_v2(e, e);
'#if @DEBUG';
	b2Assert(den > 0.0);
'#endif';
	var Px = (1.0 / den) * ((u * A.x) + (v * B.x));
	var Py = (1.0 / den) * ((u * A.y) + (v * B.y));//b2Vec2.Multiply((1.0 / den), b2Vec2.Add(b2Vec2.Multiply(u, A), b2Vec2.Multiply(v, B)));
	var dx = Q.x - Px;//b2Vec2.Subtract(Q, P);
	var dy = Q.y - Py;
	var dd = dx * dx + dy * dy;//b2Dot_v2_v2(d, d);
	if (dd > radius * radius)
	{
		return;
	}

	var nx = -ey;//new b2Vec2(-ey, ex);
	var ny = ex;
	if (nx * (Q.x - A.x) + ny * (Q.y - A.y) < 0.0)//b2Dot_v2_v2(n, b2Vec2.Subtract(Q, A)) < 0.0)
	{
		nx = -nx;//.Set(-n.x, -n.y);
		ny = -ny;
	}
	//n.Normalize();

	cf.indexA = 0;
	cf.typeA = b2ContactID.e_face;
	manifold.pointCount = 1;
	manifold.type = b2Manifold.e_faceA;
	manifold.localNormal.x = nx;
	manifold.localNormal.y = ny;
	manifold.localNormal.Normalize();
	manifold.localPoint.x = A.x;
	manifold.localPoint.y = A.y;
	manifold.points[0] = new b2ManifoldPoint();
	manifold.points[0].id.Assign(cf);
	manifold.points[0].localPoint.x = circleB.m_p.x;
	manifold.points[0].localPoint.y = circleB.m_p.y;
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

b2EPCollider._temp_edge = new b2Vec2();
b2EPCollider._temp_edge0 = new b2Vec2();
b2EPCollider._temp_edge2 = new b2Vec2();

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

		//this.m_centroidB.Assign(b2Mul_t_v2(this.m_xf, polygonB.m_centroid));
		this.m_centroidB.x = (this.m_xf.q.c * polygonB.m_centroid.x - this.m_xf.q.s * polygonB.m_centroid.y) + this.m_xf.p.x;
		this.m_centroidB.y = (this.m_xf.q.s * polygonB.m_centroid.x + this.m_xf.q.c * polygonB.m_centroid.y) + this.m_xf.p.y;

		this.m_v0.x = edgeA.m_vertex0.x;
		this.m_v0.y = edgeA.m_vertex0.y;
		this.m_v1.x = edgeA.m_vertex1.x;
		this.m_v1.y = edgeA.m_vertex1.y;
		this.m_v2.x = edgeA.m_vertex2.x;
		this.m_v2.y = edgeA.m_vertex2.y;
		this.m_v3.x = edgeA.m_vertex3.x;
		this.m_v3.y = edgeA.m_vertex3.y;

		var hasVertex0 = edgeA.m_hasVertex0;
		var hasVertex3 = edgeA.m_hasVertex3;

		b2EPCollider._temp_edge.x = this.m_v2.x - this.m_v1.x;// = b2Vec2.Subtract(this.m_v2, this.m_v1);
		b2EPCollider._temp_edge.y = this.m_v2.y - this.m_v1.y;
		b2EPCollider._temp_edge.Normalize();
		this.m_normal1.x = b2EPCollider._temp_edge.y;
		this.m_normal1.y = -b2EPCollider._temp_edge.x;
		//var offset1 = b2Dot_v2_v2(this.m_normal1, b2Vec2.Subtract(this.m_centroidB, this.m_v1));
		var offset1 = this.m_normal1.x * (this.m_centroidB.x - this.m_v1.x) + this.m_normal1.y * (this.m_centroidB.y - this.m_v1.y);
		var offset0 = 0.0, offset2 = 0.0;
		var convex1 = false, convex2 = false;

		// Is there a preceding edge?
		if (hasVertex0)
		{
			b2EPCollider._temp_edge0.x = this.m_v1.x - this.m_v0.x;
			b2EPCollider._temp_edge0.y = this.m_v1.y - this.m_v0.y;
			b2EPCollider._temp_edge0.Normalize();
			this.m_normal0.x = b2EPCollider._temp_edge0.y;
			this.m_normal0.y = -b2EPCollider._temp_edge0.x;
			//convex1 = b2Cross_v2_v2(b2EPCollider._temp_edge0, b2EPCollider._temp_edge) >= 0.0;
			convex1 = (b2EPCollider._temp_edge0.x * b2EPCollider._temp_edge.y - b2EPCollider._temp_edge0.y * b2EPCollider._temp_edge.x) >= 0;
			//offset0 = b2Dot_v2_v2(this.m_normal0, b2Vec2.Subtract(this.m_centroidB, this.m_v0));
			offset0 = this.m_normal0.x * (this.m_centroidB.x - this.m_v0.x) + this.m_normal0.y * (this.m_centroidB.y - this.m_v0.y);
		}

		// Is there a following edge?
		if (hasVertex3)
		{
			b2EPCollider._temp_edge2.x = this.m_v3.x - this.m_v2.x;
			b2EPCollider._temp_edge2.y = this.m_v3.y - this.m_v2.y;
			b2EPCollider._temp_edge2.Normalize();
			this.m_normal2.x = b2EPCollider._temp_edge2.y;
			this.m_normal2.y = -b2EPCollider._temp_edge2.x;
			//convex2 = b2Cross_v2_v2(b2EPCollider._temp_edge, b2EPCollider._temp_edge2) > 0.0;
			convex2 = (b2EPCollider._temp_edge.x * b2EPCollider._temp_edge2.y - b2EPCollider._temp_edge.y * b2EPCollider._temp_edge2.x) > 0.0;
			//offset2 = b2Dot_v2_v2(this.m_normal2, b2Vec2.Subtract(this.m_centroidB, this.m_v2));
			offset2 = this.m_normal2.x * (this.m_centroidB.x - this.m_v2.x) + this.m_normal2.y * (this.m_centroidB.y - this.m_v2.y);
		}

		// Determine front or back collision. Determine collision normal limits.
		if (hasVertex0 && hasVertex3)
		{
			if (convex1 && convex2)
			{
				this.m_front = offset0 >= 0.0 || offset1 >= 0.0 || offset2 >= 0.0;
				if (this.m_front)
				{
					this.m_normal.x = this.m_normal1.x;
					this.m_normal.y = this.m_normal1.y;
					this.m_lowerLimit.x = this.m_normal0.x;
					this.m_lowerLimit.y = this.m_normal0.y;
					this.m_upperLimit.x = this.m_normal2.x;
					this.m_upperLimit.y = this.m_normal2.y;
				}
				else
				{
					this.m_normal.x = -this.m_normal1.x;
					this.m_normal.y = -this.m_normal1.y;
					this.m_lowerLimit.x = -this.m_normal1.x;
					this.m_lowerLimit.y = -this.m_normal1.y;
					this.m_upperLimit.x = -this.m_normal1.x;
					this.m_upperLimit.y = -this.m_normal1.y;
				}
			}
			else if (convex1)
			{
				this.m_front = offset0 >= 0.0 || (offset1 >= 0.0 && offset2 >= 0.0);
				if (this.m_front)
				{
					this.m_normal.x = this.m_normal1.x;
					this.m_normal.y = this.m_normal1.y;
					this.m_lowerLimit.x = this.m_normal0.x;
					this.m_lowerLimit.y = this.m_normal0.y;
					this.m_upperLimit.x = this.m_normal1.x;
					this.m_upperLimit.y = this.m_normal1.y;
				}
				else
				{
					this.m_normal.x = -this.m_normal1.x;
					this.m_normal.y = -this.m_normal1.y;
					this.m_lowerLimit.x = -this.m_normal2.x;
					this.m_lowerLimit.y = -this.m_normal2.y;
					this.m_upperLimit.x = -this.m_normal1.x;
					this.m_upperLimit.y = -this.m_normal1.y;
				}
			}
			else if (convex2)
			{
				this.m_front = offset2 >= 0.0 || (offset0 >= 0.0 && offset1 >= 0.0);
				if (this.m_front)
				{
					this.m_normal.x = this.m_normal1.x;
					this.m_normal.y = this.m_normal1.y;
					this.m_lowerLimit.x = this.m_normal1.x;
					this.m_lowerLimit.y = this.m_normal1.y;
					this.m_upperLimit.x = this.m_normal2.x;
					this.m_upperLimit.y = this.m_normal2.y;
				}
				else
				{
					this.m_normal.x = -this.m_normal1.x;
					this.m_normal.y = -this.m_normal1.y;
					this.m_lowerLimit.x = -this.m_normal1.x;
					this.m_lowerLimit.y = -this.m_normal1.y;
					this.m_upperLimit.x = -this.m_normal0.x;
					this.m_upperLimit.y = -this.m_normal0.y;
				}
			}
			else
			{
				this.m_front = offset0 >= 0.0 && offset1 >= 0.0 && offset2 >= 0.0;
				if (this.m_front)
				{
					this.m_normal.x = this.m_normal1.x;
					this.m_normal.y = this.m_normal1.y;
					this.m_lowerLimit.x = this.m_normal1.x;
					this.m_lowerLimit.y = this.m_normal1.y;
					this.m_upperLimit.x = this.m_normal1.x;
					this.m_upperLimit.y = this.m_normal1.y;
				}
				else
				{
					this.m_normal.x = -this.m_normal1.x;
					this.m_normal.y = -this.m_normal1.y;
					this.m_lowerLimit.x = -this.m_normal2.x;
					this.m_lowerLimit.y = -this.m_normal2.y;
					this.m_upperLimit.x = -this.m_normal0.x;
					this.m_upperLimit.y = -this.m_normal0.y;
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
					this.m_normal.x = this.m_normal1.x;
					this.m_normal.y = this.m_normal1.y;
					this.m_lowerLimit.x = this.m_normal0.x;
					this.m_lowerLimit.y = this.m_normal0.y;
					this.m_upperLimit.x = -this.m_normal1.x;
					this.m_upperLimit.y = -this.m_normal1.y;
				}
				else
				{
					this.m_normal.x = -this.m_normal1.x;
					this.m_normal.y = -this.m_normal1.y;
					this.m_lowerLimit.x = this.m_normal1.x;
					this.m_lowerLimit.y = this.m_normal1.y;
					this.m_upperLimit.x = -this.m_normal1.x;
					this.m_upperLimit.y = -this.m_normal1.y;
				}
			}
			else
			{
				this.m_front = offset0 >= 0.0 && offset1 >= 0.0;
				if (this.m_front)
				{
					this.m_normal.x = this.m_normal1.x;
					this.m_normal.y = this.m_normal1.y;
					this.m_lowerLimit.x = this.m_normal1.x;
					this.m_lowerLimit.y = this.m_normal1.y;
					this.m_upperLimit.x = -this.m_normal1.x;
					this.m_upperLimit.y = -this.m_normal1.y;
				}
				else
				{
					this.m_normal.x = -this.m_normal1.x;
					this.m_normal.y = -this.m_normal1.y;
					this.m_lowerLimit.x = this.m_normal1.x;
					this.m_lowerLimit.y = this.m_normal1.y;
					this.m_upperLimit.x = -this.m_normal0.x;
					this.m_upperLimit.y = -this.m_normal0.y;
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
					this.m_normal.x = this.m_normal1.x;
					this.m_normal.y = this.m_normal1.y;
					this.m_lowerLimit.x = -this.m_normal1.x;
					this.m_lowerLimit.y = -this.m_normal1.y;
					this.m_upperLimit.x = this.m_normal2.x;
					this.m_upperLimit.y = this.m_normal2.y;
				}
				else
				{
					this.m_normal.x = -this.m_normal1.x;
					this.m_normal.y = -this.m_normal1.y;
					this.m_lowerLimit.x = -this.m_normal1.x;
					this.m_lowerLimit.y = -this.m_normal1.y;
					this.m_upperLimit.x = this.m_normal1.x;
					this.m_upperLimit.y = this.m_normal1.y;
				}
			}
			else
			{
				this.m_front = offset1 >= 0.0 && offset2 >= 0.0;
				if (this.m_front)
				{
					this.m_normal.x = this.m_normal1.x;
					this.m_normal.y = this.m_normal1.y;
					this.m_lowerLimit.x = -this.m_normal1.x;
					this.m_lowerLimit.y = -this.m_normal1.y;
					this.m_upperLimit.x = this.m_normal1.x;
					this.m_upperLimit.y = this.m_normal1.y;
				}
				else
				{
					this.m_normal.x = -this.m_normal1.x;
					this.m_normal.y = -this.m_normal1.y;
					this.m_lowerLimit.x = -this.m_normal2.x;
					this.m_lowerLimit.y = -this.m_normal2.y;
					this.m_upperLimit.x = this.m_normal1.x;
					this.m_upperLimit.y = this.m_normal1.y;
				}
			}
		}
		else
		{
			this.m_front = offset1 >= 0.0;
			if (this.m_front)
			{
				this.m_normal.x = this.m_normal1.x;
				this.m_normal.y = this.m_normal1.y;
				this.m_lowerLimit.x = -this.m_normal1.x;
				this.m_lowerLimit.y = -this.m_normal1.y;
				this.m_upperLimit.x = -this.m_normal1.x;
				this.m_upperLimit.y = -this.m_normal1.y;
			}
			else
			{
				this.m_normal.x = -this.m_normal1.x;
				this.m_normal.y = -this.m_normal1.y;
				this.m_lowerLimit.x = this.m_normal1.x;
				this.m_lowerLimit.y = this.m_normal1.y;
				this.m_upperLimit.x = this.m_normal1.x;
				this.m_upperLimit.y = this.m_normal1.y;
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
			var bestValue = this.m_normal.x * this.m_polygonB.normals[0].x + this.m_normal.y * this.m_polygonB.normals[0].y;//b2Dot_v2_v2(this.m_normal, this.m_polygonB.normals[0]);
			for (var i = 1; i < this.m_polygonB.count; ++i)
			{
				var value = this.m_normal.x * this.m_polygonB.normals[i].x + this.m_normal.y * this.m_polygonB.normals[i].y;//b2Dot_v2_v2(this.m_normal, this.m_polygonB.normals[i]);
				if (value < bestValue)
				{
					bestValue = value;
					bestIndex = i;
				}
			}

			var i1 = bestIndex;
			var i2 = i1 + 1 < this.m_polygonB.count ? i1 + 1 : 0;

			ie[0] = new b2ClipVertex();
			ie[0].v.x = this.m_polygonB.vertices[i1].x;
			ie[0].v.y = this.m_polygonB.vertices[i1].y;
			ie[0].id.indexA = 0;
			ie[0].id.indexB = i1;
			ie[0].id.typeA = b2ContactID.e_face;
			ie[0].id.typeB = b2ContactID.e_vertex;

			ie[1] = new b2ClipVertex();
			ie[1].v.x = this.m_polygonB.vertices[i2].x;
			ie[1].v.y = this.m_polygonB.vertices[i2].y;
			ie[1].id.indexA = 0;
			ie[1].id.indexB = i2;
			ie[1].id.typeA = b2ContactID.e_face;
			ie[1].id.typeB = b2ContactID.e_vertex;

			if (this.m_front)
			{
				rf.i1 = 0;
				rf.i2 = 1;
				rf.v1.x = this.m_v1.x;
				rf.v1.y = this.m_v1.y;
				rf.v2.x = this.m_v2.x;
				rf.v2.y = this.m_v2.y;
				rf.normal.x = this.m_normal1.x;
				rf.normal.y = this.m_normal1.y;
			}
			else
			{
				rf.i1 = 1;
				rf.i2 = 0;
				rf.v1.x = this.m_v2.x;
				rf.v1.y = this.m_v2.y;
				rf.v2.x = this.m_v1.x;
				rf.v2.y = this.m_v1.y;
				rf.normal.x = -this.m_normal1.x;
				rf.normal.y = -this.m_normal1.y;
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
			rf.v1.x = this.m_polygonB.vertices[rf.i1].x;
			rf.v1.y = this.m_polygonB.vertices[rf.i1].y;
			rf.v2.x = this.m_polygonB.vertices[rf.i2].x;
			rf.v2.y = this.m_polygonB.vertices[rf.i2].y;
			rf.normal.x = this.m_polygonB.normals[rf.i1].x;
			rf.normal.y = this.m_polygonB.normals[rf.i1].y;
		}

		rf.sideNormal1.x = rf.normal.y;
		rf.sideNormal1.y = -rf.normal.x;
		rf.sideNormal2.x = -rf.sideNormal1.x;
		rf.sideNormal2.y = -rf.sideNormal1.y;
		rf.sideOffset1 = rf.sideNormal1.x * rf.v1.x + rf.sideNormal1.y * rf.v1.y;//b2Dot_v2_v2(rf.sideNormal1, rf.v1);
		rf.sideOffset2 = rf.sideNormal2.x * rf.v2.x + rf.sideNormal2.y * rf.v2.y;//b2Dot_v2_v2(rf.sideNormal2, rf.v2);

		// Clip incident edge against extruded edge1 side edges.
		var clipPoints1 = new Array(2);
		var clipPoints2 = new Array(2);
		var np;

		// Clip to box side 1
		np = b2ClipSegmentToLine(clipPoints1, ie, rf.sideNormal1.x, rf.sideNormal1.y, rf.sideOffset1, rf.i1);

		if (np < b2_maxManifoldPoints)
		{
			return;
		}

		// Clip to negative box side 1
		np = b2ClipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2.x, rf.sideNormal2.y, rf.sideOffset2, rf.i2);

		if (np < b2_maxManifoldPoints)
		{
			return;
		}

		// Now clipPoints2 contains the clipped points.
		if (primaryAxis.type == b2EPAxis.e_edgeA)
		{
			manifold.localNormal.x = rf.normal.x;
			manifold.localNormal.y = rf.normal.y;
			manifold.localPoint.x = rf.v1.x;
			manifold.localPoint.y = rf.v1.y;
		}
		else
		{
			manifold.localNormal.x = polygonB.m_normals[rf.i1].x;
			manifold.localNormal.y = polygonB.m_normals[rf.i1].y;
			manifold.localPoint.x = polygonB.m_vertices[rf.i1].x;
			manifold.localPoint.y = polygonB.m_vertices[rf.i1].y;
		}

		var pointCount = 0;
		for (var i = 0; i < b2_maxManifoldPoints; ++i)
		{
			//var separation;
			//separation = b2Dot_v2_v2(rf.normal, b2Vec2.Subtract(clipPoints2[i].v, rf.v1));
			var separation = rf.normal.x * (clipPoints2[i].v.x - rf.v1.x) + rf.normal.y * (clipPoints2[i].v.y - rf.v1.y);

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
					//cp.localPoint.Assign(clipPoints2[i].v);
					cp.localPoint.x = clipPoints2[i].v.x;
					cp.localPoint.y = clipPoints2[i].v.y;
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
			//var s = b2Dot_v2_v2(this.m_normal, b2Vec2.Subtract(this.m_polygonB.vertices[i], this.m_v1));
			var s = this.m_normal.x * (this.m_polygonB.vertices[i].x - this.m_v1.x) + this.m_normal.y * (this.m_polygonB.vertices[i].y - this.m_v1.y);
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

		var perpx = -this.m_normal.y;// = new b2Vec2(-this.m_normal.y, this.m_normal.x);
		var perpy = this.m_normal.x;

		for (var i = 0; i < this.m_polygonB.count; ++i)
		{
			var nx = -this.m_polygonB.normals[i].x;//this.m_polygonB.normals[i].Negate();
			var ny = -this.m_polygonB.normals[i].y;

			//var s1 = b2Dot_v2_v2(n, b2Vec2.Subtract(this.m_polygonB.vertices[i], this.m_v1));
			var s1 = nx * (this.m_polygonB.vertices[i].x - this.m_v1.x) + ny * (this.m_polygonB.vertices[i].y - this.m_v1.y);
			//var s2 = b2Dot_v2_v2(n, b2Vec2.Subtract(this.m_polygonB.vertices[i], this.m_v2));
			var s2 = nx * (this.m_polygonB.vertices[i].x - this.m_v2.x) + ny * (this.m_polygonB.vertices[i].y - this.m_v2.y);
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
			if (/*b2Dot_v2_v2(n, perp)*/nx * perpx + ny * perpy >= 0.0)
			{
				if (/*b2Dot_v2_v2(b2Vec2.Subtract(n, this.m_upperLimit), this.m_normal)*/
					(nx - this.m_upperLimit.x) * this.m_normal.x + (ny - this.m_upperLimit.y) * this.m_normal.y < -b2_angularSlop)
				{
					continue;
				}
			}
			else
			{
				if (
					/*b2Dot_v2_v2(b2Vec2.Subtract(n, this.m_lowerLimit), this.m_normal)*/
					(nx - this.m_lowerLimit.x) * this.m_normal.x + (ny - this.m_lowerLimit.y) * this.m_normal.y< -b2_angularSlop)
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
	b2CollideEdgeAndPolygon.collider.Collide(manifold, edgeA, xfA, polygonB, xfB);
}

b2CollideEdgeAndPolygon.collider = new b2EPCollider();

/// Clipping for contact manifolds.
function b2ClipSegmentToLine(vOut, vIn,
							normalx, normaly, offset, vertexIndexA)
{
	// Start with no output points
	var numOut = 0;

	// Calculate the distance of end points to the line
	var distance0 = /*b2Dot_v2_v2(normal, vIn[0].v)*/(normalx * vIn[0].v.x + normaly * vIn[0].v.y) - offset;
	var distance1 = /*b2Dot_v2_v2(normal, vIn[1].v)*/(normalx * vIn[1].v.x + normaly * vIn[1].v.y) - offset;

	// If the points are behind the plane
	if (distance0 <= 0.0) vOut[numOut++] = vIn[0];
	if (distance1 <= 0.0) vOut[numOut++] = vIn[1];

	// If the points are on different sides of the plane
	if (distance0 * distance1 < 0.0)
	{
		// Find intersection point of edge and plane
		var interp = distance0 / (distance0 - distance1);
		vOut[numOut] = new b2ClipVertex();
		//vOut[numOut].v.Assign(b2Vec2.Add(vIn[0].v, b2Vec2.Multiply(interp, b2Vec2.Subtract(vIn[1].v, vIn[0].v))));
		vOut[numOut].v.x = vIn[0].v.x + (interp * (vIn[1].v.x - vIn[0].v.x));
		vOut[numOut].v.y = vIn[0].v.y + (interp * (vIn[1].v.y - vIn[0].v.y));

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
	b2TestShapeOverlap.input.proxyA.Set(shapeA, indexA);
	b2TestShapeOverlap.input.proxyB.Set(shapeB, indexB);
	b2TestShapeOverlap.input.transformA = xfA;
	b2TestShapeOverlap.input.transformB = xfB;
	b2TestShapeOverlap.input.useRadii = true;

	b2TestShapeOverlap.cache.count = 0;

	b2DistanceFunc(b2TestShapeOverlap.output, b2TestShapeOverlap.cache, b2TestShapeOverlap.input);

	return b2TestShapeOverlap.output.distance < 10.0 * b2_epsilon;
}

b2TestShapeOverlap.input = new b2DistanceInput();
b2TestShapeOverlap.cache = new b2SimplexCache();
b2TestShapeOverlap.output = new b2DistanceOutput();

function b2TestOverlap(a, b)
{
	return !((b.lowerBound.x - a.upperBound.x) > 0.0 || (b.lowerBound.y - a.upperBound.y) > 0.0 || (a.lowerBound.x - b.upperBound.x) > 0.0 || (a.lowerBound.y - b.upperBound.y) > 0.0);
}