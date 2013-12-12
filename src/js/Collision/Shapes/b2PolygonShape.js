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

"use strict";

/// A convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
function b2PolygonShape()
{
	this.m_type = b2Shape.e_polygon;
	this.m_radius = b2_polygonRadius;
	this.m_count = 0;
	this.m_centroid = new b2Vec2();

	this.m_vertices = new Array(b2_maxPolygonVertices);
	this.m_normals = new Array(b2_maxPolygonVertices);
}

b2PolygonShape.prototype =
{
	/// Implement b2Shape.
	Clone: function()
	{
		var shape = new b2PolygonShape();
		shape.m_count = this.m_count;
		shape.m_centroid = this.m_centroid.Clone();

		for (var i = 0; i < this.m_count; ++i)
		{
			shape.m_vertices[i] = this.m_vertices[i].Clone();
			shape.m_normals[i] = this.m_normals[i].Clone();
		}

		return shape;
	},

	/// @see b2Shape::GetChildCount
	GetChildCount: function()
	{
		return 1;
	},

	/// Create a convex hull from the given array of local points.
	/// The count must be in the range [3, b2_maxPolygonVertices].
	/// @warning the points may be re-ordered, even if they form a convex polygon
	/// @warning collinear points are handled but not removed. Collinear points
	/// may lead to poor stacking behavior.
	Set: function(vertices, count)
	{
		b2Assert(3 <= count && count <= b2_maxPolygonVertices);

		if (count < 3)
		{
			this.SetAsBox(1.0, 1.0);
			return;
		}

		var n = b2Min(count, b2_maxPolygonVertices);

		// Perform welding and copy vertices into local buffer.
		var ps = new Array(b2_maxPolygonVertices);
		var tempCount = 0;

		for (var i = 0; i < n; ++i)
		{
			var v = vertices[i];
			var unique = true;

			for (var j = 0; j < tempCount; ++j)
			{
				if (b2DistanceSquared(v, ps[j]) < 0.5 * b2_linearSlop)
				{
					unique = false;
					break;
				}
			}

			if (unique)
			{
				ps[tempCount++] = v.Clone();
			}
		}

		n = tempCount;
		if (n < 3)
		{
			// Polygon is degenerate.
			b2Assert(false);
			this.SetAsBox(1.0, 1.0);
			return;
		}

		// Create the convex hull using the Gift wrapping algorithm
		// http://en.wikipedia.org/wiki/Gift_wrapping_algorithm

		// Find the right most point on the hull
		var i0 = 0;
		var x0 = ps[0].x;
		for (i = 1; i < n; ++i)
		{
			var x = ps[i].x;
			if (x > x0 || (x == x0 && ps[i].y < ps[i0].y))
			{
				i0 = i;
				x0 = x;
			}
		}

		var hull = new Array(b2_maxPolygonVertices);
		var m = 0;
		var ih = i0;

		for (;;)
		{
			hull[m] = ih;

			var ie = 0;
			for (j = 1; j < n; ++j)
			{
				if (ie == ih)
				{
					ie = j;
					continue;
				}

				var r = b2Vec2.Subtract(ps[ie], ps[hull[m]]);
				var v = b2Vec2.Subtract(ps[j], ps[hull[m]]);
				var c = b2Cross_v2_v2(r, v);
				if (c < 0.0)
				{
					ie = j;
				}

				// Collinearity check
				if (c == 0.0 && v.LengthSquared() > r.LengthSquared())
				{
					ie = j;
				}
			}

			++m;
			ih = ie;

			if (ie == i0)
			{
				break;
			}
		}

		this.m_count = m;

		// Copy vertices.
		for (i = 0; i < m; ++i)
		{
			this.m_vertices[i] = ps[hull[i]].Clone();
		}

		// Compute normals. Ensure the edges have non-zero length.
		for (i = 0; i < m; ++i)
		{
			var i1 = i;
			var i2 = i + 1 < m ? i + 1 : 0;
			var edge = b2Vec2.Subtract(this.m_vertices[i2], this.m_vertices[i1]);
			b2Assert(edge.LengthSquared() > b2_epsilon * b2_epsilon);
			this.m_normals[i] = b2Cross_v2_f(edge, 1.0).Clone();
			this.m_normals[i].Normalize();
		}

		// Compute the polygon centroid.
		this.m_centroid = b2PolygonShape.ComputeCentroid(this.m_vertices, m);
	},

	/// Build vertices to represent an axis-aligned box centered on the local origin.
	/// @param hx the half-width.
	/// @param hy the half-height.
	SetAsBox: function(hx, hy, center, angle)
	{
		this.m_count = 4;
		this.m_vertices[0] = new b2Vec2(-hx, -hy);
		this.m_vertices[1] = new b2Vec2( hx, -hy);
		this.m_vertices[2] = new b2Vec2( hx,  hy);
		this.m_vertices[3] = new b2Vec2(-hx,  hy);
		this.m_normals[0] = new b2Vec2(0.0, -1.0);
		this.m_normals[1] = new b2Vec2(1.0, 0.0);
		this.m_normals[2] = new b2Vec2(0.0, 1.0);
		this.m_normals[3] = new b2Vec2(-1.0, 0.0);

		if (!center)
			return;

		this.m_centroid.Assign(center);

		var xf = new b2Transform();
		xf.p = center;
		xf.q.Set(angle);

		// Transform vertices and normals.
		for (var i = 0; i < this.m_count; ++i)
		{
			this.m_vertices[i].Assign(b2Mul_t_v2(xf, this.m_vertices[i]));
			this.m_normals[i].Assign(b2Mul_r_v2(xf.q, this.m_normals[i]));
		}
	},

	/// @see b2Shape::TestPoint
	TestPoint: function(xf, p)
	{
		var pLocal = b2MulT_r_v2(xf.q, b2Vec2.Subtract(p, xf.p));

		for (var i = 0; i < this.m_count; ++i)
		{
			var dot = b2Dot_v2_v2(this.m_normals[i], b2Vec2.Subtract(pLocal, this.m_vertices[i]));
			if (dot > 0.0)
			{
				return false;
			}
		}

		return true;
	},

	/// Implement b2Shape.
	RayCast: function(output, input,
					xf, childIndex)
	{
		// Put the ray into the polygon's frame of reference.
		var p1 = b2MulT_r_v2(xf.q, b2Vec2.Subtract(input.p1, xf.p));
		var p2 = b2MulT_r_v2(xf.q, b2Vec2.Subtract(input.p2, xf.p));
		var d = b2Vec2.Subtract(p2, p1);

		var lower = 0.0, upper = input.maxFraction;

		var index = -1;

		for (var i = 0; i < this.m_count; ++i)
		{
			// p = p1 + a * d
			// dot(normal, p - v) = 0
			// dot(normal, p1 - v) + a * dot(normal, d) = 0
			var numerator = b2Dot_v2_v2(this.m_normals[i], b2Vec2.Subtract(this.m_vertices[i], p1));
			var denominator = b2Dot_v2_v2(this.m_normals[i], d);

			if (denominator == 0.0)
			{
				if (numerator < 0.0)
				{
					return false;
				}
			}
			else
			{
				// Note: we want this predicate without division:
				// lower < numerator / denominator, where denominator < 0
				// Since denominator < 0, we have to flip the inequality:
				// lower < numerator / denominator <==> denominator * lower > numerator.
				if (denominator < 0.0 && numerator < lower * denominator)
				{
					// Increase lower.
					// The segment enters this half-space.
					lower = numerator / denominator;
					index = i;
				}
				else if (denominator > 0.0 && numerator < upper * denominator)
				{
					// Decrease upper.
					// The segment exits this half-space.
					upper = numerator / denominator;
				}
			}

			// The use of epsilon here causes the assert on lower to trip
			// in some cases. Apparently the use of epsilon was to make edge
			// shapes work, but now those are handled separately.
			//if (upper < lower - b2_epsilon)
			if (upper < lower)
			{
				return false;
			}
		}

		b2Assert(0.0 <= lower && lower <= input.maxFraction);

		if (index >= 0)
		{
			output.fraction = lower;
			output.normal = b2Mul_r_v2(xf.q, this.m_normals[index]);
			return true;
		}

		return false;
	},

	/// @see b2Shape::ComputeAABB
	ComputeAABB: function(aabb, xf, childIndex)
	{
		var lower = b2Mul_t_v2(xf, this.m_vertices[0]);
		var upper = lower.Clone();

		for (var i = 1; i < this.m_count; ++i)
		{
			var v = b2Mul_t_v2(xf, this.m_vertices[i]);
			lower = b2Min_v2(lower, v);
			upper = b2Max_v2(upper, v);
		}

		var r = new b2Vec2(this.m_radius, this.m_radius);
		aabb.lowerBound = b2Vec2.Subtract(lower, r);
		aabb.upperBound = b2Vec2.Add(upper, r);
	},

	/// @see b2Shape::ComputeMass
	ComputeMass: function(massData, density)
	{
		// Polygon mass, centroid, and inertia.
		// Let rho be the polygon density in mass per unit area.
		// Then:
		// mass = rho * int(dA)
		// centroid.x = (1/mass) * rho * int(x * dA)
		// centroid.y = (1/mass) * rho * int(y * dA)
		// I = rho * int((x*x + y*y) * dA)
		//
		// We can compute these integrals by summing all the integrals
		// for each triangle of the polygon. To evaluate the integral
		// for a single triangle, we make a change of variables to
		// the (u,v) coordinates of the triangle:
		// x = x0 + e1x * u + e2x * v
		// y = y0 + e1y * u + e2y * v
		// where 0 <= u && 0 <= v && u + v <= 1.
		//
		// We integrate u from [0,1-v] and then v from [0,1].
		// We also need to use the Jacobian of the transformation:
		// D = cross(e1, e2)
		//
		// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
		//
		// The rest of the derivation is handled by computer algebra.

		b2Assert(this.m_count >= 3);

		var center = new b2Vec2(0.0, 0.0);
		var area = 0.0;
		var I = 0.0;

		// s is the reference point for forming triangles.
		// It's location doesn't change the result (except for rounding error).
		var s = new b2Vec2(0.0, 0.0);

		// This code would put the reference point inside the polygon.
		for (var i = 0; i < this.m_count; ++i)
		{
			s.Add(this.m_vertices[i]);
		}
		s.Multiply(1.0 / this.m_count);

		var k_inv3 = 1.0 / 3.0;

		for (var i = 0; i < this.m_count; ++i)
		{
			// Triangle vertices.
			var e1 = b2Vec2.Subtract(this.m_vertices[i], s);
			var e2 = i + 1 < this.m_count ? b2Vec2.Subtract(this.m_vertices[i + 1], s) : b2Vec2.Subtract(this.m_vertices[0], s);

			var D = b2Cross_v2_v2(e1, e2);

			var triangleArea = 0.5 * D;
			area += triangleArea;

			// Area weighted centroid
			center.Add(b2Vec2.Multiply(triangleArea * k_inv3, b2Vec2.Add(e1, e2)));

			var ex1 = e1.x, ey1 = e1.y;
			var ex2 = e2.x, ey2 = e2.y;

			var intx2 = ex1*ex1 + ex2*ex1 + ex2*ex2;
			var inty2 = ey1*ey1 + ey2*ey1 + ey2*ey2;

			I += (0.25 * k_inv3 * D) * (intx2 + inty2);
		}

		// Total mass
		massData.mass = density * area;

		// Center of mass
		b2Assert(area > b2_epsilon);
		center.Multiply(1.0 / area);
		massData.center = b2Vec2.Add(center, s);

		// Inertia tensor relative to the local origin (point s).
		massData.I = density * I;

		// Shift to center of mass then to original body origin.
		massData.I += massData.mass * (b2Dot_v2_v2(massData.center, massData.center) - b2Dot_v2_v2(center, center));
	},

	/// Get the vertex count.
	GetVertexCount: function() { return this.m_count; },

	/// Get a vertex by index.
	GetVertex: function(index)
	{
		b2Assert(0 <= index && index < this.m_count);
		return this.m_vertices[index];
	},

	/// Validate convexity. This is a very time consuming operation.
	/// @returns true if valid
	Validate: function()
	{
		for (var i = 0; i < this.m_count; ++i)
		{
			var i1 = i;
			var i2 = i < this.m_count - 1 ? i1 + 1 : 0;
			var p = this.m_vertices[i1];
			var e = b2Vec2.Subtract(this.m_vertices[i2], p);

			for (var j = 0; j < this.m_count; ++j)
			{
				if (j == i1 || j == i2)
				{
					continue;
				}

				var v = b2Vec2.Subtract(this.m_vertices[j], p);
				var c = b2Cross_v2_v2(e, v);
				if (c < 0.0)
				{
					return false;
				}
			}
		}

		return true;
	}
};

b2PolygonShape.ComputeCentroid = function(vs, count)
{
	b2Assert(count >= 3);

	var c = new b2Vec2();
	var area = 0.0;

	// pRef is the reference point for forming triangles.
	// It's location doesn't change the result (except for rounding error).
	var pRef = new b2Vec2(0.0, 0.0);
/*#if 0
	// This code would put the reference point inside the polygon.
	for (int32 i = 0; i < count; ++i)
	{
		pRef += vs[i];
	}
	pRef *= 1.0 / count;
#endif*/

	var inv3 = 1.0 / 3.0;

	for (var i = 0; i < count; ++i)
	{
		// Triangle vertices.
		var p1 = pRef;
		var p2 = vs[i];
		var p3 = i + 1 < count ? vs[i+1] : vs[0];

		var e1 = b2Vec2.Subtract(p2, p1);
		var e2 = b2Vec2.Subtract(p3, p1);

		var D = b2Cross_v2_v2(e1, e2);

		var triangleArea = 0.5 * D;
		area += triangleArea;

		// Area weighted centroid
		c.Add(b2Vec2.Multiply(triangleArea, b2Vec2.Multiply(inv3, b2Vec2.Add(b2Vec2.Add(p1, p2), p3))));
	}

	// Centroid
	b2Assert(area > b2_epsilon);
	c.Multiply(1.0 / area);
	return c;
};

b2PolygonShape._extend(b2Shape);