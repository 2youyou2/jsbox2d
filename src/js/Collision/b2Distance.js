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

/// A distance proxy is used by the GJK algorithm.
/// It encapsulates any shape.
function b2DistanceProxy()
{
	this.m_vertices = null;
	this.m_count = 0;
	this.m_radius = 0;
}

b2DistanceProxy.prototype =
{
	Assign: function(l)
	{
		this.m_vertices = l.m_vertices;
		this.m_count = l.m_count;
		this.m_radius = l.m_radius;
	},

	/// Initialize the proxy using the given shape. The shape
	/// must remain in scope while the proxy is in use.
	Set: function(shape, index)
	{
		switch (shape.GetType())
		{
		case b2Shape.e_circle:
			{
				var circle = shape;
				this.m_vertices = [ circle.m_p ];
				this.m_count = 1;
				this.m_radius = circle.m_radius;
			}
			break;

		case b2Shape.e_polygon:
			{
				var polygon = shape;

				this.m_vertices = polygon.m_vertices;
				this.m_count = polygon.m_count;
				this.m_radius = polygon.m_radius;
			}
			break;

		case b2Shape.e_chain:
			{
				var chain = shape;
'#if @DEBUG';
				b2Assert(0 <= index && index < chain.m_count);
'#endif';
				this.m_vertices = [ chain.m_vertices[index] ];
				if (index + 1 < chain.m_count)
				{
					this.m_vertices[1] = chain.m_vertices[index + 1];
				}
				else
				{
					this.m_vertices[1] = chain.m_vertices[0];
				}

				this.m_count = 2;
				this.m_radius = chain.m_radius;
			}
			break;

		case b2Shape.e_edge:
			{
				var edge = shape;
				this.m_vertices = [ edge.m_vertex1, edge.m_vertex2 ];
				this.m_count = 2;
				this.m_radius = edge.m_radius;
			}
			break;

'#if @DEBUG';
		default:
			b2Assert(false);
'#endif';
		}
	},

	/// Get the supporting vertex index in the given direction.
	GetSupport: function(dx, dy)
	{
		var bestIndex = 0;
		var bestValue = this.m_vertices[0].x * dx + this.m_vertices[0].y * dy;//b2Dot_v2_v2(this.m_vertices[0], d);
		for (var i = 1; i < this.m_count; ++i)
		{
			var value = this.m_vertices[i].x * dx + this.m_vertices[i].y * dy;//b2Dot_v2_v2(this.m_vertices[i], d);
			if (value > bestValue)
			{
				bestIndex = i;
				bestValue = value;
			}
		}

		return bestIndex;
	},

	/// Get the supporting vertex in the given direction.
	GetSupportVertex: function(dx, dy)
	{
		return this.m_vertices[this.GetSupport(dx, dy)];
	},

	/// Get the vertex count.
	GetVertexCount: function()
	{
		return this.m_count;
	},

	/// Get a vertex by index. Used by b2Distance.
	GetVertex: function(index)
	{
'#if @DEBUG';
		b2Assert(0 <= index && index < this.m_count);
'#endif';
		return this.m_vertices[index];
	}
};

/// Used to warm start b2Distance.
/// Set count to zero on first call.
function b2SimplexCache()
{
	this.metric = 0;		///< length or area
	this.count = 0;
	this.indexA = [0, 0, 0];	///< vertices on shape A
	this.indexB = [0, 0, 0];	///< vertices on shape B
};

/// Input for b2Distance.
/// You have to option to use the shape radii
/// in the computation. Even
function b2DistanceInput()
{
	this.proxyA = new b2DistanceProxy();
	this.proxyB = new b2DistanceProxy();
	this.transformA = new b2Transform();
	this.transformB = new b2Transform();
	this.useRadii = false;
};

/// Output for b2Distance.
function b2DistanceOutput()
{
	this.pointA = new b2Vec2();		///< closest point on shapeA
	this.pointB = new b2Vec2();		///< closest point on shapeB
	this.distance = 0;
	this.iterations = 0;	///< number of GJK iterations used
};

function b2SimplexVertex()
{
	this.wA = new b2Vec2();		// support point in proxyA
	this.wB = new b2Vec2();		// support point in proxyB
	this.w = new b2Vec2();		// wB - wA
	this.a = 0;		// barycentric coordinate for closest point
	this.indexA = 0;	// wA index
	this.indexB = 0;	// wB index
}

b2SimplexVertex.prototype =
{
	Assign: function(l)
	{
		this.wA.x = l.wA.x;//.Assign(l.wA);
		this.wA.y = l.wA.y;
		this.wB.x = l.wB.x;//.Assign(l.wB);
		this.wB.y = l.wB.y;
		this.w.x = l.w.x;//.Assign(l.w);
		this.w.y = l.w.y;
		this.a = l.a;
		this.indexA = l.indexA;
		this.indexB = l.indexB;
	}
};

function b2Simplex()
{
	this.m_v = [new b2SimplexVertex(), new b2SimplexVertex(), new b2SimplexVertex()];
	this.m_count = 0;
}

b2Simplex.prototype =
{
	ReadCache: function(cache,
					proxyA, transformA,
					proxyB, transformB)
	{
'#if @DEBUG';
		b2Assert(cache.count <= 3);
'#endif';

		// Copy data from cache.
		this.m_count = cache.count;
		var vertices = this.m_v;
		for (var i = 0; i < this.m_count; ++i)
		{
			var v = vertices[i];
			v.indexA = cache.indexA[i];
			v.indexB = cache.indexB[i];
			var wALocal = proxyA.GetVertex(v.indexA);
			var wBLocal = proxyB.GetVertex(v.indexB);
			//v.wA.Assign(b2Mul_t_v2(transformA, wALocal));
			v.wA.x = (transformA.q.c * wALocal.x - transformA.q.s * wALocal.y) + transformA.p.x;
			v.wA.y = (transformA.q.s * wALocal.x + transformA.q.c * wALocal.y) + transformA.p.y;
			//v.wB.Assign(b2Mul_t_v2(transformB, wBLocal));
			v.wB.x = (transformB.q.c * wBLocal.x - transformB.q.s * wBLocal.y) + transformB.p.x;
			v.wB.y = (transformB.q.s * wBLocal.x + transformB.q.c * wBLocal.y) + transformB.p.y;
			//v.w.Assign(b2Vec2.Subtract(v.wB, v.wA));
			v.w.x = v.wB.x - v.wA.x;
			v.w.y = v.wB.y - v.wA.y;
			v.a = 0.0;
		}

		// Compute the new simplex metric, if it is substantially different than
		// old metric then flush the simplex.
		if (this.m_count > 1)
		{
			var metric1 = cache.metric;
			var metric2 = this.GetMetric();
			if (metric2 < 0.5 * metric1 || 2.0 * metric1 < metric2 || metric2 < b2_epsilon)
			{
				// Reset the simplex.
				this.m_count = 0;
			}
		}

		// If the cache is empty or invalid ...
		if (this.m_count == 0)
		{
			var v = vertices[0];
			v.indexA = 0;
			v.indexB = 0;
			var wALocal = proxyA.GetVertex(0);
			var wBLocal = proxyB.GetVertex(0);
			//v.wA.Assign(b2Mul_t_v2(transformA, wALocal));
			v.wA.x = (transformA.q.c * wALocal.x - transformA.q.s * wALocal.y) + transformA.p.x;
			v.wA.y = (transformA.q.s * wALocal.x + transformA.q.c * wALocal.y) + transformA.p.y;
			//v.wB.Assign(b2Mul_t_v2(transformB, wBLocal));
			v.wB.x = (transformB.q.c * wBLocal.x - transformB.q.s * wBLocal.y) + transformB.p.x;
			v.wB.y = (transformB.q.s * wBLocal.x + transformB.q.c * wBLocal.y) + transformB.p.y;
			//v.w.Assign(b2Vec2.Subtract(v.wB, v.wA));
			v.w.x = v.wB.x - v.wA.x;
			v.w.y = v.wB.y - v.wA.y;
			v.a = 1.0;
			this.m_count = 1;
		}
	},

	WriteCache: function(cache)
	{
		cache.metric = this.GetMetric();
		cache.count = this.m_count;
		var vertices = this.m_v;
		for (var i = 0; i < this.m_count; ++i)
		{
			cache.indexA[i] = vertices[i].indexA;
			cache.indexB[i] = vertices[i].indexB;
		}
	},

	GetSearchDirection: function(p)
	{
		switch (this.m_count)
		{
		case 1:
			//return this.m_v[0].w.Negate();
			p.x = -this.m_v[0].w.x;
			p.y = -this.m_v[0].w.y;
			break;

		case 2:
			{
				var e12x = this.m_v[1].w.x - this.m_v[0].w.x;//b2Vec2.Subtract(this.m_v[1].w, this.m_v[0].w);
				var e12y = this.m_v[1].w.y - this.m_v[0].w.y;
				var sgn = e12x * -this.m_v[0].w.y - e12y * -this.m_v[0].w.x;//b2Cross_v2_v2(e12, this.m_v[0].w.Negate());
				if (sgn > 0.0)
				{
					// Origin is left of e12.
					p.x = -1.0 * e12y; p.y = 1.0 * e12x;//b2Cross_f_v2(1.0, e12);
				}
				else
				{
					// Origin is right of e12.
					p.x = 1.0 * e12y; p.y = -1.0 * e12x;//b2Cross_v2_f(e12, 1.0);
				}
			}
			break;

'#if @DEBUG';
		default:
			b2Assert(false);
			p.x = p.y = 0;
			break;
'#endif';
		}
	},

	GetClosestPoint: function(p)
	{
		switch (this.m_count)
		{
		case 1:
			//return this.m_v[0].w;
			p.x = this.m_v[0].w.x;
			p.y = this.m_v[0].w.y;
			break;

		case 2:
			//return b2Vec2.Add(b2Vec2.Multiply(this.m_v[0].a, this.m_v[0].w), b2Vec2.Multiply(this.m_v[1].a, this.m_v[1].w));
			p.x = (this.m_v[0].a * this.m_v[0].w.x) + (this.m_v[1].a *this.m_v[1].w.x);
			p.y = (this.m_v[0].a * this.m_v[0].w.y) + (this.m_v[1].a *this.m_v[1].w.y);
			break;

		case 3:
			p.x = p.y = 0;
			break;

'#if @DEBUG';
		default:
			b2Assert(false);
			p.x = p.y = 0;
			break;
'#endif';
		}
	},

	GetWitnessPoints: function(pA, pB)
	{
		switch (this.m_count)
		{
		case 1:
			pA.x = this.m_v[0].wA.x;//.Assign(this.m_v[0].wA);
			pA.y = this.m_v[0].wA.y;
			pB.x = this.m_v[0].wB.x;//.Assign(this.m_v[0].wB);
			pB.y = this.m_v[0].wB.y;
			break;

		case 2:
			//pA.Assign(b2Vec2.Add(b2Vec2.Multiply(this.m_v[0].a, this.m_v[0].wA), b2Vec2.Multiply(this.m_v[1].a, this.m_v[1].wA)));
			pA.x = (this.m_v[0].a * this.m_v[0].wA.x) + (this.m_v[1].a * this.m_v[1].wA.x);
			pA.y = (this.m_v[0].a * this.m_v[0].wA.y) + (this.m_v[1].a * this.m_v[1].wA.y);
			//pB.Assign(b2Vec2.Add(b2Vec2.Multiply(this.m_v[0].a, this.m_v[0].wB), b2Vec2.Multiply(this.m_v[1].a, this.m_v[1].wB)));
			pB.x = (this.m_v[0].a * this.m_v[0].wB.x) + (this.m_v[1].a * this.m_v[1].wB.x);
			pB.y = (this.m_v[0].a * this.m_v[0].wB.y) + (this.m_v[1].a * this.m_v[1].wB.y);
			break;

		case 3:
			//pA.Assign(b2Vec2.Add(b2Vec2.Add(b2Vec2.Multiply(this.m_v[0].a, this.m_v[0].wA), b2Vec2.Multiply(this.m_v[1].a, this.m_v[1].wA)), b2Vec2.Multiply(this.m_v[2].a, this.m_v[2].wA)));
			pA.x = (this.m_v[0].a * this.m_v[0].wA.x) + (this.m_v[1].a * this.m_v[1].wA.x) + (this.m_v[2].a * this.m_v[2].wA.x);
			pA.y = (this.m_v[0].a * this.m_v[0].wA.y) + (this.m_v[1].a * this.m_v[1].wA.y) + (this.m_v[2].a * this.m_v[2].wA.y);
			pB.x = pA.x;//.Assign(pA);
			pB.y = pA.y;
			break;

'#if @DEBUG';
		default:
			b2Assert(false);
			break;
'#endif';
		}
	},

	GetMetric: function()
	{
		switch (this.m_count)
		{
		case 1:
			return 0.0;

		case 2:
			return b2Distance(this.m_v[0].w, this.m_v[1].w);

		case 3:
			//return b2Cross_v2_v2(b2Vec2.Subtract(this.m_v[1].w, this.m_v[0].w), b2Vec2.Subtract(this.m_v[2].w, this.m_v[0].w));
			return (this.m_v[1].w.x - this.m_v[0].w.x) * (this.m_v[2].w.y - this.m_v[0].w.y) - (this.m_v[1].w.y - this.m_v[0].w.y) * (this.m_v[2].w.x - this.m_v[0].w.x);

'#if @DEBUG';
		default:
			b2Assert(false);
			return 0.0;
'#endif';
		}
	},

	// Solve a line segment using barycentric coordinates.
	//
	// p = a1 * w1 + a2 * w2
	// a1 + a2 = 1
	//
	// The vector from the origin to the closest point on the line is
	// perpendicular to the line.
	// e12 = w2 - w1
	// dot(p, e) = 0
	// a1 * dot(w1, e) + a2 * dot(w2, e) = 0
	//
	// 2-by-2 linear system
	// [1      1     ][a1] = [1]
	// [w1.e12 w2.e12][a2] = [0]
	//
	// Define
	// d12_1 =  dot(w2, e12)
	// d12_2 = -dot(w1, e12)
	// d12 = d12_1 + d12_2
	//
	// Solution
	// a1 = d12_1 / d12
	// a2 = d12_2 / d12
	Solve2: function()
	{
		var w1 = this.m_v[0].w;
		var w2 = this.m_v[1].w;
		//var e12 = b2Vec2.Subtract(w2, w1);
		var e12x = w2.x - w1.x;
		var e12y = w2.y - w1.y;

		// w1 region
		var d12_2 = -(w1.x * e12x + w1.y * e12y);//-b2Dot_v2_v2(w1, e12);
		if (d12_2 <= 0.0)
		{
			// a2 <= 0, so we clamp it to 0
			this.m_v[0].a = 1.0;
			this.m_count = 1;
			return;
		}

		// w2 region
		var d12_1 = w2.x * e12x + w2.y * e12y;//b2Dot_v2_v2(w2, e12);
		if (d12_1 <= 0.0)
		{
			// a1 <= 0, so we clamp it to 0
			this.m_v[1].a = 1.0;
			this.m_count = 1;
			this.m_v[0].Assign(this.m_v[1]);
			return;
		}

		// Must be in e12 region.
		var inv_d12 = 1.0 / (d12_1 + d12_2);
		this.m_v[0].a = d12_1 * inv_d12;
		this.m_v[1].a = d12_2 * inv_d12;
		this.m_count = 2;
	},

	// Possible regions:
	// - points[2]
	// - edge points[0]-points[2]
	// - edge points[1]-points[2]
	// - inside the triangle
	Solve3: function()
	{
		var w1 = this.m_v[0].w;
		var w2 = this.m_v[1].w;
		var w3 = this.m_v[2].w;

		// Edge12
		// [1      1     ][a1] = [1]
		// [w1.e12 w2.e12][a2] = [0]
		// a3 = 0
		//var e12 = b2Vec2.Subtract(w2, w1);
		var e12x = w2.x - w1.x;
		var e12y = w2.y - w1.y;
		var w1e12 = w1.x * e12x + w1.y * e12y;//b2Dot_v2_v2(w1, e12);
		var w2e12 = w2.x * e12x + w2.y * e12y;//b2Dot_v2_v2(w2, e12);
		var d12_1 = w2e12;
		var d12_2 = -w1e12;

		// Edge13
		// [1      1     ][a1] = [1]
		// [w1.e13 w3.e13][a3] = [0]
		// a2 = 0
		//var e13 = b2Vec2.Subtract(w3, w1);
		var e13x = w3.x - w1.x;
		var e13y = w3.y - w1.y;
		var w1e13 = w1.x * e13x + w1.y * e13y;//b2Dot_v2_v2(w1, e13);
		var w3e13 = w3.x * e13x + w3.y * e13y;//b2Dot_v2_v2(w3, e13);
		var d13_1 = w3e13;
		var d13_2 = -w1e13;

		// Edge23
		// [1      1     ][a2] = [1]
		// [w2.e23 w3.e23][a3] = [0]
		// a1 = 0
		//var e23 = b2Vec2.Subtract(w3, w2);
		var e23x = w3.x - w2.x;
		var e23y = w3.y - w2.y;
		var w2e23 = w2.x * e23x + w2.y * e23y;//b2Dot_v2_v2(w2, e23);
		var w3e23 = w3.x * e23x + w3.y * e23y;//b2Dot_v2_v2(w3, e23);
		var d23_1 = w3e23;
		var d23_2 = -w2e23;

		// Triangle123
		var n123 = e12x * e13y - e12y * e13x;//b2Cross_v2_v2(e12, e13);

		var d123_1 = n123 * (w2.x * w3.y - w2.y * w3.x);//b2Cross_v2_v2(w2, w3);
		var d123_2 = n123 * (w3.x * w1.y - w3.y * w1.x);//b2Cross_v2_v2(w3, w1);
		var d123_3 = n123 * (w1.x * w2.y - w1.y * w2.x);//b2Cross_v2_v2(w1, w2);

		// w1 region
		if (d12_2 <= 0.0 && d13_2 <= 0.0)
		{
			this.m_v[0].a = 1.0;
			this.m_count = 1;
			return;
		}

		// e12
		if (d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0)
		{
			var inv_d12 = 1.0 / (d12_1 + d12_2);
			this.m_v[0].a = d12_1 * inv_d12;
			this.m_v[1].a = d12_2 * inv_d12;
			this.m_count = 2;
			return;
		}

		// e13
		if (d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0)
		{
			var inv_d13 = 1.0 / (d13_1 + d13_2);
			this.m_v[0].a = d13_1 * inv_d13;
			this.m_v[2].a = d13_2 * inv_d13;
			this.m_count = 2;
			this.m_v[1].Assign(this.m_v[2]);
			return;
		}

		// w2 region
		if (d12_1 <= 0.0 && d23_2 <= 0.0)
		{
			this.m_v[1].a = 1.0;
			this.m_count = 1;
			this.m_v[0].Assign(this.m_v[1]);
			return;
		}

		// w3 region
		if (d13_1 <= 0.0 && d23_1 <= 0.0)
		{
			this.m_v[2].a = 1.0;
			this.m_count = 1;
			this.m_v[0].Assign(this.m_v[2]);
			return;
		}

		// e23
		if (d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0)
		{
			var inv_d23 = 1.0 / (d23_1 + d23_2);
			this.m_v[1].a = d23_1 * inv_d23;
			this.m_v[2].a = d23_2 * inv_d23;
			this.m_count = 2;
			this.m_v[0].Assign(this.m_v[2]);
			return;
		}

		// Must be in triangle123
		var inv_d123 = 1.0 / (d123_1 + d123_2 + d123_3);
		this.m_v[0].a = d123_1 * inv_d123;
		this.m_v[1].a = d123_2 * inv_d123;
		this.m_v[2].a = d123_3 * inv_d123;
		this.m_count = 3;
	}
};

/// Compute the closest points between two shapes. Supports any combination of:
/// b2CircleShape, b2PolygonShape, b2EdgeShape. The simplex cache is input/output.
/// On the first call set b2SimplexCache.count to zero.
var _b2Distance_simplex = new b2Simplex();
var _b2Distance_normal = new b2Vec2();
var _b2Distance_p = new b2Vec2();
function b2DistanceFunc(output,
				cache,
				input)
{
	++b2DistanceFunc.b2_gjkCalls;

	var proxyA = input.proxyA;
	var proxyB = input.proxyB;

	var transformA = input.transformA;
	var transformB = input.transformB;

	// Initialize the simplex.
	_b2Distance_simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB);

	// Get simplex vertices as an array.
	var vertices = _b2Distance_simplex.m_v;
	var k_maxIters = 20;

	// These store the vertices of the last simplex so that we
	// can check for duplicates and prevent cycling.
	var saveA = [0, 0, 0], saveB = [0, 0, 0];
	var saveCount = 0;

	var distanceSqr1 = b2_maxFloat;
	var distanceSqr2 = distanceSqr1;

	// Main iteration loop.
	var iter = 0;
	while (iter < k_maxIters)
	{
		// Copy simplex so we can identify duplicates.
		saveCount = _b2Distance_simplex.m_count;
		for (var i = 0; i < saveCount; ++i)
		{
			saveA[i] = vertices[i].indexA;
			saveB[i] = vertices[i].indexB;
		}

		switch (_b2Distance_simplex.m_count)
		{
		case 1:
			break;

		case 2:
			_b2Distance_simplex.Solve2();
			break;

		case 3:
			_b2Distance_simplex.Solve3();
			break;

'#if @DEBUG';
		default:
			b2Assert(false);
'#endif';
		}

		// If we have 3 points, then the origin is in the corresponding triangle.
		if (_b2Distance_simplex.m_count == 3)
		{
			break;
		}

		// Compute closest point.
		_b2Distance_simplex.GetClosestPoint(_b2Distance_p);
		distanceSqr2 = _b2Distance_p.LengthSquared();

		// Ensure progress
		if (distanceSqr2 >= distanceSqr1)
		{
			//break;
		}
		distanceSqr1 = distanceSqr2;

		// Get search direction.
		_b2Distance_simplex.GetSearchDirection(_b2Distance_p);

		// Ensure the search direction is numerically fit.
		if (_b2Distance_p.LengthSquared() < b2_epsilon * b2_epsilon)
		{
			// The origin is probably contained by a line segment
			// or triangle. Thus the shapes are overlapped.

			// We can't return zero here even though there may be overlap.
			// In case the simplex is a point, segment, or triangle it is difficult
			// to determine if the origin is contained in the CSO or very close to it.
			break;
		}

		// Compute a tentative new simplex vertex using support points.
		var vertex = vertices[_b2Distance_simplex.m_count];
		vertex.indexA = proxyA.GetSupport(transformA.q.c * -_b2Distance_p.x + transformA.q.s * -_b2Distance_p.y, -transformA.q.s * -_b2Distance_p.x + transformA.q.c * -_b2Distance_p.y);//b2MulT_r_v2(transformA.q, d.Negate()));
		//vertex.wA.Assign(b2Mul_t_v2(transformA, proxyA.GetVertex(vertex.indexA)));
		var pva = proxyA.GetVertex(vertex.indexA);
		vertex.wA.x = (transformA.q.c * pva.x - transformA.q.s * pva.y) + transformA.p.x;
		vertex.wA.y = (transformA.q.s * pva.x + transformA.q.c * pva.y) + transformA.p.y;

		vertex.indexB = proxyB.GetSupport(transformB.q.c * _b2Distance_p.x + transformB.q.s * _b2Distance_p.y, -transformB.q.s * _b2Distance_p.x + transformB.q.c * _b2Distance_p.y);//b2MulT_r_v2(transformB.q, d));
		//vertex.wB.Assign(b2Mul_t_v2(transformB, proxyB.GetVertex(vertex.indexB)));
		var pvb = proxyB.GetVertex(vertex.indexB);
		vertex.wB.x = (transformB.q.c * pvb.x - transformB.q.s * pvb.y) + transformB.p.x;
		vertex.wB.y = (transformB.q.s * pvb.x + transformB.q.c * pvb.y) + transformB.p.y;
		vertex.w.x = vertex.wB.x - vertex.wA.x;//b2Vec2.Subtract(vertex.wB, vertex.wA));
		vertex.w.y = vertex.wB.y - vertex.wA.y;

		// Iteration count is equated to the number of support point calls.
		++iter;
		++b2DistanceFunc.b2_gjkIters;

		// Check for duplicate support points. This is the main termination criteria.
		var duplicate = false;
		for (var i = 0; i < saveCount; ++i)
		{
			if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i])
			{
				duplicate = true;
				break;
			}
		}

		// If we found a duplicate support point we must exit to avoid cycling.
		if (duplicate)
		{
			break;
		}

		// New vertex is ok and needed.
		++_b2Distance_simplex.m_count;
	}

	b2DistanceFunc.b2_gjkMaxIters = b2Max(b2DistanceFunc.b2_gjkMaxIters, iter);

	// Prepare output.
	_b2Distance_simplex.GetWitnessPoints(output.pointA, output.pointB);
	output.distance = b2Distance(output.pointA, output.pointB);
	output.iterations = iter;

	// Cache the simplex.
	_b2Distance_simplex.WriteCache(cache);

	// Apply radii if requested.
	if (input.useRadii)
	{
		var rA = proxyA.m_radius;
		var rB = proxyB.m_radius;

		if (output.distance > rA + rB && output.distance > b2_epsilon)
		{
			// Shapes are still no overlapped.
			// Move the witness points to the outer surface.
			output.distance -= rA + rB;
			//var normal = b2Vec2.Subtract(output.pointB, output.pointA);
			_b2Distance_normal.x = output.pointB.x - output.pointA.x;
			_b2Distance_normal.y = output.pointB.y - output.pointA.y;
			_b2Distance_normal.Normalize();
			output.pointA.x += (rA * _b2Distance_normal.x);//.Add(b2Vec2.Multiply(rA, normal));
			output.pointA.y += (rA * _b2Distance_normal.y);
			output.pointB.x -= (rB * _b2Distance_normal.x);//.Subtract(b2Vec2.Multiply(rB, normal));
			output.pointB.y -= (rB * _b2Distance_normal.y);
		}
		else
		{
			// Shapes are overlapped when radii are considered.
			// Move the witness points to the middle.
			//var p = b2Vec2.Multiply(0.5, b2Vec2.Add(output.pointA, output.pointB));
			var px = (0.5 * (output.pointA.x + output.pointB.x));
			var py = (0.5 * (output.pointA.y + output.pointB.y));
			output.pointA.x = px;//.Assign(p);
			output.pointA.y = py;
			output.pointB.x = px;//.Assign(p);
			output.pointB.y = py;
			output.distance = 0.0;
		}
	}
}

b2DistanceFunc.b2_gjkCalls = 0;
b2DistanceFunc.b2_gjkIters = 0;
b2DistanceFunc.b2_gjkMaxIters = 0;