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

/// A line segment (edge) shape. These can be connected in chains or loops
/// to other edge shapes. The connectivity information is used to ensure
/// correct contact normals.
function b2EdgeShape()
{
	this.parent.call(this);

	this.m_type = b2Shape.e_edge;
	this.m_radius = b2_polygonRadius;
	this.m_vertex0 = new b2Vec2();
	this.m_vertex1 = new b2Vec2();
	this.m_vertex2 = new b2Vec2();
	this.m_vertex3 = new b2Vec2();
	this.m_hasVertex0 = false;
	this.m_hasVertex3 = false;

	Object.seal(this);
}

b2EdgeShape.prototype =
{
	/// Set this as an isolated edge.
	Set: function(v1, v2)
	{
		this.m_vertex1.Assign(v1);
		this.m_vertex2.Assign(v2);
		this.m_hasVertex0 = false;
		this.m_hasVertex3 = false;
	},

	/// Implement b2Shape.
	Clone: function()
	{
		var shape = new b2EdgeShape();
		shape.m_vertex0 = this.m_vertex0.Clone();
		shape.m_vertex1 = this.m_vertex1.Clone();
		shape.m_vertex2 = this.m_vertex2.Clone();
		shape.m_vertex3 = this.m_vertex3.Clone();
		shape.m_hasVertex0 = this.m_hasVertex0;
		shape.m_hasVertex3 = this.m_hasVertex3;
		return shape;
	},

	/// @see b2Shape::GetChildCount
	GetChildCount: function()
	{
		return 1;
	},

	/// @see b2Shape::TestPoint
	TestPoint: function(transform, p)
	{
		return false;
	},

	/// Implement b2Shape.
	RayCast: function(output, input,
				xf, childIndex)
	{
		// Put the ray into the edge's frame of reference.
		var p1 = b2MulT_r_v2(xf.q, b2Vec2.Subtract(input.p1, xf.p));
		var p2 = b2MulT_r_v2(xf.q, b2Vec2.Subtract(input.p2, xf.p));
		var d = b2Vec2.Subtract(p2, p1);

		var v1 = this.m_vertex1;
		var v2 = this.m_vertex2;
		var e = b2Vec2.Subtract(v2, v1);
		var normal = new b2Vec2(e.y, -e.x);
		normal.Normalize();

		// q = p1 + t * d
		// dot(normal, q - v1) = 0
		// dot(normal, p1 - v1) + t * dot(normal, d) = 0
		var numerator = b2Dot_v2_v2(normal, b2Vec2.Subtract(v1, p1));
		var denominator = b2Dot_v2_v2(normal, d);

		if (denominator == 0.0)
		{
			return false;
		}

		var t = numerator / denominator;
		if (t < 0.0 || input.maxFraction < t)
		{
			return false;
		}

		var q = b2Vec2.Add(p1, b2Vec2.Multiply(t, d));

		// q = v1 + s * r
		// s = dot(q - v1, r) / dot(r, r)
		var r = b2Vec2.Subtract(v2, v1);
		var rr = b2Dot_v2_v2(r, r);
		if (rr == 0.0)
		{
			return false;
		}

		var s = b2Dot_v2_v2(b2Vec2.Subtract(q, v1), r) / rr;
		if (s < 0.0 || 1.0 < s)
		{
			return false;
		}

		output.fraction = t;
		if (numerator > 0.0)
		{
			output.normal = b2Mul_r_v2(xf.q, normal).Negate();
		}
		else
		{
			output.normal = b2Mul_r_v2(xf.q, normal);
		}
		return true;
	},

	/// @see b2Shape::ComputeAABB
	ComputeAABB: function(aabb, xf, childIndex)
	{
		var v1x = (xf.q.c * this.m_vertex1.x - xf.q.s * this.m_vertex1.y) + xf.p.x;//b2Mul_t_v2(xf, this.m_vertex1);
		var v1y = (xf.q.s * this.m_vertex1.x + xf.q.c * this.m_vertex1.y) + xf.p.y;
		var v2x = (xf.q.c * this.m_vertex2.x - xf.q.s * this.m_vertex2.y) + xf.p.x;//b2Mul_t_v2(xf, this.m_vertex2);
		var v2y = (xf.q.s * this.m_vertex2.x + xf.q.c * this.m_vertex2.y) + xf.p.y;

		var lowerx = b2Min(v1x, v2x);// = b2Min_v2(v1, v2);
		var lowery = b2Min(v1y, v2y);
		var upperx = b2Max(v1x, v2x); //= b2Max_v2(v1, v2);
		var uppery = b2Max(v1y, v2y);

		//var r = new b2Vec2(this.m_radius, this.m_radius);
		aabb.lowerBound.x = lowerx - this.m_radius; //= b2Vec2.Subtract(lower, r);
		aabb.lowerBound.y = lowery - this.m_radius;
		aabb.upperBound.x = upperx + this.m_radius; //= b2Vec2.Add(upper, r);
		aabb.upperBound.y = uppery + this.m_radius;
	},

	/// @see b2Shape::ComputeMass
	ComputeMass: function(massData, density)
	{
		massData.mass = 0.0;
		massData.center = b2Vec2.Multiply(0.5, b2Vec2.Add(this.m_vertex1, this.m_vertex2));
		massData.I = 0.0;
	},

//'#if @LIQUIDFUN';
	ComputeDistance: function(xf, p, distance, normal, childIndex)
	{
		var v1 = b2Mul_t_v2(xf, this.m_vertex1);
		var v2 = b2Mul_t_v2(xf, this.m_vertex2);

		var d = b2Vec2.Subtract(p, v1);
		var s = b2Vec2.Subtract(v2, v1);
		var ds = b2Dot_v2_v2(d, s);
		if (ds > 0)
		{
			var s2 = b2Dot_v2_v2(s, s);
			if (ds > s2)
			{
				d.Assign(b2Vec2.Subtract(p, v2));
			}
			else
			{
				d.Subtract(b2Vec2.Multiply(ds / s2, s));
			}
		}

		var d1 = d.Length();
		distance[0] = d1;
		normal.Assign(d1 > 0 ? b2Vec2.Multiply(1 / d1, d) : new b2Vec2(0, 0));
	},
//'#endif';

	_serialize: function(out)
	{
		var obj = out || {};

		this.parent.prototype._serialize.call(this, obj);

		obj['m_vertex1'] = this.m_vertex1._serialize();
		obj['m_vertex2'] = this.m_vertex2._serialize();

		obj['m_hasVertex0'] = this.m_hasVertex0;

		if (this.m_hasVertex0)
			obj['m_vertex0'] = this.m_vertex0._serialize();

		obj['m_hasVertex3'] = this.m_hasVertex3;

		if (this.m_hasVertex3)
			obj['m_vertex3'] = this.m_vertex3._serialize();

		return obj;
	},

	_deserialize: function(data)
	{
		this.parent.prototype._deserialize.call(this, data);

		this.m_vertex1._deserialize(data['m_vertex1']);
		this.m_vertex2._deserialize(data['m_vertex2']);

		this.m_hasVertex0 = data['m_hasVertex0'];

		if (this.m_hasVertex0)
			this.m_vertex0._deserialize(data['m_vertex0']);

		this.m_hasVertex3 = data['m_hasVertex3'];

		if (this.m_hasVertex3)
			this.m_vertex3._deserialize(data['m_vertex3']);
	}
};

b2EdgeShape._extend(b2Shape);