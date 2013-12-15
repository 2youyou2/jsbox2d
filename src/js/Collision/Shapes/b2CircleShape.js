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

/// A circle shape.
function b2CircleShape()
{
	this.parent.call(this);
	
	this.m_type = b2Shape.e_circle;
	this.m_radius = 0;
	this.m_p = new b2Vec2();

	Object.seal(this);
}

b2CircleShape.prototype =
{
	/// Implement b2Shape.
	Clone: function()
	{
		var shape = new b2CircleShape();
		shape.m_radius = this.m_radius;
		shape.m_p = this.m_p.Clone();
		return shape;
	},

	/// @see b2Shape::GetChildCount
	GetChildCount: function() { return 1; },

	/// Implement b2Shape.
	TestPoint: function(transform, p)
	{
		var center = b2Vec2.Add(transform.p, b2Mul_r_v2(transform.q, this.m_p));
		var d = b2Vec2.Subtract(p, center);
		return b2Dot_v2_v2(d, d) <= this.m_radius * this.m_radius;
	},

	/// Implement b2Shape.
	RayCast: function(output, input,
				transform, childIndex)
	{
		var position = b2Vec2.Add(transform.p, b2Mul_r_v2(transform.q, this.m_p));
		var s = b2Vec2.Subtract(input.p1, position);
		var b = b2Dot_v2_v2(s, s) - this.m_radius * this.m_radius;

		// Solve quadratic equation.
		var r = b2Vec2.Subtract(input.p2, input.p1);
		var c = b2Dot_v2_v2(s, r);
		var rr = b2Dot_v2_v2(r, r);
		var sigma = c * c - rr * b;

		// Check for negative discriminant and short segment.
		if (sigma < 0.0 || rr < b2_epsilon)
		{
			return false;
		}

		// Find the point of intersection of the line with the circle.
		var a = -(c + b2Sqrt(sigma));

		// Is the intersection point on the segment?
		if (0.0 <= a && a <= input.maxFraction * rr)
		{
			a /= rr;
			output.fraction = a;
			output.normal = b2Vec2.Add(s, b2Vec2.Multiply(a, r));
			output.normal.Normalize();
			return true;
		}

		return false;
	},

	/// @see b2Shape::ComputeAABB
	ComputeAABB: function(aabb, transform, childIndex)
	{
		var p = b2Vec2.Add(transform.p, b2Mul_r_v2(transform.q, this.m_p));
		aabb.lowerBound.Set(p.x - this.m_radius, p.y - this.m_radius);
		aabb.upperBound.Set(p.x + this.m_radius, p.y + this.m_radius);
	},

	/// @see b2Shape::ComputeMass
	ComputeMass: function(massData, density)
	{
		massData.mass = density * b2_pi * this.m_radius * this.m_radius;
		massData.center = this.m_p;

		// inertia about the local origin
		massData.I = massData.mass * (0.5 * this.m_radius * this.m_radius + b2Dot_v2_v2(this.m_p, this.m_p));
	},

	/// Get the supporting vertex index in the given direction.
	GetSupport: function(d) { return 0; },

	/// Get the supporting vertex in the given direction.
	GetSupportVertex: function(d) { return this.m_p; },

	/// Get the vertex count.
	GetVertexCount: function() { return 1; },

	/// Get a vertex by index. Used by b2Distance.
	GetVertex: function(index)
	{
		b2Assert(index == 0);
		return this.m_p;
	},

	_serialize: function(out)
	{
		var obj = out || {};

		this.parent.prototype._serialize.call(this, obj);

		obj['m_p'] = this.m_p._serialize();

		return obj;
	},

	_deserialize: function(data)
	{
		this.parent.prototype._deserialize.call(this, data);

		this.m_p._deserialize(data['m_p']);
	}
};

b2CircleShape._extend(b2Shape);