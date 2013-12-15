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

"use strict";

/// A chain shape is a free form sequence of line segments.
/// The chain has two-sided collision, so you can use inside and outside collision.
/// Therefore, you may use any winding order.
/// Since there may be many vertices, they are allocated using b2Alloc.
/// Connectivity information is used to create smooth collisions.
/// WARNING: The chain will not collide properly if there are self-intersections.
function b2ChainShape()
{
	this.parent.call(this);

	this.m_type = b2Shape.e_chain;
	this.m_radius = b2_polygonRadius;
	this.m_vertices = null;
	this.m_count = 0;
	this.m_prevVertex = new b2Vec2();
	this.m_nextVertex = new b2Vec2();
	this.m_hasPrevVertex = false;
	this.m_hasNextVertex = false;

	Object.seal(this);
}

b2ChainShape.prototype =
{
	/// Create a loop. This automatically adjusts connectivity.
	/// @param vertices an array of vertices, these are copied
	/// @param count the vertex count
	CreateLoop: function(vertices, count)
	{
		b2Assert(this.m_vertices == null && this.m_count == 0);
		b2Assert(count >= 3);

		for (var i = 1; i < count; ++i)
		{
			var v1 = vertices[i-1];
			var v2 = vertices[i];

			// If the code crashes here, it means your vertices are too close together.
			b2Assert(b2DistanceSquared(v1, v2) > b2_linearSlop * b2_linearSlop);
		}

		this.m_count = count + 1;

		this.m_vertices = new Array(this.m_count);

		for (var i = 0; i < count; ++i)
			this.m_vertices[i] = vertices[i].Clone();

		this.m_vertices[count] = this.m_vertices[0].Clone();
		this.m_prevVertex.Assign(this.m_vertices[this.m_count - 2]);
		this.m_nextVertex.Assign(this.m_vertices[1]);
		this.m_hasPrevVertex = true;
		this.m_hasNextVertex = true;
	},

	/// Create a chain with isolated end vertices.
	/// @param vertices an array of vertices, these are copied
	/// @param count the vertex count
	CreateChain: function(vertices, count)
	{
		b2Assert(this.m_vertices == null && this.m_count == 0);
		b2Assert(count >= 2);

		for (var i = 1; i < count; ++i)
		{
			var v1 = vertices[i-1];
			var v2 = vertices[i];

			// If the code crashes here, it means your vertices are too close together.
			b2Assert(b2DistanceSquared(v1, v2) > b2_linearSlop * b2_linearSlop);
		}

		this.m_count = count;
		this.m_vertices = new Array(count);

		for (var i = 0; i < count; ++i)
			this.m_vertices[i] = vertices[i].Clone();

		this.m_hasPrevVertex = false;
		this.m_hasNextVertex = false;

		this.m_prevVertex.SetZero();
		this.m_nextVertex.SetZero();
	},

	/// Establish connectivity to a vertex that precedes the first vertex.
	/// Don't call this for loops.
	SetPrevVertex: function(prevVertex)
	{
		this.m_prevVertex.Assign(prevVertex);
		this.m_hasPrevVertex = true;
	},

	/// Establish connectivity to a vertex that follows the last vertex.
	/// Don't call this for loops.
	SetNextVertex: function(nextVertex)
	{
		this.m_nextVertex.Assign(nextVertex);
		this.m_hasNextVertex = true;
	},

	/// Implement b2Shape. Vertices are cloned using b2Alloc.
	Clone: function()
	{
		var shape = new b2ChainShape();

		shape.m_count = this.m_count;
		shape.m_vertices = new Array(this.m_count);

		for (var i = 0; i < this.m_count; ++i)
			shape.m_vertices[i] = this.m_vertices[i].Clone();

		shape.m_prevVertex = this.m_prevVertex.Clone();
		shape.m_nextVertex = this.m_nextVertex.Clone();
		shape.m_hasPrevVertex = this.m_hasPrevVertex;
		shape.m_hasNextVertex = this.m_hasNextVertex;

		return shape;
	},

	/// @see b2Shape::GetChildCount
	GetChildCount: function()
	{
		return this.m_count - 1;
	},

	/// Get a child edge.
	GetChildEdge: function(edge, index)
	{
		b2Assert(0 <= index && index < this.m_count - 1);
		edge.m_type = b2Shape.e_edge;
		edge.m_radius = this.m_radius;

		edge.m_vertex1 = this.m_vertices[index + 0];
		edge.m_vertex2 = this.m_vertices[index + 1];

		if (index > 0)
		{
			edge.m_vertex0 = this.m_vertices[index - 1];
			edge.m_hasVertex0 = true;
		}
		else
		{
			edge.m_vertex0 = this.m_prevVertex;
			edge.m_hasVertex0 = this.m_hasPrevVertex;
		}

		if (index < this.m_count - 2)
		{
			edge.m_vertex3 = this.m_vertices[index + 2];
			edge.m_hasVertex3 = true;
		}
		else
		{
			edge.m_vertex3 = this.m_nextVertex;
			edge.m_hasVertex3 = this.m_hasNextVertex;
		}
	},

	/// This always return false.
	/// @see b2Shape::TestPoint
	TestPoint: function(transform, p)
	{
		return false;
	},

	/// Implement b2Shape.
	RayCast: function(output, input,
					xf, childIndex)
	{
		b2Assert(childIndex < this.m_count);

		var edgeShape = new b2EdgeShape();

		var i1 = childIndex;
		var i2 = childIndex + 1;
		if (i2 == this.m_count)
		{
			i2 = 0;
		}

		edgeShape.m_vertex1 = this.m_vertices[i1].Clone();
		edgeShape.m_vertex2 = this.m_vertices[i2].Clone();

		return edgeShape.RayCast(output, input, xf, 0);
	},

	/// @see b2Shape::ComputeAABB
	ComputeAABB: function(aabb, xf, childIndex)
	{
		b2Assert(childIndex < this.m_count);

		var i1 = childIndex;
		var i2 = childIndex + 1;
		if (i2 == this.m_count)
		{
			i2 = 0;
		}

		var v1 = b2Mul_t_v2(xf, this.m_vertices[i1]);
		var v2 = b2Mul_t_v2(xf, this.m_vertices[i2]);

		aabb.lowerBound = b2Min_v2(v1, v2);
		aabb.upperBound = b2Max_v2(v1, v2);
	},

	/// Chains have zero mass.
	/// @see b2Shape::ComputeMass
	ComputeMass: function(massData, density)
	{
		massData.mass = 0.0;
		massData.center.SetZero();
		massData.I = 0.0;
	},

	_serialize: function(out)
	{
		var obj = out || {};

		this.parent.prototype._serialize.call(this, obj);

		obj['m_count'] = this.m_count;

		obj['m_vertices'] = [];

		for (var i = 0; i < this.m_count; ++i)
			obj['m_vertices'].push(this.m_vertices[i]._serialize());

		obj['m_hasPrevVertex'] = this.m_hasPrevVertex;

		if (this.m_hasPrevVertex)
			obj['m_prevVertex'] = this.m_prevVertex._serialize();

		obj['m_hasNextVertex'] = this.m_hasNextVertex;

		if (this.m_hasNextVertex)
			obj['m_nextVertex'] = this.m_nextVertex._serialize();

		return obj;
	},

	_deserialize: function(data)
	{
		this.parent.prototype._deserialize.call(this, data);

		this.m_count = data['m_count'];
		this.m_vertices = [];

		for (var i = 0; i < this.m_count; ++i)
		{
			this.m_vertices[i] = new b2Vec2();
			this.m_vertices[i]._deserialize(data['m_vertices'][i]);
		}

		this.m_hasPrevVertex = data['m_hasPrevVertex'];

		if (this.m_hasPrevVertex)
			this.m_prevVertex._deserialize(data['m_prevVertex']);

		this.m_hasNextVertex = data['m_hasNextVertex'];

		if (this.m_hasNextVertex)
			this.m_nextVertex._deserialize(data['m_nextVertex']);
	}
};

b2ChainShape._extend(b2Shape);