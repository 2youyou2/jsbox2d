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

/// This holds the mass data computed for a shape.
function b2MassData()
{
	/// The mass of the shape, usually in kilograms.
	this.mass = 0;

	/// The position of the shape's centroid relative to the shape's origin.
	this.center = new b2Vec2();

	/// The rotational inertia of the shape about the local origin.
	this.I = 0;
}

/// A shape is used for collision detection. You can create a shape however you like.
/// Shapes used for simulation in b2World are created automatically when a b2Fixture
/// is created. Shapes may encapsulate a one or more child shapes.
function b2Shape()
{
	this.m_type = 0;
	this.m_radius = 0;
}

b2Shape.prototype =
{
	/// Clone the concrete shape using the provided allocator.
	Clone: function() { },

	/// Get the type of this shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
	GetType: function() { return this.m_type; },

	/// Get the number of child primitives.
	GetChildCount: function() { },

	/// Test a point for containment in this shape. This only works for convex shapes.
	/// @param xf the shape world transform.
	/// @param p a point in world coordinates.
	TestPoint: function(xf, p) { },

	/// Cast a ray against a child shape.
	/// @param output the ray-cast results.
	/// @param input the ray-cast input parameters.
	/// @param transform the transform to be applied to the shape.
	/// @param childIndex the child shape index
	RayCast: function(output, input,
						transform, childIndex) { },

	/// Given a transform, compute the associated axis aligned bounding box for a child shape.
	/// @param aabb returns the axis aligned box.
	/// @param xf the world transform of the shape.
	/// @param childIndex the child shape
	ComputeAABB: function(aabb, xf, childIndex) { },

	/// Compute the mass properties of this shape using its dimensions and density.
	/// The inertia tensor is computed about the local origin.
	/// @param massData returns the mass data for this shape.
	/// @param density the density in kilograms per meter squared.
	ComputeMass: function(massData, density) { },

	_serialize: function(out)
	{
		var obj = out || {};

		obj['m_type'] = this.m_type;
		obj['m_radius'] = this.m_radius;

		return obj;
	},

	_deserialize: function(data)
	{
		this.m_radius = data['m_radius'];
	}
};

b2Shape.e_circle = 0;
b2Shape.e_edge = 1;
b2Shape.e_polygon = 2;
b2Shape.e_chain = 3;
b2Shape.e_typeCount = 4;
