/*
* Copyright (c) 2011 Erin Catto http://box2d.org
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

/// Color for debug drawing. Each value has the range [0,1].
function b2Color(r, g, b)
{
	this.r = r || 0;
	this.g = g || 0;
	this.b = b || 0;
}

b2Color.prototype =
{
	Set: function(r, g, b)
	{
		this.r = r;
		this.g = g;
		this.b = b;
	}
};

/// Implement and register this class with a b2World to provide debug drawing of physics
/// entities in your game.
function b2Draw()
{
}

b2Draw.prototype =
{
	/// Set the drawing flags.
	SetFlags: function(flags) { this.m_drawFlags = flags; },

	/// Get the drawing flags.
	GetFlags: function() { return this.m_drawFlags; },

	/// Append flags to the current flags.
	AppendFlags: function(flags) { this.m_drawFlags |= flags; },

	/// Clear flags from the current flags.
	ClearFlags: function(flags) { this.m_drawFlags &= ~flags; },

	/// Toggle flags
	ToggleFlags: function(flags) { this.m_drawFlags ^= flags; },

	/// Draw a closed polygon provided in CCW order.
	DrawPolygon: function(vertices, vertexCount, color) { },

	/// Draw a solid closed polygon provided in CCW order.
	DrawSolidPolygon: function(vertices, vertexCount, color) { },

	/// Draw a circle.
	DrawCircle: function(center, radius, color) { },

	/// Draw a solid circle.
	DrawSolidCircle: function(center, radius, axis, color) { },

	/// Draw a line segment.
	DrawSegment: function(p1, p2, color) { },

	/// Draw a transform. Choose your own length scale.
	/// @param xf a transform.
	DrawTransform: function(xf) { },

//'#if @LIQUIDFUN';
	/// Draw a particle array
	DrawParticles: function(centers, radius, colors, count) { },
//'#endif';

	m_drawFlags: 0
};

b2Draw.e_shapeBit = 1;	///< draw shapes
b2Draw.e_jointBit = 2;	///< draw joint connections
b2Draw.e_aabbBit = 4;	///< draw axis aligned bounding boxes
b2Draw.e_centerOfMassBit = 8;	///< draw center of mass frame
b2Draw.e_contactPoints = 16;
b2Draw.e_contactNormals = 32;
b2Draw.e_contactImpulses = 64;
b2Draw.e_frictionImpulses = 128;
b2Draw.e_statistics = 256;
b2Draw.e_profile = 512;
b2Draw.e_pairBit = 1024;