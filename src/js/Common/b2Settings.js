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

'#if @DEBUG';
function b2Assert(A)
{
	if (!A)
	{
		console.log("Assertion failed! Pls debug.");
		debugger;
	}
}
'#endif';

var b2_maxFloat		= Number.MAX_VALUE;
var b2_epsilon		= 2.2204460492503131e-016;
var b2_pi			= Math.PI;

/// @file
/// Global tuning constants based on meters-kilograms-seconds (MKS) units.
///

// Collision

/// The maximum number of contact points between two convex shapes. Do
/// not change this value.
var b2_maxManifoldPoints		= 2;

/// The maximum number of vertices on a convex polygon. You cannot increase
/// this too much because b2BlockAllocator has a maximum object size.
var b2_maxPolygonVertices		= 8;

/// This is used to fatten AABBs in the dynamic tree. This allows proxies
/// to move by a small amount without triggering a tree adjustment.
/// This is in meters.
var b2_aabbExtension			= 0.1;

/// This is used to fatten AABBs in the dynamic tree. This is used to predict
/// the future position based on the current displacement.
/// This is a dimensionless multiplier.
var b2_aabbMultiplier			= 2.0;

/// A small length used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
var b2_linearSlop				= 0.005;

/// A small angle used as a collision and constraint tolerance. Usually it is
/// chosen to be numerically significant, but visually insignificant.
var b2_angularSlop				= (2.0 / 180.0 * b2_pi);

/// The radius of the polygon/edge shape skin. This should not be modified. Making
/// this smaller means polygons will have an insufficient buffer for continuous collision.
/// Making it larger may create artifacts for vertex collision.
var b2_polygonRadius			= (2.0 * b2_linearSlop);

/// Maximum number of sub-steps per contact in continuous physics simulation.
var b2_maxSubSteps				= 8;


// Dynamics

/// Maximum number of contacts to be handled to solve a TOI impact.
var b2_maxTOIContacts			= 32;

/// A velocity threshold for elastic collisions. Any collision with a relative linear
/// velocity below this threshold will be treated as inelastic.
var b2_velocityThreshold		= 1.0;

/// The maximum linear position correction used when solving constraints. This helps to
/// prevent overshoot.
var b2_maxLinearCorrection		= 0.2;

/// The maximum angular position correction used when solving constraints. This helps to
/// prevent overshoot.
var b2_maxAngularCorrection		= (8.0 / 180.0 * b2_pi);

/// The maximum linear velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
var b2_maxTranslation			= 2.0;
var b2_maxTranslationSquared	= (b2_maxTranslation * b2_maxTranslation);

/// The maximum angular velocity of a body. This limit is very large and is used
/// to prevent numerical problems. You shouldn't need to adjust this.
var b2_maxRotation				= (0.5 * b2_pi);
var b2_maxRotationSquared		= (b2_maxRotation * b2_maxRotation);

/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
/// that overlap is removed in one time step. However using values close to 1 often lead
/// to overshoot.
var b2_baumgarte				= 0.2;
var b2_toiBaugarte				= 0.75;


// Sleep

/// The time that a body must be still before it will go to sleep.
var b2_timeToSleep				= 0.5;

/// A body cannot sleep if its linear velocity is above this tolerance.
var b2_linearSleepTolerance		= 0.01;

/// A body cannot sleep if its angular velocity is above this tolerance.
var b2_angularSleepTolerance	= (2.0 / 180.0 * b2_pi);

/// Version numbering scheme.
/// See http://en.wikipedia.org/wiki/Software_versioning
function b2Version(ma, mi, re)
{
	this.major = ma;		///< significant changes
	this.minor = mi;		///< incremental changes
	this.revision = re;		///< bug fixes
}

b2Version.prototype =
{
	toString: function()
	{
		return this.major + '.' + this.minor + '.' + this.revision;
	}
};

/// Current version.
var b2_version = new b2Version(2, 3, 1);