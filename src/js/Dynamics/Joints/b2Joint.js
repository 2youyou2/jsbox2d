/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

function b2Jacobian()
{
	this.linear = new b2Vec2();
	this.angularA = 0;
	this.angularB = 0;
};

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
function b2JointEdge()
{
	this.other = null;			///< provides quick access to the other body attached.
	this.joint = null;			///< the joint
	this.prev = null;			///< the previous joint edge in the body's joint list
	this.next = null;			///< the next joint edge in the body's joint list
};

/// Joint definitions are used to construct joints.
function b2JointDef()
{
	this.type = b2Joint.e_unknownJoint;
	this.userData = null;
	this.bodyA = null;
	this.bodyB = null;
	this.collideConnected = false;
};

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
function b2Joint(def)
{
	b2Assert(def.bodyA != def.bodyB);

	this.m_type = def.type;
	this.m_prev = null;
	this.m_next = null;
	this.m_bodyA = def.bodyA;
	this.m_bodyB = def.bodyB;
	this.m_index = 0;
	this.m_collideConnected = def.collideConnected;
	this.m_islandFlag = false;
	this.m_userData = def.userData;

	this.m_edgeA = new b2JointEdge();
	this.m_edgeA.joint = null;
	this.m_edgeA.other = null;
	this.m_edgeA.prev = null;
	this.m_edgeA.next = null;

	this.m_edgeB = new b2JointEdge();
	this.m_edgeB.joint = null;
	this.m_edgeB.other = null;
	this.m_edgeB.prev = null;
	this.m_edgeB.next = null;
}

b2Joint.prototype =
{
	/// Get the type of the concrete joint.
	GetType: function()
	{
		return this.m_type;
	},

	/// Get the first body attached to this joint.
	GetBodyA: function()
	{
		return this.m_bodyA;
	},

	/// Get the second body attached to this joint.
	GetBodyB: function()
	{
		return this.m_bodyB;
	},

	/// Get the anchor point on bodyA in world coordinates.
	GetAnchorA: function() { },

	/// Get the anchor point on bodyB in world coordinates.
	GetAnchorB: function() { },

	/// Get the reaction force on bodyB at the joint anchor in Newtons.
	GetReactionForce: function(inv_dt) { },

	/// Get the reaction torque on bodyB in N*m.
	GetReactionTorque: function(inv_dt) { },

	/// Get the next joint the world joint list.
	GetNext: function()
	{
		return this.m_next;
	},

	/// Get the user data pointer.
	GetUserData: function()
	{
		return this.m_userData;
	},

	/// Set the user data pointer.
	SetUserData: function(data)
	{
		this.m_userData = data;
	},

	/// Short-cut function to determine if either body is inactive.
	IsActive: function()
	{
		return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
	},

	/// Get collide connected.
	/// Note: modifying the collide connect flag won't work correctly because
	/// the flag is only checked when fixture AABBs begin to overlap.
	GetCollideConnected: function()
	{
		return this.m_collideConnected;
	},

	/// Shift the origin for any points stored in world coordinates.
	ShiftOrigin: function(newOrigin) { },

	InitVelocityConstraints: function(data) { },
	SolveVelocityConstraints: function(data) { },

	// This returns true if the position errors are within tolerance.
	SolvePositionConstraints: function(data) { }
};

b2Joint.e_inactiveLimit = 0;
b2Joint.e_atLowerLimit = 1;
b2Joint.e_atUpperLimit = 2;
b2Joint.e_equalLimits = 3;

b2Joint.e_unknownJoint = 0;
b2Joint.e_revoluteJoint = 1;
b2Joint.e_prismaticJoint = 2;
b2Joint.e_distanceJoint = 3;
b2Joint.e_pulleyJoint = 4;
b2Joint.e_mouseJoint = 5;
b2Joint.e_gearJoint = 6;
b2Joint.e_wheelJoint = 7;
b2Joint.e_weldJoint = 8;
b2Joint.e_frictionJoint = 9;
b2Joint.e_ropeJoint = 10;
b2Joint.e_motorJoint = 11;

b2Joint.Create = function(def)
{
	var joint = null;

	switch (def.type)
	{
	case b2Joint.e_distanceJoint:
		joint = new b2DistanceJoint(def);
		break;

	case b2Joint.e_mouseJoint:
		joint = new b2MouseJoint(def);
		break;

	case b2Joint.e_prismaticJoint:
		joint = new b2PrismaticJoint(def);
		break;

	case b2Joint.e_revoluteJoint:
		joint = new b2RevoluteJoint(def);
		break;

	case b2Joint.e_pulleyJoint:
		joint = new b2PulleyJoint(def);
		break;

	case b2Joint.e_gearJoint:
		joint = new b2GearJoint(def);
		break;

	case b2Joint.e_wheelJoint:
		joint = new b2WheelJoint(def);
		break;

	case b2Joint.e_weldJoint:
		joint = new b2WeldJoint(def);
		break;

	case b2Joint.e_frictionJoint:
		joint = new b2FrictionJoint(def);
		break;

	case b2Joint.e_ropeJoint:
		joint = new b2RopeJoint(def);
		break;

	case b2Joint.e_motorJoint:
		joint = new b2MotorJoint(def);
		break;

	default:
		b2Assert(false);
		break;
	}

	return joint;
};

b2Joint.Destroy = function(joint)
{
};