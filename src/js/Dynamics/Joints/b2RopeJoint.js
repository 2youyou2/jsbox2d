/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

/// Rope joint definition. This requires two body anchor points and
/// a maximum lengths.
/// Note: by default the connected objects will not collide.
/// see collideConnected in b2JointDef.
function b2RopeJointDef()
{
	this.parent.call(this);

	this.type = b2Joint.e_ropeJoint;
	this.localAnchorA = new b2Vec2(-1.0, 0.0);
	this.localAnchorB = new b2Vec2(1.0, 0.0);
	this.maxLength = 0.0;
}

b2RopeJointDef._extend(b2JointDef);

/// A rope joint enforces a maximum distance between two points
/// on two bodies. It has no other effect.
/// Warning: if you attempt to change the maximum length during
/// the simulation you will get some non-physical behavior.
/// A model that would allow you to dynamically modify the length
/// would have some sponginess, so I chose not to implement it
/// that way. See b2DistanceJoint if you want to dynamically
/// control length.
function b2RopeJoint(def)
{
	this.parent.call(this, def);

	this.m_localAnchorA = def.localAnchorA.Clone();
	this.m_localAnchorB = def.localAnchorB.Clone();

	this.m_maxLength = def.maxLength;

	this.m_mass = 0.0;
	this.m_impulse = 0.0;
	this.m_state = b2Joint.e_inactiveLimit;
	this.m_length = 0.0;

	// Solver temp
	this.m_indexA = 0;
	this.m_indexB = 0;
	this.m_u = new b2Vec2();
	this.m_rA = new b2Vec2();
	this.m_rB = new b2Vec2();
	this.m_localCenterA = new b2Vec2();
	this.m_localCenterB = new b2Vec2();
	this.m_invMassA = 0;
	this.m_invMassB = 0;
	this.m_invIA = 0;
	this.m_invIB = 0;
}

b2RopeJoint.prototype =
{
	GetAnchorA: function() { return this.m_bodyA.GetWorldPoint(this.m_localAnchorA); },
	GetAnchorB: function() { return this.m_bodyB.GetWorldPoint(this.m_localAnchorB); },

	GetReactionForce: function(inv_dt)
	{
		var F = b2Vec2.Multiply((inv_dt * this.m_impulse), this.m_u);
		return F;
	},
	GetReactionTorque: function(inv_dt) { return 0.0; },

	/// The local anchor point relative to bodyA's origin.
	GetLocalAnchorA: function() { return this.m_localAnchorA; },

	/// The local anchor point relative to bodyB's origin.
	GetLocalAnchorB: function() { return this.m_localAnchorB; },

	/// Set/Get the maximum length of the rope.
	SetMaxLength: function(length) { this.m_maxLength = length; },
	GetMaxLength: function() { return this.m_maxLength; },

	GetLimitState: function() { return this.m_state; },

	InitVelocityConstraints: function(data)
	{
		this.m_indexA = this.m_bodyA.m_islandIndex;
		this.m_indexB = this.m_bodyB.m_islandIndex;
		this.m_localCenterA.Assign(this.m_bodyA.m_sweep.localCenter);
		this.m_localCenterB.Assign(this.m_bodyB.m_sweep.localCenter);
		this.m_invMassA = this.m_bodyA.m_invMass;
		this.m_invMassB = this.m_bodyB.m_invMass;
		this.m_invIA = this.m_bodyA.m_invI;
		this.m_invIB = this.m_bodyB.m_invI;

		var cA = data.positions[this.m_indexA].c.Clone();
		var aA = data.positions[this.m_indexA].a;
		var vA = data.velocities[this.m_indexA].v.Clone();
		var wA = data.velocities[this.m_indexA].w;

		var cB = data.positions[this.m_indexB].c.Clone();
		var aB = data.positions[this.m_indexB].a;
		var vB = data.velocities[this.m_indexB].v.Clone();
		var wB = data.velocities[this.m_indexB].w;

		var qA = new b2Rot(aA), qB = new b2Rot(aB);

		this.m_rA.Assign(b2Mul_r_v2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA)));
		this.m_rB.Assign(b2Mul_r_v2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB)));
		this.m_u.Assign(b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(cB, this.m_rB), cA), this.m_rA));

		this.m_length = this.m_u.Length();

		var C = this.m_length - this.m_maxLength;
		if (C > 0.0)
		{
			this.m_state = b2Joint.e_atUpperLimit;
		}
		else
		{
			this.m_state = b2Joint.e_inactiveLimit;
		}

		if (this.m_length > b2_linearSlop)
		{
			this.m_u.Multiply(1.0 / this.m_length);
		}
		else
		{
			this.m_u.SetZero();
			this.m_mass = 0.0;
			this.m_impulse = 0.0;
			return;
		}

		// Compute effective mass.
		var crA = b2Cross_v2_v2(this.m_rA, this.m_u);
		var crB = b2Cross_v2_v2(this.m_rB, this.m_u);
		var invMass = this.m_invMassA + this.m_invIA * crA * crA + this.m_invMassB + this.m_invIB * crB * crB;

		this.m_mass = invMass != 0.0 ? 1.0 / invMass : 0.0;

		if (data.step.warmStarting)
		{
			// Scale the impulse to support a variable time step.
			this.m_impulse *= data.step.dtRatio;

			var P = b2Vec2.Multiply(this.m_impulse, this.m_u);
			vA.Subtract(b2Vec2.Multiply(this.m_invMassA, P));
			wA -= this.m_invIA * b2Cross_v2_v2(this.m_rA, P);
			vB.Add(b2Vec2.Multiply(this.m_invMassB, P));
			wB += this.m_invIB * b2Cross_v2_v2(this.m_rB, P);
		}
		else
		{
			this.m_impulse = 0.0;
		}

		data.velocities[this.m_indexA].v.Assign(vA);
		data.velocities[this.m_indexA].w = wA;
		data.velocities[this.m_indexB].v.Assign(vB);
		data.velocities[this.m_indexB].w = wB;
	},
	SolveVelocityConstraints: function(data)
	{
		var vA = data.velocities[this.m_indexA].v.Clone();
		var wA = data.velocities[this.m_indexA].w;
		var vB = data.velocities[this.m_indexB].v.Clone();
		var wB = data.velocities[this.m_indexB].w;

		// Cdot = dot(u, v + cross(w, r))
		var vpA = b2Vec2.Add(vA, b2Cross_f_v2(wA, this.m_rA));
		var vpB = b2Vec2.Add(vB, b2Cross_f_v2(wB, this.m_rB));
		var C = this.m_length - this.m_maxLength;
		var Cdot = b2Dot_v2_v2(this.m_u, b2Vec2.Subtract(vpB, vpA));

		// Predictive constraint.
		if (C < 0.0)
		{
			Cdot += data.step.inv_dt * C;
		}

		var impulse = -this.m_mass * Cdot;
		var oldImpulse = this.m_impulse;
		this.m_impulse = b2Min(0.0, this.m_impulse + impulse);
		impulse = this.m_impulse - oldImpulse;

		var P = b2Vec2.Multiply(impulse, this.m_u);
		vA.Subtract(b2Vec2.Multiply(this.m_invMassA, P));
		wA -= this.m_invIA * b2Cross_v2_v2(this.m_rA, P);
		vB.Add(b2Vec2.Multiply(this.m_invMassB, P));
		wB += this.m_invIB * b2Cross_v2_v2(this.m_rB, P);

		data.velocities[this.m_indexA].v.Assign(vA);
		data.velocities[this.m_indexA].w = wA;
		data.velocities[this.m_indexB].v.Assign(vB);
		data.velocities[this.m_indexB].w = wB;
	},
	SolvePositionConstraints: function(data)
	{
		var cA = data.positions[this.m_indexA].c.Clone();
		var aA = data.positions[this.m_indexA].a;
		var cB = data.positions[this.m_indexB].c.Clone();
		var aB = data.positions[this.m_indexB].a;

		var qA = new b2Rot(aA), qB = new b2Rot(aB);

		var rA = b2Mul_r_v2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA));
		var rB = b2Mul_r_v2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB));
		var u = b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(cB, rB), cA), rA);

		var length = u.Normalize();
		var C = length - this.m_maxLength;

		C = b2Clamp(C, 0.0, b2_maxLinearCorrection);

		var impulse = -this.m_mass * C;
		var P = b2Vec2.Multiply(impulse, u);

		cA.Subtract(b2Vec2.Multiply(this.m_invMassA, P));
		aA -= this.m_invIA * b2Cross_v2_v2(rA, P);
		cB.Add(b2Vec2.Multiply(this.m_invMassB, P));
		aB += this.m_invIB * b2Cross_v2_v2(rB, P);

		data.positions[this.m_indexA].c.Assign(cA);
		data.positions[this.m_indexA].a = aA;
		data.positions[this.m_indexB].c.Assign(cB);
		data.positions[this.m_indexB].a = aB;

		return length - this.m_maxLength < b2_linearSlop;
	}
};

b2RopeJoint._extend(b2Joint);