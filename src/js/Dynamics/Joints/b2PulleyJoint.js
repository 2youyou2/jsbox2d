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

var b2_minPulleyLength = 2.0;

/// Pulley joint definition. This requires two ground anchors,
/// two dynamic body anchor points, and a pulley ratio.
function b2PulleyJointDef()
{
	this.parent.call(this);

	this.type = b2Joint.e_pulleyJoint;
	this.groundAnchorA = new b2Vec2(-1.0, 1.0);
	this.groundAnchorB = new b2Vec2(1.0, 1.0);
	this.localAnchorA = new b2Vec2(-1.0, 0.0);
	this.localAnchorB = new b2Vec2(1.0, 0.0);
	this.lengthA = 0.0;
	this.lengthB = 0.0;
	this.ratio = 1.0;
	this.collideConnected = true;
}

b2PulleyJointDef.prototype =
{
	/// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
	Initialize: function(bA, bB,
					groundA, groundB,
					anchorA, anchorB,
					r)
	{
		this.bodyA = bA;
		this.bodyB = bB;
		this.groundAnchorA.Assign(groundA);
		this.groundAnchorB.Assign(groundB);
		this.localAnchorA.Assign(this.bodyA.GetLocalPoint(anchorA));
		this.localAnchorB.Assign(this.bodyB.GetLocalPoint(anchorB));
		var dA = b2Vec2.Subtract(anchorA, groundA);
		this.lengthA = dA.Length();
		var dB = b2Vec2.Subtract(anchorB, groundB);
		this.lengthB = dB.Length();
		this.ratio = r;
		b2Assert(this.ratio > b2_epsilon);
	}
};

b2PulleyJointDef._extend(b2JointDef);

/// The pulley joint is connected to two bodies and two fixed ground points.
/// The pulley supports a ratio such that:
/// length1 + ratio * length2 <= constant
/// Yes, the force transmitted is scaled by the ratio.
/// Warning: the pulley joint can get a bit squirrelly by itself. They often
/// work better when combined with prismatic joints. You should also cover the
/// the anchor points with static shapes to prevent one side from going to
/// zero length.
function b2PulleyJoint(def)
{
	this.parent.call(this, def);

	// Solver temp
	this.m_indexA = 0;
	this.m_indexB = 0;
	this.m_uA = new b2Vec2();
	this.m_uB = new b2Vec2();
	this.m_rA = new b2Vec2();
	this.m_rB = new b2Vec2();
	this.m_localCenterA = new b2Vec2();
	this.m_localCenterB = new b2Vec2();
	this.m_invMassA = 0;
	this.m_invMassB = 0;
	this.m_invIA = 0;
	this.m_invIB = 0;
	this.m_mass = 0;

	this.m_groundAnchorA = def.groundAnchorA.Clone();
	this.m_groundAnchorB = def.groundAnchorB.Clone();
	this.m_localAnchorA = def.localAnchorA.Clone();
	this.m_localAnchorB = def.localAnchorB.Clone();

	this.m_lengthA = def.lengthA;
	this.m_lengthB = def.lengthB;

	b2Assert(def.ratio != 0.0);
	this.m_ratio = def.ratio;

	this.m_constant = def.lengthA + this.m_ratio * def.lengthB;

	this.m_impulse = 0.0;
}

b2PulleyJoint.prototype =
{
	GetAnchorA: function() { return this.m_bodyA.GetWorldPoint(this.m_localAnchorA); },
	GetAnchorB: function() { return this.m_bodyB.GetWorldPoint(this.m_localAnchorB); },

	GetReactionForce: function(inv_dt)
	{
		var P = b2Vec2.Multiply(this.m_impulse, this.m_uB);
		return b2Vec2.Multiply(inv_dt, P);
	},
	GetReactionTorque: function(inv_dt)
	{
		return 0.0;
	},

	/// Get the first ground anchor.
	GetGroundAnchorA: function() { return this.m_groundAnchorA; },

	/// Get the second ground anchor.
	GetGroundAnchorB: function() { return this.m_groundAnchorB; },

	/// Get the current length of the segment attached to bodyA.
	GetLengthA: function() { return this.m_lengthA; },

	/// Get the current length of the segment attached to bodyB.
	GetLengthB: function() { return this.m_lengthB; },

	/// Get the pulley ratio.
	GetRatio: function() { return this.m_ratio; },

	/// Get the current length of the segment attached to bodyA.
	GetCurrentLengthA: function()
	{
		var p = this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
		var s = this.m_groundAnchorA;
		var d = b2Vec2.Subtract(p, s);
		return d.Length();
	},

	/// Get the current length of the segment attached to bodyB.
	GetCurrentLengthB: function()
	{
		var p = this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
		var s = this.m_groundAnchorB;
		var d = b2Vec2.Subtract(p, s);
		return d.Length();
	},

	/// Implement b2Joint.ShiftOrigin
	ShiftOrigin: function(newOrigin)
	{
		this.m_groundAnchorA.Subtract(newOrigin);
		this.m_groundAnchorB.Subtract(newOrigin);
	},

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

		// Get the pulley axes.
		this.m_uA.Assign(b2Vec2.Add(cA, b2Vec2.Subtract(this.m_rA, this.m_groundAnchorA)));
		this.m_uB.Assign(b2Vec2.Add(cB, b2Vec2.Subtract(this.m_rB, this.m_groundAnchorB)));

		var lengthA = this.m_uA.Length();
		var lengthB = this.m_uB.Length();

		if (lengthA > 10.0 * b2_linearSlop)
		{
			this.m_uA.Multiply(1.0 / lengthA);
		}
		else
		{
			this.m_uA.SetZero();
		}

		if (lengthB > 10.0 * b2_linearSlop)
		{
			this.m_uB.Multiply(1.0 / lengthB);
		}
		else
		{
			this.m_uB.SetZero();
		}

		// Compute effective mass.
		var ruA = b2Cross_v2_v2(this.m_rA, this.m_uA);
		var ruB = b2Cross_v2_v2(this.m_rB, this.m_uB);

		var mA = this.m_invMassA + this.m_invIA * ruA * ruA;
		var mB = this.m_invMassB + this.m_invIB * ruB * ruB;

		this.m_mass = mA + this.m_ratio * this.m_ratio * mB;

		if (this.m_mass > 0.0)
		{
			this.m_mass = 1.0 / this.m_mass;
		}

		if (data.step.warmStarting)
		{
			// Scale impulses to support variable time steps.
			this.m_impulse *= data.step.dtRatio;

			// Warm starting.
			var PA = b2Vec2.Multiply(-(this.m_impulse), this.m_uA);
			var PB = b2Vec2.Multiply((-this.m_ratio * this.m_impulse), this.m_uB);

			vA.Add(b2Vec2.Multiply(this.m_invMassA, PA));
			wA += this.m_invIA * b2Cross_v2_v2(this.m_rA, PA);
			vB.Add(b2Vec2.Multiply(this.m_invMassB, PB));
			wB += this.m_invIB * b2Cross_v2_v2(this.m_rB, PB);
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

		var vpA = b2Vec2.Add(vA, b2Cross_f_v2(wA, this.m_rA));
		var vpB = b2Vec2.Add(vB, b2Cross_f_v2(wB, this.m_rB));

		var Cdot = -b2Dot_v2_v2(this.m_uA, vpA) - this.m_ratio * b2Dot_v2_v2(this.m_uB, vpB);
		var impulse = -this.m_mass * Cdot;
		this.m_impulse += impulse;

		var PA = b2Vec2.Multiply(-impulse, this.m_uA);
		var PB = b2Vec2.Multiply(-this.m_ratio, b2Vec2.Multiply(impulse, this.m_uB));
		vA.Add(b2Vec2.Multiply(this.m_invMassA, PA));
		wA += this.m_invIA * b2Cross_v2_v2(this.m_rA, PA);
		vB.Add(b2Vec2.Multiply(this.m_invMassB, PB));
		wB += this.m_invIB * b2Cross_v2_v2(this.m_rB, PB);

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

		// Get the pulley axes.
		var uA = b2Vec2.Add(cA, b2Vec2.Subtract(rA, this.m_groundAnchorA));
		var uB = b2Vec2.Add(cB, b2Vec2.Subtract(rB, this.m_groundAnchorB));

		var lengthA = uA.Length();
		var lengthB = uB.Length();

		if (lengthA > 10.0 * b2_linearSlop)
		{
			uA.Multiply(1.0 / lengthA);
		}
		else
		{
			uA.SetZero();
		}

		if (lengthB > 10.0 * b2_linearSlop)
		{
			uB.Multiply(1.0 / lengthB);
		}
		else
		{
			uB.SetZero();
		}

		// Compute effective mass.
		var ruA = b2Cross_v2_v2(rA, uA);
		var ruB = b2Cross_v2_v2(rB, uB);

		var mA = this.m_invMassA + this.m_invIA * ruA * ruA;
		var mB = this.m_invMassB + this.m_invIB * ruB * ruB;

		var mass = mA + this.m_ratio * this.m_ratio * mB;

		if (mass > 0.0)
		{
			mass = 1.0 / mass;
		}

		var C = this.m_constant - lengthA - this.m_ratio * lengthB;
		var linearError = b2Abs(C);

		var impulse = -mass * C;

		var PA = b2Vec2.Multiply(-impulse, uA);
		var PB = b2Vec2.Multiply(-this.m_ratio, b2Vec2.Multiply(impulse, uB));

		cA.Add(b2Vec2.Multiply(this.m_invMassA, PA));
		aA += this.m_invIA * b2Cross_v2_v2(rA, PA);
		cB.Add(b2Vec2.Multiply(this.m_invMassB, PB));
		aB += this.m_invIB * b2Cross_v2_v2(rB, PB);

		data.positions[this.m_indexA].c.Assign(cA);
		data.positions[this.m_indexA].a = aA;
		data.positions[this.m_indexB].c.Assign(cB);
		data.positions[this.m_indexB].a = aB;

		return linearError < b2_linearSlop;
	}
};

b2PulleyJoint._extend(b2Joint);