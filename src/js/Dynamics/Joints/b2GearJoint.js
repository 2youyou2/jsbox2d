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

/// Gear joint definition. This definition requires two existing
/// revolute or prismatic joints (any combination will work).
function b2GearJointDef()
{
	this.parent.call(this);

	this.type = b2Joint.e_gearJoint;
	this.joint1 = null;
	this.joint2 = null;
	this.ratio = 1.0;

	Object.seal(this);
}

b2GearJointDef.prototype =
{
	_deserialize: function(data, bodies, joints)
	{
		this.parent.prototype._deserialize.call(this, data, bodies, joints);

		// set up later on
		this.joint1 = data['joint1'];
		this.joint2 = data['joint2'];
		this.ratio = data['ratio'];
	}
};

b2GearJointDef._extend(b2JointDef);

/// A gear joint is used to connect two joints together. Either joint
/// can be a revolute or prismatic joint. You specify a gear ratio
/// to bind the motions together:
/// coordinate1 + ratio * coordinate2 = constant
/// The ratio can be negative or positive. If one joint is a revolute joint
/// and the other joint is a prismatic joint, then the ratio will have units
/// of length or units of 1/length.
/// @warning You have to manually destroy the gear joint if joint1 or joint2
/// is destroyed.
function b2GearJoint(def)
{
	this.parent.call(this, def);

	this.m_joint1 = def.joint1;
	this.m_joint2 = def.joint2;

	this.m_typeA = this.m_joint1.GetType();
	this.m_typeB = this.m_joint2.GetType();

'#if @DEBUG';
	b2Assert(this.m_typeA == b2Joint.e_revoluteJoint || this.m_typeA == b2Joint.e_prismaticJoint);
	b2Assert(this.m_typeB == b2Joint.e_revoluteJoint || this.m_typeB == b2Joint.e_prismaticJoint);
'#endif';

	var coordinateA, coordinateB;

	// TODO_ERIN there might be some problem with the joint edges in b2Joint.

	this.m_bodyC = this.m_joint1.GetBodyA();
	this.m_bodyA = this.m_joint1.GetBodyB();

	// Get geometry of joint1
	var xfA = this.m_bodyA.m_xf;
	var aA = this.m_bodyA.m_sweep.a;
	var xfC = this.m_bodyC.m_xf;
	var aC = this.m_bodyC.m_sweep.a;

	this.m_localAnchorA = new b2Vec2();
	this.m_localAnchorB = new b2Vec2();
	this.m_localAnchorC = new b2Vec2();
	this.m_localAnchorD = new b2Vec2();

	this.m_localAxisC = new b2Vec2();
	this.m_localAxisD = new b2Vec2();

	if (this.m_typeA == b2Joint.e_revoluteJoint)
	{
		var revolute = def.joint1;
		this.m_localAnchorC.Assign(revolute.m_localAnchorA);
		this.m_localAnchorA.Assign(revolute.m_localAnchorB);
		this.m_referenceAngleA = revolute.m_referenceAngle;
		this.m_localAxisC.SetZero();

		coordinateA = aA - aC - this.m_referenceAngleA;
	}
	else
	{
		var prismatic = def.joint1;
		this.m_localAnchorC.Assign(prismatic.m_localAnchorA);
		this.m_localAnchorA.Assign(prismatic.m_localAnchorB);
		this.m_referenceAngleA = prismatic.m_referenceAngle;
		this.m_localAxisC.Assign(prismatic.m_localXAxisA);

		var pC = this.m_localAnchorC;
		var pA = b2MulT_r_v2(xfC.q, b2Vec2.Add(b2Mul_r_v2(xfA.q, this.m_localAnchorA), b2Vec2.Subtract(xfA.p, xfC.p)));
		coordinateA = b2Dot_v2_v2(b2Vec2.Subtract(pA, pC), this.m_localAxisC);
	}

	this.m_bodyD = this.m_joint2.GetBodyA();
	this.m_bodyB = this.m_joint2.GetBodyB();

	// Get geometry of joint2
	var xfB = this.m_bodyB.m_xf;
	var aB = this.m_bodyB.m_sweep.a;
	var xfD = this.m_bodyD.m_xf;
	var aD = this.m_bodyD.m_sweep.a;

	if (this.m_typeB == b2Joint.e_revoluteJoint)
	{
		var revolute = def.joint2;
		this.m_localAnchorD.Assign(revolute.m_localAnchorA);
		this.m_localAnchorB.Assign(revolute.m_localAnchorB);
		this.m_referenceAngleB = revolute.m_referenceAngle;
		this.m_localAxisD.SetZero();

		coordinateB = aB - aD - this.m_referenceAngleB;
	}
	else
	{
		var prismatic = def.joint2;
		this.m_localAnchorD.Assign(prismatic.m_localAnchorA);
		this.m_localAnchorB.Assign(prismatic.m_localAnchorB);
		this.m_referenceAngleB = prismatic.m_referenceAngle;
		this.m_localAxisD.Assign(prismatic.m_localXAxisA);

		var pD = this.m_localAnchorD;
		var pB = b2MulT_r_v2(xfD.q, b2Vec2.Add(b2Mul_r_v2(xfB.q, this.m_localAnchorB), b2Vec2.Subtract(xfB.p, xfD.p)));
		coordinateB = b2Dot_v2_v2(b2Vec2.Subtract(pB, pD), this.m_localAxisD);
	}

	this.m_ratio = def.ratio;

	this.m_constant = coordinateA + this.m_ratio * coordinateB;

	this.m_impulse = 0.0;

	// Solver temp
	this.m_indexA = this.m_indexB = this.m_indexC = this.m_indexD = 0;
	this.m_lcA = new b2Vec2(); this.m_lcB = new b2Vec2(); this.m_lcC = new b2Vec2(); this.m_lcD = new b2Vec2();
	this.m_mA = this.m_mB = this.m_mC = this.m_mD = 0;
	this.m_iA = this.m_iB = this.m_iC = this.m_iD = 0;
	this.m_JvAC = new b2Vec2(), this.m_JvBD = new b2Vec2();
	this.m_JwA = this.m_JwB = this.m_JwC = this.m_JwD = 0;
	this.m_mass = 0;
}

b2GearJoint.prototype =
{
	GetAnchorA: function() { return this.m_bodyA.GetWorldPoint(this.m_localAnchorA); },
	GetAnchorB: function() { return this.m_bodyB.GetWorldPoint(this.m_localAnchorB); },

	GetReactionForce: function(inv_dt)
	{
		var P = b2Vec2.Multiply(this.m_impulse, this.m_JvAC);
		return b2Vec2.Multiply(inv_dt, P);
	},
	GetReactionTorque: function(inv_dt)
	{
		var L = this.m_impulse * this.m_JwA;
		return inv_dt * L;
	},

	/// Get the first joint.
	GetJoint1: function() { return this.m_joint1; },

	/// Get the second joint.
	GetJoint2: function() { return this.m_joint2; },

	/// Set/Get the gear ratio.
	SetRatio: function(ratio)
	{
'#if @DEBUG';
		b2Assert(b2IsValid(ratio));
'#endif';
		this.m_ratio = ratio;
	},
	GetRatio: function()
	{
		return this.m_ratio;
	},

	InitVelocityConstraints: function(data)
		{
		this.m_indexA = this.m_bodyA.m_islandIndex;
		this.m_indexB = this.m_bodyB.m_islandIndex;
		this.m_indexC = this.m_bodyC.m_islandIndex;
		this.m_indexD = this.m_bodyD.m_islandIndex;
		this.m_lcA.Assign(this.m_bodyA.m_sweep.localCenter);
		this.m_lcB.Assign(this.m_bodyB.m_sweep.localCenter);
		this.m_lcC.Assign(this.m_bodyC.m_sweep.localCenter);
		this.m_lcD.Assign(this.m_bodyD.m_sweep.localCenter);
		this.m_mA = this.m_bodyA.m_invMass;
		this.m_mB = this.m_bodyB.m_invMass;
		this.m_mC = this.m_bodyC.m_invMass;
		this.m_mD = this.m_bodyD.m_invMass;
		this.m_iA = this.m_bodyA.m_invI;
		this.m_iB = this.m_bodyB.m_invI;
		this.m_iC = this.m_bodyC.m_invI;
		this.m_iD = this.m_bodyD.m_invI;

		var aA = data.positions[this.m_indexA].a;
		var vA = data.velocities[this.m_indexA].v.Clone();
		var wA = data.velocities[this.m_indexA].w;

		var aB = data.positions[this.m_indexB].a;
		var vB = data.velocities[this.m_indexB].v.Clone();
		var wB = data.velocities[this.m_indexB].w;

		var aC = data.positions[this.m_indexC].a;
		var vC = data.velocities[this.m_indexC].v.Clone();
		var wC = data.velocities[this.m_indexC].w;

		var aD = data.positions[this.m_indexD].a;
		var vD = data.velocities[this.m_indexD].v.Clone();
		var wD = data.velocities[this.m_indexD].w;

		var qA = new b2Rot(aA), qB = new b2Rot(aB), qC = new b2Rot(aC), qD = new b2Rot(aD);

		this.m_mass = 0.0;

		if (this.m_typeA == b2Joint.e_revoluteJoint)
		{
			this.m_JvAC.SetZero();
			this.m_JwA = 1.0;
			this.m_JwC = 1.0;
			this.m_mass += this.m_iA + this.m_iC;
		}
		else
		{
			var u = b2Mul_r_v2(qC, this.m_localAxisC);
			var rC = b2Mul_r_v2(qC, b2Vec2.Subtract(this.m_localAnchorC, this.m_lcC));
			var rA = b2Mul_r_v2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_lcA));
			this.m_JvAC.Assign(u);
			this.m_JwC = b2Cross_v2_v2(rC, u);
			this.m_JwA = b2Cross_v2_v2(rA, u);
			this.m_mass += this.m_mC + this.m_mA + this.m_iC * this.m_JwC * this.m_JwC + this.m_iA * this.m_JwA * this.m_JwA;
		}

		if (this.m_typeB == b2Joint.e_revoluteJoint)
		{
			this.m_JvBD.SetZero();
			this.m_JwB = this.m_ratio;
			this.m_JwD = this.m_ratio;
			this.m_mass += this.m_ratio * this.m_ratio * (this.m_iB + this.m_iD);
		}
		else
		{
			var u = b2Mul_r_v2(qD, this.m_localAxisD);
			var rD = b2Mul_r_v2(qD, b2Vec2.Subtract(this.m_localAnchorD, this.m_lcD));
			var rB = b2Mul_r_v2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_lcB));
			this.m_JvBD.Assign(b2Vec2.Multiply(this.m_ratio, u));
			this.m_JwD = this.m_ratio * b2Cross_v2_v2(rD, u);
			this.m_JwB = this.m_ratio * b2Cross_v2_v2(rB, u);
			this.m_mass += this.m_ratio * this.m_ratio * (this.m_mD + this.m_mB) + this.m_iD * this.m_JwD * this.m_JwD + this.m_iB * this.m_JwB * this.m_JwB;
		}

		// Compute effective mass.
		this.m_mass = this.m_mass > 0.0 ? 1.0 / this.m_mass : 0.0;

		if (data.step.warmStarting)
		{
			vA.Add(b2Vec2.Multiply((this.m_mA * this.m_impulse), this.m_JvAC));
			wA += this.m_iA * this.m_impulse * this.m_JwA;
			vB.Add(b2Vec2.Multiply((this.m_mB * this.m_impulse), this.m_JvBD));
			wB += this.m_iB * this.m_impulse * this.m_JwB;
			vC.Subtract(b2Vec2.Multiply((this.m_mC * this.m_impulse), this.m_JvAC));
			wC -= this.m_iC * this.m_impulse * this.m_JwC;
			vD.Subtract(b2Vec2.Multiply((this.m_mD * this.m_impulse), this.m_JvBD));
			wD -= this.m_iD * this.m_impulse * this.m_JwD;
		}
		else
		{
			this.m_impulse = 0.0;
		}

		data.velocities[this.m_indexA].v.Assign(vA);
		data.velocities[this.m_indexA].w = wA;
		data.velocities[this.m_indexB].v.Assign(vB);
		data.velocities[this.m_indexB].w = wB;
		data.velocities[this.m_indexC].v.Assign(vC);
		data.velocities[this.m_indexC].w = wC;
		data.velocities[this.m_indexD].v.Assign(vD);
		data.velocities[this.m_indexD].w = wD;
	},
	SolveVelocityConstraints: function(data)
	{
		var vA = data.velocities[this.m_indexA].v.Clone();
		var wA = data.velocities[this.m_indexA].w;
		var vB = data.velocities[this.m_indexB].v.Clone();
		var wB = data.velocities[this.m_indexB].w;
		var vC = data.velocities[this.m_indexC].v.Clone();
		var wC = data.velocities[this.m_indexC].w;
		var vD = data.velocities[this.m_indexD].v.Clone();
		var wD = data.velocities[this.m_indexD].w;

		var Cdot = b2Dot_v2_v2(this.m_JvAC, b2Vec2.Subtract(vA, vC)) + b2Dot_v2_v2(this.m_JvBD, b2Vec2.Subtract(vB, vD));
		Cdot += (this.m_JwA * wA - this.m_JwC * wC) + (this.m_JwB * wB - this.m_JwD * wD);

		var impulse = -this.m_mass * Cdot;
		this.m_impulse += impulse;

		vA.Add(b2Vec2.Multiply((this.m_mA * impulse), this.m_JvAC));
		wA += this.m_iA * impulse * this.m_JwA;
		vB.Add(b2Vec2.Multiply((this.m_mB * impulse), this.m_JvBD));
		wB += this.m_iB * impulse * this.m_JwB;
		vC.Subtract(b2Vec2.Multiply((this.m_mC * impulse), this.m_JvAC));
		wC -= this.m_iC * impulse * this.m_JwC;
		vD.Subtract(b2Vec2.Multiply((this.m_mD * impulse), this.m_JvBD));
		wD -= this.m_iD * impulse * this.m_JwD;

		data.velocities[this.m_indexA].v.Assign(vA);
		data.velocities[this.m_indexA].w = wA;
		data.velocities[this.m_indexB].v.Assign(vB);
		data.velocities[this.m_indexB].w = wB;
		data.velocities[this.m_indexC].v.Assign(vC);
		data.velocities[this.m_indexC].w = wC;
		data.velocities[this.m_indexD].v.Assign(vD);
		data.velocities[this.m_indexD].w = wD;
	},
	SolvePositionConstraints: function(data)
	{
		var cA = data.positions[this.m_indexA].c.Clone();
		var aA = data.positions[this.m_indexA].a;
		var cB = data.positions[this.m_indexB].c.Clone();
		var aB = data.positions[this.m_indexB].a;
		var cC = data.positions[this.m_indexC].c.Clone();
		var aC = data.positions[this.m_indexC].a;
		var cD = data.positions[this.m_indexD].c.Clone();
		var aD = data.positions[this.m_indexD].a;

		var qA = new b2Rot(aA), qB = new b2Rot(aB), qC = new b2Rot(aC), qD = new b2Rot(aD);

		var linearError = 0.0;

		var coordinateA, coordinateB;

		var JvAC = new b2Vec2(), JvBD = new b2Vec2();
		var JwA, JwB, JwC, JwD;
		var mass = 0.0;

		if (this.m_typeA == b2Joint.e_revoluteJoint)
		{
			JvAC.SetZero();
			JwA = 1.0;
			JwC = 1.0;
			mass += this.m_iA + this.m_iC;

			coordinateA = aA - aC - this.m_referenceAngleA;
		}
		else
		{
			var u = b2Mul_r_v2(qC, this.m_localAxisC);
			var rC = b2Mul_r_v2(qC, b2Vec2.Subtract(this.m_localAnchorC, this.m_lcC));
			var rA = b2Mul_r_v2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_lcA));
			JvAC.Assign(u);
			JwC = b2Cross_v2_v2(rC, u);
			JwA = b2Cross_v2_v2(rA, u);
			mass += this.m_mC + this.m_mA + this.m_iC * JwC * JwC + this.m_iA * JwA * JwA;

			var pC = b2Vec2.Subtract(this.m_localAnchorC, this.m_lcC);
			var pA = b2MulT_r_v2(qC, b2Vec2.Add(rA, b2Vec2.Subtract(cA, cC)));
			coordinateA = b2Dot_v2_v2(b2Vec2.Subtract(pA, pC), this.m_localAxisC);
		}

		if (this.m_typeB == b2Joint.e_revoluteJoint)
		{
			JvBD.SetZero();
			JwB = this.m_ratio;
			JwD = this.m_ratio;
			mass += this.m_ratio * this.m_ratio * (this.m_iB + this.m_iD);

			coordinateB = aB - aD - this.m_referenceAngleB;
		}
		else
		{
			var u = b2Mul_r_v2(qD, this.m_localAxisD);
			var rD = b2Mul_r_v2(qD, b2Vec2.Subtract(this.m_localAnchorD, this.m_lcD));
			var rB = b2Mul_r_v2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_lcB));
			JvBD.Assign(b2Vec2.Multiply(this.m_ratio, u));
			JwD = this.m_ratio * b2Cross_v2_v2(rD, u);
			JwB = this.m_ratio * b2Cross_v2_v2(rB, u);
			mass += this.m_ratio * this.m_ratio * (this.m_mD + this.m_mB) + this.m_iD * JwD * JwD + this.m_iB * JwB * JwB;

			var pD = b2Vec2.Subtract(this.m_localAnchorD, this.m_lcD);
			var pB = b2MulT_r_v2(qD, b2Vec2.Add(rB, b2Vec2.Subtract(cB, cD)));
			coordinateB = b2Dot_v2_v2(b2Vec2.Subtract(pB, pD), this.m_localAxisD);
		}

		var C = (coordinateA + this.m_ratio * coordinateB) - this.m_constant;

		var impulse = 0.0;
		if (mass > 0.0)
		{
			impulse = -C / mass;
		}

		cA.Add(b2Vec2.Multiply(this.m_mA, b2Vec2.Multiply(impulse, JvAC)));
		aA += this.m_iA * impulse * JwA;
		cB.Add(b2Vec2.Multiply(this.m_mB, b2Vec2.Multiply(impulse, JvBD)));
		aB += this.m_iB * impulse * JwB;
		cC.Subtract(b2Vec2.Multiply(this.m_mC, b2Vec2.Multiply(impulse, JvAC)));
		aC -= this.m_iC * impulse * JwC;
		cD.Subtract(b2Vec2.Multiply(this.m_mD, b2Vec2.Multiply(impulse, JvBD)));
		aD -= this.m_iD * impulse * JwD;

		data.positions[this.m_indexA].c.Assign(cA);
		data.positions[this.m_indexA].a = aA;
		data.positions[this.m_indexB].c.Assign(cB);
		data.positions[this.m_indexB].a = aB;
		data.positions[this.m_indexC].c.Assign(cC);
		data.positions[this.m_indexC].a = aC;
		data.positions[this.m_indexD].c.Assign(cD);
		data.positions[this.m_indexD].a = aD;

		// TODO_ERIN not implemented
		return linearError < b2_linearSlop;
	},

	_serialize: function(out)
	{
		var obj = out || {};

		this.parent.prototype._serialize.call(this, obj);

		obj['joint1'] = this.m_joint1.__temp_joint_id;
		obj['joint2'] = this.m_joint2.__temp_joint_id;
		obj['ratio'] = this.m_ratio;

		return obj;
	}
};

b2GearJoint._extend(b2Joint);