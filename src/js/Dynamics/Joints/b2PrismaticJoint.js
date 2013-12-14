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

/// Prismatic joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
function b2PrismaticJointDef()
{
	this.parent.call(this);

	this.type = b2Joint.e_prismaticJoint;
	this.localAnchorA = new b2Vec2();
	this.localAnchorB = new b2Vec2();
	this.localAxisA = new b2Vec2(1.0, 0.0);
	this.referenceAngle = 0.0;
	this.enableLimit = false;
	this.lowerTranslation = 0.0;
	this.upperTranslation = 0.0;
	this.enableMotor = false;
	this.maxMotorForce = 0.0;
	this.motorSpeed = 0.0;
}

b2PrismaticJointDef.prototype =
{
	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and unit world axis.
	Initialize: function(bA, bB, anchor, axis)
	{
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
		this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
		this.localAxisA = this.bodyA.GetLocalVector(axis);
		this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
	},

	_deserialize: function(data, bodies, joints)
	{
		this.parent.prototype._deserialize.call(this, data, bodies, joints);

		this.localAnchorA._deserialize(data['localAnchorA']);
		this.localAnchorB._deserialize(data['localAnchorB']);
		this.localAxisA._deserialize(data['localAxisA']);
		this.referenceAngle = data['referenceAngle'];
		this.enableLimit = data['enableLimit'];
		this.lowerTranslation = data['lowerTranslation'];
		this.upperTranslation = data['upperTranslation'];
		this.enableMotor = data['enableMotor'];
		this.maxMotorForce = data['maxMotorForce'];
		this.motorSpeed = data['motorSpeed'];
	}
};

b2PrismaticJointDef._extend(b2JointDef);

/// A prismatic joint. This joint provides one degree of freedom: translation
/// along an axis fixed in bodyA. Relative rotation is prevented. You can
/// use a joint limit to restrict the range of motion and a joint motor to
/// drive the motion or to model joint friction.
function b2PrismaticJoint(def)
{
	this.parent.call(this, def);

	this.m_localAnchorA = def.localAnchorA.Clone();
	this.m_localAnchorB = def.localAnchorB.Clone();
	this.m_localXAxisA = def.localAxisA.Clone();
	this.m_localXAxisA.Normalize();
	this.m_localYAxisA = b2Cross_f_v2(1.0, this.m_localXAxisA);
	this.m_referenceAngle = def.referenceAngle;

	this.m_impulse = new b2Vec3();
	this.m_motorMass = 0.0;
	this.m_motorImpulse = 0.0;

	this.m_lowerTranslation = def.lowerTranslation;
	this.m_upperTranslation = def.upperTranslation;
	this.m_maxMotorForce = def.maxMotorForce;
	this.m_motorSpeed = def.motorSpeed;
	this.m_enableLimit = def.enableLimit;
	this.m_enableMotor = def.enableMotor;
	this.m_limitState = b2Joint.e_inactiveLimit;

	this.m_axis = new b2Vec2();
	this.m_perp = new b2Vec2();

	// Solver temp
	this.m_indexA = 0;
	this.m_indexB = 0;
	this.m_localCenterA = new b2Vec2();
	this.m_localCenterB = new b2Vec2();
	this.m_invMassA = 0;
	this.m_invMassB = 0;
	this.m_invIA = 0;
	this.m_invIB = 0;
	this.m_s1 = 0, this.m_s2 = 0;
	this.m_a1 = 0, this.m_a2 = 0;
	this.m_K = new b2Mat33();
	this.m_motorMass = 0;
}

b2PrismaticJoint.prototype =
{
	GetAnchorA: function()
	{
		return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
	},
	GetAnchorB: function()
	{
		return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
	},

	GetReactionForce: function(inv_dt)
	{
		return b2Vec2.Multiply(inv_dt, b2Vec2.Add(b2Vec2.Multiply(this.m_impulse.x, this.m_perp), b2Vec2.Multiply((this.m_motorImpulse + this.m_impulse.z), this.m_axis)));
	},
	GetReactionTorque: function(inv_dt)
	{
		return inv_dt * this.m_impulse.y;
	},

	/// The local anchor point relative to bodyA's origin.
	GetLocalAnchorA: function() { return this.m_localAnchorA; },

	/// The local anchor point relative to bodyB's origin.
	GetLocalAnchorB: function() { return this.m_localAnchorB; },

	/// The local joint axis relative to bodyA.
	GetLocalAxisA: function() { return this.m_localXAxisA; },

	/// Get the reference angle.
	GetReferenceAngle: function() { return this.m_referenceAngle; },

	/// Get the current joint translation, usually in meters.
	GetJointTranslation: function()
	{
		var pA = this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
		var pB = this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
		var d = b2Vec2.Subtract(pB, pA);
		var axis = this.m_bodyA.GetWorldVector(this.m_localXAxisA);

		var translation = b2Dot_v2_v2(d, axis);
		return translation;
	},

	/// Get the current joint translation speed, usually in meters per second.
	GetJointSpeed: function()
	{
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;

		var rA = b2Mul_r_v2(bA.m_xf.q, b2Vec2.Subtract(this.m_localAnchorA, bA.m_sweep.localCenter));
		var rB = b2Mul_r_v2(bB.m_xf.q, b2Vec2.Subtract(this.m_localAnchorB, bB.m_sweep.localCenter));
		var p1 = b2Vec2.Add(bA.m_sweep.c, rA);
		var p2 = b2Vec2.Add(bB.m_sweep.c, rB);
		var d = b2Vec2.Subtract(p2, p1);
		var axis = b2Mul_r_v2(bA.m_xf.q, this.m_localXAxisA);

		var vA = bA.m_linearVelocity;
		var vB = bB.m_linearVelocity;
		var wA = bA.m_angularVelocity;
		var wB = bB.m_angularVelocity;

		var speed = b2Dot_v2_v2(d, b2Cross_f_v2(wA, axis)) + b2Dot_v2_v2(axis, b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(vB, b2Cross_f_v2(wB, rB)), vA), b2Cross_f_v2(wA, rA)));
		return speed;
	},

	/// Is the joint limit enabled?
	IsLimitEnabled: function()
	{
		return this.m_enableLimit;
	},

	/// Enable/disable the joint limit.
	EnableLimit: function(flag)
	{
		if (flag != this.m_enableLimit)
		{
			this.m_bodyA.SetAwake(true);
			this.m_bodyB.SetAwake(true);
			this.m_enableLimit = flag;
			this.m_impulse.z = 0.0;
		}
	},

	/// Get the lower joint limit, usually in meters.
	GetLowerLimit: function()
	{
		return this.m_lowerTranslation;
	},

	/// Get the upper joint limit, usually in meters.
	GetUpperLimit: function()
	{
		return this.m_upperTranslation;
	},

	/// Set the joint limits, usually in meters.
	SetLimits: function(lower, upper)
	{
		b2Assert(lower <= upper);
		if (lower != this.m_lowerTranslation || upper != this.m_upperTranslation)
		{
			this.m_bodyA.SetAwake(true);
			this.m_bodyB.SetAwake(true);
			this.m_lowerTranslation = lower;
			this.m_upperTranslation = upper;
			this.m_impulse.z = 0.0;
		}
	},

	/// Is the joint motor enabled?
	IsMotorEnabled: function()
	{
		return this.m_enableMotor;
	},

	/// Enable/disable the joint motor.
	EnableMotor: function(flag)
	{
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_enableMotor = flag;
	},

	/// Set the motor speed, usually in meters per second.
	SetMotorSpeed: function(speed)
	{
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_motorSpeed = speed;
	},

	/// Get the motor speed, usually in meters per second.
	GetMotorSpeed: function()
	{
		return this.m_motorSpeed;
	},

	/// Set the maximum motor force, usually in N.
	SetMaxMotorForce: function(force)
	{
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_maxMotorForce = force;
	},
	GetMaxMotorForce: function() { return this.m_maxMotorForce; },

	/// Get the current motor force given the inverse time step, usually in N.
	GetMotorForce: function(inv_dt)
	{
		return inv_dt * this.m_motorImpulse;
	},

	InitVelocityConstraints: function(data)
	{
		this.m_indexA = this.m_bodyA.m_islandIndex;
		this.m_indexB = this.m_bodyB.m_islandIndex;
		this.m_localCenterA = this.m_bodyA.m_sweep.localCenter;
		this.m_localCenterB = this.m_bodyB.m_sweep.localCenter;
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

		// Compute the effective masses.
		var rA = b2Mul_r_v2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA));
		var rB = b2Mul_r_v2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB));
		var d = b2Vec2.Add(b2Vec2.Subtract(cB, cA), b2Vec2.Subtract(rB, rA));

		var mA = this.m_invMassA, mB = this.m_invMassB;
		var iA = this.m_invIA, iB = this.m_invIB;

		// Compute motor Jacobian and effective mass.
		{
			this.m_axis = b2Mul_r_v2(qA, this.m_localXAxisA);
			this.m_a1 = b2Cross_v2_v2(b2Vec2.Add(d, rA), this.m_axis);
			this.m_a2 = b2Cross_v2_v2(rB, this.m_axis);

			this.m_motorMass = mA + mB + iA * this.m_a1 * this.m_a1 + iB * this.m_a2 * this.m_a2;
			if (this.m_motorMass > 0.0)
			{
				this.m_motorMass = 1.0 / this.m_motorMass;
			}
		}

		// Prismatic constraint.
		{
			this.m_perp = b2Mul_r_v2(qA, this.m_localYAxisA);

			this.m_s1 = b2Cross_v2_v2(b2Vec2.Add(d, rA), this.m_perp);
			this.m_s2 = b2Cross_v2_v2(rB, this.m_perp);

			var k11 = mA + mB + iA * this.m_s1 * this.m_s1 + iB * this.m_s2 * this.m_s2;
			var k12 = iA * this.m_s1 + iB * this.m_s2;
			var k13 = iA * this.m_s1 * this.m_a1 + iB * this.m_s2 * this.m_a2;
			var k22 = iA + iB;
			if (k22 == 0.0)
			{
				// For bodies with fixed rotation.
				k22 = 1.0;
			}
			var k23 = iA * this.m_a1 + iB * this.m_a2;
			var k33 = mA + mB + iA * this.m_a1 * this.m_a1 + iB * this.m_a2 * this.m_a2;

			this.m_K.ex.Set(k11, k12, k13);
			this.m_K.ey.Set(k12, k22, k23);
			this.m_K.ez.Set(k13, k23, k33);
		}

		// Compute motor and limit terms.
		if (this.m_enableLimit)
		{
			var jointTranslation = b2Dot_v2_v2(this.m_axis, d);
			if (b2Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * b2_linearSlop)
			{
				this.m_limitState = b2Joint.e_equalLimits;
			}
			else if (jointTranslation <= this.m_lowerTranslation)
			{
				if (this.m_limitState != b2Joint.e_atLowerLimit)
				{
					this.m_limitState = b2Joint.e_atLowerLimit;
					this.m_impulse.z = 0.0;
				}
			}
			else if (jointTranslation >= this.m_upperTranslation)
			{
				if (this.m_limitState != b2Joint.e_atUpperLimit)
				{
					this.m_limitState = b2Joint.e_atUpperLimit;
					this.m_impulse.z = 0.0;
				}
			}
			else
			{
				this.m_limitState = b2Joint.e_inactiveLimit;
				this.m_impulse.z = 0.0;
			}
		}
		else
		{
			this.m_limitState = b2Joint.e_inactiveLimit;
			this.m_impulse.z = 0.0;
		}

		if (this.m_enableMotor == false)
		{
			this.m_motorImpulse = 0.0;
		}

		if (data.step.warmStarting)
		{
			// Account for variable time step.
			this.m_impulse.Multiply(data.step.dtRatio);
			this.m_motorImpulse *= data.step.dtRatio;

			var P = b2Vec2.Add(b2Vec2.Multiply(this.m_impulse.x, this.m_perp), b2Vec2.Multiply((this.m_motorImpulse + this.m_impulse.z), this.m_axis));
			var LA = this.m_impulse.x * this.m_s1 + this.m_impulse.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_a1;
			var LB = this.m_impulse.x * this.m_s2 + this.m_impulse.y + (this.m_motorImpulse + this.m_impulse.z) * this.m_a2;

			vA.Subtract(b2Vec2.Multiply(mA, P));
			wA -= iA * LA;

			vB.Add(b2Vec2.Multiply(mB, P));
			wB += iB * LB;
		}
		else
		{
			this.m_impulse.SetZero();
			this.m_motorImpulse = 0.0;
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

		var mA = this.m_invMassA, mB = this.m_invMassB;
		var iA = this.m_invIA, iB = this.m_invIB;

		// Solve linear motor constraint.
		if (this.m_enableMotor && this.m_limitState != b2Joint.e_equalLimits)
		{
			var Cdot = b2Dot_v2_v2(this.m_axis, b2Vec2.Subtract(vB, vA)) + this.m_a2 * wB - this.m_a1 * wA;
			var impulse = this.m_motorMass * (this.m_motorSpeed - Cdot);
			var oldImpulse = this.m_motorImpulse;
			var maxImpulse = data.step.dt * this.m_maxMotorForce;
			this.m_motorImpulse = b2Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = this.m_motorImpulse - oldImpulse;

			var P = b2Vec2.Multiply(impulse, this.m_axis);
			var LA = impulse * this.m_a1;
			var LB = impulse * this.m_a2;

			vA.Subtract(b2Vec2.Multiply(mA, P));
			wA -= iA * LA;

			vB.Add(b2Vec2.Multiply(mB, P));
			wB += iB * LB;
		}

		var Cdot1 = new b2Vec2();
		Cdot1.x = b2Dot_v2_v2(this.m_perp, b2Vec2.Subtract(vB, vA)) + this.m_s2 * wB - this.m_s1 * wA;
		Cdot1.y = wB - wA;

		if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit)
		{
			// Solve prismatic and limit constraint in block form.
			var Cdot2;
			Cdot2 = b2Dot_v2_v2(this.m_axis, b2Vec2.Subtract(vB, vA)) + this.m_a2 * wB - this.m_a1 * wA;
			var Cdot = new b2Vec3(Cdot1.x, Cdot1.y, Cdot2);

			var f1 = this.m_impulse.Clone();
			var df =  this.m_K.Solve33(Cdot.Negate());
			this.m_impulse.Add(df);

			if (this.m_limitState == b2Joint.e_atLowerLimit)
			{
				this.m_impulse.z = b2Max(this.m_impulse.z, 0.0);
			}
			else if (this.m_limitState == b2Joint.e_atUpperLimit)
			{
				this.m_impulse.z = b2Min(this.m_impulse.z, 0.0);
			}

			// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
			var b = b2Vec2.Subtract(Cdot1.Negate(), b2Vec2.Multiply((this.m_impulse.z - f1.z), new b2Vec2(this.m_K.ez.x, this.m_K.ez.y)));
			var f2r = b2Vec2.Add(this.m_K.Solve22(b), new b2Vec2(f1.x, f1.y));
			this.m_impulse.x = f2r.x;
			this.m_impulse.y = f2r.y;

			df = b2Vec3.Subtract(this.m_impulse, f1);

			var P = b2Vec2.Add(b2Vec2.Multiply(df.x, this.m_perp), b2Vec2.Multiply(df.z, this.m_axis));
			var LA = df.x * this.m_s1 + df.y + df.z * this.m_a1;
			var LB = df.x * this.m_s2 + df.y + df.z * this.m_a2;

			vA.Subtract(b2Vec2.Multiply(mA, P));
			wA -= iA * LA;

			vB.Add(b2Vec2.Multiply(mB, P));
			wB += iB * LB;
		}
		else
		{
			// Limit is inactive, just solve the prismatic constraint in block form.
			var df = this.m_K.Solve22(Cdot1.Negate());
			this.m_impulse.x += df.x;
			this.m_impulse.y += df.y;

			var P = b2Vec2.Multiply(df.x, this.m_perp);
			var LA = df.x * this.m_s1 + df.y;
			var LB = df.x * this.m_s2 + df.y;

			vA.Subtract(b2Vec2.Multiply(mA, P));
			wA -= iA * LA;

			vB.Add(b2Vec2.Multiply(mB, P));
			wB += iB * LB;
		}

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

		var mA = this.m_invMassA, mB = this.m_invMassB;
		var iA = this.m_invIA, iB = this.m_invIB;

		// Compute fresh Jacobians
		var rA = b2Mul_r_v2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA));
		var rB = b2Mul_r_v2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB));
		var d = b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(cB, rB), cA), rA);

		var axis = b2Mul_r_v2(qA, this.m_localXAxisA);
		var a1 = b2Cross_v2_v2(b2Vec2.Add(d, rA), axis);
		var a2 = b2Cross_v2_v2(rB, axis);
		var perp = b2Mul_r_v2(qA, this.m_localYAxisA);

		var s1 = b2Cross_v2_v2(b2Vec2.Add(d, rA), perp);
		var s2 = b2Cross_v2_v2(rB, perp);

		var impulse = new b2Vec3();
		var C1 = new b2Vec2();
		C1.x = b2Dot_v2_v2(perp, d);
		C1.y = aB - aA - this.m_referenceAngle;

		var linearError = b2Abs(C1.x);
		var angularError = b2Abs(C1.y);

		var active = false;
		var C2 = 0.0;
		if (this.m_enableLimit)
		{
			var translation = b2Dot_v2_v2(axis, d);
			if (b2Abs(this.m_upperTranslation - this.m_lowerTranslation) < 2.0 * b2_linearSlop)
			{
				// Prevent large angular corrections
				C2 = b2Clamp(translation, -b2_maxLinearCorrection, b2_maxLinearCorrection);
				linearError = b2Max(linearError, b2Abs(translation));
				active = true;
			}
			else if (translation <= this.m_lowerTranslation)
			{
				// Prevent large linear corrections and allow some slop.
				C2 = b2Clamp(translation - this.m_lowerTranslation + b2_linearSlop, -b2_maxLinearCorrection, 0.0);
				linearError = b2Max(linearError, this.m_lowerTranslation - translation);
				active = true;
			}
			else if (translation >= this.m_upperTranslation)
			{
				// Prevent large linear corrections and allow some slop.
				C2 = b2Clamp(translation - this.m_upperTranslation - b2_linearSlop, 0.0, b2_maxLinearCorrection);
				linearError = b2Max(linearError, translation - this.m_upperTranslation);
				active = true;
			}
		}

		if (active)
		{
			var k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
			var k12 = iA * s1 + iB * s2;
			var k13 = iA * s1 * a1 + iB * s2 * a2;
			var k22 = iA + iB;
			if (k22 == 0.0)
			{
				// For fixed rotation
				k22 = 1.0;
			}
			var k23 = iA * a1 + iB * a2;
			var k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

			var K = new b2Mat33();
			K.ex.Set(k11, k12, k13);
			K.ey.Set(k12, k22, k23);
			K.ez.Set(k13, k23, k33);

			var C = new b2Vec3();
			C.x = C1.x;
			C.y = C1.y;
			C.z = C2;

			impulse = K.Solve33(C.Negate());
		}
		else
		{
			var k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
			var k12 = iA * s1 + iB * s2;
			var k22 = iA + iB;
			if (k22 == 0.0)
			{
				k22 = 1.0;
			}

			var K = new b2Mat22();
			K.ex.Set(k11, k12);
			K.ey.Set(k12, k22);

			var impulse1 = K.Solve(C1.Negate());
			impulse.x = impulse1.x;
			impulse.y = impulse1.y;
			impulse.z = 0.0;
		}

		var P = b2Vec2.Add(b2Vec2.Multiply(impulse.x, perp), b2Vec2.Multiply(impulse.z, axis));
		var LA = impulse.x * s1 + impulse.y + impulse.z * a1;
		var LB = impulse.x * s2 + impulse.y + impulse.z * a2;

		cA.Subtract(b2Vec2.Multiply(mA, P));
		aA -= iA * LA;
		cB.Add(b2Vec2.Multiply(mB, P));
		aB += iB * LB;

		data.positions[this.m_indexA].c.Assign(cA);
		data.positions[this.m_indexA].a = aA;
		data.positions[this.m_indexB].c.Assign(cB);
		data.positions[this.m_indexB].a = aB;

		return linearError <= b2_linearSlop && angularError <= b2_angularSlop;
	},

	_serialize: function(out)
	{
		var obj = out || {};

		this.parent.prototype._serialize.call(this, obj);

		obj['localAnchorA'] = this.m_localAnchorA._serialize();
		obj['localAnchorB'] = this.m_localAnchorB._serialize();
		obj['localAxisA'] = this.m_localXAxisA._serialize();
		obj['referenceAngle'] = this.m_referenceAngle;
		obj['enableLimit'] = this.m_enableLimit;
		obj['lowerTranslation'] = this.m_lowerTranslation;
		obj['upperTranslation'] = this.m_upperTranslation;
		obj['enableMotor'] = this.m_enableMotor;
		obj['maxMotorForce'] = this.m_maxMotorForce;
		obj['motorSpeed'] = this.m_motorSpeed;

		return obj;
	}
};

b2PrismaticJoint._extend(b2Joint);