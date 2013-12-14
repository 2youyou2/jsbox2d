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

/// Revolute joint definition. This requires defining an
/// anchor point where the bodies are joined. The definition
/// uses local anchor points so that the initial configuration
/// can violate the constraint slightly. You also need to
/// specify the initial relative angle for joint limits. This
/// helps when saving and loading a game.
/// The local anchor points are measured from the body's origin
/// rather than the center of mass because:
/// 1. you might not know where the center of mass will be.
/// 2. if you add/remove shapes from a body and recompute the mass,
///    the joints will be broken.
function b2RevoluteJointDef()
{
	this.parent.call(this);

	this.type = b2Joint.e_revoluteJoint;
	this.localAnchorA = new b2Vec2();
	this.localAnchorB = new b2Vec2();
	this.referenceAngle = 0.0;
	this.lowerAngle = 0.0;
	this.upperAngle = 0.0;
	this.maxMotorTorque = 0.0;
	this.motorSpeed = 0.0;
	this.enableLimit = false;
	this.enableMotor = false;
}

b2RevoluteJointDef.prototype =
{
	/// Initialize the bodies, anchors, and reference angle using a world
	/// anchor point.
	Initialize: function(bA, bB, anchor)
	{
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA = this.bodyA.GetLocalPoint(anchor);
		this.localAnchorB = this.bodyB.GetLocalPoint(anchor);
		this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
	},

	_deserialize: function(data, bodies, joints)
	{
		this.parent.prototype._deserialize.call(this, data, bodies, joints);

		this.localAnchorA._deserialize(data['localAnchorA']);
		this.localAnchorB._deserialize(data['localAnchorB']);
		this.referenceAngle = data['referenceAngle'];
		this.lowerAngle = data['lowerAngle'];
		this.upperAngle = data['upperAngle'];
		this.maxMotorTorque = data['maxMotorTorque'];
		this.motorSpeed = data['motorSpeed'];
		this.enableLimit = data['enableLimit'];
		this.enableMotor = data['enableMotor'];
	}
};

b2RevoluteJointDef._extend(b2JointDef);

/// A revolute joint constrains two bodies to share a common point while they
/// are free to rotate about the point. The relative rotation about the shared
/// point is the joint angle. You can limit the relative rotation with
/// a joint limit that specifies a lower and upper angle. You can use a motor
/// to drive the relative rotation about the shared point. A maximum motor torque
/// is provided so that infinite forces are not generated.
function b2RevoluteJoint(def)
{
	this.parent.call(this, def);

	this.m_localAnchorA = def.localAnchorA.Clone();
	this.m_localAnchorB = def.localAnchorB.Clone();
	this.m_referenceAngle = def.referenceAngle;

	this.m_impulse = new b2Vec3();
	this.m_motorImpulse = 0.0;

	this.m_lowerAngle = def.lowerAngle;
	this.m_upperAngle = def.upperAngle;
	this.m_maxMotorTorque = def.maxMotorTorque;
	this.m_motorSpeed = def.motorSpeed;
	this.m_enableLimit = def.enableLimit;
	this.m_enableMotor = def.enableMotor;
	this.m_limitState = b2Joint.e_inactiveLimit;

	// Solver temp
	this.m_indexA = 0;
	this.m_indexB = 0;
	this.m_rA = new b2Vec2();
	this.m_rB = new b2Vec2();
	this.m_localCenterA = new b2Vec2();
	this.m_localCenterB = new b2Vec2();
	this.m_invMassA = 0;
	this.m_invMassB = 0;
	this.m_invIA = 0;
	this.m_invIB = 0;
	this.m_mass = new b2Mat33();			// effective mass for point-to-point constraint.
	this.m_motorMass = 0;	// effective mass for motor/limit angular constraint.
}

b2RevoluteJoint.prototype =
{
	GetAnchorA: function()
	{
		return this.m_bodyA.GetWorldPoint(this.m_localAnchorA);
	},
	GetAnchorB: function()
	{
		return this.m_bodyB.GetWorldPoint(this.m_localAnchorB);
	},

	/// The local anchor point relative to bodyA's origin.
	GetLocalAnchorA: function() { return this.m_localAnchorA; },

	/// The local anchor point relative to bodyB's origin.
	GetLocalAnchorB: function() { return this.m_localAnchorB; },

	/// Get the reference angle.
	GetReferenceAngle: function() { return this.m_referenceAngle; },

	/// Get the current joint angle in radians.
	GetJointAngle: function()
	{
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		return bB.m_sweep.a - bA.m_sweep.a - this.m_referenceAngle;
	},

	/// Get the current joint angle speed in radians per second.
	GetJointSpeed: function()
	{
		var bA = this.m_bodyA;
		var bB = this.m_bodyB;
		return bB.m_angularVelocity - bA.m_angularVelocity;
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

	/// Get the lower joint limit in radians.
	GetLowerLimit: function()
	{
		return this.m_lowerAngle;
	},

	/// Get the upper joint limit in radians.
	GetUpperLimit: function()
	{
		return this.m_upperAngle;
	},

	/// Set the joint limits in radians.
	SetLimits: function(lower, upper)
	{
		b2Assert(lower <= upper);

		if (lower != this.m_lowerAngle || upper != this.m_upperAngle)
		{
			this.m_bodyA.SetAwake(true);
			this.m_bodyB.SetAwake(true);
			this.m_impulse.z = 0.0;
			this.m_lowerAngle = lower;
			this.m_upperAngle = upper;
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

	/// Set the motor speed in radians per second.
	SetMotorSpeed: function(speed)
	{
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_motorSpeed = speed;
	},

	/// Get the motor speed in radians per second.
	GetMotorSpeed: function()
	{
		return this.m_motorSpeed;
	},

	/// Set the maximum motor torque, usually in N-m.
	SetMaxMotorTorque: function(torque)
	{
		this.m_bodyA.SetAwake(true);
		this.m_bodyB.SetAwake(true);
		this.m_maxMotorTorque = torque;
	},
	GetMaxMotorTorque: function() { return this.m_maxMotorTorque; },

	/// Get the reaction force given the inverse time step.
	/// Unit is N.
	GetReactionForce: function(inv_dt)
	{
		var P = new b2Vec2(this.m_impulse.x, this.m_impulse.y);
		return b2Vec2.Multiply(inv_dt, P);
	},

	/// Get the reaction torque due to the joint limit given the inverse time step.
	/// Unit is N*m.
	GetReactionTorque: function(inv_dt) { return inv_dt * this.m_impulse.z; },

	/// Get the current motor torque given the inverse time step.
	/// Unit is N*m.
	GetMotorTorque: function(inv_dt)
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

		var aA = data.positions[this.m_indexA].a;
		var vA = data.velocities[this.m_indexA].v.Clone();
		var wA = data.velocities[this.m_indexA].w;

		var aB = data.positions[this.m_indexB].a;
		var vB = data.velocities[this.m_indexB].v.Clone();
		var wB = data.velocities[this.m_indexB].w;

		var qA = new b2Rot(aA), qB = new b2Rot(aB);

		this.m_rA = b2Mul_r_v2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA));
		this.m_rB = b2Mul_r_v2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB));

		// J = [-I -r1_skew I r2_skew]
		//     [ 0       -1 0       1]
		// r_skew = [-ry; rx]

		// Matlab
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
		//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

		var mA = this.m_invMassA, mB = this.m_invMassB;
		var iA = this.m_invIA, iB = this.m_invIB;

		var fixedRotation = (iA + iB == 0.0);

		this.m_mass.ex.x = mA + mB + this.m_rA.y * this.m_rA.y * iA + this.m_rB.y * this.m_rB.y * iB;
		this.m_mass.ey.x = -this.m_rA.y * this.m_rA.x * iA - this.m_rB.y * this.m_rB.x * iB;
		this.m_mass.ez.x = -this.m_rA.y * iA - this.m_rB.y * iB;
		this.m_mass.ex.y = this.m_mass.ey.x;
		this.m_mass.ey.y = mA + mB + this.m_rA.x * this.m_rA.x * iA + this.m_rB.x * this.m_rB.x * iB;
		this.m_mass.ez.y = this.m_rA.x * iA + this.m_rB.x * iB;
		this.m_mass.ex.z = this.m_mass.ez.x;
		this.m_mass.ey.z = this.m_mass.ez.y;
		this.m_mass.ez.z = iA + iB;

		this.m_motorMass = iA + iB;
		if (this.m_motorMass > 0.0)
		{
			this.m_motorMass = 1.0 / this.m_motorMass;
		}

		if (this.m_enableMotor == false || fixedRotation)
		{
			this.m_motorImpulse = 0.0;
		}

		if (this.m_enableLimit && fixedRotation == false)
		{
			var jointAngle = aB - aA - this.m_referenceAngle;
			if (b2Abs(this.m_upperAngle - this.m_lowerAngle) < 2.0 * b2_angularSlop)
			{
				this.m_limitState = b2Joint.e_equalLimits;
			}
			else if (jointAngle <= this.m_lowerAngle)
			{
				if (this.m_limitState != b2Joint.e_atLowerLimit)
				{
					this.m_impulse.z = 0.0;
				}
				this.m_limitState = b2Joint.e_atLowerLimit;
			}
			else if (jointAngle >= this.m_upperAngle)
			{
				if (this.m_limitState != b2Joint.e_atUpperLimit)
				{
					this.m_impulse.z = 0.0;
				}
				this.m_limitState = b2Joint.e_atUpperLimit;
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
		}

		if (data.step.warmStarting)
		{
			// Scale impulses to support a variable time step.
			this.m_impulse.Multiply(data.step.dtRatio);
			this.m_motorImpulse *= data.step.dtRatio;

			var P = new b2Vec2(this.m_impulse.x, this.m_impulse.y);

			vA.Subtract(b2Vec2.Multiply(mA, P));
			wA -= iA * (b2Cross_v2_v2(this.m_rA, P) + this.m_motorImpulse + this.m_impulse.z);

			vB.Add(b2Vec2.Multiply(mB, P));
			wB += iB * (b2Cross_v2_v2(this.m_rB, P) + this.m_motorImpulse + this.m_impulse.z);
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

		var fixedRotation = (iA + iB == 0.0);

		// Solve motor constraint.
		if (this.m_enableMotor && this.m_limitState != b2Joint.e_equalLimits && fixedRotation == false)
		{
			var Cdot = wB - wA - this.m_motorSpeed;
			var impulse = -this.m_motorMass * Cdot;
			var oldImpulse = this.m_motorImpulse;
			var maxImpulse = data.step.dt * this.m_maxMotorTorque;
			this.m_motorImpulse = b2Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = this.m_motorImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		// Solve limit constraint.
		if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit && fixedRotation == false)
		{
			var Cdot1 = b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(vB, b2Cross_f_v2(wB, this.m_rB)), vA), b2Cross_f_v2(wA, this.m_rA));
			var Cdot2 = wB - wA;
			var Cdot = new b2Vec3(Cdot1.x, Cdot1.y, Cdot2);

			var impulse = this.m_mass.Solve33(Cdot).Negate();

			if (this.m_limitState == b2Joint.e_equalLimits)
			{
				this.m_impulse.Add(impulse);
			}
			else if (this.m_limitState == b2Joint.e_atLowerLimit)
			{
				var newImpulse = this.m_impulse.z + impulse.z;
				if (newImpulse < 0.0)
				{
					var rhs = b2Vec2.Add(Cdot1.Negate(), b2Vec2.Multiply(this.m_impulse.z, new b2Vec2(this.m_mass.ez.x, this.m_mass.ez.y)));
					var reduced = this.m_mass.Solve22(rhs);
					impulse.x = reduced.x;
					impulse.y = reduced.y;
					impulse.z = -this.m_impulse.z;
					this.m_impulse.x += reduced.x;
					this.m_impulse.y += reduced.y;
					this.m_impulse.z = 0.0;
				}
				else
				{
					this.m_impulse.Add(impulse);
				}
			}
			else if (this.m_limitState == b2Joint.e_atUpperLimit)
			{
				var newImpulse = this.m_impulse.z + impulse.z;
				if (newImpulse > 0.0)
				{
					var rhs = b2Vec2.Add(Cdot1.Negate(), b2Vec2.Multiply(this.m_impulse.z, new b2Vec2(this.m_mass.ez.x, this.m_mass.ez.y)));
					var reduced = this.m_mass.Solve22(rhs);
					impulse.x = reduced.x;
					impulse.y = reduced.y;
					impulse.z = -this.m_impulse.z;
					this.m_impulse.x += reduced.x;
					this.m_impulse.y += reduced.y;
					this.m_impulse.z = 0.0;
				}
				else
				{
					this.m_impulse.Add(impulse);
				}
			}

			var P = new b2Vec2(impulse.x, impulse.y);

			vA.Subtract(b2Vec2.Multiply(mA, P));
			wA -= iA * (b2Cross_v2_v2(this.m_rA, P) + impulse.z);

			vB.Add(b2Vec2.Multiply(mB, P));
			wB += iB * (b2Cross_v2_v2(this.m_rB, P) + impulse.z);
		}
		else
		{
			// Solve point-to-point constraint
			var Cdot = b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(vB, b2Cross_f_v2(wB, this.m_rB)), vA), b2Cross_f_v2(wA, this.m_rA));
			var impulse = this.m_mass.Solve22(Cdot.Negate());

			this.m_impulse.x += impulse.x;
			this.m_impulse.y += impulse.y;

			vA.Subtract(b2Vec2.Multiply(mA, impulse));
			wA -= iA * b2Cross_v2_v2(this.m_rA, impulse);

			vB.Add(b2Vec2.Multiply(mB, impulse));
			wB += iB * b2Cross_v2_v2(this.m_rB, impulse);
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

		var angularError = 0.0;
		var positionError = 0.0;

		var fixedRotation = (this.m_invIA + this.m_invIB == 0.0);

		// Solve angular limit constraint.
		if (this.m_enableLimit && this.m_limitState != b2Joint.e_inactiveLimit && fixedRotation == false)
		{
			var angle = aB - aA - this.m_referenceAngle;
			var limitImpulse = 0.0;

			if (this.m_limitState == b2Joint.e_equalLimits)
			{
				// Prevent large angular corrections
				var C = b2Clamp(angle - this.m_lowerAngle, -b2_maxAngularCorrection, b2_maxAngularCorrection);
				limitImpulse = -this.m_motorMass * C;
				angularError = b2Abs(C);
			}
			else if (this.m_limitState == b2Joint.e_atLowerLimit)
			{
				var C = angle - this.m_lowerAngle;
				angularError = -C;

				// Prevent large angular corrections and allow some slop.
				C = b2Clamp(C + b2_angularSlop, -b2_maxAngularCorrection, 0.0);
				limitImpulse = -this.m_motorMass * C;
			}
			else if (this.m_limitState == b2Joint.e_atUpperLimit)
			{
				var C = angle - this.m_upperAngle;
				angularError = C;

				// Prevent large angular corrections and allow some slop.
				C = b2Clamp(C - b2_angularSlop, 0.0, b2_maxAngularCorrection);
				limitImpulse = -this.m_motorMass * C;
			}

			aA -= this.m_invIA * limitImpulse;
			aB += this.m_invIB * limitImpulse;
		}

		// Solve point-to-point constraint.
		{
			qA.Set(aA);
			qB.Set(aB);
			var rA = b2Mul_r_v2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA));
			var rB = b2Mul_r_v2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB));

			var C = b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(cB, rB), cA), rA);
			positionError = C.Length();

			var mA = this.m_invMassA, mB = this.m_invMassB;
			var iA = this.m_invIA, iB = this.m_invIB;

			var K = new b2Mat22();
			K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
			K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
			K.ey.x = K.ex.y;
			K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;

			var impulse = K.Solve(C).Negate();

			cA.Subtract(b2Vec2.Multiply(mA, impulse));
			aA -= iA * b2Cross_v2_v2(rA, impulse);

			cB.Add(b2Vec2.Multiply(mB, impulse));
			aB += iB * b2Cross_v2_v2(rB, impulse);
		}

		data.positions[this.m_indexA].c.Assign(cA);
		data.positions[this.m_indexA].a = aA;
		data.positions[this.m_indexB].c.Assign(cB);
		data.positions[this.m_indexB].a = aB;

		return positionError <= b2_linearSlop && angularError <= b2_angularSlop;
	},

	_serialize: function(out)
	{
		var obj = out || {};

		this.parent.prototype._serialize.call(this, obj);

		obj['localAnchorA'] = this.m_localAnchorA._serialize();
		obj['localAnchorB'] = this.m_localAnchorB._serialize();
		obj['referenceAngle'] = this.m_referenceAngle;
		obj['lowerAngle'] = this.m_lowerAngle;
		obj['upperAngle'] = this.m_upperAngle;
		obj['maxMotorTorque'] = this.m_maxMotorTorque;
		obj['motorSpeed'] = this.m_motorSpeed;
		obj['enableLimit'] = this.m_enableLimit;
		obj['enableMotor'] = this.m_enableMotor;

		return obj;
	}
};

b2RevoluteJoint._extend(b2Joint);

