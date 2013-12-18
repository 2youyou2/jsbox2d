/*
* Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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

function b2MotorJointDef()
{
	this.parent.call(this);

	this.type = b2Joint.e_motorJoint;
	this.linearOffset = new b2Vec2();
	this.angularOffset = 0.0;
	this.maxForce = 1.0;
	this.maxTorque = 1.0;
	this.correctionFactor = 0.3;

	Object.seal(this);
}

/// Motor joint definition.
b2MotorJointDef.prototype =
{
	/// Initialize the bodies and offsets using the current transforms.
	Initialize: function(bA, bB)
	{
		this.bodyA = bA;
		this.bodyB = bB;
		var xB = this.bodyB.GetPosition();
		this.linearOffset.Assign(this.bodyA.GetLocalPoint(xB));

		var angleA = this.bodyA.GetAngle();
		var angleB = this.bodyB.GetAngle();
		this.angularOffset = angleB - angleA;
	},

	_deserialize: function(data, bodies, joints)
	{
		this.parent.prototype._deserialize.call(this, data, bodies, joints);

		this.linearOffset._deserialize(data['linearOffset']);
		this.angularOffset = data['angularOffset'];
		this.maxForce = data['maxForce'];
		this.maxTorque = data['maxTorque'];
		this.correctionFactor = data['correctionFactor'];
	}
};

b2MotorJointDef._extend(b2JointDef);

/// A motor joint is used to control the relative motion
/// between two bodies. A typical usage is to control the movement
/// of a dynamic body with respect to the ground.
function b2MotorJoint(def)
{
	this.parent.call(this, def);

	this.m_linearOffset = def.linearOffset.Clone();
	this.m_angularOffset = def.angularOffset;

	this.m_linearImpulse = new b2Vec2();
	this.m_angularImpulse = 0.0;

	this.m_maxForce = def.maxForce;
	this.m_maxTorque = def.maxTorque;
	this.m_correctionFactor = def.correctionFactor;

	// Solver temp
	this.m_indexA = 0;
	this.m_indexB = 0;
	this.m_rA = new b2Vec2();
	this.m_rB = new b2Vec2();
	this.m_localCenterA = new b2Vec2();
	this.m_localCenterB = new b2Vec2();
	this.m_linearError = new b2Vec2();
	this.m_angularError = 0;
	this.m_invMassA = 0;
	this.m_invMassB = 0;
	this.m_invIA = 0;
	this.m_invIB = 0;
	this.m_linearMass = new b2Mat22();
	this.m_angularMass = 0;
}

b2MotorJoint.prototype =
{
	GetAnchorA: function()
	{
		return this.m_bodyA.GetPosition();
	},
	GetAnchorB: function()
	{
		return this.m_bodyB.GetPosition();
	},

	GetReactionForce: function(inv_dt)
	{
		return b2Vec2.Multiply(inv_dt, this.m_linearImpulse);
	},
	GetReactionTorque: function(inv_dt)
	{
		return inv_dt * this.m_angularImpulse;
	},

	/// Set/get the target linear offset, in frame A, in meters.
	SetLinearOffset: function(linearOffset)
	{
		if (linearOffset.x != this.m_linearOffset.x || linearOffset.y != this.m_linearOffset.y)
		{
			this.m_bodyA.SetAwake(true);
			this.m_bodyB.SetAwake(true);
			this.m_linearOffset.Assign(linearOffset);
		}
	},
	GetLinearOffset: function()
	{
		return this.m_linearOffset;
	},

	/// Set/get the target angular offset, in radians.
	SetAngularOffset: function(angularOffset)
	{
		if (angularOffset != this.m_angularOffset)
		{
			this.m_bodyA.SetAwake(true);
			this.m_bodyB.SetAwake(true);
			this.m_angularOffset = angularOffset;
		}
	},
	GetAngularOffset: function()
	{
		return this.m_angularOffset;
	},

	/// Set the maximum friction force in N.
	SetMaxForce: function(force)
	{
'#if @DEBUG';
		b2Assert(b2IsValid(force) && force >= 0.0);
'#endif';
		this.m_maxForce = force;
	},

	/// Get the maximum friction force in N.
	GetMaxForce: function()
	{
		return this.m_maxForce;
	},

	/// Set the maximum friction torque in N*m.
	SetMaxTorque: function(torque)
	{
'#if @DEBUG';
		b2Assert(b2IsValid(torque) && torque >= 0.0);
'#endif';
		this.m_maxTorque = torque;
	},

	/// Get the maximum friction torque in N*m.
	GetMaxTorque: function()
	{
		return this.m_maxTorque;
	},

	/// Set the position correction factor in the range [0,1].
	SetCorrectionFactor: function(factor)
	{
'#if @DEBUG';
		b2Assert(b2IsValid(factor) && 0.0 <= factor && factor <= 1.0);
'#endif';
		this.m_correctionFactor = factor;
	},

	/// Get the position correction factor in the range [0,1].
	GetCorrectionFactor: function()
	{
		return this.m_correctionFactor;
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

		// Compute the effective mass matrix.
		this.m_rA.Assign(b2Mul_r_v2(qA, this.m_localCenterA.Negate()));
		this.m_rB.Assign(b2Mul_r_v2(qB, this.m_localCenterB.Negate()));

		// J = [-I -r1_skew I r2_skew]
		//     [ 0       -1 0       1]
		// r_skew = [-ry; rx]

		// Matlab
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
		//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

		var mA = this.m_invMassA, mB = this.m_invMassB;
		var iA = this.m_invIA, iB = this.m_invIB;

		var K = new b2Mat22();
		K.ex.x = mA + mB + iA * this.m_rA.y * this.m_rA.y + iB * this.m_rB.y * this.m_rB.y;
		K.ex.y = -iA * this.m_rA.x * this.m_rA.y - iB * this.m_rB.x * this.m_rB.y;
		K.ey.x = K.ex.y;
		K.ey.y = mA + mB + iA * this.m_rA.x * this.m_rA.x + iB * this.m_rB.x * this.m_rB.x;

		this.m_linearMass.Assign(K.GetInverse());

		this.m_angularMass = iA + iB;
		if (this.m_angularMass > 0.0)
		{
			this.m_angularMass = 1.0 / this.m_angularMass;
		}

		this.m_linearError.Assign(b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(cB, this.m_rB), cA), this.m_rA), b2Mul_r_v2(qA, this.m_linearOffset)));
		this.m_angularError = aB - aA - this.m_angularOffset;

		if (data.step.warmStarting)
		{
			// Scale impulses to support a variable time step.
			this.m_linearImpulse.Multiply(data.step.dtRatio);
			this.m_angularImpulse *= data.step.dtRatio;

			var P = new b2Vec2(this.m_linearImpulse.x, this.m_linearImpulse.y);
			vA.Subtract(b2Vec2.Multiply(mA, P));
			wA -= iA * (b2Cross_v2_v2(this.m_rA, P) + this.m_angularImpulse);
			vB.Add(b2Vec2.Multiply(mB, P));
			wB += iB * (b2Cross_v2_v2(this.m_rB, P) + this.m_angularImpulse);
		}
		else
		{
			this.m_linearImpulse.SetZero();
			this.m_angularImpulse = 0.0;
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

		var h = data.step.dt;
		var inv_h = data.step.inv_dt;

		// Solve angular friction
		{
			var Cdot = wB - wA + inv_h * this.m_correctionFactor * this.m_angularError;
			var impulse = -this.m_angularMass * Cdot;

			var oldImpulse = this.m_angularImpulse;
			var maxImpulse = h * this.m_maxTorque;
			this.m_angularImpulse = b2Clamp(this.m_angularImpulse + impulse, -maxImpulse, maxImpulse);
			impulse = this.m_angularImpulse - oldImpulse;

			wA -= iA * impulse;
			wB += iB * impulse;
		}

		// Solve linear friction
		{
			var Cdot = b2Vec2.Add(b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(vB, b2Cross_f_v2(wB, this.m_rB)), vA), b2Cross_f_v2(wA, this.m_rA)), b2Vec2.Multiply(inv_h, b2Vec2.Multiply(this.m_correctionFactor, this.m_linearError)));

			var impulse = b2Mul_m22_v2(this.m_linearMass, Cdot).Negate();
			var oldImpulse = this.m_linearImpulse;
			this.m_linearImpulse.Add(impulse);

			var maxImpulse = h * this.m_maxForce;

			if (this.m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse)
			{
				this.m_linearImpulse.Normalize();
				this.m_linearImpulse.Multiply(maxImpulse);
			}

			impulse.Assign(b2Vec2.Subtract(this.m_linearImpulse, oldImpulse));

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
		return true;
	},

	_serialize: function(out)
	{
		var obj = out || {};

		this.parent.prototype._serialize.call(this, obj);

		obj['linearOffset'] = this.m_linearOffset._serialize();
		obj['angularOffset'] = this.m_angularOffset;
		obj['maxForce'] = this.m_maxForce;
		obj['maxTorque'] = this.m_maxTorque;
		obj['correctionFactor'] = this.m_correctionFactor;

		return obj;
	}
};

b2MotorJoint._extend(b2Joint);