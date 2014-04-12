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

/// Weld joint definition. You need to specify local anchor points
/// where they are attached and the relative body angle. The position
/// of the anchor points is important for computing the reaction torque.
function b2WeldJointDef()
{
	this.parent.call(this);

	this.type = b2Joint.e_weldJoint;
	this.localAnchorA = new b2Vec2(0.0, 0.0);
	this.localAnchorB = new b2Vec2(0.0, 0.0);
	this.referenceAngle = 0.0;
	this.frequencyHz = 0.0;
	this.dampingRatio = 0.0;

	Object.seal(this);
}

b2WeldJointDef.prototype =
{
	/// Initialize the bodies, anchors, and reference angle using a world
	/// anchor point.
	Initialize: function(bA, bB, anchor)
	{
		this.bodyA = bA;
		this.bodyB = bB;
		this.localAnchorA.Assign(this.bodyA.GetLocalPoint(anchor));
		this.localAnchorB.Assign(this.bodyB.GetLocalPoint(anchor));
		this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
	},

	_deserialize: function(data, bodies, joints)
	{
		this.parent.prototype._deserialize.call(this, data, bodies, joints);

		this.localAnchorA._deserialize(data['localAnchorA']);
		this.localAnchorB._deserialize(data['localAnchorB']);
		this.referenceAngle = data['referenceAngle'];
		this.frequencyHz = data['frequencyHz'];
		this.dampingRatio = data['dampingRatio'];
	}
};

b2WeldJointDef._extend(b2JointDef);

/// A weld joint essentially glues two bodies together. A weld joint may
/// distort somewhat because the island constraint solver is approximate.
function b2WeldJoint(def)
{
	this.parent.call(this, def);

	this.m_bias = 0;

	// Solver shared
	this.m_gamma = 0;

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
	this.m_mass = new b2Mat33();

	this.m_localAnchorA = def.localAnchorA.Clone();
	this.m_localAnchorB = def.localAnchorB.Clone();
	this.m_referenceAngle = def.referenceAngle;
	this.m_frequencyHz = def.frequencyHz;
	this.m_dampingRatio = def.dampingRatio;

	this.m_impulse = new b2Vec3();
}

b2WeldJoint.prototype =
{
	GetAnchorA: function() { return this.m_bodyA.GetWorldPoint(this.m_localAnchorA); },
	GetAnchorB: function() { return this.m_bodyB.GetWorldPoint(this.m_localAnchorB); },

	GetReactionForce: function(inv_dt)
	{
		var P = new b2Vec2(this.m_impulse.x, this.m_impulse.y);
		return b2Vec2.Multiply(inv_dt, P);
	},
	GetReactionTorque: function(inv_dt) { return inv_dt * this.m_impulse.z; },

	/// The local anchor point relative to bodyA's origin.
	GetLocalAnchorA: function() { return this.m_localAnchorA; },

	/// The local anchor point relative to bodyB's origin.
	GetLocalAnchorB: function() { return this.m_localAnchorB; },

	/// Get the reference angle.
	GetReferenceAngle: function() { return this.m_referenceAngle; },

	/// Set/get frequency in Hz.
	SetFrequency: function(hz) { this.m_frequencyHz = hz; },
	GetFrequency: function() { return this.m_frequencyHz; },

	/// Set/get damping ratio.
	SetDampingRatio: function(ratio) { this.m_dampingRatio = ratio; },
	GetDampingRatio: function() { return this.m_dampingRatio; },

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

		var aA = data.positions[this.m_indexA].a;
		var vA = data.velocities[this.m_indexA].v.Clone();
		var wA = data.velocities[this.m_indexA].w;

		var aB = data.positions[this.m_indexB].a;
		var vB = data.velocities[this.m_indexB].v.Clone();
		var wB = data.velocities[this.m_indexB].w;

		var qA = new b2Rot(aA), qB = new b2Rot(aB);

		this.m_rA.Assign(b2Mul_r_v2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA)));
		this.m_rB.Assign(b2Mul_r_v2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB)));

		// J = [-I -r1_skew I r2_skew]
		//     [ 0       -1 0       1]
		// r_skew = [-ry; rx]

		// Matlab
		// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
		//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
		//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

		var mA = this.m_invMassA, mB = this.m_invMassB;
		var iA = this.m_invIA, iB = this.m_invIB;

		var K = new b2Mat33();
		K.ex.x = mA + mB + this.m_rA.y * this.m_rA.y * iA + this.m_rB.y * this.m_rB.y * iB;
		K.ey.x = -this.m_rA.y * this.m_rA.x * iA - this.m_rB.y * this.m_rB.x * iB;
		K.ez.x = -this.m_rA.y * iA - this.m_rB.y * iB;
		K.ex.y = K.ey.x;
		K.ey.y = mA + mB + this.m_rA.x * this.m_rA.x * iA + this.m_rB.x * this.m_rB.x * iB;
		K.ez.y = this.m_rA.x * iA + this.m_rB.x * iB;
		K.ex.z = K.ez.x;
		K.ey.z = K.ez.y;
		K.ez.z = iA + iB;

		if (this.m_frequencyHz > 0.0)
		{
			K.GetInverse22(this.m_mass);

			var invM = iA + iB;
			var m = invM > 0.0 ? 1.0 / invM : 0.0;

			var C = aB - aA - this.m_referenceAngle;

			// Frequency
			var omega = 2.0 * b2_pi * this.m_frequencyHz;

			// Damping coefficient
			var d = 2.0 * m * this.m_dampingRatio * omega;

			// Spring stiffness
			var k = m * omega * omega;

			// magic formulas
			var h = data.step.dt;
			this.m_gamma = h * (d + h * k);
			this.m_gamma = this.m_gamma != 0.0 ? 1.0 / this.m_gamma : 0.0;
			this.m_bias = C * h * k * this.m_gamma;

			invM += this.m_gamma;
			this.m_mass.ez.z = invM != 0.0 ? 1.0 / invM : 0.0;
		}
		else if (K.ez.z == 0.0)
		{
			K.GetInverse22(this.m_mass);
			this.m_gamma = 0.0;
			this.m_bias = 0.0;
		}
		else
		{
			K.GetSymInverse33(this.m_mass);
			this.m_gamma = 0.0;
			this.m_bias = 0.0;
		}

		if (data.step.warmStarting)
		{
			// Scale impulses to support a variable time step.
			this.m_impulse.Multiply(data.step.dtRatio);

			var P = new b2Vec2(this.m_impulse.x, this.m_impulse.y);

			vA.Subtract(b2Vec2.Multiply(mA, P));
			wA -= iA * (b2Cross_v2_v2(this.m_rA, P) + this.m_impulse.z);

			vB.Add(b2Vec2.Multiply(mB, P));
			wB += iB * (b2Cross_v2_v2(this.m_rB, P) + this.m_impulse.z);
		}
		else
		{
			this.m_impulse.SetZero();
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

		if (this.m_frequencyHz > 0.0)
		{
			var Cdot2 = wB - wA;

			var impulse2 = -this.m_mass.ez.z * (Cdot2 + this.m_bias + this.m_gamma * this.m_impulse.z);
			this.m_impulse.z += impulse2;

			wA -= iA * impulse2;
			wB += iB * impulse2;

			var Cdot1 = b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(vB, b2Cross_f_v2(wB, this.m_rB)), vA), b2Cross_f_v2(wA, this.m_rA));

			var impulse1 = b2Mul22_m33_v2(this.m_mass, Cdot1).Negate();
			this.m_impulse.x += impulse1.x;
			this.m_impulse.y += impulse1.y;

			var P = impulse1.Clone();

			vA.Subtract(b2Vec2.Multiply(mA, P));
			wA -= iA * b2Cross_v2_v2(this.m_rA, P);

			vB.Add(b2Vec2.Multiply(mB, P));
			wB += iB * b2Cross_v2_v2(this.m_rB, P);
		}
		else
		{
			var Cdot1 = b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(vB, b2Cross_f_v2(wB, this.m_rB)), vA), b2Cross_f_v2(wA, this.m_rA));
			var Cdot2 = wB - wA;
			var Cdot = new b2Vec3(Cdot1.x, Cdot1.y, Cdot2);

			var impulse = b2Mul_m33_v3(this.m_mass, Cdot).Negate();
			this.m_impulse.Add(impulse);

			var P = new b2Vec2(impulse.x, impulse.y);

			vA.Subtract(b2Vec2.Multiply(mA, P));
			wA -= iA * (b2Cross_v2_v2(this.m_rA, P) + impulse.z);

			vB.Add(b2Vec2.Multiply(mB, P));
			wB += iB * (b2Cross_v2_v2(this.m_rB, P) + impulse.z);
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

		var rA = b2Mul_r_v2(qA, b2Vec2.Subtract(this.m_localAnchorA, this.m_localCenterA));
		var rB = b2Mul_r_v2(qB, b2Vec2.Subtract(this.m_localAnchorB, this.m_localCenterB));

		var positionError, angularError;

		var K = new b2Mat33();
		K.ex.x = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
		K.ey.x = -rA.y * rA.x * iA - rB.y * rB.x * iB;
		K.ez.x = -rA.y * iA - rB.y * iB;
		K.ex.y = K.ey.x;
		K.ey.y = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
		K.ez.y = rA.x * iA + rB.x * iB;
		K.ex.z = K.ez.x;
		K.ey.z = K.ez.y;
		K.ez.z = iA + iB;

		if (this.m_frequencyHz > 0.0)
		{
			var C1 = b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(cB, rB), cA), rA);

			positionError = C1.Length();
			angularError = 0.0;

			var P = K.Solve22(C1).Negate();

			cA.Subtract(b2Vec2.Multiply(mA, P));
			aA -= iA * b2Cross_v2_v2(rA, P);

			cB.Add(b2Vec2.Multiply(mB, P));
			aB += iB * b2Cross_v2_v2(rB, P);
		}
		else
		{
			var C1 = b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(cB, rB), cA), rA);
			var C2 = aB - aA - this.m_referenceAngle;

			positionError = C1.Length();
			angularError = b2Abs(C2);

			var C = new b2Vec3(C1.x, C1.y, C2);

			var impulse;
			if (K.ez.z > 0.0)
			{
				impulse = K.Solve33(C).Invert();
			}
			else
			{
				var impulse2 = K.Solve22(C1).Invert();
				impulse = new b2Vec3(impulse2.x, impulse2.y, 0.0);
			}

			var P = new b2Vec2(impulse.x, impulse.y);

			cA.Subtract(b2Vec2.Multiply(mA, P));
			aA -= iA * (b2Cross_v2_v2(rA, P) + impulse.z);

			cB.Add(b2Vec2.Multiply(mB, P));
			aB += iB * (b2Cross_v2_v2(rB, P) + impulse.z);
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
		obj['frequencyHz'] = this.m_frequencyHz;
		obj['dampingRatio'] = this.m_dampingRatio;

		return obj;
	}
};

b2WeldJoint._extend(b2Joint);
