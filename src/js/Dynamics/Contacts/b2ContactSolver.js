function b2VelocityConstraintPoint()
{
	this.rA = new b2Vec2();
	this.rB = new b2Vec2();
	this.normalImpulse = 0;
	this.tangentImpulse = 0;
	this.normalMass = 0;
	this.tangentMass = 0;
	this.velocityBias = 0;
}

function b2ContactPositionConstraint()
{
	this.localPoints = new Array(b2_maxManifoldPoints);
	this.localNormal = new b2Vec2();
	this.localPoint = new b2Vec2();
	this.indexA = 0;
	this.indexB = 0;
	this.invMassA = 0, this.invMassB = 0;
	this.localCenterA = new b2Vec2(), this.localCenterB = new b2Vec2();
	this.invIA = 0, this.invIB = 0;
	this.type = 0;
	this.radiusA = 0, this.radiusB = 0;
	this.pointCount = 0;
};

function b2ContactVelocityConstraint()
{
	this.points = new Array(b2_maxManifoldPoints);

	for (var i = 0; i < this.points.length; ++i)
		this.points[i] = new b2VelocityConstraintPoint();

	this.normal = new b2Vec2();
	this.normalMass = new b2Mat22();
	this.K = new b2Mat22();
	this.indexA = 0;
	this.indexB = 0;
	this.invMassA = 0, this.invMassB = 0;
	this.invIA = 0, this.invIB = 0;
	this.friction = 0;
	this.restitution = 0;
	this.tangentSpeed = 0;
	this.pointCount = 0;
	this.contactIndex = 0;
}

function b2PositionSolverManifold()
{
	this.normal = new b2Vec2();
	this.point = new b2Vec2();
	this.separation = 0;
}

b2PositionSolverManifold.prototype =
{
	Initialize: function(pc, xfA, xfB, index)
	{
		b2Assert(pc.pointCount > 0);

		switch (pc.type)
		{
		case b2Manifold.e_circles:
			{
				var pointA = b2Mul_t_v2(xfA, pc.localPoint);
				var pointB = b2Mul_t_v2(xfB, pc.localPoints[0]);
				this.normal.Assign(b2Vec2.Subtract(pointB, pointA));
				this.normal.Normalize();
				this.point.Assign(b2Vec2.Multiply(0.5, b2Vec2.Add(pointA, pointB)));
				this.separation = b2Dot_v2_v2(b2Vec2.Subtract(pointB, pointA), this.normal) - pc.radiusA - pc.radiusB;
			}
			break;

		case b2Manifold.e_faceA:
			{
				this.normal.Assign(b2Mul_r_v2(xfA.q, pc.localNormal));
				var planePoint = b2Mul_t_v2(xfA, pc.localPoint);

				var clipPoint = b2Mul_t_v2(xfB, pc.localPoints[index]);
				this.separation = b2Dot_v2_v2(b2Vec2.Subtract(clipPoint, planePoint), this.normal) - pc.radiusA - pc.radiusB;
				this.point.Assign(clipPoint);
			}
			break;

		case b2Manifold.e_faceB:
			{
				this.normal = b2Mul_r_v2(xfB.q, pc.localNormal);
				var planePoint = b2Mul_t_v2(xfB, pc.localPoint);

				var clipPoint = b2Mul_t_v2(xfA, pc.localPoints[index]);
				this.separation = b2Dot_v2_v2(b2Vec2.Subtract(clipPoint, planePoint), this.normal) - pc.radiusA - pc.radiusB;
				this.point.Assign(clipPoint);

				// Ensure normal points from A to B
				this.normal.Assign(this.normal.Negate());
			}
			break;
		}
	}
};

function b2ContactSolverDef()
{
	this.step = new b2TimeStep();
	this.contacts = null;
	this.count = 0;
	this.positions = null;
	this.velocities = null;
}

function b2ContactSolver(def)
{
	this.m_step = def.step;
	this.m_count = def.count;
	this.m_positionConstraints = new Array(this.m_count);
	this.m_velocityConstraints = new Array(this.m_count);
	this.m_positions = def.positions;
	this.m_velocities = def.velocities;
	this.m_contacts = def.contacts;

	// Initialize position independent portions of the constraints.
	for (var i = 0; i < this.m_count; ++i)
	{
		var contact = this.m_contacts[i];

		var fixtureA = contact.m_fixtureA;
		var fixtureB = contact.m_fixtureB;
		var shapeA = fixtureA.GetShape();
		var shapeB = fixtureB.GetShape();
		var radiusA = shapeA.m_radius;
		var radiusB = shapeB.m_radius;
		var bodyA = fixtureA.GetBody();
		var bodyB = fixtureB.GetBody();
		var manifold = contact.GetManifold();

		var pointCount = manifold.pointCount;
		b2Assert(pointCount > 0);

		var vc = this.m_velocityConstraints[i] = new b2ContactVelocityConstraint();
		vc.friction = contact.m_friction;
		vc.restitution = contact.m_restitution;
		vc.tangentSpeed = contact.m_tangentSpeed;
		vc.indexA = bodyA.m_islandIndex;
		vc.indexB = bodyB.m_islandIndex;
		vc.invMassA = bodyA.m_invMass;
		vc.invMassB = bodyB.m_invMass;
		vc.invIA = bodyA.m_invI;
		vc.invIB = bodyB.m_invI;
		vc.contactIndex = i;
		vc.pointCount = pointCount;
		vc.K.SetZero();
		vc.normalMass.SetZero();

		var pc = this.m_positionConstraints[i] = new b2ContactPositionConstraint();
		pc.indexA = bodyA.m_islandIndex;
		pc.indexB = bodyB.m_islandIndex;
		pc.invMassA = bodyA.m_invMass;
		pc.invMassB = bodyB.m_invMass;
		pc.localCenterA.Assign(bodyA.m_sweep.localCenter);
		pc.localCenterB.Assign(bodyB.m_sweep.localCenter);
		pc.invIA = bodyA.m_invI;
		pc.invIB = bodyB.m_invI;
		pc.localNormal.Assign(manifold.localNormal);
		pc.localPoint.Assign(manifold.localPoint);
		pc.pointCount = pointCount;
		pc.radiusA = radiusA;
		pc.radiusB = radiusB;
		pc.type = manifold.type;

		for (var j = 0; j < pointCount; ++j)
		{
			var cp = manifold.points[j];
			var vcp = vc.points[j];

			if (this.m_step.warmStarting)
			{
				vcp.normalImpulse = this.m_step.dtRatio * cp.normalImpulse;
				vcp.tangentImpulse = this.m_step.dtRatio * cp.tangentImpulse;
			}
			else
			{
				vcp.normalImpulse = 0.0;
				vcp.tangentImpulse = 0.0;
			}

			vcp.rA.SetZero();
			vcp.rB.SetZero();
			vcp.normalMass = 0.0;
			vcp.tangentMass = 0.0;
			vcp.velocityBias = 0.0;

			pc.localPoints[j] = cp.localPoint;
		}
	}
}

b2ContactSolver.prototype =
{
	InitializeVelocityConstraints: function()
	{
		for (var i = 0; i < this.m_count; ++i)
		{
			var vc = this.m_velocityConstraints[i];
			var pc = this.m_positionConstraints[i];

			var radiusA = pc.radiusA;
			var radiusB = pc.radiusB;
			var manifold = this.m_contacts[vc.contactIndex].GetManifold();

			var indexA = vc.indexA;
			var indexB = vc.indexB;

			var mA = vc.invMassA;
			var mB = vc.invMassB;
			var iA = vc.invIA;
			var iB = vc.invIB;
			var localCenterA = pc.localCenterA;
			var localCenterB = pc.localCenterB;

			var cA = this.m_positions[indexA].c;
			var aA = this.m_positions[indexA].a;
			var vA = this.m_velocities[indexA].v;
			var wA = this.m_velocities[indexA].w;

			var cB = this.m_positions[indexB].c;
			var aB = this.m_positions[indexB].a;
			var vB = this.m_velocities[indexB].v;
			var wB = this.m_velocities[indexB].w;

			b2Assert(manifold.pointCount > 0);

			var xfA = new b2Transform(), xfB = new b2Transform();
			xfA.q.Set(aA);
			xfB.q.Set(aB);
			xfA.p.Assign(b2Vec2.Subtract(cA, b2Mul_r_v2(xfA.q, localCenterA)));
			xfB.p.Assign(b2Vec2.Subtract(cB, b2Mul_r_v2(xfB.q, localCenterB)));

			var worldManifold = new b2WorldManifold();
			worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB);

			vc.normal.Assign(worldManifold.normal);

			var pointCount = vc.pointCount;
			for (var j = 0; j < pointCount; ++j)
			{
				var vcp = vc.points[j];

				vcp.rA = b2Vec2.Subtract(worldManifold.points[j], cA);
				vcp.rB = b2Vec2.Subtract(worldManifold.points[j], cB);

				var rnA = b2Cross_v2_v2(vcp.rA, vc.normal);
				var rnB = b2Cross_v2_v2(vcp.rB, vc.normal);

				var kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

				vcp.normalMass = kNormal > 0.0 ? 1.0 / kNormal : 0.0;

				var tangent = b2Cross_v2_f(vc.normal, 1.0);

				var rtA = b2Cross_v2_v2(vcp.rA, tangent);
				var rtB = b2Cross_v2_v2(vcp.rB, tangent);

				var kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

				vcp.tangentMass = kTangent > 0.0 ? 1.0 /  kTangent : 0.0;

				// Setup a velocity bias for restitution.
				vcp.velocityBias = 0.0;
				var vRel = b2Dot_v2_v2(vc.normal, b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(vB, b2Cross_f_v2(wB, vcp.rB)), vA), b2Cross_f_v2(wA, vcp.rA)));
				if (vRel < -b2_velocityThreshold)
				{
					vcp.velocityBias = -vc.restitution * vRel;
				}
			}

			// If we have two points, then prepare the block solver.
			if (vc.pointCount == 2)
			{
				var vcp1 = vc.points[0];
				var vcp2 = vc.points[1];

				var rn1A = b2Cross_v2_v2(vcp1.rA, vc.normal);
				var rn1B = b2Cross_v2_v2(vcp1.rB, vc.normal);
				var rn2A = b2Cross_v2_v2(vcp2.rA, vc.normal);
				var rn2B = b2Cross_v2_v2(vcp2.rB, vc.normal);

				var k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
				var k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
				var k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

				// Ensure a reasonable condition number.
				var k_maxConditionNumber = 1000.0;
				if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
				{
					// K is safe to invert.
					vc.K.ex.Set(k11, k12);
					vc.K.ey.Set(k12, k22);
					vc.normalMass.Assign(vc.K.GetInverse());
				}
				else
				{
					// The constraints are redundant, just use one.
					// TODO_ERIN use deepest?
					vc.pointCount = 1;
				}
			}
		}
	},

	WarmStart: function()
	{
		// Warm start.
		for (var i = 0; i < this.m_count; ++i)
		{
			var vc = this.m_velocityConstraints[i];

			var indexA = vc.indexA;
			var indexB = vc.indexB;
			var mA = vc.invMassA;
			var iA = vc.invIA;
			var mB = vc.invMassB;
			var iB = vc.invIB;
			var pointCount = vc.pointCount;

			var vA = this.m_velocities[indexA].v;
			var wA = this.m_velocities[indexA].w;
			var vB = this.m_velocities[indexB].v;
			var wB = this.m_velocities[indexB].w;

			var normal = vc.normal;
			var tangent = b2Cross_v2_f(normal, 1.0);

			for (var j = 0; j < pointCount; ++j)
			{
				var vcp = vc.points[j];
				var P = b2Vec2.Add(b2Vec2.Multiply(vcp.normalImpulse, normal), b2Vec2.Multiply(vcp.tangentImpulse, tangent));
				wA -= iA * b2Cross_v2_v2(vcp.rA, P);
				vA.Subtract(b2Vec2.Multiply(mA, P));
				wB += iB * b2Cross_v2_v2(vcp.rB, P);
				vB.Add(b2Vec2.Multiply(mB, P));
			}

			this.m_velocities[indexA].v.Assign(vA);
			this.m_velocities[indexA].w = wA;
			this.m_velocities[indexB].v.Assign(vB);
			this.m_velocities[indexB].w = wB;
		}
	},
	SolveVelocityConstraints: function()
	{
		for (var i = 0; i < this.m_count; ++i)
		{
			var vc = this.m_velocityConstraints[i];

			var indexA = vc.indexA;
			var indexB = vc.indexB;
			var mA = vc.invMassA;
			var iA = vc.invIA;
			var mB = vc.invMassB;
			var iB = vc.invIB;
			var pointCount = vc.pointCount;

			var vA = this.m_velocities[indexA].v;
			var wA = this.m_velocities[indexA].w;
			var vB = this.m_velocities[indexB].v;
			var wB = this.m_velocities[indexB].w;

			var normal = vc.normal;
			var tangent = b2Cross_v2_f(normal, 1.0);
			var friction = vc.friction;

			b2Assert(pointCount == 1 || pointCount == 2);

			// Solve tangent constraints first because non-penetration is more important
			// than friction.
			for (var j = 0; j < pointCount; ++j)
			{
				var vcp = vc.points[j];

				// Relative velocity at contact
				var dv = b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(vB, b2Cross_f_v2(wB, vcp.rB)), vA), b2Cross_f_v2(wA, vcp.rA));

				// Compute tangent force
				var vt = b2Dot_v2_v2(dv, tangent) - vc.tangentSpeed;
				var lambda = vcp.tangentMass * (-vt);

				// b2Clamp the accumulated force
				var maxFriction = friction * vcp.normalImpulse;
				var newImpulse = b2Clamp(vcp.tangentImpulse + lambda, -maxFriction, maxFriction);
				lambda = newImpulse - vcp.tangentImpulse;
				vcp.tangentImpulse = newImpulse;

				// Apply contact impulse
				var P = b2Vec2.Multiply(lambda, tangent);

				vA.Subtract(b2Vec2.Multiply(mA, P));
				wA -= iA * b2Cross_v2_v2(vcp.rA, P);

				vB.Add(b2Vec2.Multiply(mB, P));
				wB += iB * b2Cross_v2_v2(vcp.rB, P);
			}

			// Solve normal constraints
			if (vc.pointCount == 1)
			{
				var vcp = vc.points[0];

				// Relative velocity at contact
				var dv = b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(vB, b2Cross_f_v2(wB, vcp.rB)), vA), b2Cross_f_v2(wA, vcp.rA));

				// Compute normal impulse
				var vn = b2Dot_v2_v2(dv, normal);
				var lambda = -vcp.normalMass * (vn - vcp.velocityBias);

				// b2Clamp the accumulated impulse
				var newImpulse = b2Max(vcp.normalImpulse + lambda, 0.0);
				lambda = newImpulse - vcp.normalImpulse;
				vcp.normalImpulse = newImpulse;

				// Apply contact impulse
				var P = b2Vec2.Multiply(lambda, normal);
				vA.Subtract(b2Vec2.Multiply(mA, P));
				wA -= iA * b2Cross_v2_v2(vcp.rA, P);

				vB.Add(b2Vec2.Multiply(mB, P));
				wB += iB * b2Cross_v2_v2(vcp.rB, P);
			}
			else
			{
				// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
				// Build the mini LCP for this contact patch
				//
				// vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
				//
				// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
				// b = vn0 - velocityBias
				//
				// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
				// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
				// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
				// solution that satisfies the problem is chosen.
				//
				// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
				// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
				//
				// Substitute:
				//
				// x = a + d
				//
				// a := old total impulse
				// x := new total impulse
				// d := incremental impulse
				//
				// For the current iteration we extend the formula for the incremental impulse
				// to compute the new total impulse:
				//
				// vn = A * d + b
				//    = A * (x - a) + b
				//    = A * x + b - A * a
				//    = A * x + b'
				// b' = b - A * a;

				var cp1 = vc.points[0];
				var cp2 = vc.points[1];

				var a = new b2Vec2(cp1.normalImpulse, cp2.normalImpulse);
				b2Assert(a.x >= 0.0 && a.y >= 0.0);

				// Relative velocity at contact
				var dv1 = b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(vB, b2Cross_f_v2(wB, cp1.rB)), vA), b2Cross_f_v2(wA, cp1.rA));
				var dv2 = b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(vB, b2Cross_f_v2(wB, cp2.rB)), vA), b2Cross_f_v2(wA, cp2.rA));

				// Compute normal velocity
				var vn1 = b2Dot_v2_v2(dv1, normal);
				var vn2 = b2Dot_v2_v2(dv2, normal);

				var b = new b2Vec2();
				b.x = vn1 - cp1.velocityBias;
				b.y = vn2 - cp2.velocityBias;

				// Compute b'
				b.Subtract(b2Mul_m22_v2(vc.K, a));

				var k_errorTol = 1e-3;

				for (;;)
				{
					//
					// Case 1: vn = 0
					//
					// 0 = A * x + b'
					//
					// Solve for x:
					//
					// x = - inv(A) * b'
					//
					var x = b2Mul_m22_v2(vc.normalMass, b).Negate();

					if (x.x >= 0.0 && x.y >= 0.0)
					{
						// Get the incremental impulse
						var d = b2Vec2.Subtract(x, a);

						// Apply incremental impulse
						var P1 = b2Vec2.Multiply(d.x, normal);
						var P2 = b2Vec2.Multiply(d.y, normal);
						vA.Subtract(b2Vec2.Multiply(mA, b2Vec2.Add(P1, P2)));
						wA -= iA * (b2Cross_v2_v2(cp1.rA, P1) + b2Cross_v2_v2(cp2.rA, P2));

						vB.Add(b2Vec2.Multiply(mB, b2Vec2.Add(P1, P2)));
						wB += iB * (b2Cross_v2_v2(cp1.rB, P1) + b2Cross_v2_v2(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

/*
	#if B2_DEBUG_SOLVER == 1
						// Postconditions
						dv1 = vB + b2Cross(wB, cp1.rB) - vA - b2Cross(wA, cp1.rA);
						dv2 = vB + b2Cross(wB, cp2.rB) - vA - b2Cross(wA, cp2.rA);

						// Compute normal velocity
						vn1 = b2Dot(dv1, normal);
						vn2 = b2Dot(dv2, normal);

						b2Assert(b2Abs(vn1 - cp1.velocityBias) < k_errorTol);
						b2Assert(b2Abs(vn2 - cp2.velocityBias) < k_errorTol);
	#endif
*/
						break;
					}

					//
					// Case 2: vn1 = 0 and x2 = 0
					//
					//   0 = a11 * x1 + a12 * 0 + b1'
					// vn2 = a21 * x1 + a22 * 0 + b2'
					//
					x.x = - cp1.normalMass * b.x;
					x.y = 0.0;
					vn1 = 0.0;
					vn2 = vc.K.ex.y * x.x + b.y;

					if (x.x >= 0.0 && vn2 >= 0.0)
					{
						// Get the incremental impulse
						var d = b2Vec2.Subtract(x, a);

						// Apply incremental impulse
						var P1 = b2Vec2.Multiply(d.x, normal);
						var P2 = b2Vec2.Multiply(d.y, normal);
						vA.Subtract(b2Vec2.Multiply(mA, b2Vec2.Add(P1, P2)));
						wA -= iA * (b2Cross_v2_v2(cp1.rA, P1) + b2Cross_v2_v2(cp2.rA, P2));

						vB.Add(b2Vec2.Multiply(mB, b2Vec2.Add(P1, P2)));
						wB += iB * (b2Cross_v2_v2(cp1.rB, P1) + b2Cross_v2_v2(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

/*	#if B2_DEBUG_SOLVER == 1
						// Postconditions
						dv1 = vB + b2Cross(wB, cp1.rB) - vA - b2Cross(wA, cp1.rA);

						// Compute normal velocity
						vn1 = b2Dot(dv1, normal);

						b2Assert(b2Abs(vn1 - cp1.velocityBias) < k_errorTol);
	#endif*/
						break;
					}


					//
					// Case 3: vn2 = 0 and x1 = 0
					//
					// vn1 = a11 * 0 + a12 * x2 + b1'
					//   0 = a21 * 0 + a22 * x2 + b2'
					//
					x.x = 0.0;
					x.y = - cp2.normalMass * b.y;
					vn1 = vc.K.ey.x * x.y + b.x;
					vn2 = 0.0;

					if (x.y >= 0.0 && vn1 >= 0.0)
					{
						// Resubstitute for the incremental impulse
						var d = b2Vec2.Subtract(x, a);

						// Apply incremental impulse
						var P1 = b2Vec2.Multiply(d.x, normal);
						var P2 = b2Vec2.Multiply(d.y, normal);
						vA.Subtract(b2Vec2.Multiply(mA, b2Vec2.Add(P1, P2)));
						wA -= iA * (b2Cross_v2_v2(cp1.rA, P1) + b2Cross_v2_v2(cp2.rA, P2));

						vB.Add(b2Vec2.Multiply(mB, b2Vec2.Add(P1, P2)));
						wB += iB * (b2Cross_v2_v2(cp1.rB, P1) + b2Cross_v2_v2(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

/*	#if B2_DEBUG_SOLVER == 1
						// Postconditions
						dv2 = vB + b2Cross(wB, cp2.rB) - vA - b2Cross(wA, cp2.rA);

						// Compute normal velocity
						vn2 = b2Dot(dv2, normal);

						b2Assert(b2Abs(vn2 - cp2.velocityBias) < k_errorTol);
	#endif*/
						break;
					}

					//
					// Case 4: x1 = 0 and x2 = 0
					//
					// vn1 = b1
					// vn2 = b2;
					x.x = 0.0;
					x.y = 0.0;
					vn1 = b.x;
					vn2 = b.y;

					if (vn1 >= 0.0 && vn2 >= 0.0 )
					{
						// Resubstitute for the incremental impulse
						var d = b2Vec2.Subtract(x, a);

						// Apply incremental impulse
						var P1 = b2Vec2.Multiply(d.x, normal);
						var P2 = b2Vec2.Multiply(d.y, normal);
						vA.Subtract(b2Vec2.Multiply(mA, b2Vec2.Add(P1, P2)));
						wA -= iA * (b2Cross_v2_v2(cp1.rA, P1) + b2Cross_v2_v2(cp2.rA, P2));

						vB.Add(b2Vec2.Multiply(mB, b2Vec2.Add(P1, P2)));
						wB += iB * (b2Cross_v2_v2(cp1.rB, P1) + b2Cross_v2_v2(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = x.x;
						cp2.normalImpulse = x.y;

						break;
					}

					// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
					break;
				}
			}

			this.m_velocities[indexA].w = wA;
			this.m_velocities[indexB].w = wB;
		}
	},
	StoreImpulses: function()
	{
		for (var i = 0; i < this.m_count; ++i)
		{
			var vc = this.m_velocityConstraints[i];
			var manifold = this.m_contacts[vc.contactIndex].GetManifold();

			for (var j = 0; j < vc.pointCount; ++j)
			{
				manifold.points[j].normalImpulse = vc.points[j].normalImpulse;
				manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse;
			}
		}
	},

	SolvePositionConstraints: function()
	{
		var minSeparation = 0.0;

		for (var i = 0; i < this.m_count; ++i)
		{
			var pc = this.m_positionConstraints[i];

			var indexA = pc.indexA;
			var indexB = pc.indexB;
			var localCenterA = pc.localCenterA;
			var mA = pc.invMassA;
			var iA = pc.invIA;
			var localCenterB = pc.localCenterB;
			var mB = pc.invMassB;
			var iB = pc.invIB;
			var pointCount = pc.pointCount;

			var cA = this.m_positions[indexA].c;
			var aA = this.m_positions[indexA].a;

			var cB = this.m_positions[indexB].c;
			var aB = this.m_positions[indexB].a;

			// Solve normal constraints
			for (var j = 0; j < pointCount; ++j)
			{
				var xfA = new b2Transform(), xfB = new b2Transform();
				xfA.q.Set(aA);
				xfB.q.Set(aB);
				xfA.p.Assign(b2Vec2.Subtract(cA, b2Mul_r_v2(xfA.q, localCenterA)));
				xfB.p.Assign(b2Vec2.Subtract(cB, b2Mul_r_v2(xfB.q, localCenterB)));

				var psm = new b2PositionSolverManifold();
				psm.Initialize(pc, xfA, xfB, j);
				var normal = psm.normal;

				var point = psm.point;
				var separation = psm.separation;

				var rA = b2Vec2.Subtract(point, cA);
				var rB = b2Vec2.Subtract(point, cB);

				// Track max constraint error.
				minSeparation = b2Min(minSeparation, separation);

				// Prevent large corrections and allow slop.
				var C = b2Clamp(b2_baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0);

				// Compute the effective mass.
				var rnA = b2Cross_v2_v2(rA, normal);
				var rnB = b2Cross_v2_v2(rB, normal);
				var K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

				// Compute normal impulse
				var impulse = K > 0.0 ? - C / K : 0.0;

				var P = b2Vec2.Multiply(impulse, normal);

				cA.Subtract(b2Vec2.Multiply(mA, P));
				aA -= iA * b2Cross_v2_v2(rA, P);

				cB.Add(b2Vec2.Multiply(mB, P));
				aB += iB * b2Cross_v2_v2(rB, P);
			}

			this.m_positions[indexA].a = aA;

			this.m_positions[indexB].a = aB;
		}

		// We can't expect minSpeparation >= -b2_linearSlop because we don't
		// push the separation above -b2_linearSlop.
		return minSeparation >= -3.0 * b2_linearSlop;
	},
	SolveTOIPositionConstraints: function(toiIndexA, toiIndexB)
	{
		var minSeparation = 0.0;

		for (var i = 0; i < this.m_count; ++i)
		{
			var pc = this.m_positionConstraints[i];

			var indexA = pc.indexA;
			var indexB = pc.indexB;
			var localCenterA = pc.localCenterA;
			var localCenterB = pc.localCenterB;
			var pointCount = pc.pointCount;

			var mA = 0.0;
			var iA = 0.0;
			if (indexA == toiIndexA || indexA == toiIndexB)
			{
				mA = pc.invMassA;
				iA = pc.invIA;
			}

			var mB = 0.0;
			var iB = 0.0;
			if (indexB == toiIndexA || indexB == toiIndexB)
			{
				mB = pc.invMassB;
				iB = pc.invIB;
			}

			var cA = this.m_positions[indexA].c;
			var aA = this.m_positions[indexA].a;

			var cB = this.m_positions[indexB].c;
			var aB = this.m_positions[indexB].a;

			// Solve normal constraints
			for (var j = 0; j < pointCount; ++j)
			{
				var xfA = new b2Transform(), xfB = new b2Transform();
				xfA.q.Set(aA);
				xfB.q.Set(aB);
				xfA.p.Assign(b2Vec2.Subtract(cA, b2Mul_r_v2(xfA.q, localCenterA)));
				xfB.p.Assign(b2Vec2.Subtract(cB, b2Mul_r_v2(xfB.q, localCenterB)));

				var psm = new b2PositionSolverManifold();
				psm.Initialize(pc, xfA, xfB, j);
				var normal = psm.normal;

				var point = psm.point;
				var separation = psm.separation;

				var rA = b2Vec2.Subtract(point, cA);
				var rB = b2Vec2.Subtract(point, cB);

				// Track max constraint error.
				minSeparation = b2Min(minSeparation, separation);

				// Prevent large corrections and allow slop.
				var C = b2Clamp(b2_toiBaugarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0);

				// Compute the effective mass.
				var rnA = b2Cross_v2_v2(rA, normal);
				var rnB = b2Cross_v2_v2(rB, normal);
				var K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

				// Compute normal impulse
				var impulse = K > 0.0 ? - C / K : 0.0;

				var P = b2Vec2.Multiply(impulse, normal);

				cA.Subtract(b2Vec2.Multiply(mA, P));
				aA -= iA * b2Cross_v2_v2(rA, P);

				cB.Add(b2Vec2.Multiply(mB, P));
				aB += iB * b2Cross_v2_v2(rB, P);
			}

			this.m_positions[indexA].a = aA;

			this.m_positions[indexB].a = aB;
		}

		// We can't expect minSpeparation >= -b2_linearSlop because we don't
		// push the separation above -b2_linearSlop.
		return minSeparation >= -1.5 * b2_linearSlop;
	}
};