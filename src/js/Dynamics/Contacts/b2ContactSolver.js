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
'#if @DEBUG';
		b2Assert(pc.pointCount > 0);
'#endif';

		switch (pc.type)
		{
		case b2Manifold.e_circles:
			{
				var pointAx = (xfA.q.c * pc.localPoint.x - xfA.q.s * pc.localPoint.y) + xfA.p.x;
				var pointAy = (xfA.q.s * pc.localPoint.x + xfA.q.c * pc.localPoint.y) + xfA.p.y;//b2Mul_t_v2(xfA, pc.localPoint);
				var pointBx = (xfB.q.c * pc.localPoints[0].x - xfB.q.s * pc.localPoints[0].y) + xfB.p.x;
				var pointBy = (xfB.q.s * pc.localPoints[0].x + xfB.q.c * pc.localPoints[0].y) + xfB.p.y;//b2Mul_t_v2(xfB, pc.localPoints[0]);
				this.point.x = 0.5 * (pointAx + pointBx); //.Assign(b2Vec2.Multiply(0.5, b2Vec2.Add(pointA, pointB)));
				this.point.y = 0.5 * (pointAy + pointBy);
				this.normal.x = pointBx - pointAx; //Assign(b2Vec2.Subtract(pointB, pointA));
				this.normal.y = pointBy - pointAy;
				var tempnx = this.normal.x;
				var tempny = this.normal.y;
				this.normal.Normalize();
				this.separation = /*b2Dot_v2_v2(b2Vec2.Subtract(pointB, pointA), this.normal)*/ (tempnx * this.normal.x + tempny * this.normal.y) - pc.radiusA - pc.radiusB;
			}
			break;

		case b2Manifold.e_faceA:
			{
				//this.normal.Assign(b2Mul_r_v2(xfA.q, pc.localNormal));
				this.normal.x = xfA.q.c * pc.localNormal.x - xfA.q.s * pc.localNormal.y;
				this.normal.y = xfA.q.s * pc.localNormal.x + xfA.q.c * pc.localNormal.y;

				//var planePoint = b2Mul_t_v2(xfA, pc.localPoint);
				var planePointx = (xfA.q.c * pc.localPoint.x - xfA.q.s * pc.localPoint.y) + xfA.p.x;
				var planePointy = (xfA.q.s * pc.localPoint.x + xfA.q.c * pc.localPoint.y) + xfA.p.y;

				//var clipPoint = b2Mul_t_v2(xfB, pc.localPoints[index]);
				var clipPointx = (xfB.q.c * pc.localPoints[index].x - xfB.q.s * pc.localPoints[index].y) + xfB.p.x;
				var clipPointy = (xfB.q.s * pc.localPoints[index].x + xfB.q.c * pc.localPoints[index].y) + xfB.p.y;
				this.separation = ((clipPointx - planePointx) * this.normal.x + (clipPointy - planePointy) * this.normal.y) /*b2Dot_v2_v2(b2Vec2.Subtract(clipPoint, planePoint), this.normal)*/ - pc.radiusA - pc.radiusB;

				this.point.x = clipPointx;
				this.point.y = clipPointy;
			}
			break;

		case b2Manifold.e_faceB:
			{
				//this.normal.Assign(b2Mul_r_v2(xfB.q, pc.localNormal));
				this.normal.x = xfB.q.c * pc.localNormal.x - xfB.q.s * pc.localNormal.y;
				this.normal.y = xfB.q.s * pc.localNormal.x + xfB.q.c * pc.localNormal.y;

				//var planePoint = b2Mul_t_v2(xfB, pc.localPoint);
				var planePointx = (xfB.q.c * pc.localPoint.x - xfB.q.s * pc.localPoint.y) + xfB.p.x;
				var planePointy = (xfB.q.s * pc.localPoint.x + xfB.q.c * pc.localPoint.y) + xfB.p.y;

				//var clipPoint = b2Mul_t_v2(xfA, pc.localPoints[index]);
				var clipPointx = (xfA.q.c * pc.localPoints[index].x - xfA.q.s * pc.localPoints[index].y) + xfA.p.x;
				var clipPointy = (xfA.q.s * pc.localPoints[index].x + xfA.q.c * pc.localPoints[index].y) + xfA.p.y;
				this.separation = ((clipPointx - planePointx) * this.normal.x + (clipPointy - planePointy) * this.normal.y) /*b2Dot_v2_v2(b2Vec2.Subtract(clipPoint, planePoint), this.normal)*/ - pc.radiusA - pc.radiusB;

				this.point.x = clipPointx;
				this.point.y = clipPointy;

				// Ensure normal points from A to B
				//this.normal.Assign(this.normal.Negate());
				this.normal.x = -this.normal.x;
				this.normal.y = -this.normal.y;
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

function b2ContactSolver()
{
	this.m_positionConstraints = [];
	this.m_velocityConstraints = [];
}

b2ContactSolver.cs_xfA = new b2Transform();
b2ContactSolver.cs_xfB = new b2Transform();

b2ContactSolver.temp_solver_manifold = new b2PositionSolverManifold();

b2ContactSolver.prototype =
{
	Init: function(def)
	{
		this.m_step = def.step;
		this.m_count = def.count;
		this.m_positionConstraints.length = this.m_count;
		this.m_velocityConstraints.length = this.m_count;
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
'#if @DEBUG';
			b2Assert(pointCount > 0);
'#endif';

			var vc = this.m_velocityConstraints[i] || new b2ContactVelocityConstraint();
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
			this.m_velocityConstraints[i] = vc;

			var pc = this.m_positionConstraints[i] || new b2ContactPositionConstraint();
			pc.indexA = bodyA.m_islandIndex;
			pc.indexB = bodyB.m_islandIndex;
			pc.invMassA = bodyA.m_invMass;
			pc.invMassB = bodyB.m_invMass;
			pc.localCenterA.x = bodyA.m_sweep.localCenter.x;
			pc.localCenterA.y = bodyA.m_sweep.localCenter.y;
			pc.localCenterB.x = bodyB.m_sweep.localCenter.x;
			pc.localCenterB.y = bodyB.m_sweep.localCenter.y;
			pc.invIA = bodyA.m_invI;
			pc.invIB = bodyB.m_invI;
			pc.localNormal.x = manifold.localNormal.x;
			pc.localNormal.y = manifold.localNormal.y;
			pc.localPoint.x = manifold.localPoint.x;
			pc.localPoint.y = manifold.localPoint.y;
			pc.pointCount = pointCount;
			pc.radiusA = radiusA;
			pc.radiusB = radiusB;
			pc.type = manifold.type;
			this.m_positionConstraints[i] = pc;

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
	},

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

'#if @DEBUG';
			b2Assert(manifold.pointCount > 0);
'#endif';

			b2ContactSolver.cs_xfA.q.Set(aA);
			b2ContactSolver.cs_xfB.q.Set(aB);
			b2ContactSolver.cs_xfA.p.x = cA.x - (b2ContactSolver.cs_xfA.q.c * localCenterA.x - b2ContactSolver.cs_xfA.q.s * localCenterA.y);//Assign(b2Vec2.Subtract(cA, b2Mul_r_v2(b2ContactSolver.cs_xfA.q, localCenterA)));
			b2ContactSolver.cs_xfA.p.y = cA.y - (b2ContactSolver.cs_xfA.q.s * localCenterA.x + b2ContactSolver.cs_xfA.q.c * localCenterA.y);
			b2ContactSolver.cs_xfB.p.x = cB.x - (b2ContactSolver.cs_xfB.q.c * localCenterB.x - b2ContactSolver.cs_xfB.q.s * localCenterB.y);//.Assign(b2Vec2.Subtract(cB, b2Mul_r_v2(b2ContactSolver.cs_xfB.q, localCenterB)));
			b2ContactSolver.cs_xfB.p.y = cB.y - (b2ContactSolver.cs_xfB.q.s * localCenterB.x + b2ContactSolver.cs_xfB.q.c * localCenterB.y);

			var worldManifold = new b2WorldManifold();
			worldManifold.Initialize(manifold, b2ContactSolver.cs_xfA, radiusA, b2ContactSolver.cs_xfB, radiusB);

			vc.normal.x = worldManifold.normal.x;//.Assign(worldManifold.normal);
			vc.normal.y = worldManifold.normal.y;

			var pointCount = vc.pointCount;
			for (var j = 0; j < pointCount; ++j)
			{
				var vcp = vc.points[j];

				vcp.rA.x = worldManifold.points[j].x - cA.x;// = b2Vec2.Subtract(worldManifold.points[j], cA);
				vcp.rA.y = worldManifold.points[j].y - cA.y;
				vcp.rB.x = worldManifold.points[j].x - cB.x;//b2Vec2.Subtract(worldManifold.points[j], cB);
				vcp.rB.y = worldManifold.points[j].y - cB.y;

				var rnA = vcp.rA.x * vc.normal.y - vcp.rA.y * vc.normal.x;//b2Cross_v2_v2(vcp.rA, vc.normal);
				var rnB = vcp.rB.x * vc.normal.y - vcp.rB.y * vc.normal.x;//b2Cross_v2_v2(vcp.rB, vc.normal);

				var kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

				vcp.normalMass = kNormal > 0.0 ? 1.0 / kNormal : 0.0;

				var tangentx = 1.0 * vc.normal.y;//b2Cross_v2_f(vc.normal, 1.0);
				var tangenty = -1.0 * vc.normal.x;

				var rtA = vcp.rA.x * tangenty - vcp.rA.y * tangentx;//b2Cross_v2_v2(vcp.rA, tangent);
				var rtB = vcp.rB.x * tangenty - vcp.rB.y * tangentx;//b2Cross_v2_v2(vcp.rB, tangent);

				var kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

				vcp.tangentMass = kTangent > 0.0 ? 1.0 /  kTangent : 0.0;

				// Setup a velocity bias for restitution.
				vcp.velocityBias = 0.0;
				//var tax = -wB * vcp.rB.y;//b2Cross_f_v2(wB, vcp.rB);
				//var tay = wB * vcp.rB.x;
				//var tbx = -wA * vcp.rA.y;//b2Cross_f_v2(wA, vcp.rA);
				//var tby = wA * vcp.rA.x;

				//var vRel = b2Dot_v2_v2(vc.normal, b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(vB, ta), vA), tb));
				var vRel = vc.normal.x * (((vB.x + (-wB * vcp.rB.y)) - vA.x) - (-wA * vcp.rA.y)) + vc.normal.y * (((vB.y + (wB * vcp.rB.x)) - vA.y) - (wA * vcp.rA.x));
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

				var rn1A = vcp1.rA.x * vc.normal.y - vcp1.rA.y * vc.normal.x;//b2Cross_v2_v2(vcp1.rA, vc.normal);
				var rn1B = vcp1.rB.x * vc.normal.y - vcp1.rB.y * vc.normal.x;//b2Cross_v2_v2(vcp1.rB, vc.normal);
				var rn2A = vcp2.rA.x * vc.normal.y - vcp2.rA.y * vc.normal.x;//b2Cross_v2_v2(vcp2.rA, vc.normal);
				var rn2B = vcp2.rB.x * vc.normal.y - vcp2.rB.y * vc.normal.x;//b2Cross_v2_v2(vcp2.rB, vc.normal);

				var k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
				var k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
				var k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

				// Ensure a reasonable condition number.
				var k_maxConditionNumber = 1000.0;
				if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
				{
					// K is safe to invert.
					vc.K.ex.x = k11;//.Set(k11, k12);
					vc.K.ex.y = k12;
					vc.K.ey.x = k12;//.Set(k12, k22);
					vc.K.ey.y = k22;
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
			var tangentx = 1.0 * normal.y;
			var tangenty = -1.0 * normal.x;//b2Cross_v2_f(normal, 1.0);

			for (var j = 0; j < pointCount; ++j)
			{
				var vcp = vc.points[j];
				var Px = (vcp.normalImpulse * normal.x) + (vcp.tangentImpulse * tangentx);
				var Py = (vcp.normalImpulse * normal.y) + (vcp.tangentImpulse * tangenty);
					//b2Vec2.Add(b2Vec2.Multiply(vcp.normalImpulse, normal), b2Vec2.Multiply(vcp.tangentImpulse, tangent));
				wA -= iA * (vcp.rA.x * Py - vcp.rA.y * Px);//b2Cross_v2_v2(vcp.rA, P);
				//vA.Subtract(b2Vec2.Multiply(mA, P));
				vA.x -= mA * Px;
				vA.y -= mA * Py;
				wB += iB * (vcp.rB.x * Py - vcp.rB.y * Px);//b2Cross_v2_v2(vcp.rB, P);
				//vB.Add(b2Vec2.Multiply(mB, P));
				vB.x += mB * Px;
				vB.y += mB * Py;
			}

			this.m_velocities[indexA].w = wA;
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
			var tangentx = 1.0 * normal.y;//b2Cross_v2_f(normal, 1.0);
			var tangenty = -1.0 * normal.x;
			var friction = vc.friction;

'#if @DEBUG';
			b2Assert(pointCount == 1 || pointCount == 2);
'#endif';

			// Solve tangent constraints first because non-penetration is more important
			// than friction.
			for (var j = 0; j < pointCount; ++j)
			{
				var vcp = vc.points[j];

				// Relative velocity at contact
				var dvx = vB.x + (-wB * vcp.rB.y) - vA.x - (-wA * vcp.rA.y);
				var dvy = vB.y + (wB * vcp.rB.x) - vA.y - (wA * vcp.rA.x);

				//b2Assert(b2Vec2.Equals(dv, b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(vB, b2Cross_f_v2(wB, vcp.rB)), vA), b2Cross_f_v2(wA, vcp.rA))));

				// Compute tangent force
				var vt = (dvx * tangentx + dvy * tangenty) /*b2Dot_v2_v2(dv, tangent)*/ - vc.tangentSpeed;
				var lambda = vcp.tangentMass * (-vt);

				// b2Clamp the accumulated force
				var maxFriction = friction * vcp.normalImpulse;
				var newImpulse = b2Clamp(vcp.tangentImpulse + lambda, -maxFriction, maxFriction);
				lambda = newImpulse - vcp.tangentImpulse;
				vcp.tangentImpulse = newImpulse;

				// Apply contact impulse
				var Px = lambda * tangentx;
				var Py = lambda * tangenty;

				//b2Assert(b2Vec2.Equals(P, b2Vec2.Multiply(lambda, tangent)));

				//vA.Subtract(b2Vec2.Multiply(mA, P));
				vA.x -= mA * Px;
				vA.y -= mA * Py;
				wA -= iA * (vcp.rA.x * Py - vcp.rA.y * Px);//b2Cross_v2_v2(vcp.rA, P);

				//vB.Add(b2Vec2.Multiply(mB, P));
				vB.x += mB * Px;
				vB.y += mB * Py;
				wB += iB * (vcp.rB.x * Py - vcp.rB.y * Px);//b2Cross_v2_v2(vcp.rB, P);
			}

			// Solve normal constraints
			if (vc.pointCount == 1)
			{
				vcp = vc.points[0];

				// Relative velocity at contact
				dvx = vB.x + (-wB * vcp.rB.y) - vA.x - (-wA * vcp.rA.y);
				dvy = vB.y + (wB * vcp.rB.x) - vA.y - (wA * vcp.rA.x);

				//b2Assert(b2Vec2.Equals(dv, b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(vB, b2Cross_f_v2(wB, vcp.rB)), vA), b2Cross_f_v2(wA, vcp.rA))));

				// Compute normal impulse
				var vn = dvx * normal.x + dvy * normal.y;//b2Dot_v2_v2(dv, normal);
				var lambda = -vcp.normalMass * (vn - vcp.velocityBias);

				// b2Clamp the accumulated impulse
				var newImpulse = b2Max(vcp.normalImpulse + lambda, 0.0);
				lambda = newImpulse - vcp.normalImpulse;
				vcp.normalImpulse = newImpulse;

				// Apply contact impulse
				Px = lambda * normal.x;
				Py = lambda * normal.y;//b2Vec2.Multiply(lambda, normal);
				//vA.Subtract(b2Vec2.Multiply(mA, P));
				vA.x -= mA * Px;
				vA.y -= mA * Py;
				wA -= iA * (vcp.rA.x * Py - vcp.rA.y * Px);//b2Cross_v2_v2(vcp.rA, P);

				//vB.Add(b2Vec2.Multiply(mB, P));
				vB.x += mB * Px;
				vB.y += mB * Py;
				wB += iB * (vcp.rB.x * Py - vcp.rB.y * Px);//b2Cross_v2_v2(vcp.rB, P);
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

				var ax = cp1.normalImpulse;
				var ay = cp2.normalImpulse;
'#if @DEBUG';
				b2Assert(ax >= 0.0 && ay >= 0.0);
'#endif';

				// Relative velocity at contact
				var dv1x = vB.x + (-wB * cp1.rB.y) - vA.x - (-wA * cp1.rA.y);
				var dv1y = vB.y + (wB * cp1.rB.x) - vA.y - (wA * cp1.rA.x);

				//b2Assert(b2Vec2.Equals(dv1, b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(vB, b2Cross_f_v2(wB, cp1.rB)), vA), b2Cross_f_v2(wA, cp1.rA))));
				var dv2x = vB.x + (-wB * cp2.rB.y) - vA.x - (-wA * cp2.rA.y);
				var dv2y = vB.y + (wB * cp2.rB.x) - vA.y - (wA * cp2.rA.x);
				//b2Assert(b2Vec2.Equals(dv2, b2Vec2.Subtract(b2Vec2.Subtract(b2Vec2.Add(vB, b2Cross_f_v2(wB, cp2.rB)), vA), b2Cross_f_v2(wA, cp2.rA))));

				// Compute normal velocity
				var vn1 = dv1x * normal.x + dv1y * normal.y;//b2Dot_v2_v2(dv1, normal);
				var vn2 = dv2x * normal.x + dv2y * normal.y;//b2Dot_v2_v2(dv2, normal);

				var bx = vn1 - cp1.velocityBias;
				var by = vn2 - cp2.velocityBias;

				// Compute b'
				//b.Subtract(b2Mul_m22_v2(vc.K, a));
				bx -= vc.K.ex.x * ax + vc.K.ey.x * ay;
				by -= vc.K.ex.y * ax + vc.K.ey.y * ay;

				//var k_errorTol = 1e-3;

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
					var xx = -(vc.normalMass.ex.x * bx + vc.normalMass.ey.x * by);
					var xy = -(vc.normalMass.ex.y * bx + vc.normalMass.ey.y * by);
						//b2Mul_m22_v2(vc.normalMass, b).Negate();

					if (xx >= 0.0 && xy >= 0.0)
					{
						// Get the incremental impulse
						var dx = xx - ax;
						var dy = xy - ay;//b2Vec2.Subtract(x, a);

						// Apply incremental impulse
						var P1x = dx * normal.x;
						var P1y = dx * normal.y;
						var P2x = dy * normal.x;
						var P2y = dy * normal.y;
						//vA.Subtract(b2Vec2.Multiply(mA, b2Vec2.Add(P1, P2)));
						vA.x -= mA * (P1x + P2x);
						vA.y -= mA * (P1y + P2y);
						wA -= iA * ((cp1.rA.x * P1y - cp1.rA.y * P1x) + (cp2.rA.x * P2y - cp2.rA.y * P2x));//(b2Cross_v2_v2(cp1.rA, P1) + b2Cross_v2_v2(cp2.rA, P2));

						//vB.Add(b2Vec2.Multiply(mB, b2Vec2.Add(P1, P2)));
						vB.x += mB * (P1x + P2x);
						vB.y += mB * (P1y + P2y);
						wB += iB * ((cp1.rB.x * P1y - cp1.rB.y * P1x) + (cp2.rB.x * P2y - cp2.rB.y * P2x));//(b2Cross_v2_v2(cp1.rB, P1) + b2Cross_v2_v2(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = xx;
						cp2.normalImpulse = xy;

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
					xx = - cp1.normalMass * bx;
					xy = 0.0;
					vn1 = 0.0;
					vn2 = vc.K.ex.y * xx + by;

					if (xx >= 0.0 && vn2 >= 0.0)
					{
						// Get the incremental impulse
						dx = xx - ax;
						dy = xy - ay;//b2Vec2.Subtract(x, a);

						// Apply incremental impulse
						P1x = dx * normal.x;
						P1y = dx * normal.y;//b2Vec2.Multiply(d.x, normal);
						P2x = dy * normal.x;
						P2y = dy * normal.y;//b2Vec2.Multiply(d.y, normal);
						//vA.Subtract(b2Vec2.Multiply(mA, b2Vec2.Add(P1, P2)));
						vA.x -= mA * (P1x + P2x);
						vA.y -= mA * (P1y + P2y);
						wA -= iA * ((cp1.rA.x * P1y - cp1.rA.y * P1x) + (cp2.rA.x * P2y - cp2.rA.y * P2x));//(b2Cross_v2_v2(cp1.rA, P1) + b2Cross_v2_v2(cp2.rA, P2));

						//vB.Add(b2Vec2.Multiply(mB, b2Vec2.Add(P1, P2)));
						vB.x += mB * (P1x + P2x);
						vB.y += mB * (P1y + P2y);
						wB += iB * ((cp1.rB.x * P1y - cp1.rB.y * P1x) + (cp2.rB.x * P2y - cp2.rB.y * P2x));//(b2Cross_v2_v2(cp1.rB, P1) + b2Cross_v2_v2(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = xx;
						cp2.normalImpulse = xy;

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
					xx = 0.0;
					xy = - cp2.normalMass * by;
					vn1 = vc.K.ey.x * xy + bx;
					vn2 = 0.0;

					if (xy >= 0.0 && vn1 >= 0.0)
					{
						// Resubstitute for the incremental impulse
						dx = xx - ax;
						dy = xy - ay;//b2Vec2.Subtract(x, a);

						// Apply incremental impulse
						P1x = dx * normal.x;
						P1y = dx * normal.y;//b2Vec2.Multiply(d.x, normal);
						P2x = dy * normal.x;
						P2y = dy * normal.y;//b2Vec2.Multiply(d.y, normal);
						//vA.Subtract(b2Vec2.Multiply(mA, b2Vec2.Add(P1, P2)));
						vA.x -= mA * (P1x + P2x);
						vA.y -= mA * (P1y + P2y);
						wA -= iA * ((cp1.rA.x * P1y - cp1.rA.y * P1x) + (cp2.rA.x * P2y - cp2.rA.y * P2x));//(b2Cross_v2_v2(cp1.rA, P1) + b2Cross_v2_v2(cp2.rA, P2));

						//vB.Add(b2Vec2.Multiply(mB, b2Vec2.Add(P1, P2)));
						vB.x += mB * (P1x + P2x);
						vB.y += mB * (P1y + P2y);
						wB += iB * ((cp1.rB.x * P1y - cp1.rB.y * P1x) + (cp2.rB.x * P2y - cp2.rB.y * P2x));//(b2Cross_v2_v2(cp1.rB, P1) + b2Cross_v2_v2(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = xx;
						cp2.normalImpulse = xy;

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
					xx = 0.0;
					xy = 0.0;
					vn1 = bx;
					vn2 = by;

					if (vn1 >= 0.0 && vn2 >= 0.0 )
					{
						// Resubstitute for the incremental impulse
						dx = xx - ax;
						dy = xy - ay;//b2Vec2.Subtract(x, a);

						// Apply incremental impulse
						P1x = dx * normal.x;
						P1y = dx * normal.y;//b2Vec2.Multiply(d.x, normal);
						P2x = dy * normal.x;
						P2y = dy * normal.y;//b2Vec2.Multiply(d.y, normal);
						//vA.Subtract(b2Vec2.Multiply(mA, b2Vec2.Add(P1, P2)));
						vA.x -= mA * (P1x + P2x);
						vA.y -= mA * (P1y + P2y);
						wA -= iA * ((cp1.rA.x * P1y - cp1.rA.y * P1x) + (cp2.rA.x * P2y - cp2.rA.y * P2x));//(b2Cross_v2_v2(cp1.rA, P1) + b2Cross_v2_v2(cp2.rA, P2));

						//vB.Add(b2Vec2.Multiply(mB, b2Vec2.Add(P1, P2)));
						vB.x += mB * (P1x + P2x);
						vB.y += mB * (P1y + P2y);
						wB += iB * ((cp1.rB.x * P1y - cp1.rB.y * P1x) + (cp2.rB.x * P2y - cp2.rB.y * P2x));//(b2Cross_v2_v2(cp1.rB, P1) + b2Cross_v2_v2(cp2.rB, P2));

						// Accumulate
						cp1.normalImpulse = xx;
						cp2.normalImpulse = xy;

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
				b2ContactSolver.cs_xfA.q.Set(aA);
				b2ContactSolver.cs_xfB.q.Set(aB);
				//b2ContactSolver.cs_xfA.p.Assign(b2Vec2.Subtract(cA, b2Mul_r_v2(b2ContactSolver.cs_xfA.q, localCenterA)));
				b2ContactSolver.cs_xfA.p.x = cA.x - (b2ContactSolver.cs_xfA.q.c * localCenterA.x - b2ContactSolver.cs_xfA.q.s * localCenterA.y);
				b2ContactSolver.cs_xfA.p.y = cA.y - (b2ContactSolver.cs_xfA.q.s * localCenterA.x + b2ContactSolver.cs_xfA.q.c * localCenterA.y);
				//b2ContactSolver.cs_xfB.p.Assign(b2Vec2.Subtract(cB, b2Mul_r_v2(b2ContactSolver.cs_xfB.q, localCenterB)));
				b2ContactSolver.cs_xfB.p.x = cB.x - (b2ContactSolver.cs_xfB.q.c * localCenterB.x - b2ContactSolver.cs_xfB.q.s * localCenterB.y);
				b2ContactSolver.cs_xfB.p.y = cB.y - (b2ContactSolver.cs_xfB.q.s * localCenterB.x + b2ContactSolver.cs_xfB.q.c * localCenterB.y);

				b2ContactSolver.temp_solver_manifold.Initialize(pc, b2ContactSolver.cs_xfA, b2ContactSolver.cs_xfB, j);
				var normal = b2ContactSolver.temp_solver_manifold.normal;

				var point = b2ContactSolver.temp_solver_manifold.point;
				var separation = b2ContactSolver.temp_solver_manifold.separation;

				var rAx = point.x - cA.x;
				var rAy = point.y - cA.y; //b2Vec2.Subtract(point, cA);
				var rBx = point.x - cB.x; //b2Vec2.Subtract(point, cB);
				var rBy = point.y - cB.y;

				// Track max constraint error.
				minSeparation = b2Min(minSeparation, separation);

				// Prevent large corrections and allow slop.
				var C = b2Clamp(b2_baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0);

				// Compute the effective mass.
				var rnA = rAx * normal.y - rAy * normal.x;//b2Cross_v2_v2(rA, normal);
				var rnB = rBx * normal.y - rBy * normal.x;//b2Cross_v2_v2(rB, normal);
				var K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

				// Compute normal impulse
				var impulse = K > 0.0 ? - C / K : 0.0;

				var Px = impulse * normal.x;
				var Py = impulse * normal.y;//b2Vec2.Multiply(impulse, normal);

				//cA.Subtract(b2Vec2.Multiply(mA, P));
				cA.x -= mA * Px;
				cA.y -= mA * Py;
				aA -= iA * (rAx * Py - rAy * Px);//b2Cross_v2_v2(rA, P);

				//cB.Add(b2Vec2.Multiply(mB, P));
				cB.x += mB * Px;
				cB.y += mB * Py;
				aB += iB * (rBx * Py - rBy * Px);//b2Cross_v2_v2(rB, P);
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
				b2ContactSolver.cs_xfA.q.Set(aA);
				b2ContactSolver.cs_xfB.q.Set(aB);
				b2ContactSolver.cs_xfA.p.Assign(b2Vec2.Subtract(cA, b2Mul_r_v2(b2ContactSolver.cs_xfA.q, localCenterA)));
				b2ContactSolver.cs_xfB.p.Assign(b2Vec2.Subtract(cB, b2Mul_r_v2(b2ContactSolver.cs_xfB.q, localCenterB)));

				b2ContactSolver.temp_solver_manifold.Initialize(pc, b2ContactSolver.cs_xfA, b2ContactSolver.cs_xfB, j);
				var normal = b2ContactSolver.temp_solver_manifold.normal;

				var point = b2ContactSolver.temp_solver_manifold.point;
				var separation = b2ContactSolver.temp_solver_manifold.separation;

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