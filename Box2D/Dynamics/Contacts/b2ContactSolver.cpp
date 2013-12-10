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

#include <Box2D/Dynamics/Contacts/b2ContactSolver.h>

#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Common/b2StackAllocator.h>

#define B2_DEBUG_SOLVER 0

struct b2ContactPositionConstraint
{
	b2Vec2 localPoints[b2_maxManifoldPoints];
	b2Vec2 localNormal;
	b2Vec2 localPoint;
	int32 indexA;
	int32 indexB;
	float32 invMassA, invMassB;
	b2Vec2 localCenterA, localCenterB;
	float32 invIA, invIB;
	int type;
	float32 radiusA, radiusB;
	int32 pointCount;
};

b2ContactSolver::b2ContactSolver(b2ContactSolverDef* def)
{
	this->m_step = def->step;
	this->m_allocator = def->allocator;
	this->m_count = def->count;
	this->m_positionConstraints = (b2ContactPositionConstraint*)this->m_allocator->Allocate(this->m_count * sizeof(b2ContactPositionConstraint));
	this->m_velocityConstraints = (b2ContactVelocityConstraint*)this->m_allocator->Allocate(this->m_count * sizeof(b2ContactVelocityConstraint));
	this->m_positions = def->positions;
	this->m_velocities = def->velocities;
	this->m_contacts = def->contacts;

	// Initialize position independent portions of the constraints.
	for (int32 i = 0; i < this->m_count; ++i)
	{
		b2Contact* contact = this->m_contacts[i];

		b2Fixture* fixtureA = contact->m_fixtureA;
		b2Fixture* fixtureB = contact->m_fixtureB;
		b2Shape* shapeA = fixtureA->GetShape();
		b2Shape* shapeB = fixtureB->GetShape();
		float32 radiusA = shapeA->m_radius;
		float32 radiusB = shapeB->m_radius;
		b2Body* bodyA = fixtureA->GetBody();
		b2Body* bodyB = fixtureB->GetBody();
		b2Manifold* manifold = contact->GetManifold();

		int32 pointCount = manifold->pointCount;
		b2Assert(pointCount > 0);

		b2ContactVelocityConstraint* vc = this->m_velocityConstraints + i;
		vc->friction = contact->m_friction;
		vc->restitution = contact->m_restitution;
		vc->tangentSpeed = contact->m_tangentSpeed;
		vc->indexA = bodyA->m_islandIndex;
		vc->indexB = bodyB->m_islandIndex;
		vc->invMassA = bodyA->m_invMass;
		vc->invMassB = bodyB->m_invMass;
		vc->invIA = bodyA->m_invI;
		vc->invIB = bodyB->m_invI;
		vc->contactIndex = i;
		vc->pointCount = pointCount;
		vc->K.SetZero();
		vc->normalMass.SetZero();

		b2ContactPositionConstraint* pc = this->m_positionConstraints + i;
		pc->indexA = bodyA->m_islandIndex;
		pc->indexB = bodyB->m_islandIndex;
		pc->invMassA = bodyA->m_invMass;
		pc->invMassB = bodyB->m_invMass;
		pc->localCenterA = bodyA->m_sweep.localCenter;
		pc->localCenterB = bodyB->m_sweep.localCenter;
		pc->invIA = bodyA->m_invI;
		pc->invIB = bodyB->m_invI;
		pc->localNormal = manifold->localNormal;
		pc->localPoint = manifold->localPoint;
		pc->pointCount = pointCount;
		pc->radiusA = radiusA;
		pc->radiusB = radiusB;
		pc->type = manifold->type;

		for (int32 j = 0; j < pointCount; ++j)
		{
			b2ManifoldPoint* cp = manifold->points + j;
			b2VelocityConstraintPoint* vcp = vc->points + j;
	
			if (this->m_step.warmStarting)
			{
				vcp->normalImpulse = this->m_step.dtRatio * cp->normalImpulse;
				vcp->tangentImpulse = this->m_step.dtRatio * cp->tangentImpulse;
			}
			else
			{
				vcp->normalImpulse = 0.0;
				vcp->tangentImpulse = 0.0;
			}

			vcp->rA.SetZero();
			vcp->rB.SetZero();
			vcp->normalMass = 0.0;
			vcp->tangentMass = 0.0;
			vcp->velocityBias = 0.0;

			pc->localPoints[j] = cp->localPoint;
		}
	}
}

b2ContactSolver::~b2ContactSolver()
{
	this->m_allocator->Free(this->m_velocityConstraints);
	this->m_allocator->Free(this->m_positionConstraints);
}

// Initialize position dependent portions of the velocity constraints.
void b2ContactSolver::InitializeVelocityConstraints()
{
	for (int32 i = 0; i < this->m_count; ++i)
	{
		b2ContactVelocityConstraint* vc = this->m_velocityConstraints + i;
		b2ContactPositionConstraint* pc = this->m_positionConstraints + i;

		float32 radiusA = pc->radiusA;
		float32 radiusB = pc->radiusB;
		b2Manifold* manifold = this->m_contacts[vc->contactIndex]->GetManifold();

		int32 indexA = vc->indexA;
		int32 indexB = vc->indexB;

		float32 mA = vc->invMassA;
		float32 mB = vc->invMassB;
		float32 iA = vc->invIA;
		float32 iB = vc->invIB;
		b2Vec2 localCenterA = pc->localCenterA;
		b2Vec2 localCenterB = pc->localCenterB;

		b2Vec2 cA = this->m_positions[indexA].c;
		float32 aA = this->m_positions[indexA].a;
		b2Vec2 vA = this->m_velocities[indexA].v;
		float32 wA = this->m_velocities[indexA].w;

		b2Vec2 cB = this->m_positions[indexB].c;
		float32 aB = this->m_positions[indexB].a;
		b2Vec2 vB = this->m_velocities[indexB].v;
		float32 wB = this->m_velocities[indexB].w;

		b2Assert(manifold->pointCount > 0);

		b2Transform xfA, xfB;
		xfA.q.Set(aA);
		xfB.q.Set(aB);
		xfA.p = b2Vec2::Subtract(cA, b2Mul_r_v2(xfA.q, localCenterA));
		xfB.p = b2Vec2::Subtract(cB, b2Mul_r_v2(xfB.q, localCenterB));

		b2WorldManifold worldManifold;
		worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB);

		vc->normal = worldManifold.normal;

		int32 pointCount = vc->pointCount;
		for (int32 j = 0; j < pointCount; ++j)
		{
			b2VelocityConstraintPoint* vcp = vc->points + j;

			vcp->rA = b2Vec2::Subtract(worldManifold.points[j], cA);
			vcp->rB = b2Vec2::Subtract(worldManifold.points[j], cB);

			float32 rnA = b2Cross_v2_v2(vcp->rA, vc->normal);
			float32 rnB = b2Cross_v2_v2(vcp->rB, vc->normal);

			float32 kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			vcp->normalMass = kNormal > 0.0 ? 1.0 / kNormal : 0.0;

			b2Vec2 tangent = b2Cross_v2_f(vc->normal, 1.0);

			float32 rtA = b2Cross_v2_v2(vcp->rA, tangent);
			float32 rtB = b2Cross_v2_v2(vcp->rB, tangent);

			float32 kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

			vcp->tangentMass = kTangent > 0.0 ? 1.0 /  kTangent : 0.0;

			// Setup a velocity bias for restitution.
			vcp->velocityBias = 0.0;
			float32 vRel = b2Dot_v2_v2(vc->normal, b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(vB, b2Cross_f_v2(wB, vcp->rB)), vA), b2Cross_f_v2(wA, vcp->rA)));
			if (vRel < -b2_velocityThreshold)
			{
				vcp->velocityBias = -vc->restitution * vRel;
			}
		}

		// If we have two points, then prepare the block solver.
		if (vc->pointCount == 2)
		{
			b2VelocityConstraintPoint* vcp1 = vc->points + 0;
			b2VelocityConstraintPoint* vcp2 = vc->points + 1;

			float32 rn1A = b2Cross_v2_v2(vcp1->rA, vc->normal);
			float32 rn1B = b2Cross_v2_v2(vcp1->rB, vc->normal);
			float32 rn2A = b2Cross_v2_v2(vcp2->rA, vc->normal);
			float32 rn2B = b2Cross_v2_v2(vcp2->rB, vc->normal);

			float32 k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
			float32 k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
			float32 k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

			// Ensure a reasonable condition number.
			const float32 k_maxConditionNumber = 1000.0;
			if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
			{
				// K is safe to invert.
				vc->K.ex.Set(k11, k12);
				vc->K.ey.Set(k12, k22);
				vc->normalMass = vc->K.GetInverse();
			}
			else
			{
				// The constraints are redundant, just use one.
				// TODO_ERIN use deepest?
				vc->pointCount = 1;
			}
		}
	}
}

void b2ContactSolver::WarmStart()
{
	// Warm start.
	for (int32 i = 0; i < this->m_count; ++i)
	{
		b2ContactVelocityConstraint* vc = this->m_velocityConstraints + i;

		int32 indexA = vc->indexA;
		int32 indexB = vc->indexB;
		float32 mA = vc->invMassA;
		float32 iA = vc->invIA;
		float32 mB = vc->invMassB;
		float32 iB = vc->invIB;
		int32 pointCount = vc->pointCount;

		b2Vec2 vA = this->m_velocities[indexA].v;
		float32 wA = this->m_velocities[indexA].w;
		b2Vec2 vB = this->m_velocities[indexB].v;
		float32 wB = this->m_velocities[indexB].w;

		b2Vec2 normal = vc->normal;
		b2Vec2 tangent = b2Cross_v2_f(normal, 1.0);

		for (int32 j = 0; j < pointCount; ++j)
		{
			b2VelocityConstraintPoint* vcp = vc->points + j;
			b2Vec2 P = b2Vec2::Add(b2Vec2::Multiply(vcp->normalImpulse, normal), b2Vec2::Multiply(vcp->tangentImpulse, tangent));
			wA -= iA * b2Cross_v2_v2(vcp->rA, P);
			vA.Subtract(b2Vec2::Multiply(mA, P));
			wB += iB * b2Cross_v2_v2(vcp->rB, P);
			vB.Add(b2Vec2::Multiply(mB, P));
		}

		this->m_velocities[indexA].v = vA;
		this->m_velocities[indexA].w = wA;
		this->m_velocities[indexB].v = vB;
		this->m_velocities[indexB].w = wB;
	}
}

void b2ContactSolver::SolveVelocityConstraints()
{
	for (int32 i = 0; i < this->m_count; ++i)
	{
		b2ContactVelocityConstraint* vc = this->m_velocityConstraints + i;

		int32 indexA = vc->indexA;
		int32 indexB = vc->indexB;
		float32 mA = vc->invMassA;
		float32 iA = vc->invIA;
		float32 mB = vc->invMassB;
		float32 iB = vc->invIB;
		int32 pointCount = vc->pointCount;

		b2Vec2 vA = this->m_velocities[indexA].v;
		float32 wA = this->m_velocities[indexA].w;
		b2Vec2 vB = this->m_velocities[indexB].v;
		float32 wB = this->m_velocities[indexB].w;

		b2Vec2 normal = vc->normal;
		b2Vec2 tangent = b2Cross_v2_f(normal, 1.0);
		float32 friction = vc->friction;

		b2Assert(pointCount == 1 || pointCount == 2);

		// Solve tangent constraints first because non-penetration is more important
		// than friction.
		for (int32 j = 0; j < pointCount; ++j)
		{
			b2VelocityConstraintPoint* vcp = vc->points + j;

			// Relative velocity at contact
			b2Vec2 dv = b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(vB, b2Cross_f_v2(wB, vcp->rB)), vA), b2Cross_f_v2(wA, vcp->rA));

			// Compute tangent force
			float32 vt = b2Dot_v2_v2(dv, tangent) - vc->tangentSpeed;
			float32 lambda = vcp->tangentMass * (-vt);

			// b2Clamp the accumulated force
			float32 maxFriction = friction * vcp->normalImpulse;
			float32 newImpulse = b2Clamp(vcp->tangentImpulse + lambda, -maxFriction, maxFriction);
			lambda = newImpulse - vcp->tangentImpulse;
			vcp->tangentImpulse = newImpulse;

			// Apply contact impulse
			b2Vec2 P = b2Vec2::Multiply(lambda, tangent);

			vA.Subtract(b2Vec2::Multiply(mA, P));
			wA -= iA * b2Cross_v2_v2(vcp->rA, P);

			vB.Add(b2Vec2::Multiply(mB, P));
			wB += iB * b2Cross_v2_v2(vcp->rB, P);
		}

		// Solve normal constraints
		if (vc->pointCount == 1)
		{
			b2VelocityConstraintPoint* vcp = vc->points + 0;

			// Relative velocity at contact
			b2Vec2 dv = b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(vB, b2Cross_f_v2(wB, vcp->rB)), vA), b2Cross_f_v2(wA, vcp->rA));

			// Compute normal impulse
			float32 vn = b2Dot_v2_v2(dv, normal);
			float32 lambda = -vcp->normalMass * (vn - vcp->velocityBias);

			// b2Clamp the accumulated impulse
			float32 newImpulse = b2Max(vcp->normalImpulse + lambda, 0.0);
			lambda = newImpulse - vcp->normalImpulse;
			vcp->normalImpulse = newImpulse;

			// Apply contact impulse
			b2Vec2 P = b2Vec2::Multiply(lambda, normal);
			vA.Subtract(b2Vec2::Multiply(mA, P));
			wA -= iA * b2Cross_v2_v2(vcp->rA, P);

			vB.Add(b2Vec2::Multiply(mB, P));
			wB += iB * b2Cross_v2_v2(vcp->rB, P);
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

			b2VelocityConstraintPoint* cp1 = vc->points + 0;
			b2VelocityConstraintPoint* cp2 = vc->points + 1;

			b2Vec2 a(cp1->normalImpulse, cp2->normalImpulse);
			b2Assert(a.x >= 0.0 && a.y >= 0.0);

			// Relative velocity at contact
			b2Vec2 dv1 = b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(vB, b2Cross_f_v2(wB, cp1->rB)), vA), b2Cross_f_v2(wA, cp1->rA));
			b2Vec2 dv2 = b2Vec2::Subtract(b2Vec2::Subtract(b2Vec2::Add(vB, b2Cross_f_v2(wB, cp2->rB)), vA), b2Cross_f_v2(wA, cp2->rA));

			// Compute normal velocity
			float32 vn1 = b2Dot_v2_v2(dv1, normal);
			float32 vn2 = b2Dot_v2_v2(dv2, normal);

			b2Vec2 b;
			b.x = vn1 - cp1->velocityBias;
			b.y = vn2 - cp2->velocityBias;

			// Compute b'
			b.Subtract(b2Mul_m22_v2(vc->K, a));

			const float32 k_errorTol = 1e-3f;
			B2_NOT_USED(k_errorTol);

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
				b2Vec2 x = b2Mul_m22_v2(vc->normalMass, b).Negate();

				if (x.x >= 0.0 && x.y >= 0.0)
				{
					// Get the incremental impulse
					b2Vec2 d = b2Vec2::Subtract(x, a);

					// Apply incremental impulse
					b2Vec2 P1 = b2Vec2::Multiply(d.x, normal);
					b2Vec2 P2 = b2Vec2::Multiply(d.y, normal);
					vA.Subtract(b2Vec2::Multiply(mA, b2Vec2::Add(P1, P2)));
					wA -= iA * (b2Cross_v2_v2(cp1->rA, P1) + b2Cross_v2_v2(cp2->rA, P2));

					vB.Add(b2Vec2::Multiply(mB, b2Vec2::Add(P1, P2)));
					wB += iB * (b2Cross_v2_v2(cp1->rB, P1) + b2Cross_v2_v2(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

#if B2_DEBUG_SOLVER == 1
					// Postconditions
					dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);
					dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);

					// Compute normal velocity
					vn1 = b2Dot(dv1, normal);
					vn2 = b2Dot(dv2, normal);

					b2Assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
					b2Assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
#endif
					break;
				}

				//
				// Case 2: vn1 = 0 and x2 = 0
				//
				//   0 = a11 * x1 + a12 * 0 + b1' 
				// vn2 = a21 * x1 + a22 * 0 + b2'
				//
				x.x = - cp1->normalMass * b.x;
				x.y = 0.0;
				vn1 = 0.0;
				vn2 = vc->K.ex.y * x.x + b.y;

				if (x.x >= 0.0 && vn2 >= 0.0)
				{
					// Get the incremental impulse
					b2Vec2 d = b2Vec2::Subtract(x, a);

					// Apply incremental impulse
					b2Vec2 P1 = b2Vec2::Multiply(d.x, normal);
					b2Vec2 P2 = b2Vec2::Multiply(d.y, normal);
					vA.Subtract(b2Vec2::Multiply(mA, b2Vec2::Add(P1, P2)));
					wA -= iA * (b2Cross_v2_v2(cp1->rA, P1) + b2Cross_v2_v2(cp2->rA, P2));

					vB.Add(b2Vec2::Multiply(mB, b2Vec2::Add(P1, P2)));
					wB += iB * (b2Cross_v2_v2(cp1->rB, P1) + b2Cross_v2_v2(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

#if B2_DEBUG_SOLVER == 1
					// Postconditions
					dv1 = vB + b2Cross(wB, cp1->rB) - vA - b2Cross(wA, cp1->rA);

					// Compute normal velocity
					vn1 = b2Dot(dv1, normal);

					b2Assert(b2Abs(vn1 - cp1->velocityBias) < k_errorTol);
#endif
					break;
				}


				//
				// Case 3: vn2 = 0 and x1 = 0
				//
				// vn1 = a11 * 0 + a12 * x2 + b1' 
				//   0 = a21 * 0 + a22 * x2 + b2'
				//
				x.x = 0.0;
				x.y = - cp2->normalMass * b.y;
				vn1 = vc->K.ey.x * x.y + b.x;
				vn2 = 0.0;

				if (x.y >= 0.0 && vn1 >= 0.0)
				{
					// Resubstitute for the incremental impulse
					b2Vec2 d = b2Vec2::Subtract(x, a);

					// Apply incremental impulse
					b2Vec2 P1 = b2Vec2::Multiply(d.x, normal);
					b2Vec2 P2 = b2Vec2::Multiply(d.y, normal);
					vA.Subtract(b2Vec2::Multiply(mA, b2Vec2::Add(P1, P2)));
					wA -= iA * (b2Cross_v2_v2(cp1->rA, P1) + b2Cross_v2_v2(cp2->rA, P2));

					vB.Add(b2Vec2::Multiply(mB, b2Vec2::Add(P1, P2)));
					wB += iB * (b2Cross_v2_v2(cp1->rB, P1) + b2Cross_v2_v2(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

#if B2_DEBUG_SOLVER == 1
					// Postconditions
					dv2 = vB + b2Cross(wB, cp2->rB) - vA - b2Cross(wA, cp2->rA);

					// Compute normal velocity
					vn2 = b2Dot(dv2, normal);

					b2Assert(b2Abs(vn2 - cp2->velocityBias) < k_errorTol);
#endif
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
					b2Vec2 d = b2Vec2::Subtract(x, a);

					// Apply incremental impulse
					b2Vec2 P1 = b2Vec2::Multiply(d.x, normal);
					b2Vec2 P2 = b2Vec2::Multiply(d.y, normal);
					vA.Subtract(b2Vec2::Multiply(mA, b2Vec2::Add(P1, P2)));
					wA -= iA * (b2Cross_v2_v2(cp1->rA, P1) + b2Cross_v2_v2(cp2->rA, P2));

					vB.Add(b2Vec2::Multiply(mB, b2Vec2::Add(P1, P2)));
					wB += iB * (b2Cross_v2_v2(cp1->rB, P1) + b2Cross_v2_v2(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

					break;
				}

				// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
				break;
			}
		}

		this->m_velocities[indexA].v = vA;
		this->m_velocities[indexA].w = wA;
		this->m_velocities[indexB].v = vB;
		this->m_velocities[indexB].w = wB;
	}
}

void b2ContactSolver::StoreImpulses()
{
	for (int32 i = 0; i < this->m_count; ++i)
	{
		b2ContactVelocityConstraint* vc = this->m_velocityConstraints + i;
		b2Manifold* manifold = this->m_contacts[vc->contactIndex]->GetManifold();

		for (int32 j = 0; j < vc->pointCount; ++j)
		{
			manifold->points[j].normalImpulse = vc->points[j].normalImpulse;
			manifold->points[j].tangentImpulse = vc->points[j].tangentImpulse;
		}
	}
}

struct b2PositionSolverManifold
{
	void Initialize(b2ContactPositionConstraint* pc, const b2Transform& xfA, const b2Transform& xfB, int32 index)
	{
		b2Assert(pc->pointCount > 0);

		switch (pc->type)
		{
		case b2Manifold::e_circles:
			{
				b2Vec2 pointA = b2Mul_t_v2(xfA, pc->localPoint);
				b2Vec2 pointB = b2Mul_t_v2(xfB, pc->localPoints[0]);
				this->normal = b2Vec2::Subtract(pointB, pointA);
				this->normal.Normalize();
				this->point = b2Vec2::Multiply(0.5, b2Vec2::Add(pointA, pointB));
				this->separation = b2Dot_v2_v2(b2Vec2::Subtract(pointB, pointA), this->normal) - pc->radiusA - pc->radiusB;
			}
			break;

		case b2Manifold::e_faceA:
			{
				this->normal = b2Mul_r_v2(xfA.q, pc->localNormal);
				b2Vec2 planePoint = b2Mul_t_v2(xfA, pc->localPoint);

				b2Vec2 clipPoint = b2Mul_t_v2(xfB, pc->localPoints[index]);
				this->separation = b2Dot_v2_v2(b2Vec2::Subtract(clipPoint, planePoint), this->normal) - pc->radiusA - pc->radiusB;
				this->point = clipPoint;
			}
			break;

		case b2Manifold::e_faceB:
			{
				this->normal = b2Mul_r_v2(xfB.q, pc->localNormal);
				b2Vec2 planePoint = b2Mul_t_v2(xfB, pc->localPoint);

				b2Vec2 clipPoint = b2Mul_t_v2(xfA, pc->localPoints[index]);
				this->separation = b2Dot_v2_v2(b2Vec2::Subtract(clipPoint, planePoint), this->normal) - pc->radiusA - pc->radiusB;
				this->point = clipPoint;

				// Ensure normal points from A to B
				this->normal = this->normal.Negate();
			}
			break;
		}
	}

	b2Vec2 normal;
	b2Vec2 point;
	float32 separation;
};

// Sequential solver.
bool b2ContactSolver::SolvePositionConstraints()
{
	float32 minSeparation = 0.0;

	for (int32 i = 0; i < this->m_count; ++i)
	{
		b2ContactPositionConstraint* pc = this->m_positionConstraints + i;

		int32 indexA = pc->indexA;
		int32 indexB = pc->indexB;
		b2Vec2 localCenterA = pc->localCenterA;
		float32 mA = pc->invMassA;
		float32 iA = pc->invIA;
		b2Vec2 localCenterB = pc->localCenterB;
		float32 mB = pc->invMassB;
		float32 iB = pc->invIB;
		int32 pointCount = pc->pointCount;

		b2Vec2 cA = this->m_positions[indexA].c;
		float32 aA = this->m_positions[indexA].a;

		b2Vec2 cB = this->m_positions[indexB].c;
		float32 aB = this->m_positions[indexB].a;

		// Solve normal constraints
		for (int32 j = 0; j < pointCount; ++j)
		{
			b2Transform xfA, xfB;
			xfA.q.Set(aA);
			xfB.q.Set(aB);
			xfA.p = b2Vec2::Subtract(cA, b2Mul_r_v2(xfA.q, localCenterA));
			xfB.p = b2Vec2::Subtract(cB, b2Mul_r_v2(xfB.q, localCenterB));

			b2PositionSolverManifold psm;
			psm.Initialize(pc, xfA, xfB, j);
			b2Vec2 normal = psm.normal;

			b2Vec2 point = psm.point;
			float32 separation = psm.separation;

			b2Vec2 rA = b2Vec2::Subtract(point, cA);
			b2Vec2 rB = b2Vec2::Subtract(point, cB);

			// Track max constraint error.
			minSeparation = b2Min(minSeparation, separation);

			// Prevent large corrections and allow slop.
			float32 C = b2Clamp(b2_baumgarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0);

			// Compute the effective mass.
			float32 rnA = b2Cross_v2_v2(rA, normal);
			float32 rnB = b2Cross_v2_v2(rB, normal);
			float32 K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			// Compute normal impulse
			float32 impulse = K > 0.0 ? - C / K : 0.0;

			b2Vec2 P = b2Vec2::Multiply(impulse, normal);

			cA.Subtract(b2Vec2::Multiply(mA, P));
			aA -= iA * b2Cross_v2_v2(rA, P);

			cB.Add(b2Vec2::Multiply(mB, P));
			aB += iB * b2Cross_v2_v2(rB, P);
		}

		this->m_positions[indexA].c = cA;
		this->m_positions[indexA].a = aA;

		this->m_positions[indexB].c = cB;
		this->m_positions[indexB].a = aB;
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -3.0 * b2_linearSlop;
}

// Sequential position solver for position constraints.
bool b2ContactSolver::SolveTOIPositionConstraints(int32 toiIndexA, int32 toiIndexB)
{
	float32 minSeparation = 0.0;

	for (int32 i = 0; i < this->m_count; ++i)
	{
		b2ContactPositionConstraint* pc = this->m_positionConstraints + i;

		int32 indexA = pc->indexA;
		int32 indexB = pc->indexB;
		b2Vec2 localCenterA = pc->localCenterA;
		b2Vec2 localCenterB = pc->localCenterB;
		int32 pointCount = pc->pointCount;

		float32 mA = 0.0;
		float32 iA = 0.0;
		if (indexA == toiIndexA || indexA == toiIndexB)
		{
			mA = pc->invMassA;
			iA = pc->invIA;
		}

		float32 mB = 0.0;
		float32 iB = 0.;
		if (indexB == toiIndexA || indexB == toiIndexB)
		{
			mB = pc->invMassB;
			iB = pc->invIB;
		}

		b2Vec2 cA = this->m_positions[indexA].c;
		float32 aA = this->m_positions[indexA].a;

		b2Vec2 cB = this->m_positions[indexB].c;
		float32 aB = this->m_positions[indexB].a;

		// Solve normal constraints
		for (int32 j = 0; j < pointCount; ++j)
		{
			b2Transform xfA, xfB;
			xfA.q.Set(aA);
			xfB.q.Set(aB);
			xfA.p = b2Vec2::Subtract(cA, b2Mul_r_v2(xfA.q, localCenterA));
			xfB.p = b2Vec2::Subtract(cB, b2Mul_r_v2(xfB.q, localCenterB));

			b2PositionSolverManifold psm;
			psm.Initialize(pc, xfA, xfB, j);
			b2Vec2 normal = psm.normal;

			b2Vec2 point = psm.point;
			float32 separation = psm.separation;

			b2Vec2 rA = b2Vec2::Subtract(point, cA);
			b2Vec2 rB = b2Vec2::Subtract(point, cB);

			// Track max constraint error.
			minSeparation = b2Min(minSeparation, separation);

			// Prevent large corrections and allow slop.
			float32 C = b2Clamp(b2_toiBaugarte * (separation + b2_linearSlop), -b2_maxLinearCorrection, 0.0);

			// Compute the effective mass.
			float32 rnA = b2Cross_v2_v2(rA, normal);
			float32 rnB = b2Cross_v2_v2(rB, normal);
			float32 K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			// Compute normal impulse
			float32 impulse = K > 0.0 ? - C / K : 0.0;

			b2Vec2 P = b2Vec2::Multiply(impulse, normal);

			cA.Subtract(b2Vec2::Multiply(mA, P));
			aA -= iA * b2Cross_v2_v2(rA, P);

			cB.Add(b2Vec2::Multiply(mB, P));
			aB += iB * b2Cross_v2_v2(rB, P);
		}

		this->m_positions[indexA].c = cA;
		this->m_positions[indexA].a = aA;

		this->m_positions[indexB].c = cB;
		this->m_positions[indexB].a = aB;
	}

	// We can't expect minSpeparation >= -b2_linearSlop because we don't
	// push the separation above -b2_linearSlop.
	return minSeparation >= -1.5 * b2_linearSlop;
}
