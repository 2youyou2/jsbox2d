/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <Box2D/Dynamics/b2Body.h>
#include <Box2D/Dynamics/b2Fixture.h>
#include <Box2D/Dynamics/b2World.h>
#include <Box2D/Dynamics/Contacts/b2Contact.h>
#include <Box2D/Dynamics/Joints/b2Joint.h>

b2BodyDef::b2BodyDef()
{
	this->userData = null;
	this->position.Set(0.0, 0.0);
	this->angle = 0.0;
	this->linearVelocity.Set(0.0, 0.0);
	this->angularVelocity = 0.0;
	this->linearDamping = 0.0;
	this->angularDamping = 0.0;
	this->allowSleep = true;
	this->awake = true;
	this->fixedRotation = false;
	this->bullet = false;
	this->type = b2Body::b2_staticBody;
	this->active = true;
	this->gravityScale = 1.0;
}

b2Body::b2Body(const b2BodyDef* bd, b2World* world)
{
	b2Assert(bd->position.IsValid());
	b2Assert(bd->linearVelocity.IsValid());
	b2Assert(b2IsValid(bd->angle));
	b2Assert(b2IsValid(bd->angularVelocity));
	b2Assert(b2IsValid(bd->angularDamping) && bd->angularDamping >= 0.0);
	b2Assert(b2IsValid(bd->linearDamping) && bd->linearDamping >= 0.0);

	this->m_flags = 0;

	if (bd->bullet)
	{
		this->m_flags |= b2Body::e_bulletFlag;
	}
	if (bd->fixedRotation)
	{
		this->m_flags |= b2Body::e_fixedRotationFlag;
	}
	if (bd->allowSleep)
	{
		this->m_flags |= b2Body::e_autoSleepFlag;
	}
	if (bd->awake)
	{
		this->m_flags |= b2Body::e_awakeFlag;
	}
	if (bd->active)
	{
		this->m_flags |= b2Body::e_activeFlag;
	}

	this->m_world = world;

	this->m_xf.p.Assign(bd->position);
	this->m_xf.q.Set(bd->angle);

	this->m_sweep.localCenter.SetZero();
	this->m_sweep.c0.Assign(this->m_xf.p);
	this->m_sweep.c.Assign(this->m_xf.p);
	this->m_sweep.a0 = bd->angle;
	this->m_sweep.a = bd->angle;
	this->m_sweep.alpha0 = 0.0;

	this->m_jointList = null;
	this->m_contactList = null;
	this->m_prev = null;
	this->m_next = null;

	this->m_linearVelocity.Assign(bd->linearVelocity);
	this->m_angularVelocity = bd->angularVelocity;

	this->m_linearDamping = bd->linearDamping;
	this->m_angularDamping = bd->angularDamping;
	this->m_gravityScale = bd->gravityScale;

	this->m_force.SetZero();
	this->m_torque = 0.0;

	this->m_sleepTime = 0.0;

	this->m_type = bd->type;

	if (this->m_type == b2_dynamicBody)
	{
		this->m_mass = 1.0;
		this->m_invMass = 1.0;
	}
	else
	{
		this->m_mass = 0.0;
		this->m_invMass = 0.0;
	}

	this->m_I = 0.0;
	this->m_invI = 0.0;

	this->m_userData = bd->userData;

	this->m_fixtureList = null;
	this->m_fixtureCount = 0;
}

b2Body::~b2Body()
{
	// shapes and joints are destroyed in b2World::Destroy
}

void b2Body::SetType(int type)
{
	b2Assert(this->m_world->IsLocked() == false);
	if (this->m_world->IsLocked() == true)
	{
		return;
	}

	if (this->m_type == type)
	{
		return;
	}

	this->m_type = type;

	ResetMassData();

	if (this->m_type == b2_staticBody)
	{
		this->m_linearVelocity.SetZero();
		this->m_angularVelocity = 0.0;
		this->m_sweep.a0 = this->m_sweep.a;
		this->m_sweep.c0.Assign(this->m_sweep.c);
		SynchronizeFixtures();
	}

	SetAwake(true);

	this->m_force.SetZero();
	this->m_torque = 0.0;

	// Delete the attached contacts.
	b2ContactEdge* ce = this->m_contactList;
	while (ce)
	{
		b2ContactEdge* ce0 = ce;
		ce = ce->next;
		this->m_world->m_contactManager.Destroy(ce0->contact);
	}
	this->m_contactList = null;

	// Touch the proxies so that new contacts will be created (when appropriate)
	b2BroadPhase* broadPhase = &this->m_world->m_contactManager.m_broadPhase;
	for (b2Fixture* f = this->m_fixtureList; f; f = f->m_next)
	{
		int32 proxyCount = f->m_proxyCount;
		for (int32 i = 0; i < proxyCount; ++i)
		{
			broadPhase->TouchProxy(f->m_proxies[i].proxyId);
		}
	}
}

b2Fixture* b2Body::CreateFixture(const b2FixtureDef* def)
{
	b2Assert(this->m_world->IsLocked() == false);
	if (this->m_world->IsLocked() == true)
	{
		return null;
	}

	b2BlockAllocator* allocator = &this->m_world->m_blockAllocator;

	void* memory = allocator->Allocate(sizeof(b2Fixture));
	b2Fixture* fixture = new (memory) b2Fixture;
	fixture->Create(allocator, this, def);

	if (this->m_flags & b2Body::e_activeFlag)
	{
		b2BroadPhase* broadPhase = &this->m_world->m_contactManager.m_broadPhase;
		fixture->CreateProxies(broadPhase, this->m_xf);
	}

	fixture->m_next = this->m_fixtureList;
	this->m_fixtureList = fixture;
	++this->m_fixtureCount;

	fixture->m_body = this;

	// Adjust mass properties if needed.
	if (fixture->m_density > 0.0)
	{
		ResetMassData();
	}

	// Let the world know we have a new fixture. This will cause new contacts
	// to be created at the beginning of the next time step.
	this->m_world->m_flags |= b2World::e_newFixture;

	return fixture;
}

b2Fixture* b2Body::CreateFixture(const b2Shape* shape, float32 density)
{
	b2FixtureDef def;
	def.shape = shape;
	def.density = density;

	return CreateFixture(&def);
}

void b2Body::DestroyFixture(b2Fixture* fixture)
{
	b2Assert(this->m_world->IsLocked() == false);
	if (this->m_world->IsLocked() == true)
	{
		return;
	}

	b2Assert(fixture->m_body == this);

	// Remove the fixture from this body's singly linked list.
	b2Assert(this->m_fixtureCount > 0);
	b2Fixture** node = &this->m_fixtureList;
	bool found = false;
	while (*node != null)
	{
		if (*node == fixture)
		{
			*node = fixture->m_next;
			found = true;
			break;
		}

		node = &(*node)->m_next;
	}

	// You tried to remove a shape that is not attached to this body.
	b2Assert(found);

	// Destroy any contacts associated with the fixture.
	b2ContactEdge* edge = this->m_contactList;
	while (edge)
	{
		b2Contact* c = edge->contact;
		edge = edge->next;

		b2Fixture* fixtureA = c->GetFixtureA();
		b2Fixture* fixtureB = c->GetFixtureB();

		if (fixture == fixtureA || fixture == fixtureB)
		{
			// This destroys the contact and removes it from
			// this body's contact list.
			this->m_world->m_contactManager.Destroy(c);
		}
	}

	b2BlockAllocator* allocator = &this->m_world->m_blockAllocator;

	if (this->m_flags & b2Body::e_activeFlag)
	{
		b2BroadPhase* broadPhase = &this->m_world->m_contactManager.m_broadPhase;
		fixture->DestroyProxies(broadPhase);
	}

	fixture->Destroy(allocator);
	fixture->m_body = null;
	fixture->m_next = null;
	fixture->~b2Fixture();
	allocator->Free(fixture, sizeof(b2Fixture));

	--this->m_fixtureCount;

	// Reset the mass data.
	ResetMassData();
}

void b2Body::ResetMassData()
{
	// Compute mass data from shapes. Each shape has its own density.
	this->m_mass = 0.0;
	this->m_invMass = 0.0;
	this->m_I = 0.0;
	this->m_invI = 0.0;
	this->m_sweep.localCenter.SetZero();

	// Static and kinematic bodies have zero mass.
	if (this->m_type == b2_staticBody || this->m_type == b2_kinematicBody)
	{
		this->m_sweep.c0.Assign(this->m_xf.p);
		this->m_sweep.c.Assign(this->m_xf.p);
		this->m_sweep.a0 = this->m_sweep.a;
		return;
	}

	b2Assert(this->m_type == b2_dynamicBody);

	// Accumulate mass over all fixtures.
	b2Vec2 localCenter = b2Vec2(0, 0);
	for (b2Fixture* f = this->m_fixtureList; f; f = f->m_next)
	{
		if (f->m_density == 0.0)
		{
			continue;
		}

		b2MassData massData;
		f->GetMassData(&massData);
		this->m_mass += massData.mass;
		localCenter.Add(b2Vec2::Multiply(massData.mass, massData.center));
		this->m_I += massData.I;
	}

	// Compute center of mass.
	if (this->m_mass > 0.0)
	{
		this->m_invMass = 1.0 / this->m_mass;
		localCenter.Multiply(this->m_invMass);
	}
	else
	{
		// Force all dynamic bodies to have a positive mass.
		this->m_mass = 1.0;
		this->m_invMass = 1.0;
	}

	if (this->m_I > 0.0 && (this->m_flags & b2Body::e_fixedRotationFlag) == 0)
	{
		// Center the inertia about the center of mass.
		this->m_I -= this->m_mass * b2Dot_v2_v2(localCenter, localCenter);
		b2Assert(this->m_I > 0.0);
		this->m_invI = 1.0 / this->m_I;

	}
	else
	{
		this->m_I = 0.0;
		this->m_invI = 0.0;
	}

	// Move center of mass.
	b2Vec2 oldCenter = this->m_sweep.c;
	this->m_sweep.localCenter.Assign(localCenter);
	this->m_sweep.c.Assign(b2Mul_t_v2(this->m_xf, this->m_sweep.localCenter));
	this->m_sweep.c0.Assign(this->m_sweep.c);

	// Update center of mass velocity.
	this->m_linearVelocity.Add(b2Cross_f_v2(this->m_angularVelocity, b2Vec2::Subtract(this->m_sweep.c, oldCenter)));
}

void b2Body::SetMassData(const b2MassData* massData)
{
	b2Assert(this->m_world->IsLocked() == false);
	if (this->m_world->IsLocked() == true)
	{
		return;
	}

	if (this->m_type != b2_dynamicBody)
	{
		return;
	}

	this->m_invMass = 0.0;
	this->m_I = 0.0;
	this->m_invI = 0.0;

	this->m_mass = massData->mass;
	if (this->m_mass <= 0.0)
	{
		this->m_mass = 1.0;
	}

	this->m_invMass = 1.0 / this->m_mass;

	if (massData->I > 0.0 && (this->m_flags & b2Body::e_fixedRotationFlag) == 0)
	{
		this->m_I = massData->I - this->m_mass * b2Dot_v2_v2(massData->center, massData->center);
		b2Assert(this->m_I > 0.0);
		this->m_invI = 1.0 / this->m_I;
	}

	// Move center of mass.
	b2Vec2 oldCenter = this->m_sweep.c;
	this->m_sweep.localCenter.Assign(massData->center);
	this->m_sweep.c.Assign(b2Mul_t_v2(this->m_xf, this->m_sweep.localCenter));
	this->m_sweep.c0.Assign(this->m_sweep.c);

	// Update center of mass velocity.
	this->m_linearVelocity.Add(b2Cross_f_v2(this->m_angularVelocity, b2Vec2::Subtract(this->m_sweep.c, oldCenter)));
}

bool b2Body::ShouldCollide(const b2Body* other) const
{
	// At least one body should be dynamic.
	if (this->m_type != b2_dynamicBody && other->m_type != b2_dynamicBody)
	{
		return false;
	}

	// Does a joint prevent collision?
	for (b2JointEdge* jn = this->m_jointList; jn; jn = jn->next)
	{
		if (jn->other == other)
		{
			if (jn->joint->m_collideConnected == false)
			{
				return false;
			}
		}
	}

	return true;
}

void b2Body::SetTransform(const b2Vec2& position, float32 angle)
{
	b2Assert(this->m_world->IsLocked() == false);
	if (this->m_world->IsLocked() == true)
	{
		return;
	}

	this->m_xf.q.Set(angle);
	this->m_xf.p.Assign(position);

	this->m_sweep.c.Assign(b2Mul_t_v2(this->m_xf, this->m_sweep.localCenter));
	this->m_sweep.a = angle;

	this->m_sweep.c0.Assign(this->m_sweep.c);
	this->m_sweep.a0 = angle;

	b2BroadPhase* broadPhase = &this->m_world->m_contactManager.m_broadPhase;
	for (b2Fixture* f = this->m_fixtureList; f; f = f->m_next)
	{
		f->Synchronize(broadPhase, this->m_xf, this->m_xf);
	}
}

void b2Body::SynchronizeFixtures()
{
	b2Transform xf1;
	xf1.q.Set(this->m_sweep.a0);
	xf1.p.Assign(b2Vec2::Subtract(this->m_sweep.c0, b2Mul_r_v2(xf1.q, this->m_sweep.localCenter)));

	b2BroadPhase* broadPhase = &this->m_world->m_contactManager.m_broadPhase;
	for (b2Fixture* f = this->m_fixtureList; f; f = f->m_next)
	{
		f->Synchronize(broadPhase, xf1, this->m_xf);
	}
}

void b2Body::SetActive(bool flag)
{
	b2Assert(this->m_world->IsLocked() == false);

	if (flag == IsActive())
	{
		return;
	}

	if (flag)
	{
		this->m_flags |= b2Body::e_activeFlag;

		// Create all proxies.
		b2BroadPhase* broadPhase = &this->m_world->m_contactManager.m_broadPhase;
		for (b2Fixture* f = this->m_fixtureList; f; f = f->m_next)
		{
			f->CreateProxies(broadPhase, this->m_xf);
		}

		// Contacts are created the next time step.
	}
	else
	{
		this->m_flags &= ~b2Body::e_activeFlag;

		// Destroy all proxies.
		b2BroadPhase* broadPhase = &this->m_world->m_contactManager.m_broadPhase;
		for (b2Fixture* f = this->m_fixtureList; f; f = f->m_next)
		{
			f->DestroyProxies(broadPhase);
		}

		// Destroy the attached contacts.
		b2ContactEdge* ce = this->m_contactList;
		while (ce)
		{
			b2ContactEdge* ce0 = ce;
			ce = ce->next;
			this->m_world->m_contactManager.Destroy(ce0->contact);
		}
		this->m_contactList = null;
	}
}

void b2Body::SetFixedRotation(bool flag)
{
	bool status = (this->m_flags & b2Body::e_fixedRotationFlag) == b2Body::e_fixedRotationFlag;
	if (status == flag)
	{
		return;
	}

	if (flag)
	{
		this->m_flags |= b2Body::e_fixedRotationFlag;
	}
	else
	{
		this->m_flags &= ~b2Body::e_fixedRotationFlag;
	}

	this->m_angularVelocity = 0.0;

	ResetMassData();
}

void b2Body::Dump()
{
	int32 bodyIndex = this->m_islandIndex;

	b2Log("{\n");
	b2Log("  b2BodyDef bd;\n");
	b2Log("  bd.type = b2BodyType(%d);\n", this->m_type);
	b2Log("  bd.position.Set(%.15lef, %.15lef);\n", this->m_xf.p.x, this->m_xf.p.y);
	b2Log("  bd.angle = %.15lef;\n", this->m_sweep.a);
	b2Log("  bd.linearVelocity.Set(%.15lef, %.15lef);\n", this->m_linearVelocity.x, this->m_linearVelocity.y);
	b2Log("  bd.angularVelocity = %.15lef;\n", this->m_angularVelocity);
	b2Log("  bd.linearDamping = %.15lef;\n", this->m_linearDamping);
	b2Log("  bd.angularDamping = %.15lef;\n", this->m_angularDamping);
	b2Log("  bd.allowSleep = bool(%d);\n", this->m_flags & b2Body::e_autoSleepFlag);
	b2Log("  bd.awake = bool(%d);\n", this->m_flags & b2Body::e_awakeFlag);
	b2Log("  bd.fixedRotation = bool(%d);\n", this->m_flags & b2Body::e_fixedRotationFlag);
	b2Log("  bd.bullet = bool(%d);\n", this->m_flags & b2Body::e_bulletFlag);
	b2Log("  bd.active = bool(%d);\n", this->m_flags & b2Body::e_activeFlag);
	b2Log("  bd.gravityScale = %.15lef;\n", this->m_gravityScale);
	b2Log("  bodies[%d] = this->m_world->CreateBody(&bd);\n", this->m_islandIndex);
	b2Log("\n");
	for (b2Fixture* f = this->m_fixtureList; f; f = f->m_next)
	{
		b2Log("  {\n");
		f->Dump(bodyIndex);
		b2Log("  }\n");
	}
	b2Log("}\n");
}
