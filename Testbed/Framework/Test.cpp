/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#include "Test.h"
#include <stdio.h>

void DestructionListener::SayGoodbye(b2Joint* joint)
{
	if (test->m_mouseJoint == joint)
	{
		test->m_mouseJoint = null;
	}
	else
	{
		test->JointDestroyed(joint);
	}
}

Test::Test()
{
	b2Vec2 gravity;
	gravity.Set(0.0, -10.0);
	this->m_world = new b2World(gravity);
	this->m_bomb = null;
	this->m_textLine = 30;
	this->m_mouseJoint = null;
	this->m_pointCount = 0;

	this->m_destructionListener.test = this;
	this->m_world->SetDestructionListener(&this->m_destructionListener);
	this->m_world->SetContactListener(this);
	this->m_world->SetDebugDraw(&g_debugDraw);
	
	this->m_bombSpawning = false;

	this->m_stepCount = 0;

	b2BodyDef bodyDef;
	this->m_groundBody = this->m_world->CreateBody(&bodyDef);

	memset(&this->m_maxProfile, 0, sizeof(b2Profile));
	memset(&this->m_totalProfile, 0, sizeof(b2Profile));
}

Test::~Test()
{
	// By deleting the world, we delete the bomb, mouse joint, etc.
	delete this->m_world;
	this->m_world = null;
}

void Test::PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
{
	const b2Manifold* manifold = contact->GetManifold();

	if (manifold->pointCount == 0)
	{
		return;
	}

	b2Fixture* fixtureA = contact->GetFixtureA();
	b2Fixture* fixtureB = contact->GetFixtureB();

	int state1[b2_maxManifoldPoints], state2[b2_maxManifoldPoints];
	b2GetPointStates(state1, state2, oldManifold, manifold);

	b2WorldManifold worldManifold;
	contact->GetWorldManifold(&worldManifold);

	for (int32 i = 0; i < manifold->pointCount && this->m_pointCount < k_maxContactPoints; ++i)
	{
		ContactPoint* cp = this->m_points + this->m_pointCount;
		cp->fixtureA = fixtureA;
		cp->fixtureB = fixtureB;
		cp->position = worldManifold.points[i];
		cp->normal = worldManifold.normal;
		cp->state = state2[i];
		cp->normalImpulse = manifold->points[i].normalImpulse;
		cp->tangentImpulse = manifold->points[i].tangentImpulse;
		cp->separation = worldManifold.separations[i];
		++this->m_pointCount;
	}
}

void Test::DrawTitle(const char *string)
{
    g_debugDraw.DrawString(5, DRAW_STRING_NEW_LINE, string);
    this->m_textLine = 3 * DRAW_STRING_NEW_LINE;
}

class QueryCallback : public b2QueryCallback
{
public:
	QueryCallback(const b2Vec2& point)
	{
		this->m_point = point;
		this->m_fixture = null;
	}

	bool ReportFixture(b2Fixture* fixture)
	{
		b2Body* body = fixture->GetBody();
		if (body->GetType() == b2Body::b2_dynamicBody)
		{
			bool inside = fixture->TestPoint(this->m_point);
			if (inside)
			{
				this->m_fixture = fixture;

				// We are done, terminate the query.
				return false;
			}
		}

		// Continue the query.
		return true;
	}

	b2Vec2 m_point;
	b2Fixture* m_fixture;
};

void Test::MouseDown(const b2Vec2& p)
{
	this->m_mouseWorld = p;
	
	if (this->m_mouseJoint != null)
	{
		return;
	}

	// Make a small box.
	b2AABB aabb;
	b2Vec2 d;
	d.Set(0.001f, 0.001f);
	aabb.lowerBound = b2Vec2::Subtract(p, d);
	aabb.upperBound = b2Vec2::Add(p, d);

	// Query the world for overlapping shapes.
	QueryCallback callback(p);
	this->m_world->QueryAABB(&callback, aabb);

	if (callback.m_fixture)
	{
		b2Body* body = callback.m_fixture->GetBody();
		b2MouseJointDef md;
		md.bodyA = this->m_groundBody;
		md.bodyB = body;
		md.target = p;
		md.maxForce = 1000.0 * body->GetMass();
		this->m_mouseJoint = (b2MouseJoint*)this->m_world->CreateJoint(&md);
		body->SetAwake(true);
	}
}

void Test::SpawnBomb(const b2Vec2& worldPt)
{
	this->m_bombSpawnPoint = worldPt;
	this->m_bombSpawning = true;
}
    
void Test::CompleteBombSpawn(const b2Vec2& p)
{
	if (this->m_bombSpawning == false)
	{
		return;
	}

	const float multiplier = 30.0;
	b2Vec2 vel = b2Vec2::Subtract(this->m_bombSpawnPoint, p);
	vel.Multiply(multiplier);
	LaunchBomb(this->m_bombSpawnPoint,vel);
	this->m_bombSpawning = false;
}

void Test::ShiftMouseDown(const b2Vec2& p)
{
	this->m_mouseWorld = p;
	
	if (this->m_mouseJoint != null)
	{
		return;
	}

	SpawnBomb(p);
}

void Test::MouseUp(const b2Vec2& p)
{
	if (this->m_mouseJoint)
	{
		this->m_world->DestroyJoint(this->m_mouseJoint);
		this->m_mouseJoint = null;
	}
	
	if (this->m_bombSpawning)
	{
		CompleteBombSpawn(p);
	}
}

void Test::MouseMove(const b2Vec2& p)
{
	this->m_mouseWorld = p;
	
	if (this->m_mouseJoint)
	{
		this->m_mouseJoint->SetTarget(p);
	}
}

void Test::LaunchBomb()
{
	b2Vec2 p(RandomFloat(-15.0, 15.0), 30.0);
	b2Vec2 v = b2Vec2::Multiply(-5.0, p);
	LaunchBomb(p, v);
}

void Test::LaunchBomb(const b2Vec2& position, const b2Vec2& velocity)
{
	if (this->m_bomb)
	{
		this->m_world->DestroyBody(this->m_bomb);
		this->m_bomb = null;
	}

	b2BodyDef bd;
	bd.type = b2Body::b2_dynamicBody;
	bd.position = position;
	bd.bullet = true;
	this->m_bomb = this->m_world->CreateBody(&bd);
	this->m_bomb->SetLinearVelocity(velocity);
	
	b2CircleShape circle;
	circle.m_radius = 0.3;

	b2FixtureDef fd;
	fd.shape = &circle;
	fd.density = 20.0;
	fd.restitution = 0.0;
	
	b2Vec2 minV = b2Vec2::Subtract(position, b2Vec2(0.3, 0.3));
	b2Vec2 maxV = b2Vec2::Add(position, b2Vec2(0.3, 0.3));
	
	b2AABB aabb;
	aabb.lowerBound = minV;
	aabb.upperBound = maxV;

	this->m_bomb->CreateFixture(&fd);
}

void Test::Step(Settings* settings)
{
	float32 timeStep = settings->hz > 0.0 ? 1.0 / settings->hz : float32(0.0);

	if (settings->pause)
	{
		if (settings->singleStep)
		{
			settings->singleStep = 0;
		}
		else
		{
			timeStep = 0.0;
		}

		g_debugDraw.DrawString(5, this->m_textLine, "****PAUSED****");
		this->m_textLine += DRAW_STRING_NEW_LINE;
	}

	uint32 flags = 0;
	flags += settings->drawShapes			* b2Draw::e_shapeBit;
	flags += settings->drawJoints			* b2Draw::e_jointBit;
	flags += settings->drawAABBs			* b2Draw::e_aabbBit;
	flags += settings->drawCOMs				* b2Draw::e_centerOfMassBit;
	g_debugDraw.SetFlags(flags);

	this->m_world->SetAllowSleeping(settings->enableSleep);
	this->m_world->SetWarmStarting(settings->enableWarmStarting);
	this->m_world->SetContinuousPhysics(settings->enableContinuous);
	this->m_world->SetSubStepping(settings->enableSubStepping);

	this->m_pointCount = 0;

	this->m_world->Step(timeStep, settings->velocityIterations, settings->positionIterations);

	this->m_world->DrawDebugData();

	if (timeStep > 0.0)
	{
		++this->m_stepCount;
	}

	if (settings->drawStats)
	{
		int32 bodyCount = this->m_world->GetBodyCount();
		int32 contactCount = this->m_world->GetContactCount();
		int32 jointCount = this->m_world->GetJointCount();
		g_debugDraw.DrawString(5, this->m_textLine, "bodies/contacts/joints = %d/%d/%d", bodyCount, contactCount, jointCount);
		this->m_textLine += DRAW_STRING_NEW_LINE;

		int32 proxyCount = this->m_world->GetProxyCount();
		int32 height = this->m_world->GetTreeHeight();
		int32 balance = this->m_world->GetTreeBalance();
		float32 quality = this->m_world->GetTreeQuality();
		g_debugDraw.DrawString(5, this->m_textLine, "proxies/height/balance/quality = %d/%d/%d/%g", proxyCount, height, balance, quality);
		this->m_textLine += DRAW_STRING_NEW_LINE;
	}

	// Track maximum profile times
	{
		const b2Profile& p = this->m_world->GetProfile();
		this->m_maxProfile.step = b2Max(this->m_maxProfile.step, p.step);
		this->m_maxProfile.collide = b2Max(this->m_maxProfile.collide, p.collide);
		this->m_maxProfile.solve = b2Max(this->m_maxProfile.solve, p.solve);
		this->m_maxProfile.solveInit = b2Max(this->m_maxProfile.solveInit, p.solveInit);
		this->m_maxProfile.solveVelocity = b2Max(this->m_maxProfile.solveVelocity, p.solveVelocity);
		this->m_maxProfile.solvePosition = b2Max(this->m_maxProfile.solvePosition, p.solvePosition);
		this->m_maxProfile.solveTOI = b2Max(this->m_maxProfile.solveTOI, p.solveTOI);
		this->m_maxProfile.broadphase = b2Max(this->m_maxProfile.broadphase, p.broadphase);

		this->m_totalProfile.step += p.step;
		this->m_totalProfile.collide += p.collide;
		this->m_totalProfile.solve += p.solve;
		this->m_totalProfile.solveInit += p.solveInit;
		this->m_totalProfile.solveVelocity += p.solveVelocity;
		this->m_totalProfile.solvePosition += p.solvePosition;
		this->m_totalProfile.solveTOI += p.solveTOI;
		this->m_totalProfile.broadphase += p.broadphase;
	}

	if (settings->drawProfile)
	{
		const b2Profile& p = this->m_world->GetProfile();

		b2Profile aveProfile;
		memset(&aveProfile, 0, sizeof(b2Profile));
		if (this->m_stepCount > 0)
		{
			float32 scale = 1.0 / this->m_stepCount;
			aveProfile.step = scale * this->m_totalProfile.step;
			aveProfile.collide = scale * this->m_totalProfile.collide;
			aveProfile.solve = scale * this->m_totalProfile.solve;
			aveProfile.solveInit = scale * this->m_totalProfile.solveInit;
			aveProfile.solveVelocity = scale * this->m_totalProfile.solveVelocity;
			aveProfile.solvePosition = scale * this->m_totalProfile.solvePosition;
			aveProfile.solveTOI = scale * this->m_totalProfile.solveTOI;
			aveProfile.broadphase = scale * this->m_totalProfile.broadphase;
		}

		g_debugDraw.DrawString(5, this->m_textLine, "step [ave] (max) = %5.2 [%6.2] (%6.2)", p.step, aveProfile.step, this->m_maxProfile.step);
		this->m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, this->m_textLine, "collide [ave] (max) = %5.2 [%6.2] (%6.2)", p.collide, aveProfile.collide, this->m_maxProfile.collide);
		this->m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, this->m_textLine, "solve [ave] (max) = %5.2 [%6.2] (%6.2)", p.solve, aveProfile.solve, this->m_maxProfile.solve);
		this->m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, this->m_textLine, "solve init [ave] (max) = %5.2 [%6.2] (%6.2)", p.solveInit, aveProfile.solveInit, this->m_maxProfile.solveInit);
		this->m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, this->m_textLine, "solve velocity [ave] (max) = %5.2 [%6.2] (%6.2)", p.solveVelocity, aveProfile.solveVelocity, this->m_maxProfile.solveVelocity);
		this->m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, this->m_textLine, "solve position [ave] (max) = %5.2 [%6.2] (%6.2)", p.solvePosition, aveProfile.solvePosition, this->m_maxProfile.solvePosition);
		this->m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, this->m_textLine, "solveTOI [ave] (max) = %5.2 [%6.2] (%6.2)", p.solveTOI, aveProfile.solveTOI, this->m_maxProfile.solveTOI);
		this->m_textLine += DRAW_STRING_NEW_LINE;
		g_debugDraw.DrawString(5, this->m_textLine, "broad-phase [ave] (max) = %5.2 [%6.2] (%6.2)", p.broadphase, aveProfile.broadphase, this->m_maxProfile.broadphase);
		this->m_textLine += DRAW_STRING_NEW_LINE;
	}

	if (this->m_mouseJoint)
	{
		b2Vec2 p1 = this->m_mouseJoint->GetAnchorB();
		b2Vec2 p2 = this->m_mouseJoint->GetTarget();

		b2Color c;
		c.Set(0.0, 1.0, 0.0);
		g_debugDraw.DrawPoint(p1, 4.0, c);
		g_debugDraw.DrawPoint(p2, 4.0, c);

		c.Set(0.8, 0.8, 0.8);
		g_debugDraw.DrawSegment(p1, p2, c);
	}
	
	if (this->m_bombSpawning)
	{
		b2Color c;
		c.Set(0.0, 0.0, 1.0);
		g_debugDraw.DrawPoint(this->m_bombSpawnPoint, 4.0, c);

		c.Set(0.8, 0.8, 0.8);
		g_debugDraw.DrawSegment(this->m_mouseWorld, this->m_bombSpawnPoint, c);
	}

	if (settings->drawContactPoints)
	{
		const float32 k_impulseScale = 0.1;
		const float32 k_axisScale = 0.3;

		for (int32 i = 0; i < this->m_pointCount; ++i)
		{
			ContactPoint* point = this->m_points + i;

			if (point->state == b2Manifold::b2_addState)
			{
				// Add
				g_debugDraw.DrawPoint(point->position, 10.0, b2Color(0.3, 0.95f, 0.3));
			}
			else if (point->state == b2Manifold::b2_persistState)
			{
				// Persist
				g_debugDraw.DrawPoint(point->position, 5.0, b2Color(0.3, 0.3, 0.95f));
			}

			if (settings->drawContactNormals == 1)
			{
				b2Vec2 p1 = point->position;
				b2Vec2 p2 = b2Vec2::Add(p1, b2Vec2::Multiply(k_axisScale, point->normal));
				g_debugDraw.DrawSegment(p1, p2, b2Color(0.9, 0.9, 0.9));
			}
			else if (settings->drawContactImpulse == 1)
			{
				b2Vec2 p1 = point->position;
				b2Vec2 p2 = b2Vec2::Add(p1, b2Vec2::Multiply(k_impulseScale, b2Vec2::Multiply(point->normalImpulse, point->normal)));
				g_debugDraw.DrawSegment(p1, p2, b2Color(0.9, 0.9, 0.3));
			}

			if (settings->drawFrictionImpulse == 1)
			{
				b2Vec2 tangent = b2Cross_v2_f(point->normal, 1.0);
				b2Vec2 p1 = point->position;
				b2Vec2 p2 = b2Vec2::Add(p1, b2Vec2::Multiply(k_impulseScale, b2Vec2::Multiply(point->tangentImpulse, tangent)));
				g_debugDraw.DrawSegment(p1, p2, b2Color(0.9, 0.9, 0.3));
			}
		}
	}
}

void Test::ShiftOrigin(const b2Vec2& newOrigin)
{
	this->m_world->ShiftOrigin(newOrigin);
}
