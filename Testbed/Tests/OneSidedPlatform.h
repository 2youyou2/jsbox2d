/*
* Copyright (c) 2008-2009 Erin Catto http://www.box2d.org
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

#ifndef ONE_SIDED_PLATFORM_H
#define ONE_SIDED_PLATFORM_H

class OneSidedPlatform : public Test
{
public:

	enum State
	{
		e_unknown,
		e_above,
		e_below
	};

	OneSidedPlatform()
	{
		// Ground
		{
			b2BodyDef bd;
			b2Body* ground = this->m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-20.0, 0.0), b2Vec2(20.0, 0.0));
			ground->CreateFixture(&shape, 0.0);
		}

		// Platform
		{
			b2BodyDef bd;
			bd.position.Set(0.0, 10.0);
			b2Body* body = this->m_world->CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsBox(3.0, 0.5);
			this->m_platform = body->CreateFixture(&shape, 0.0);

			this->m_bottom = 10.0 - 0.5;
			this->m_top = 10.0 + 0.5;
		}

		// Actor
		{
			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;
			bd.position.Set(0.0, 12.0);
			b2Body* body = this->m_world->CreateBody(&bd);

			this->m_radius = 0.5;
			b2CircleShape shape;
			shape.m_radius = this->m_radius;
			this->m_character = body->CreateFixture(&shape, 20.0);

			body->SetLinearVelocity(b2Vec2(0.0, -50.0));

			this->m_state = e_unknown;
		}
	}

	void PreSolve(b2Contact* contact, const b2Manifold* oldManifold)
	{
		Test::PreSolve(contact, oldManifold);

		b2Fixture* fixtureA = contact->GetFixtureA();
		b2Fixture* fixtureB = contact->GetFixtureB();

		if (fixtureA != this->m_platform && fixtureA != this->m_character)
		{
			return;
		}

		if (fixtureB != this->m_platform && fixtureB != this->m_character)
		{
			return;
		}

#if 1
		b2Vec2 position = this->m_character->GetBody()->GetPosition();

		if (position.y < this->m_top + this->m_radius - 3.0 * b2_linearSlop)
		{
			contact->SetEnabled(false);
		}
#else
        b2Vec2 v = this->m_character->GetBody()->GetLinearVelocity();
        if (v.y > 0.0)
		{
            contact->SetEnabled(false);
        }
#endif
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);
		g_debugDraw.DrawString(5, this->m_textLine, "Press: (c) create a shape, (d) destroy a shape.");
		this->m_textLine += DRAW_STRING_NEW_LINE;

        b2Vec2 v = this->m_character->GetBody()->GetLinearVelocity();
        g_debugDraw.DrawString(5, this->m_textLine, "Character Linear Velocity: %f", v.y);
		this->m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new OneSidedPlatform;
	}

	float32 m_radius, m_top, m_bottom;
	State m_state;
	b2Fixture* m_platform;
	b2Fixture* m_character;
};

#endif
