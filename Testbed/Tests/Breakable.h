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

#ifndef BREAKABLE_TEST_H
#define BREAKABLE_TEST_H

// This is used to test sensor shapes.
class Breakable : public Test
{
public:

	enum
	{
		e_count = 7
	};

	Breakable()
	{
		// Ground body
		{
			b2BodyDef bd;
			b2Body* ground = this->m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0, 0.0), b2Vec2(40.0, 0.0));
			ground->CreateFixture(&shape, 0.0);
		}

		// Breakable dynamic body
		{
			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;
			bd.position.Set(0.0, 40.0);
			bd.angle = 0.25f * b2_pi;
			this->m_body1 = this->m_world->CreateBody(&bd);

			this->m_shape1.SetAsBox(0.5, 0.5, b2Vec2(-0.5, 0.0), 0.0);
			this->m_piece1 = this->m_body1->CreateFixture(&this->m_shape1, 1.0);

			this->m_shape2.SetAsBox(0.5, 0.5, b2Vec2(0.5, 0.0), 0.0);
			this->m_piece2 = this->m_body1->CreateFixture(&this->m_shape2, 1.0);
		}

		this->m_break = false;
		this->m_broke = false;
	}

	void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse)
	{
		if (this->m_broke)
		{
			// The body already broke.
			return;
		}

		// Should the body break?
		int32 count = contact->GetManifold()->pointCount;

		float32 maxImpulse = 0.0;
		for (int32 i = 0; i < count; ++i)
		{
			maxImpulse = b2Max(maxImpulse, impulse->normalImpulses[i]);
		}

		if (maxImpulse > 40.0)
		{
			// Flag the body for breaking.
			this->m_break = true;
		}
	}

	void Break()
	{
		// Create two bodies from one.
		b2Body* body1 = this->m_piece1->GetBody();
		b2Vec2 center = body1->GetWorldCenter();

		body1->DestroyFixture(this->m_piece2);
		this->m_piece2 = null;

		b2BodyDef bd;
		bd.type = b2Body::b2_dynamicBody;
		bd.position.Assign(body1->GetPosition());
		bd.angle = body1->GetAngle();

		b2Body* body2 = this->m_world->CreateBody(&bd);
		this->m_piece2 = body2->CreateFixture(&this->m_shape2, 1.0);

		// Compute consistent velocities for new bodies based on
		// cached velocity.
		b2Vec2 center1 = body1->GetWorldCenter();
		b2Vec2 center2 = body2->GetWorldCenter();
		
		b2Vec2 velocity1 = b2Vec2::Add(this->m_velocity, b2Cross_f_v2(this->m_angularVelocity, b2Vec2::Subtract(center1, center)));
		b2Vec2 velocity2 = b2Vec2::Add(this->m_velocity, b2Cross_f_v2(this->m_angularVelocity, b2Vec2::Subtract(center2, center)));

		body1->SetAngularVelocity(this->m_angularVelocity);
		body1->SetLinearVelocity(velocity1);

		body2->SetAngularVelocity(this->m_angularVelocity);
		body2->SetLinearVelocity(velocity2);
	}

	void Step(Settings* settings)
	{
		if (this->m_break)
		{
			Break();
			this->m_broke = true;
			this->m_break = false;
		}

		// Cache velocities to improve movement on breakage.
		if (this->m_broke == false)
		{
			this->m_velocity.Assign(this->m_body1->GetLinearVelocity());
			this->m_angularVelocity = this->m_body1->GetAngularVelocity();
		}

		Test::Step(settings);
	}

	static Test* Create()
	{
		return new Breakable;
	}

	b2Body* m_body1;
	b2Vec2 m_velocity;
	float32 m_angularVelocity;
	b2PolygonShape m_shape1;
	b2PolygonShape m_shape2;
	b2Fixture* m_piece1;
	b2Fixture* m_piece2;

	bool m_broke;
	bool m_break;
};

#endif
