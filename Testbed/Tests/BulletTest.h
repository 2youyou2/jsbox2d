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

#ifndef BULLET_TEST_H
#define BULLET_TEST_H

class BulletTest : public Test
{
public:

	BulletTest()
	{
		{
			b2BodyDef bd;
			bd.position.Set(0.0, 0.0);
			b2Body* body = this->m_world->CreateBody(&bd);

			b2EdgeShape edge;

			edge.Set(b2Vec2(-10.0, 0.0), b2Vec2(10.0, 0.0));
			body->CreateFixture(&edge, 0.0);

			b2PolygonShape shape;
			shape.SetAsBox(0.2, 1.0, b2Vec2(0.5, 1.0), 0.0);
			body->CreateFixture(&shape, 0.0);
		}

		{
			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;
			bd.position.Set(0.0, 4.0);

			b2PolygonShape box;
			box.SetAsBox(2.0, 0.1);

			this->m_body = this->m_world->CreateBody(&bd);
			this->m_body->CreateFixture(&box, 1.0);

			box.SetAsBox(0.25f, 0.25f);

			//this->m_x = RandomFloat(-1.0, 1.0);
			this->m_x = 0.20352793f;
			bd.position.Set(this->m_x, 10.0);
			bd.bullet = true;

			this->m_bullet = this->m_world->CreateBody(&bd);
			this->m_bullet->CreateFixture(&box, 100.0);

			this->m_bullet->SetLinearVelocity(b2Vec2(0.0, -50.0));
		}
	}

	void Launch()
	{
		this->m_body->SetTransform(b2Vec2(0.0, 4.0), 0.0);
		this->m_body->SetLinearVelocity(b2Vec2(0, 0));
		this->m_body->SetAngularVelocity(0.0);

		this->m_x = RandomFloat(-1.0, 1.0);
		this->m_bullet->SetTransform(b2Vec2(this->m_x, 10.0), 0.0);
		this->m_bullet->SetLinearVelocity(b2Vec2(0.0, -50.0));
		this->m_bullet->SetAngularVelocity(0.0);

		extern int32 b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters;
		extern int32 b2_toiCalls, b2_toiIters, b2_toiMaxIters;
		extern int32 b2_toiRootIters, b2_toiMaxRootIters;

		b2_gjkCalls = 0;
		b2_gjkIters = 0;
		b2_gjkMaxIters = 0;

		b2_toiCalls = 0;
		b2_toiIters = 0;
		b2_toiMaxIters = 0;
		b2_toiRootIters = 0;
		b2_toiMaxRootIters = 0;
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		extern int32 b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters;
		extern int32 b2_toiCalls, b2_toiIters;
		extern int32 b2_toiRootIters, b2_toiMaxRootIters;

		if (b2_gjkCalls > 0)
		{
			g_debugDraw.DrawString(5, this->m_textLine, "gjk calls = %d, ave gjk iters = %3.1, max gjk iters = %d",
				b2_gjkCalls, b2_gjkIters / float32(b2_gjkCalls), b2_gjkMaxIters);
			this->m_textLine += DRAW_STRING_NEW_LINE;
		}

		if (b2_toiCalls > 0)
		{
			g_debugDraw.DrawString(5, this->m_textLine, "toi calls = %d, ave toi iters = %3.1, max toi iters = %d",
				b2_toiCalls, b2_toiIters / float32(b2_toiCalls), b2_toiMaxRootIters);
			this->m_textLine += DRAW_STRING_NEW_LINE;

			g_debugDraw.DrawString(5, this->m_textLine, "ave toi root iters = %3.1, max toi root iters = %d",
				b2_toiRootIters / float32(b2_toiCalls), b2_toiMaxRootIters);
			this->m_textLine += DRAW_STRING_NEW_LINE;
		}

		if (this->m_stepCount % 60 == 0)
		{
			Launch();
		}
	}

	static Test* Create()
	{
		return new BulletTest;
	}

	b2Body* m_body;
	b2Body* m_bullet;
	float32 m_x;
};

#endif
