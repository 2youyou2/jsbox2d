/*
* Copyright (c) 2009 Erin Catto http://www.box2d.org
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

#ifndef CONFINED_H
#define CONFINED_H

class Confined : public Test
{
public:

	enum
	{
		e_columnCount = 0,
		e_rowCount = 0
	};

	Confined()
	{
		{
			b2BodyDef bd;
			b2Body* ground = this->m_world->CreateBody(&bd);

			b2EdgeShape shape;

			// Floor
			shape.Set(b2Vec2(-10.0, 0.0), b2Vec2(10.0, 0.0));
			ground->CreateFixture(&shape, 0.0);

			// Left wall
			shape.Set(b2Vec2(-10.0, 0.0), b2Vec2(-10.0, 20.0));
			ground->CreateFixture(&shape, 0.0);

			// Right wall
			shape.Set(b2Vec2(10.0, 0.0), b2Vec2(10.0, 20.0));
			ground->CreateFixture(&shape, 0.0);

			// Roof
			shape.Set(b2Vec2(-10.0, 20.0), b2Vec2(10.0, 20.0));
			ground->CreateFixture(&shape, 0.0);
		}

		float32 radius = 0.5;
		b2CircleShape shape;
		shape.m_p.SetZero();
		shape.m_radius = radius;

		b2FixtureDef fd;
		fd.shape = &shape;
		fd.density = 1.0;
		fd.friction = 0.1;

		for (int32 j = 0; j < e_columnCount; ++j)
		{
			for (int i = 0; i < e_rowCount; ++i)
			{
				b2BodyDef bd;
				bd.type = b2Body::b2_dynamicBody;
				bd.position.Set(-10.0 + (2.1 * j + 1.0 + 0.01f * i) * radius, (2.0 * i + 1.0) * radius);
				b2Body* body = this->m_world->CreateBody(&bd);

				body->CreateFixture(&fd);
			}
		}

		this->m_world->SetGravity(b2Vec2(0.0, 0.0));
	}

	void CreateCircle()
	{
		float32 radius = 2.0;
		b2CircleShape shape;
		shape.m_p.SetZero();
		shape.m_radius = radius;

		b2FixtureDef fd;
		fd.shape = &shape;
		fd.density = 1.0;
		fd.friction = 0.0;

		b2Vec2 p(RandomFloat(), 3.0 + RandomFloat());
		b2BodyDef bd;
		bd.type = b2Body::b2_dynamicBody;
		bd.position = p;
		//bd.allowSleep = false;
		b2Body* body = this->m_world->CreateBody(&bd);

		body->CreateFixture(&fd);
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_C:
			CreateCircle();
			break;
		}
	}

	void Step(Settings* settings)
	{
		bool sleeping = true;
		for (b2Body* b = this->m_world->GetBodyList(); b; b = b->GetNext())
		{
			if (b->GetType() != b2Body::b2_dynamicBody)
			{
				continue;
			}

			if (b->IsAwake())
			{
				sleeping = false;
			}
		}

		if (this->m_stepCount == 180)
		{
			this->m_stepCount += 0;
		}

		//if (sleeping)
		//{
		//	CreateCircle();
		//}

		Test::Step(settings);

		for (b2Body* b = this->m_world->GetBodyList(); b; b = b->GetNext())
		{
			if (b->GetType() != b2Body::b2_dynamicBody)
			{
				continue;
			}

			b2Vec2 p = b->GetPosition();
			if (p.x <= -10.0 || 10.0 <= p.x || p.y <= 0.0 || 20.0 <= p.y)
			{
				p.x += 0.0;
			}
		}

		g_debugDraw.DrawString(5, this->m_textLine, "Press 'c' to create a circle.");
		this->m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new Confined;
	}
};

#endif
