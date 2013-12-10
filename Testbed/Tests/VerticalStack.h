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

#ifndef VERTICAL_STACK_H
#define VERTICAL_STACK_H

class VerticalStack : public Test
{
public:

	enum
	{
		e_columnCount = 5,
		e_rowCount = 16
		//e_columnCount = 1,
		//e_rowCount = 1
	};

	VerticalStack()
	{
		{
			b2BodyDef bd;
			b2Body* ground = this->m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0, 0.0), b2Vec2(40.0, 0.0));
			ground->CreateFixture(&shape, 0.0);

			shape.Set(b2Vec2(20.0, 0.0), b2Vec2(20.0, 20.0));
			ground->CreateFixture(&shape, 0.0);
		}

		float32 xs[5] = {0.0, -10.0, -5.0, 5.0, 10.0};

		for (int32 j = 0; j < e_columnCount; ++j)
		{
			b2PolygonShape shape;
			shape.SetAsBox(0.5, 0.5);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 1.0;
			fd.friction = 0.3;

			for (int i = 0; i < e_rowCount; ++i)
			{
				b2BodyDef bd;
				bd.type = b2Body::b2_dynamicBody;

				int32 n = j * e_rowCount + i;
				b2Assert(n < e_rowCount * e_columnCount);
				this->m_indices[n] = n;
				bd.userData = this->m_indices + n;

				float32 x = 0.0;
				//float32 x = RandomFloat(-0.02f, 0.02f);
				//float32 x = i % 2 == 0 ? -0.025f : 0.025f;
				bd.position.Set(xs[j] + x, 0.752f + 1.54f * i);
				b2Body* body = this->m_world->CreateBody(&bd);

				this->m_bodies[n] = body;

				body->CreateFixture(&fd);
			}
		}

		this->m_bullet = null;
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_COMMA:
			if (this->m_bullet != null)
			{
				this->m_world->DestroyBody(this->m_bullet);
				this->m_bullet = null;
			}

			{
				b2CircleShape shape;
				shape.m_radius = 0.25f;

				b2FixtureDef fd;
				fd.shape = &shape;
				fd.density = 20.0;
				fd.restitution = 0.05f;

				b2BodyDef bd;
				bd.type = b2Body::b2_dynamicBody;
				bd.bullet = true;
				bd.position.Set(-31.0, 5.0);

				this->m_bullet = this->m_world->CreateBody(&bd);
				this->m_bullet->CreateFixture(&fd);

				this->m_bullet->SetLinearVelocity(b2Vec2(400.0, 0.0));
			}
			break;
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);
		g_debugDraw.DrawString(5, this->m_textLine, "Press: (,) to launch a bullet.");
		this->m_textLine += DRAW_STRING_NEW_LINE;

		//if (this->m_stepCount == 300)
		//{
		//	if (this->m_bullet != null)
		//	{
		//		this->m_world->DestroyBody(this->m_bullet);
		//		this->m_bullet = null;
		//	}

		//	{
		//		b2CircleShape shape;
		//		shape.m_radius = 0.25f;

		//		b2FixtureDef fd;
		//		fd.shape = &shape;
		//		fd.density = 20.0;
		//		fd.restitution = 0.05f;

		//		b2BodyDef bd;
		//		bd.type = b2Body::b2_dynamicBody;
		//		bd.bullet = true;
		//		bd.position.Set(-31.0, 5.0);

		//		this->m_bullet = this->m_world->CreateBody(&bd);
		//		this->m_bullet->CreateFixture(&fd);

		//		this->m_bullet->SetLinearVelocity(b2Vec2(400.0, 0.0));
		//	}
		//}
	}

	static Test* Create()
	{
		return new VerticalStack;
	}

	b2Body* m_bullet;
	b2Body* m_bodies[e_rowCount * e_columnCount];
	int32 m_indices[e_rowCount * e_columnCount];
};

#endif
