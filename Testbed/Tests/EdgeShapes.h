/*
* Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#ifndef EDGE_SHAPES_H
#define EDGE_SHAPES_H

class EdgeShapesCallback : public b2RayCastCallback
{
public:
	EdgeShapesCallback()
	{
		this->m_fixture = null;
	}

	float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point,
						  const b2Vec2& normal, float32 fraction)
	{
		this->m_fixture = fixture;
		this->m_point = point;
		this->m_normal = normal;

		return fraction;
	}

	b2Fixture* m_fixture;
	b2Vec2 m_point;
	b2Vec2 m_normal;
};

class EdgeShapes : public Test
{
public:

	enum
	{
		e_maxBodies = 256
	};

	EdgeShapes()
	{
		// Ground body
		{
			b2BodyDef bd;
			b2Body* ground = this->m_world->CreateBody(&bd);

			float32 x1 = -20.0;
			float32 y1 = 2.0 * cosf(x1 / 10.0 * b2_pi);
			for (int32 i = 0; i < 80; ++i)
			{
				float32 x2 = x1 + 0.5;
				float32 y2 = 2.0 * cosf(x2 / 10.0 * b2_pi);

				b2EdgeShape shape;
				shape.Set(b2Vec2(x1, y1), b2Vec2(x2, y2));
				ground->CreateFixture(&shape, 0.0);

				x1 = x2;
				y1 = y2;
			}
		}

		{
			b2Vec2 vertices[3];
			vertices[0].Set(-0.5, 0.0);
			vertices[1].Set(0.5, 0.0);
			vertices[2].Set(0.0, 1.5);
			this->m_polygons[0].Set(vertices, 3);
		}

		{
			b2Vec2 vertices[3];
			vertices[0].Set(-0.1, 0.0);
			vertices[1].Set(0.1, 0.0);
			vertices[2].Set(0.0, 1.5);
			this->m_polygons[1].Set(vertices, 3);
		}

		{
			float32 w = 1.0;
			float32 b = w / (2.0 + b2Sqrt(2.0));
			float32 s = b2Sqrt(2.0) * b;

			b2Vec2 vertices[8];
			vertices[0].Set(0.5 * s, 0.0);
			vertices[1].Set(0.5 * w, b);
			vertices[2].Set(0.5 * w, b + s);
			vertices[3].Set(0.5 * s, w);
			vertices[4].Set(-0.5 * s, w);
			vertices[5].Set(-0.5 * w, b + s);
			vertices[6].Set(-0.5 * w, b);
			vertices[7].Set(-0.5 * s, 0.0);

			this->m_polygons[2].Set(vertices, 8);
		}

		{
			this->m_polygons[3].SetAsBox(0.5, 0.5);
		}

		{
			this->m_circle.m_radius = 0.5;
		}

		this->m_bodyIndex = 0;
		memset(this->m_bodies, 0, sizeof(this->m_bodies));

		this->m_angle = 0.0;
	}

	void Create(int32 index)
	{
		if (this->m_bodies[this->m_bodyIndex] != null)
		{
			this->m_world->DestroyBody(this->m_bodies[this->m_bodyIndex]);
			this->m_bodies[this->m_bodyIndex] = null;
		}

		b2BodyDef bd;

		float32 x = RandomFloat(-10.0, 10.0);
		float32 y = RandomFloat(10.0, 20.0);
		bd.position.Set(x, y);
		bd.angle = RandomFloat(-b2_pi, b2_pi);
		bd.type = b2Body::b2_dynamicBody;

		if (index == 4)
		{
			bd.angularDamping = 0.02f;
		}

		this->m_bodies[this->m_bodyIndex] = this->m_world->CreateBody(&bd);

		if (index < 4)
		{
			b2FixtureDef fd;
			fd.shape = this->m_polygons + index;
			fd.friction = 0.3;
			fd.density = 20.0;
			this->m_bodies[this->m_bodyIndex]->CreateFixture(&fd);
		}
		else
		{
			b2FixtureDef fd;
			fd.shape = &this->m_circle;
			fd.friction = 0.3;
			fd.density = 20.0;
			this->m_bodies[this->m_bodyIndex]->CreateFixture(&fd);
		}

		this->m_bodyIndex = (this->m_bodyIndex + 1) % e_maxBodies;
	}

	void DestroyBody()
	{
		for (int32 i = 0; i < e_maxBodies; ++i)
		{
			if (this->m_bodies[i] != null)
			{
				this->m_world->DestroyBody(this->m_bodies[i]);
				this->m_bodies[i] = null;
				return;
			}
		}
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_1:
		case GLFW_KEY_2:
		case GLFW_KEY_3:
		case GLFW_KEY_4:
		case GLFW_KEY_5:
			Create(key - GLFW_KEY_1);
			break;

		case GLFW_KEY_D:
			DestroyBody();
			break;
		}
	}

	void Step(Settings* settings)
	{
		bool advanceRay = settings->pause == 0 || settings->singleStep;

		Test::Step(settings);
		g_debugDraw.DrawString(5, this->m_textLine, "Press 1-5 to drop stuff");
		this->m_textLine += DRAW_STRING_NEW_LINE;

		float32 L = 25.0;
		b2Vec2 point1(0.0, 10.0);
		b2Vec2 d(L * cosf(this->m_angle), -L * b2Abs(sinf(this->m_angle)));
		b2Vec2 point2 = b2Vec2::Add(point1, d);

		EdgeShapesCallback callback;

		this->m_world->RayCast(&callback, point1, point2);

		if (callback.m_fixture)
		{
			g_debugDraw.DrawPoint(callback.m_point, 5.0, b2Color(0.4, 0.9, 0.4));

			g_debugDraw.DrawSegment(point1, callback.m_point, b2Color(0.8, 0.8, 0.8));

			b2Vec2 head = b2Vec2::Add(callback.m_point, b2Vec2::Multiply(0.5, callback.m_normal));
			g_debugDraw.DrawSegment(callback.m_point, head, b2Color(0.9, 0.9, 0.4));
		}
		else
		{
			g_debugDraw.DrawSegment(point1, point2, b2Color(0.8, 0.8, 0.8));
		}

		if (advanceRay)
		{
			this->m_angle += 0.25f * b2_pi / 180.0;
		}
	}

	static Test* Create()
	{
		return new EdgeShapes;
	}

	int32 m_bodyIndex;
	b2Body* m_bodies[e_maxBodies];
	b2PolygonShape m_polygons[4];
	b2CircleShape m_circle;

	float32 m_angle;
};

#endif
