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

#ifndef RAY_CAST_H
#define RAY_CAST_H

// This test demonstrates how to use the world ray-cast feature.
// NOTE: we are intentionally filtering one of the polygons, therefore
// the ray will always miss one type of polygon.

// This callback finds the closest hit. Polygon 0 is filtered.
class RayCastClosestCallback : public b2RayCastCallback
{
public:
	RayCastClosestCallback()
	{
		this->m_hit = false;
	}

	float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction)
	{
		b2Body* body = fixture->GetBody();
		void* userData = body->GetUserData();
		if (userData)
		{
			int32 index = *(int32*)userData;
			if (index == 0)
			{
				// By returning -1, we instruct the calling code to ignore this fixture and
				// continue the ray-cast to the next fixture.
				return -1.0;
			}
		}

		this->m_hit = true;
		this->m_point = point;
		this->m_normal = normal;

		// By returning the current fraction, we instruct the calling code to clip the ray and
		// continue the ray-cast to the next fixture. WARNING: do not assume that fixtures
		// are reported in order. However, by clipping, we can always get the closest fixture.
		return fraction;
	}
	
	bool m_hit;
	b2Vec2 m_point;
	b2Vec2 m_normal;
};

// This callback finds any hit. Polygon 0 is filtered. For this type of query we are usually
// just checking for obstruction, so the actual fixture and hit point are irrelevant. 
class RayCastAnyCallback : public b2RayCastCallback
{
public:
	RayCastAnyCallback()
	{
		this->m_hit = false;
	}

	float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction)
	{
		b2Body* body = fixture->GetBody();
		void* userData = body->GetUserData();
		if (userData)
		{
			int32 index = *(int32*)userData;
			if (index == 0)
			{
				// By returning -1, we instruct the calling code to ignore this fixture
				// and continue the ray-cast to the next fixture.
				return -1.0;
			}
		}

		this->m_hit = true;
		this->m_point = point;
		this->m_normal = normal;

		// At this point we have a hit, so we know the ray is obstructed.
		// By returning 0, we instruct the calling code to terminate the ray-cast.
		return 0.0;
	}

	bool m_hit;
	b2Vec2 m_point;
	b2Vec2 m_normal;
};

// This ray cast collects multiple hits along the ray. Polygon 0 is filtered.
// The fixtures are not necessary reported in order, so we might not capture
// the closest fixture.
class RayCastMultipleCallback : public b2RayCastCallback
{
public:
	enum
	{
		e_maxCount = 3
	};

	RayCastMultipleCallback()
	{
		this->m_count = 0;
	}

	float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction)
	{
		b2Body* body = fixture->GetBody();
		void* userData = body->GetUserData();
		if (userData)
		{
			int32 index = *(int32*)userData;
			if (index == 0)
			{
				// By returning -1, we instruct the calling code to ignore this fixture
				// and continue the ray-cast to the next fixture.
				return -1.0;
			}
		}

		b2Assert(this->m_count < e_maxCount);

		this->m_points[this->m_count] = point;
		this->m_normals[this->m_count] = normal;
		++this->m_count;

		if (this->m_count == e_maxCount)
		{
			// At this point the buffer is full.
			// By returning 0, we instruct the calling code to terminate the ray-cast.
			return 0.0;
		}

		// By returning 1, we instruct the caller to continue without clipping the ray.
		return 1.0;
	}

	b2Vec2 m_points[e_maxCount];
	b2Vec2 m_normals[e_maxCount];
	int32 m_count;
};


class RayCast : public Test
{
public:

	enum
	{
		e_maxBodies = 256
	};

	enum Mode
	{
		e_closest,
		e_any,
		e_multiple
	};

	RayCast()
	{
		// Ground body
		{
			b2BodyDef bd;
			b2Body* ground = this->m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0, 0.0), b2Vec2(40.0, 0.0));
			ground->CreateFixture(&shape, 0.0);
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

		{
			this->m_edge.Set(b2Vec2(-1.0, 0.0), b2Vec2(1.0, 0.0));
		}

		this->m_bodyIndex = 0;
		memset(this->m_bodies, 0, sizeof(this->m_bodies));

		this->m_angle = 0.0;

		this->m_mode = e_closest;
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
		float32 y = RandomFloat(0.0, 20.0);
		bd.position.Set(x, y);
		bd.angle = RandomFloat(-b2_pi, b2_pi);

		this->m_userData[this->m_bodyIndex] = index;
		bd.userData = this->m_userData + this->m_bodyIndex;

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
			this->m_bodies[this->m_bodyIndex]->CreateFixture(&fd);
		}
		else if (index < 5)
		{
			b2FixtureDef fd;
			fd.shape = &this->m_circle;
			fd.friction = 0.3;

			this->m_bodies[this->m_bodyIndex]->CreateFixture(&fd);
		}
		else
		{
			b2FixtureDef fd;
			fd.shape = &this->m_edge;
			fd.friction = 0.3;

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
		case GLFW_KEY_6:
			Create(key - GLFW_KEY_1);
			break;

		case GLFW_KEY_D:
			DestroyBody();
			break;

		case GLFW_KEY_M:
			if (this->m_mode == e_closest)
			{
				this->m_mode = e_any;
			}
			else if (this->m_mode == e_any)
			{
				this->m_mode = e_multiple;
			}
			else if (this->m_mode == e_multiple)
			{
				this->m_mode = e_closest;
			}
		}
	}

	void Step(Settings* settings)
	{
		bool advanceRay = settings->pause == 0 || settings->singleStep;

		Test::Step(settings);
		g_debugDraw.DrawString(5, this->m_textLine, "Press 1-6 to drop stuff, m to change the mode");
		this->m_textLine += DRAW_STRING_NEW_LINE;
		switch (this->m_mode)
		{
		case e_closest:
			g_debugDraw.DrawString(5, this->m_textLine, "Ray-cast mode: closest - find closest fixture along the ray");
			break;
		
		case e_any:
			g_debugDraw.DrawString(5, this->m_textLine, "Ray-cast mode: any - check for obstruction");
			break;

		case e_multiple:
			g_debugDraw.DrawString(5, this->m_textLine, "Ray-cast mode: multiple - gather multiple fixtures");
			break;
		}

		this->m_textLine += DRAW_STRING_NEW_LINE;

		float32 L = 11.0;
		b2Vec2 point1(0.0, 10.0);
		b2Vec2 d(L * cosf(this->m_angle), L * sinf(this->m_angle));
		b2Vec2 point2 = b2Vec2::Add(point1, d);

		if (this->m_mode == e_closest)
		{
			RayCastClosestCallback callback;
			this->m_world->RayCast(&callback, point1, point2);

			if (callback.m_hit)
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
		}
		else if (this->m_mode == e_any)
		{
			RayCastAnyCallback callback;
			this->m_world->RayCast(&callback, point1, point2);

			if (callback.m_hit)
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
		}
		else if (this->m_mode == e_multiple)
		{
			RayCastMultipleCallback callback;
			this->m_world->RayCast(&callback, point1, point2);
			g_debugDraw.DrawSegment(point1, point2, b2Color(0.8, 0.8, 0.8));

			for (int32 i = 0; i < callback.m_count; ++i)
			{
				b2Vec2 p = callback.m_points[i];
				b2Vec2 n = callback.m_normals[i];
				g_debugDraw.DrawPoint(p, 5.0, b2Color(0.4, 0.9, 0.4));
				g_debugDraw.DrawSegment(point1, p, b2Color(0.8, 0.8, 0.8));
				b2Vec2 head = b2Vec2::Add(p, b2Vec2::Multiply(0.5, n));
				g_debugDraw.DrawSegment(p, head, b2Color(0.9, 0.9, 0.4));
			}
		}

		if (advanceRay)
		{
			this->m_angle += 0.25f * b2_pi / 180.0;
		}

#if 0
		// This case was failing.
		{
			b2Vec2 vertices[4];
			//vertices[0].Set(-22.875f, -3.0);
			//vertices[1].Set(22.875f, -3.0);
			//vertices[2].Set(22.875f, 3.0);
			//vertices[3].Set(-22.875f, 3.0);

			b2PolygonShape shape;
			//shape.Set(vertices, 4);
			shape.SetAsBox(22.875f, 3.0);

			b2RayCastInput input;
			input.p1.Set(10.2725f,1.71372f);
			input.p2.Set(10.2353f,2.21807f);
			//input.maxFraction = 0.567623f;
			input.maxFraction = 0.56762173f;

			b2Transform xf;
			xf.SetIdentity();
			xf.position.Set(23.0, 5.0);

			b2RayCastOutput output;
			bool hit;
			hit = shape.RayCast(&output, input, xf);
			hit = false;

			b2Color color(1.0, 1.0, 1.0);
			b2Vec2 vs[4];
			for (int32 i = 0; i < 4; ++i)
			{
				vs[i] = b2Mul(xf, shape.m_vertices[i]);
			}

			g_debugDraw.DrawPolygon(vs, 4, color);
			g_debugDraw.DrawSegment(input.p1, input.p2, color);
		}
#endif
	}

	static Test* Create()
	{
		return new RayCast;
	}

	int32 m_bodyIndex;
	b2Body* m_bodies[e_maxBodies];
	int32 m_userData[e_maxBodies];
	b2PolygonShape m_polygons[4];
	b2CircleShape m_circle;
	b2EdgeShape m_edge;

	float32 m_angle;

	Mode m_mode;
};

#endif
