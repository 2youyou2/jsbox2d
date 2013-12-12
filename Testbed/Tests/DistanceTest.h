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

#ifndef DISTANCE_TEST_H
#define DISTANCE_TEST_H

class DistanceTest : public Test
{
public:
	DistanceTest()
	{
		{
			this->m_transformA.SetIdentity();
			this->m_transformA.p.Set(0.0, -0.2);
			this->m_polygonA.SetAsBox(10.0, 0.2);
		}

		{
			this->m_positionB.Set(12.017401f, 0.13678508f);
			this->m_angleB = -0.0109265f;
			this->m_transformB.Set(this->m_positionB, this->m_angleB);

			this->m_polygonB.SetAsBox(2.0, 0.1);
		}
	}

	static Test* Create()
	{
		return new DistanceTest;
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		b2DistanceInput input;
		input.proxyA.Set(&this->m_polygonA, 0);
		input.proxyB.Set(&this->m_polygonB, 0);
		input.transformA.Assign(this->m_transformA);
		input.transformB.Assign(this->m_transformB);
		input.useRadii = true;
		b2SimplexCache cache;
		cache.count = 0;
		b2DistanceOutput output;
		b2DistanceFunc(&output, &cache, &input);

		g_debugDraw.DrawString(5, this->m_textLine, "distance = %g", output.distance);
		this->m_textLine += DRAW_STRING_NEW_LINE;

		g_debugDraw.DrawString(5, this->m_textLine, "iterations = %d", output.iterations);
		this->m_textLine += DRAW_STRING_NEW_LINE;

		{
			b2Color color(0.9, 0.9, 0.9);
			b2Vec2 v[b2_maxPolygonVertices];
			for (int32 i = 0; i < this->m_polygonA.m_count; ++i)
			{
				v[i].Assign(b2Mul_t_v2(this->m_transformA, this->m_polygonA.m_vertices[i]));
			}
			g_debugDraw.DrawPolygon(v, this->m_polygonA.m_count, color);

			for (int32 i = 0; i < this->m_polygonB.m_count; ++i)
			{
				v[i].Assign(b2Mul_t_v2(this->m_transformB, this->m_polygonB.m_vertices[i]));
			}
			g_debugDraw.DrawPolygon(v, this->m_polygonB.m_count, color);
		}

		b2Vec2 x1 = output.pointA;
		b2Vec2 x2 = output.pointB;

		b2Color c1(1.0, 0.0, 0.0);
		g_debugDraw.DrawPoint(x1, 4.0, c1);

		b2Color c2(1.0, 1.0, 0.0);
		g_debugDraw.DrawPoint(x2, 4.0, c2);
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_A:
			this->m_positionB.x -= 0.1;
			break;

		case GLFW_KEY_D:
			this->m_positionB.x += 0.1;
			break;

		case GLFW_KEY_S:
			this->m_positionB.y -= 0.1;
			break;

		case GLFW_KEY_W:
			this->m_positionB.y += 0.1;
			break;

		case GLFW_KEY_Q:
			this->m_angleB += 0.1 * b2_pi;
			break;

		case GLFW_KEY_E:
			this->m_angleB -= 0.1 * b2_pi;
			break;
		}

		this->m_transformB.Set(this->m_positionB, this->m_angleB);
	}

	b2Vec2 m_positionB;
	float32 m_angleB;

	b2Transform m_transformA;
	b2Transform m_transformB;
	b2PolygonShape m_polygonA;
	b2PolygonShape m_polygonB;
};

#endif
