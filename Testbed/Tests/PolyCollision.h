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

#ifndef POLYCOLLISION_H
#define POLYCOLLISION_H

class PolyCollision : public Test
{
public:
	PolyCollision()
	{
		{
			this->m_polygonA.SetAsBox(0.2, 0.4);
			this->m_transformA.Set(b2Vec2(0.0, 0.0), 0.0);
		}

		{
			this->m_polygonB.SetAsBox(0.5, 0.5);
			this->m_positionB.Set(19.345284f, 1.5632932f);
			this->m_angleB = 1.9160721f;
			this->m_transformB.Set(this->m_positionB, this->m_angleB);
		}
	}

	static Test* Create()
	{
		return new PolyCollision;
	}

	void Step(Settings* settings)
	{
		B2_NOT_USED(settings);

		b2Manifold manifold;
		b2CollidePolygons(&manifold, &this->m_polygonA, this->m_transformA, &this->m_polygonB, this->m_transformB);

		b2WorldManifold worldManifold;
		worldManifold.Initialize(&manifold, this->m_transformA, this->m_polygonA.m_radius, this->m_transformB, this->m_polygonB.m_radius);

		g_debugDraw.DrawString(5, this->m_textLine, "point count = %d", manifold.pointCount);
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

		for (int32 i = 0; i < manifold.pointCount; ++i)
		{
			g_debugDraw.DrawPoint(worldManifold.points[i], 4.0, b2Color(0.9, 0.3, 0.3));
		}
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

	b2PolygonShape m_polygonA;
	b2PolygonShape m_polygonB;

	b2Transform m_transformA;
	b2Transform m_transformB;

	b2Vec2 m_positionB;
	float32 m_angleB;
};

#endif
