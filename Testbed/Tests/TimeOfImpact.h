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

#ifndef TIME_OF_IMPACT_H
#define TIME_OF_IMPACT_H

class TimeOfImpact : public Test
{
public:
	TimeOfImpact()
	{
		this->m_shapeA.SetAsBox(25.0, 5.0);
		this->m_shapeB.SetAsBox(2.5, 2.5);
	}

	static Test* Create()
	{
		return new TimeOfImpact;
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);

		b2Sweep sweepA;
		sweepA.c0.Set(24.0, -60.0);
		sweepA.a0 = 2.95f;
		sweepA.c = sweepA.c0;
		sweepA.a = sweepA.a0;
		sweepA.localCenter.SetZero();

		b2Sweep sweepB;
		sweepB.c0.Set(53.474274f, -50.252514f);
		sweepB.a0 = 513.36676f; // - 162.0 * b2_pi;
		sweepB.c.Set(54.595478f, -51.083473f);
		sweepB.a = 513.62781f; //  - 162.0 * b2_pi;
		sweepB.localCenter.SetZero();

		//sweepB.a0 -= 300.0 * b2_pi;
		//sweepB.a -= 300.0 * b2_pi;

		b2TOIInput input;
		input.proxyA.Set(&this->m_shapeA, 0);
		input.proxyB.Set(&this->m_shapeB, 0);
		input.sweepA = sweepA;
		input.sweepB = sweepB;
		input.tMax = 1.0;

		b2TOIOutput output;

		b2TimeOfImpact(&output, &input);

		g_debugDraw.DrawString(5, this->m_textLine, "toi = %g", output.t);
		this->m_textLine += DRAW_STRING_NEW_LINE;

		extern int32 b2_toiMaxIters, b2_toiMaxRootIters;
		g_debugDraw.DrawString(5, this->m_textLine, "max toi iters = %d, max root iters = %d", b2_toiMaxIters, b2_toiMaxRootIters);
		this->m_textLine += DRAW_STRING_NEW_LINE;

		b2Vec2 vertices[b2_maxPolygonVertices];

		b2Transform transformA;
		sweepA.GetTransform(&transformA, 0.0);
		for (int32 i = 0; i < this->m_shapeA.m_count; ++i)
		{
			vertices[i] = b2Mul_t_v2(transformA, this->m_shapeA.m_vertices[i]);
		}
		g_debugDraw.DrawPolygon(vertices, this->m_shapeA.m_count, b2Color(0.9, 0.9, 0.9));

		b2Transform transformB;
		sweepB.GetTransform(&transformB, 0.0);
		
		//b2Vec2 localPoint(2.0, -0.1);

		for (int32 i = 0; i < this->m_shapeB.m_count; ++i)
		{
			vertices[i] = b2Mul_t_v2(transformB, this->m_shapeB.m_vertices[i]);
		}
		g_debugDraw.DrawPolygon(vertices, this->m_shapeB.m_count, b2Color(0.5, 0.9, 0.5));

		sweepB.GetTransform(&transformB, output.t);
		for (int32 i = 0; i < this->m_shapeB.m_count; ++i)
		{
			vertices[i] = b2Mul_t_v2(transformB, this->m_shapeB.m_vertices[i]);
		}
		g_debugDraw.DrawPolygon(vertices, this->m_shapeB.m_count, b2Color(0.5, 0.7, 0.9));

		sweepB.GetTransform(&transformB, 1.0);
		for (int32 i = 0; i < this->m_shapeB.m_count; ++i)
		{
			vertices[i] = b2Mul_t_v2(transformB, this->m_shapeB.m_vertices[i]);
		}
		g_debugDraw.DrawPolygon(vertices, this->m_shapeB.m_count, b2Color(0.9, 0.5, 0.5));

#if 0
		for (float32 t = 0.0; t < 1.0; t += 0.1)
		{
			sweepB.GetTransform(&transformB, t);
			for (int32 i = 0; i < this->m_shapeB.m_count; ++i)
			{
				vertices[i] = b2Mul(transformB, this->m_shapeB.m_vertices[i]);
			}
			g_debugDraw.DrawPolygon(vertices, this->m_shapeB.m_count, b2Color(0.9, 0.5, 0.5));
		}
#endif
	}

	b2PolygonShape m_shapeA;
	b2PolygonShape m_shapeB;
};

#endif
