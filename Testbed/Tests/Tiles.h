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

#ifndef TILES_H
#define TILES_H

/// This stress tests the dynamic tree broad-phase. This also shows that tile
/// based collision is _not_ smooth due to Box2D not knowing about adjacency.
class Tiles : public Test
{
public:
	enum
	{
		e_count = 20
	};

	Tiles()
	{
		this->m_fixtureCount = 0;
		b2Timer timer;

		{
			float32 a = 0.5;
			b2BodyDef bd;
			bd.position.y = -a;
			b2Body* ground = this->m_world->CreateBody(&bd);

#if 1
			int32 N = 200;
			int32 M = 10;
			b2Vec2 position;
			position.y = 0.0;
			for (int32 j = 0; j < M; ++j)
			{
				position.x = -N * a;
				for (int32 i = 0; i < N; ++i)
				{
					b2PolygonShape shape;
					shape.SetAsBox(a, a, position, 0.0);
					ground->CreateFixture(&shape, 0.0);
					++this->m_fixtureCount;
					position.x += 2.0 * a;
				}
				position.y -= 2.0 * a;
			}
#else
			int32 N = 200;
			int32 M = 10;
			b2Vec2 position;
			position.x = -N * a;
			for (int32 i = 0; i < N; ++i)
			{
				position.y = 0.0;
				for (int32 j = 0; j < M; ++j)
				{
					b2PolygonShape shape;
					shape.SetAsBox(a, a, position, 0.0);
					ground->CreateFixture(&shape, 0.0);
					position.y -= 2.0 * a;
				}
				position.x += 2.0 * a;
			}
#endif
		}

		{
			float32 a = 0.5;
			b2PolygonShape shape;
			shape.SetAsBox(a, a);

			b2Vec2 x(-7.0, 0.75f);
			b2Vec2 y;
			b2Vec2 deltaX(0.5625f, 1.25f);
			b2Vec2 deltaY(1.125f, 0.0);

			for (int32 i = 0; i < e_count; ++i)
			{
				y.Assign(x);

				for (int32 j = i; j < e_count; ++j)
				{
					b2BodyDef bd;
					bd.type = b2Body::b2_dynamicBody;
					bd.position.Assign(y);

					//if (i == 0 && j == 0)
					//{
					//	bd.allowSleep = false;
					//}
					//else
					//{
					//	bd.allowSleep = true;
					//}

					b2Body* body = this->m_world->CreateBody(&bd);
					body->CreateFixture(&shape, 5.0);
					++this->m_fixtureCount;
					y.Add(deltaY);
				}

				x.Add(deltaX);
			}
		}

		this->m_createTime = timer.GetMilliseconds();
	}

	void Step(Settings* settings)
	{
		const b2ContactManager& cm = this->m_world->GetContactManager();
		int32 height = cm.m_broadPhase.GetTreeHeight();
		int32 leafCount = cm.m_broadPhase.GetProxyCount();
		int32 minimumNodeCount = 2 * leafCount - 1;
		float32 minimumHeight = ceilf(logf(float32(minimumNodeCount)) / logf(2.0));
		g_debugDraw.DrawString(5, this->m_textLine, "dynamic tree height = %d, min = %d", height, int32(minimumHeight));
		this->m_textLine += DRAW_STRING_NEW_LINE;

		Test::Step(settings);

		g_debugDraw.DrawString(5, this->m_textLine, "create time = %6.2 ms, fixture count = %d",
			this->m_createTime, this->m_fixtureCount);
		this->m_textLine += DRAW_STRING_NEW_LINE;

		//b2DynamicTree* tree = &this->m_world->m_contactManager.m_broadPhase.m_tree;

		//if (this->m_stepCount == 400)
		//{
		//	tree->RebuildBottomUp();
		//}
	}

	static Test* Create()
	{
		return new Tiles;
	}

	int32 m_fixtureCount;
	float32 m_createTime;
};

#endif
