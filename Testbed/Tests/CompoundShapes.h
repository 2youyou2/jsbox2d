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

#ifndef COMPOUND_SHAPES_H
#define COMPOUND_SHAPES_H

// TODO_ERIN test joints on compounds.
class CompoundShapes : public Test
{
public:
	CompoundShapes()
	{
		{
			b2BodyDef bd;
			bd.position.Set(0.0, 0.0);
			b2Body* body = this->m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(50.0, 0.0), b2Vec2(-50.0, 0.0));

			body->CreateFixture(&shape, 0.0);
		}

		{
			b2CircleShape circle1;
			circle1.m_radius = 0.5;
			circle1.m_p.Set(-0.5, 0.5);

			b2CircleShape circle2;
			circle2.m_radius = 0.5;
			circle2.m_p.Set(0.5, 0.5);

			for (int i = 0; i < 10; ++i)
			{
				float32 x = RandomFloat(-0.1, 0.1);
				b2BodyDef bd;
				bd.type = b2Body::b2_dynamicBody;
				bd.position.Set(x + 5.0, 1.05f + 2.5 * i);
				bd.angle = RandomFloat(-b2_pi, b2_pi);
				b2Body* body = this->m_world->CreateBody(&bd);
				body->CreateFixture(&circle1, 2.0);
				body->CreateFixture(&circle2, 0.0);
			}
		}

		{
			b2PolygonShape polygon1;
			polygon1.SetAsBox(0.25f, 0.5);

			b2PolygonShape polygon2;
			polygon2.SetAsBox(0.25f, 0.5, b2Vec2(0.0, -0.5), 0.5 * b2_pi);

			for (int i = 0; i < 10; ++i)
			{
				float32 x = RandomFloat(-0.1, 0.1);
				b2BodyDef bd;
				bd.type = b2Body::b2_dynamicBody;
				bd.position.Set(x - 5.0, 1.05f + 2.5 * i);
				bd.angle = RandomFloat(-b2_pi, b2_pi);
				b2Body* body = this->m_world->CreateBody(&bd);
				body->CreateFixture(&polygon1, 2.0);
				body->CreateFixture(&polygon2, 2.0);
			}
		}

		{
			b2Transform xf1;
			xf1.q.Set(0.3524f * b2_pi);
			xf1.p = xf1.q.GetXAxis();

			b2Vec2 vertices[3];

			b2PolygonShape triangle1;
			vertices[0] = b2Mul_t_v2(xf1, b2Vec2(-1.0, 0.0));
			vertices[1] = b2Mul_t_v2(xf1, b2Vec2(1.0, 0.0));
			vertices[2] = b2Mul_t_v2(xf1, b2Vec2(0.0, 0.5));
			triangle1.Set(vertices, 3);

			b2Transform xf2;
			xf2.q.Set(-0.3524f * b2_pi);
			xf2.p = xf2.q.GetXAxis().Negate();

			b2PolygonShape triangle2;
			vertices[0] = b2Mul_t_v2(xf2, b2Vec2(-1.0, 0.0));
			vertices[1] = b2Mul_t_v2(xf2, b2Vec2(1.0, 0.0));
			vertices[2] = b2Mul_t_v2(xf2, b2Vec2(0.0, 0.5));
			triangle2.Set(vertices, 3);

			for (int32 i = 0; i < 10; ++i)
			{
				float32 x = RandomFloat(-0.1, 0.1);
				b2BodyDef bd;
				bd.type = b2Body::b2_dynamicBody;
				bd.position.Set(x, 2.05f + 2.5 * i);
				bd.angle = 0.0;
				b2Body* body = this->m_world->CreateBody(&bd);
				body->CreateFixture(&triangle1, 2.0);
				body->CreateFixture(&triangle2, 2.0);
			}
		}

		{
			b2PolygonShape bottom;
			bottom.SetAsBox( 1.5, 0.15f );

			b2PolygonShape left;
			left.SetAsBox(0.15f, 2.7, b2Vec2(-1.45f, 2.35f), 0.2);

			b2PolygonShape right;
			right.SetAsBox(0.15f, 2.7, b2Vec2(1.45f, 2.35f), -0.2);

			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;
			bd.position.Set( 0.0, 2.0 );
			b2Body* body = this->m_world->CreateBody(&bd);
			body->CreateFixture(&bottom, 4.0);
			body->CreateFixture(&left, 4.0);
			body->CreateFixture(&right, 4.0);
		}
	}

	static Test* Create()
	{
		return new CompoundShapes;
	}
};

#endif
