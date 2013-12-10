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

#ifndef DOMINOS_H
#define DOMINOS_H

class Dominos : public Test
{
public:

	Dominos()
	{
		b2Body* b1;
		{
			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0, 0.0), b2Vec2(40.0, 0.0));

			b2BodyDef bd;
			b1 = this->m_world->CreateBody(&bd);
			b1->CreateFixture(&shape, 0.0);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(6.0, 0.25f);

			b2BodyDef bd;
			bd.position.Set(-1.5, 10.0);
			b2Body* ground = this->m_world->CreateBody(&bd);
			ground->CreateFixture(&shape, 0.0);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(0.1, 1.0);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 20.0;
			fd.friction = 0.1;

			for (int i = 0; i < 10; ++i)
			{
				b2BodyDef bd;
				bd.type = b2Body::b2_dynamicBody;
				bd.position.Set(-6.0 + 1.0 * i, 11.25f);
				b2Body* body = this->m_world->CreateBody(&bd);
				body->CreateFixture(&fd);
			}
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(7.0, 0.25f, b2Vec2(0, 0), 0.3);

			b2BodyDef bd;
			bd.position.Set(1.0, 6.0);
			b2Body* ground = this->m_world->CreateBody(&bd);
			ground->CreateFixture(&shape, 0.0);
		}

		b2Body* b2;
		{
			b2PolygonShape shape;
			shape.SetAsBox(0.25f, 1.5);

			b2BodyDef bd;
			bd.position.Set(-7.0, 4.0);
			b2 = this->m_world->CreateBody(&bd);
			b2->CreateFixture(&shape, 0.0);
		}

		b2Body* b3;
		{
			b2PolygonShape shape;
			shape.SetAsBox(6.0, 0.125f);

			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;
			bd.position.Set(-0.9, 1.0);
			bd.angle = -0.15f;

			b3 = this->m_world->CreateBody(&bd);
			b3->CreateFixture(&shape, 10.0);
		}

		b2RevoluteJointDef jd;
		b2Vec2 anchor;

		anchor.Set(-2.0, 1.0);
		jd.Initialize(b1, b3, anchor);
		jd.collideConnected = true;
		this->m_world->CreateJoint(&jd);

		b2Body* b4;
		{
			b2PolygonShape shape;
			shape.SetAsBox(0.25f, 0.25f);

			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;
			bd.position.Set(-10.0, 15.0);
			b4 = this->m_world->CreateBody(&bd);
			b4->CreateFixture(&shape, 10.0);
		}

		anchor.Set(-7.0, 15.0);
		jd.Initialize(b2, b4, anchor);
		this->m_world->CreateJoint(&jd);

		b2Body* b5;
		{
			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;
			bd.position.Set(6.5, 3.0);
			b5 = this->m_world->CreateBody(&bd);

			b2PolygonShape shape;
			b2FixtureDef fd;

			fd.shape = &shape;
			fd.density = 10.0;
			fd.friction = 0.1;

			shape.SetAsBox(1.0, 0.1, b2Vec2(0.0, -0.9), 0.0);
			b5->CreateFixture(&fd);

			shape.SetAsBox(0.1, 1.0, b2Vec2(-0.9, 0.0), 0.0);
			b5->CreateFixture(&fd);

			shape.SetAsBox(0.1, 1.0, b2Vec2(0.9, 0.0), 0.0);
			b5->CreateFixture(&fd);
		}

		anchor.Set(6.0, 2.0);
		jd.Initialize(b1, b5, anchor);
		this->m_world->CreateJoint(&jd);

		b2Body* b6;
		{
			b2PolygonShape shape;
			shape.SetAsBox(1.0, 0.1);

			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;
			bd.position.Set(6.5, 4.1);
			b6 = this->m_world->CreateBody(&bd);
			b6->CreateFixture(&shape, 30.0);
		}

		anchor.Set(7.5, 4.0);
		jd.Initialize(b5, b6, anchor);
		this->m_world->CreateJoint(&jd);

		b2Body* b7;
		{
			b2PolygonShape shape;
			shape.SetAsBox(0.1, 1.0);

			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;
			bd.position.Set(7.4, 1.0);

			b7 = this->m_world->CreateBody(&bd);
			b7->CreateFixture(&shape, 10.0);
		}

		b2DistanceJointDef djd;
		djd.bodyA = b3;
		djd.bodyB = b7;
		djd.localAnchorA.Set(6.0, 0.0);
		djd.localAnchorB.Set(0.0, -1.0);
		b2Vec2 d = b2Vec2::Subtract(djd.bodyB->GetWorldPoint(djd.localAnchorB), djd.bodyA->GetWorldPoint(djd.localAnchorA));
		djd.length = d.Length();
		this->m_world->CreateJoint(&djd);

		{
			float32 radius = 0.2;

			b2CircleShape shape;
			shape.m_radius = radius;

			for (int32 i = 0; i < 4; ++i)
			{
				b2BodyDef bd;
				bd.type = b2Body::b2_dynamicBody;
				bd.position.Set(5.9 + 2.0 * radius * i, 2.4);
				b2Body* body = this->m_world->CreateBody(&bd);
				body->CreateFixture(&shape, 10.0);
			}
		}
	}

	static Test* Create()
	{
		return new Dominos;
	}
};

#endif
