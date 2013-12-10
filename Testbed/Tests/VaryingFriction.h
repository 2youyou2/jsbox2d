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

#ifndef VARYING_FRICTION_H
#define VARYING_FRICTION_H

class VaryingFriction : public Test
{
public:

	VaryingFriction()
	{
		{
			b2BodyDef bd;
			b2Body* ground = this->m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0, 0.0), b2Vec2(40.0, 0.0));
			ground->CreateFixture(&shape, 0.0);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(13.0, 0.25f);

			b2BodyDef bd;
			bd.position.Set(-4.0, 22.0);
			bd.angle = -0.25f;

			b2Body* ground = this->m_world->CreateBody(&bd);
			ground->CreateFixture(&shape, 0.0);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(0.25f, 1.0);

			b2BodyDef bd;
			bd.position.Set(10.5, 19.0);

			b2Body* ground = this->m_world->CreateBody(&bd);
			ground->CreateFixture(&shape, 0.0);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(13.0, 0.25f);

			b2BodyDef bd;
			bd.position.Set(4.0, 14.0);
			bd.angle = 0.25f;

			b2Body* ground = this->m_world->CreateBody(&bd);
			ground->CreateFixture(&shape, 0.0);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(0.25f, 1.0);

			b2BodyDef bd;
			bd.position.Set(-10.5, 11.0);

			b2Body* ground = this->m_world->CreateBody(&bd);
			ground->CreateFixture(&shape, 0.0);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(13.0, 0.25f);

			b2BodyDef bd;
			bd.position.Set(-4.0, 6.0);
			bd.angle = -0.25f;

			b2Body* ground = this->m_world->CreateBody(&bd);
			ground->CreateFixture(&shape, 0.0);
		}

		{
			b2PolygonShape shape;
			shape.SetAsBox(0.5, 0.5);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 25.0;

			float friction[5] = {0.75f, 0.5, 0.35f, 0.1, 0.0};

			for (int i = 0; i < 5; ++i)
			{
				b2BodyDef bd;
				bd.type = b2Body::b2_dynamicBody;
				bd.position.Set(-15.0 + 4.0 * i, 28.0);
				b2Body* body = this->m_world->CreateBody(&bd);

				fd.friction = friction[i];
				body->CreateFixture(&fd);
			}
		}
	}

	static Test* Create()
	{
		return new VaryingFriction;
	}
};

#endif
