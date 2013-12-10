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

#ifndef BODY_TYPES_H
#define BODY_TYPES_H

class BodyTypes : public Test
{
public:
	BodyTypes()
	{
		b2Body* ground = null;
		{
			b2BodyDef bd;
			ground = this->m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-20.0, 0.0), b2Vec2(20.0, 0.0));

			b2FixtureDef fd;
			fd.shape = &shape;

			ground->CreateFixture(&fd);
		}

		// Define attachment
		{
			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;
			bd.position.Set(0.0, 3.0);
			this->m_attachment = this->m_world->CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsBox(0.5, 2.0);
			this->m_attachment->CreateFixture(&shape, 2.0);
		}

		// Define platform
		{
			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;
			bd.position.Set(-4.0, 5.0);
			this->m_platform = this->m_world->CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsBox(0.5, 4.0, b2Vec2(4.0, 0.0), 0.5 * b2_pi);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.friction = 0.6;
			fd.density = 2.0;
			this->m_platform->CreateFixture(&fd);

			b2RevoluteJointDef rjd;
			rjd.Initialize(this->m_attachment, this->m_platform, b2Vec2(0.0, 5.0));
			rjd.maxMotorTorque = 50.0;
			rjd.enableMotor = true;
			this->m_world->CreateJoint(&rjd);

			b2PrismaticJointDef pjd;
			pjd.Initialize(ground, this->m_platform, b2Vec2(0.0, 5.0), b2Vec2(1.0, 0.0));

			pjd.maxMotorForce = 1000.0;
			pjd.enableMotor = true;
			pjd.lowerTranslation = -10.0;
			pjd.upperTranslation = 10.0;
			pjd.enableLimit = true;

			this->m_world->CreateJoint(&pjd);

			this->m_speed = 3.0;
		}

		// Create a payload
		{
			b2BodyDef bd;
			bd.type = b2Body::b2_dynamicBody;
			bd.position.Set(0.0, 8.0);
			b2Body* body = this->m_world->CreateBody(&bd);

			b2PolygonShape shape;
			shape.SetAsBox(0.75f, 0.75f);

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.friction = 0.6;
			fd.density = 2.0;

			body->CreateFixture(&fd);
		}
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_D:
			this->m_platform->SetType(b2Body::b2_dynamicBody);
			break;

		case GLFW_KEY_S:
			this->m_platform->SetType(b2Body::b2_staticBody);
			break;

		case GLFW_KEY_K:
			this->m_platform->SetType(b2Body::b2_kinematicBody);
			this->m_platform->SetLinearVelocity(b2Vec2(-this->m_speed, 0.0));
			this->m_platform->SetAngularVelocity(0.0);
			break;
		}
	}

	void Step(Settings* settings)
	{
		// Drive the kinematic body.
		if (this->m_platform->GetType() == b2Body::b2_kinematicBody)
		{
			b2Vec2 p = this->m_platform->GetTransform().p;
			b2Vec2 v = this->m_platform->GetLinearVelocity();

			if ((p.x < -10.0 && v.x < 0.0) ||
				(p.x > 10.0 && v.x > 0.0))
			{
				v.x = -v.x;
				this->m_platform->SetLinearVelocity(v);
			}
		}

		Test::Step(settings);
		g_debugDraw.DrawString(5, this->m_textLine, "Keys: (d) dynamic, (s) static, (k) kinematic");
		this->m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new BodyTypes;
	}

	b2Body* m_attachment;
	b2Body* m_platform;
	float32 m_speed;
};

#endif
