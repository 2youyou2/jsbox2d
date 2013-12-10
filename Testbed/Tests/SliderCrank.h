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

#ifndef SLIDER_CRANK_H
#define SLIDER_CRANK_H

// A motor driven slider crank with joint friction.

class SliderCrank : public Test
{
public:
	SliderCrank()
	{
		b2Body* ground = null;
		{
			b2BodyDef bd;
			ground = this->m_world->CreateBody(&bd);

			b2EdgeShape shape;
			shape.Set(b2Vec2(-40.0, 0.0), b2Vec2(40.0, 0.0));
			ground->CreateFixture(&shape, 0.0);
		}

		{
			b2Body* prevBody = ground;

			// Define crank.
			{
				b2PolygonShape shape;
				shape.SetAsBox(0.5, 2.0);

				b2BodyDef bd;
				bd.type = b2Body::b2_dynamicBody;
				bd.position.Set(0.0, 7.0);
				b2Body* body = this->m_world->CreateBody(&bd);
				body->CreateFixture(&shape, 2.0);

				b2RevoluteJointDef rjd;
				rjd.Initialize(prevBody, body, b2Vec2(0.0, 5.0));
				rjd.motorSpeed = 1.0 * b2_pi;
				rjd.maxMotorTorque = 10000.0;
				rjd.enableMotor = true;
				this->m_joint1 = (b2RevoluteJoint*)this->m_world->CreateJoint(&rjd);

				prevBody = body;
			}

			// Define follower.
			{
				b2PolygonShape shape;
				shape.SetAsBox(0.5, 4.0);

				b2BodyDef bd;
				bd.type = b2Body::b2_dynamicBody;
				bd.position.Set(0.0, 13.0);
				b2Body* body = this->m_world->CreateBody(&bd);
				body->CreateFixture(&shape, 2.0);

				b2RevoluteJointDef rjd;
				rjd.Initialize(prevBody, body, b2Vec2(0.0, 9.0));
				rjd.enableMotor = false;
				this->m_world->CreateJoint(&rjd);

				prevBody = body;
			}

			// Define piston
			{
				b2PolygonShape shape;
				shape.SetAsBox(1.5, 1.5);

				b2BodyDef bd;
				bd.type = b2Body::b2_dynamicBody;
				bd.fixedRotation = true;
				bd.position.Set(0.0, 17.0);
				b2Body* body = this->m_world->CreateBody(&bd);
				body->CreateFixture(&shape, 2.0);

				b2RevoluteJointDef rjd;
				rjd.Initialize(prevBody, body, b2Vec2(0.0, 17.0));
				this->m_world->CreateJoint(&rjd);

				b2PrismaticJointDef pjd;
				pjd.Initialize(ground, body, b2Vec2(0.0, 17.0), b2Vec2(0.0, 1.0));

				pjd.maxMotorForce = 1000.0;
				pjd.enableMotor = true;

				this->m_joint2 = (b2PrismaticJoint*)this->m_world->CreateJoint(&pjd);
			}

			// Create a payload
			{
				b2PolygonShape shape;
				shape.SetAsBox(1.5, 1.5);

				b2BodyDef bd;
				bd.type = b2Body::b2_dynamicBody;
				bd.position.Set(0.0, 23.0);
				b2Body* body = this->m_world->CreateBody(&bd);
				body->CreateFixture(&shape, 2.0);
			}
		}
	}

	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_F:
			this->m_joint2->EnableMotor(!this->m_joint2->IsMotorEnabled());
			this->m_joint2->GetBodyB()->SetAwake(true);
			break;

		case GLFW_KEY_M:
			this->m_joint1->EnableMotor(!this->m_joint1->IsMotorEnabled());
			this->m_joint1->GetBodyB()->SetAwake(true);
			break;
		}
	}

	void Step(Settings* settings)
	{
		Test::Step(settings);
		g_debugDraw.DrawString(5, this->m_textLine, "Keys: (f) toggle friction, (m) toggle motor");
		this->m_textLine += DRAW_STRING_NEW_LINE;
		float32 torque = this->m_joint1->GetMotorTorque(settings->hz);
		g_debugDraw.DrawString(5, this->m_textLine, "Motor Torque = %5.0", (float) torque);
		this->m_textLine += DRAW_STRING_NEW_LINE;
	}

	static Test* Create()
	{
		return new SliderCrank;
	}

	b2RevoluteJoint* m_joint1;
	b2PrismaticJoint* m_joint2;
};

#endif
